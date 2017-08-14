#include <Arduino.h>
#include "DMAChannel.h"
#include "DMAServo.h"
DMAChannel dma0;
DMAChannel dma1;
DMAChannel dma2;

const int NUM_SERVOS = 2; //DO NOT CHANGE!
const int SERVO_PIN_1 = 22; //DO NOT CHANGE!
const int SERVO_PIN_2 = 23; //DO NOT CHANGE!
const float MIN_PWM_US = 500.0; //TODO: make this a variable set by user
const float MAX_PWM_US = 2500.0;
const float US_TO_DEG = (MAX_PWM_US - MIN_PWM_US) / 180.0;

/*  The PDB has a 16 bit counter, which can trigger a DMA transfer when the counter
    reaches the value in the IDLY register. We want the modulus of the counter to be
    20ms (the servo frame length), and ideally close to the 65535 limit, as this will
    improve the resolution of the PWM signal. For FBus at 60MHz a prescale of 2 and
    multiplication factor of 10 gives 3 ticks per microsecond.
*/
uint32_t PDB_PRESCALE = 2;     //TODO: feed this into PDB_CONFIG macro
uint32_t PDB_MULT_FACTOR = 10; //TODO: feed this into PDB_CONFIG macro

//it's easy to have integer maths errors if you you're not careful where you multiply/ divide
#define usToTicks(us) ((us * (F_BUS / 1000000)) / (PDB_PRESCALE * PDB_MULT_FACTOR))

/* Macro to configure the PDB0_SC register:
    pg 927 of the Teensy 3.5 manual
    PDB_SC_LDMOD(0): allow updates to timing registers immediately, so we can update the trigger times on the fly
    PDB_SC_TRGSEL: Trigger select: Note sure this matters, 15 is a software trigger, closest to DMA?
    PDB_SC_PDBEN: enable the PDB counter
    PDB_DMAEN: enable DMA for the interrupt (default is a software interrupt)
    PDB_SC_MULT(n): multiplication factor for counter: 1,10,20,40
    PDB_SC_PRESCALER(n): prescaler for counter: 2^(n-1)
    PDB_SC_CONT: Enable continuous operation of the counter

    These changes (and changes to some other PDB registers) don't apply until it's been ORd
    with the PDB_SC_LDOK value.
*/

#define PDB_CONFIG (PDB_SC_LDMOD(0) | PDB_SC_TRGSEL(15) | PDB_SC_PDBEN | PDB_SC_DMAEN | PDB_SC_CONT | PDB_SC_PRESCALER(1) | PDB_SC_MULT(1))

uint32_t GPIO_BUFFER[2 * NUM_SERVOS];            //values to write to GPIO_ADDR
volatile uint32_t &GPIO_ADDR = GPIOC_PTOR;       //register you write to to toggle the digital pin states
volatile uint32_t PDB_DLY_TICKS[2 * NUM_SERVOS]; //
uint32_t PDB_SC_UPDATE;                          //value to sent to PDB0_SC when we want to update IDLY

const uint32_t SERVO_OFFSET_TICKS = usToTicks(3000); // value to stagger servo PWMs at (servo N starts at (N-1)*SERVO_OFFSET_TICKS us into the frame)

//optionally show the PWM timings to check servo behaviour
bool showTimings = false;
unsigned long oldTime1, newTime1, oldTime2, newTime2;

DMAServo::DMAServo()
{
}

DMAServo::~DMAServo()
{
}

void DMAServo::init()
{
    _initPins();
    _initDMA();
    _initPDB();
}

void DMAServo::init(bool t)
{
    showTimings = t;
    init();
}

void DMAServo::writeMicroseconds(int servoNum, float val)
{
    //servoNum starts at 1...
    if (val < MIN_PWM_US)
    {
        val = MIN_PWM_US;
    }
    else if (val > MAX_PWM_US)
    {
        val = MAX_PWM_US;
    }

    uint32_t newOnTime = usToTicks((uint32_t)val) & 0xFFFF;
    uint32_t newDelayTime = newOnTime + (servoNum - 1) * SERVO_OFFSET_TICKS;
    while (PDB0_CNT <= newDelayTime)
    {
        //wait to make sure the pin has already been pulled back low this cycle
    }
    PDB_DLY_TICKS[2 * (servoNum - 1)] = newDelayTime;
}

void DMAServo::writeMicroseconds(int servoNum, int val)
{
    writeMicroseconds(servoNum,(float)(val));
}

void DMAServo::writeAngle(int servoNum, float val)
{
    writeMicroseconds(servoNum,MIN_PWM_US + US_TO_DEG*val);
}

void DMAServo::writeAngle(int servoNum, int val)
{
    writeAngle(servoNum,(float)(val));
}

void DMAServo::checkSerial(void)
{
    /* send commands of the form "X YYYY"
    where X is the servo number (1 or 2) and YYYY is the PWM value in writeMicroseconds
    The space can actually be anything, it's just discarded 
    */
    int p = Serial.available();
    if ((p < 5) | (p > 6))
    {
      //incorrect number of bytes: clear the buffer and exit
      while (Serial.available()) {
        int dummy = Serial.read();
      }
      return;
    }
    //Serial is read in as ASCII, there has to be a better way than this
    int val = 0; //PWM value
    int servoNum = Serial.read() - 48;
    if((servoNum < 1) || (servoNum > NUM_SERVOS)) {
      Serial.print("Invalid Servo Number");
      return;
    }
    int dummy = Serial.read(); //the space (or whatever)
    
    //read in the remaining 3-4 bytes and convert to PWM values
    while ((p = Serial.available())) {
      int num =  Serial.read() - 48;
      if (p == 4) {
        val = val + num * 1000;
      }
      else if (p == 3) {
        val = val + num * 100;
      }
      else if ( p == 2) {
        val = val + num * 10;
      }
      else if (p == 1) {
        val = val + num;
      }
    }
    writeMicroseconds(servoNum,val);
}

void DMAServo::_initPDB(void)
{
    SIM_SCGC6 |= SIM_SCGC6_PDB; //enable PDB clock

    PDB0_SC = PDB_CONFIG; //set most of the options
    /* The modulus  (point at which the counter loops back to zero) is the servo frame length
        interrupt delay will be udpated by DMAs, but for now it just needs to be set to a small
        value to trigger the first servo high.
        We then need to reset the software trigger for some reason? (maybe because DMA is not enabled when software triggering is?)
        Finally, write Load OK to update the modulus and delay values. This has be be done with, or after,
        the PDB enable bit has been written (this is done in PDB_CONFIG)
    */
    PDB0_MOD = usToTicks(20000);
    PDB0_IDLY = 0; //usToTicks(10);
    PDB0_SC |= PDB_SC_SWTRIG;
    PDB0_SC |= PDB_SC_LDOK;
}

void DMAServo::_initDMA(void)
{
    // The GPIO Buffer needs the same two values per cell: to toggle the pin high and low
    GPIO_BUFFER[0] = 1 << 1; //servo 1 high
    GPIO_BUFFER[1] = 1 << 1; //servo 1 low
    GPIO_BUFFER[2] = 1 << 2; //servo 2 high
    GPIO_BUFFER[3] = 1 << 2; //servo 2 low

    /* Buffer to update the PDB with new values.
        Note that they are in ascending order, but the last element is 0.
        This is because it's initialised at the point where it will next start at 0
    */
    PDB_DLY_TICKS[0] = usToTicks(1500) & 0xFFFF;                        //servo 1 low
    PDB_DLY_TICKS[1] = SERVO_OFFSET_TICKS & 0xFFFF;                     //servo 2 high
    PDB_DLY_TICKS[2] = (SERVO_OFFSET_TICKS + usToTicks(1500)) & 0xFFFF; //servo 2 low
    PDB_DLY_TICKS[3] = 0;                                               //servo 1 high

    PDB_SC_UPDATE = PDB_CONFIG | PDB_SC_LDOK; //for DMA2

    /* DMA0 is triggered by the PDB interrupt, and toggles a servo's pin.
       DMA0 triggers DMA1, which moves to the next PDB delay
       DMA1 triggers DMA2, which writes to PDB_SC to update the PDB delay value.
  
       The servo pins start low. Servo 1 is set high when the initial PDB delay triggers DMA0
       This will then set the delay value to PDB_DLY_TICKS[0] later, which will then bring the GPIO
       back low. This then continutes to the next servo.
    */

    dma0.sourceBuffer(GPIO_BUFFER, sizeof(GPIO_BUFFER));
    dma0.destination(GPIO_ADDR);
    dma0.triggerAtHardwareEvent(DMAMUX_SOURCE_PDB);

    dma1.sourceBuffer(PDB_DLY_TICKS, sizeof(PDB_DLY_TICKS));
    dma1.destination(PDB0_IDLY);
    dma1.triggerAtTransfersOf(dma0);
    dma1.triggerAtCompletionOf(dma0); //need both

    dma2.source(PDB_SC_UPDATE);
    dma2.destination(PDB0_SC);
    dma2.triggerAtTransfersOf(dma1);
    dma2.triggerAtCompletionOf(dma1); //need both
    dma2.transferCount(1);            //error if not set?

    dma0.enable();
    dma1.enable();
    dma2.enable();
}

void DMAServo::_initPins()
{
    pinMode(SERVO_PIN_1, OUTPUT); //PORTC 1
    pinMode(SERVO_PIN_2, OUTPUT); //PORTC 2

    if (showTimings)
    {
        // check timing of PWM signals using interrupts:
        oldTime1 = micros();
        oldTime2 = micros();
        attachInterrupt(digitalPinToInterrupt(SERVO_PIN_1), DMAServo::_pinTime1, CHANGE);
        attachInterrupt(digitalPinToInterrupt(SERVO_PIN_2), DMAServo::_pinTime2, CHANGE);
    }
}

void DMAServo::_pinTime1(void)
{
    newTime1 = micros();
    int val = digitalReadFast(SERVO_PIN_1);
    if (val == 0)
    {
        //pin has just gone low, let's see how long it was high for
        Serial.print("1: ");
        Serial.println(newTime1 - oldTime1);
    }
    oldTime1 = newTime1;
}

void DMAServo::_pinTime2(void)
{
    newTime2 = micros();
    int val = digitalReadFast(SERVO_PIN_2);
    if (val == 0)
    {
        //pin has just gone low, let's see how long it was high for
        Serial.print("2: ");
        Serial.println(newTime2 - oldTime2);
    }
    oldTime2 = newTime2;
}