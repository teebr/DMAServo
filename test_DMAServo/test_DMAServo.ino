#include "DMAServo.h"

DMAServo servos;
void setup() {
  Serial.begin(115200);
  servos.init(true); //true: show timings of each PWM
  servos.writeMicroseconds(1,1234.56);
  servos.writeAngle(2,78);
  Serial.println("Setup complete");
}

void loop() {
  servos.checkSerial();
  delay(50);

}
