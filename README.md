# DMAServo
Servo library for Teensy 3.5/3.6

This library uses direct memory access (DMA) and the programmable delay block (PDB) to control the servo timing.
Switching the digital pins high and low is entirely handled by DMA so there's not CPU involvement (software interrupts and delay loops), which can conflict with other interrupts (e.g. I2C) and make the servo jittery.

Currently it's configured to operate two servos on pins 22 and 23. This can be edited in the code, but currently all the servos need to be on the same GPIO port. I think this can be solved by using a DMA destination buffer rather than a single destination.
If F_BUS is changed from the default of 60MHz, the PDB prescaler / multiplication factor will have to be changed to avoid overflow.

The prinicipal of operation is as follows:
- The PDB counter overflows every 20ms (the servo frame length)
- The PDB has a register to trigger a DMA transfer when the counter reaches the register value. The delay register value is continually incremented forward to the next time a pin has to switched high or low. These values are stored in an array, and updated using the writeMicroseconds command.
- There are three DMA channels:
  - DMA0 triggered by the PDB interrupt. This toggles the GPIO port value of the current pin
  - DMA1 is triggered by DMA0, and sets the next interrupt value to trigger DMA0 at
  - DMA2 is triggered by DMA1, and updates the PDB with the new interrupt value.
  
- For this set-up the timing is as follows:
  - when the counter overflows, pin 22 is set high
  - after [PWM length] microseconds, pin 22 is set low
  - after 3000us (a constant value, could be set a bit lower, maybe 2550us?), pin 23 is set high
  - after 3000+[PWM length]us, pin 23 is set low
  - nothing happens until the PDB counter reaches 20ms
  
