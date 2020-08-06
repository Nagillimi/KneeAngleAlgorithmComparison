/* Gait cycle mimicking code for stepper motors

  Notes:
  - pin assignment for Int1, Int2, Int3, Int4 is 1,3,2,4 in order.
  - Working on realtime angle output of knee_stepper

*/
#include <AccelStepper.h>

#define STEPS_PER_REV 8
#define GEAR_RED 64

//const int angle measurement
const int STEPS_PER_OUTPUT_REV = STEPS_PER_REV * GEAR_RED;
AccelStepper knee_stepper(STEPS_PER_REV, 1, 3, 2, 4);

void setup() {
  knee_stepper.setMaxSpeed(1000.0);
  knee_stepper.setAcceleration(1000.0);
  knee_stepper.setSpeed(200);
  knee_stepper.moveTo(512);
}

void loop() {
//  knee_stepper.setSpeed(1000);
//  knee_stepper.step(100);
//  delay(500);

  if (knee_stepper.distanceToGo() == 0) 
    knee_stepper.moveTo(-knee_stepper.currentPosition());
    
  knee_stepper.run();
}
