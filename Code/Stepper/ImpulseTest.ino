#include <AccelStepper.h>

AccelStepper trigger_stepper(AccelStepper::FULL4WIRE, 0, 2, 1, 3); // 0 - 3

void setup() {
  pinMode(23, INPUT);
  trigger_stepper.setMaxSpeed(1000.0);
  trigger_stepper.setAcceleration(500.0);

  while(digitalRead(23) == LOW) {}
  
  triggerTest(); 
}

void loop() {}

void triggerTest() {
  while(true) {
    trigger_stepper.setSpeed(1000);
    trigger_stepper.moveTo(1000000);
    while(trigger_stepper.currentPosition() != 1000000) {
      trigger_stepper.run();
      delay(3);
    }

    trigger_stepper.setCurrentPosition(0);
    trigger_stepper.setMaxSpeed(1000.0);
    trigger_stepper.setAcceleration(500.0);
  }
}
