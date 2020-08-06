/* Gait cycle mimicking code for stepper motors

  Notes:
  - pin assignment for Int1, Int2, Int3, Int4 is 1,3,2,4 in order on MCU.
  - HALF4WIRE is 0.9deg accuracy -> 4096 steps/rev
  - FULL4WIRE is 1.8deg accuracy -> 2048 steps/rev

*/
#include <AccelStepper.h>

AccelStepper knee_stepper(AccelStepper::HALF4WIRE, 1, 3, 2, 4);
AccelStepper hip_stepper(AccelStepper::HALF4WIRE, 5, 7, 6, 8);

int home_pin = 23;

void setup() {
  hip_stepper.setMaxSpeed(500.0);
  knee_stepper.setMaxSpeed(500.0);

  hip_stepper.setAcceleration(200.0);
  knee_stepper.setAcceleration(200.0);
  
  pinMode(home_pin, INPUT);

  Serial.begin(9600);
  while(!Serial);

  setCalibrationPosition();
}

void loop() {
  // Home button for calibration to reset to home position
  int homie = digitalRead(home_pin);
  if(homie == HIGH) {
    setCalibrationPosition();
  }
  Serial.println(homie);  

  
}

// Calibration "Homing" procedure
// Moves both arms into a mechanical stop, and sets position as zero.
// Then moves both arms to the center.

// Notes:
// - runToPosition() is dodgy, replace with run() & while loops
// - use 1024 as center if using HALF4WIRE mode, 512 for FULL4WIRE
void setCalibrationPosition() {

  // Set knee joint parameters for calibration position  
  knee_stepper.setSpeed(100);
  knee_stepper.moveTo(2048);
  while(knee_stepper.currentPosition() != 2048) {
    knee_stepper.run();
    delay(5);
  }

  // Set hip joint parameters for calibration position
  hip_stepper.setSpeed(100);
  hip_stepper.moveTo(2048);
  while(hip_stepper.currentPosition() != 2048) {
    hip_stepper.run();
    delay(5);
  }

  // Resest current knee position as zero
  knee_stepper.setCurrentPosition(0);
  knee_stepper.setMaxSpeed(500.0);
  knee_stepper.setAcceleration(200.0);
  knee_stepper.setSpeed(100);
  knee_stepper.moveTo(-512);
  while(knee_stepper.currentPosition() != -512) {
    knee_stepper.run();
    delay(5);
  }

  // Resest current hip position as zero
  hip_stepper.setCurrentPosition(0);
  hip_stepper.setMaxSpeed(500.0);
  hip_stepper.setAcceleration(200.0);
  hip_stepper.setSpeed(100);
  hip_stepper.moveTo(-512);
  while(hip_stepper.currentPosition() != -512) {
    hip_stepper.run();
    delay(5);
  }
  
}

void setNormalGait() {
  knee_stepper.setMaxSpeed(1024.0);
  knee_stepper.setAcceleration(200.0);
  knee_stepper.setSpeed(512);
  knee_stepper.moveTo(512);

  hip_stepper.setMaxSpeed(1024.0);
  hip_stepper.setAcceleration(200.0);
  hip_stepper.setSpeed(512);
  hip_stepper.moveTo(-512);
}
