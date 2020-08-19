/* Gait cycle mimicking code for stepper motors
  Since the gear ratio of these motors is actually 63.68395

  Notes:
  - pin assignment for Int1, Int2, Int3, Int4 is 1,3,2,4 in order on MCU.
  - HALF4WIRE is 0.9deg accuracy -> 4096 steps/rev
  - FULL4WIRE is 1.8deg accuracy -> 2048 steps/rev

*/
#include <AccelStepper.h>

AccelStepper knee_stepper(AccelStepper::FULL4WIRE, 8, 14, 9, 15); // 9 - 15
AccelStepper hip_stepper(AccelStepper::FULL4WIRE, 4, 6, 5, 7); // 4 - 7
AccelStepper trigger_stepper(AccelStepper::HALF4WIRE, 0, 2, 1, 3); // 0 - 3

int home_pin = 23;

void setup() {
  hip_stepper.setMaxSpeed(500.0);
  knee_stepper.setMaxSpeed(500.0);
  trigger_stepper.setMaxSpeed(1000.0);

  hip_stepper.setAcceleration(200.0);
  knee_stepper.setAcceleration(200.0);
  trigger_stepper.setAcceleration(200.0);
  
  pinMode(home_pin, INPUT);

// Uncomment for button prompt  
//  while(digitalRead(home_pin) == LOW);
  
  Serial.begin(9600);
  delay(50);
  Serial.println("\nRecalibrating...\n");
  
  setCalibrationPosition();
  triggerTest();

  Serial.print( 
    "\nSystem is ready"
    "\nPress button to recalibrate...\n"
  );
}

void loop() {
  // Home button for calibration to reset to home position
  int recalibrate = digitalRead(home_pin);
  if(recalibrate) {
    Serial.println("\nRecalibrating...\n");
    setCalibrationPosition();
    delay(50);
    triggerTest();
    Serial.println("Done");
  }  

  setNormalGait();  



}

// Calibration "Homing" procedure
// Moves both arms into a mechanical stop, and sets position as zero.
// Then moves both arms to the center.

// Notes:
// - runToPosition() is dodgy, replace with run() & while loops
// - use 1024 as center if using HALF4WIRE mode, 512 for FULL4WIRE
void setCalibrationPosition() {
  // Set hip joint parameters for calibration position
  hip_stepper.setSpeed(100);
  hip_stepper.moveTo(1024);
  while(hip_stepper.currentPosition() != 1024) {
    hip_stepper.run();
    delay(5);
  }

  // Set knee joint parameters for calibration position  
  knee_stepper.setSpeed(100);
  knee_stepper.moveTo(1024);
  while(knee_stepper.currentPosition() != 1024) {
    knee_stepper.run();
    delay(5);
  }

  // Resest current hip and knee positions as zero
  knee_stepper.setCurrentPosition(540);
  knee_stepper.setMaxSpeed(1000.0);
  knee_stepper.setAcceleration(200.0);
  hip_stepper.setCurrentPosition(540);
  hip_stepper.setMaxSpeed(1000.0);
  hip_stepper.setAcceleration(200.0);

  // Move to 0
  knee_stepper.setSpeed(100);
  knee_stepper.moveTo(0);
  hip_stepper.setSpeed(100);
  hip_stepper.moveTo(0);
  while(knee_stepper.currentPosition() != 0 && hip_stepper.currentPosition() != 0) {
    knee_stepper.run();
    delay(5);
    hip_stepper.run();
    delay(5);
  }
}


void triggerTest() {
  trigger_stepper.setSpeed(500);
  trigger_stepper.moveTo(4096);
  while(trigger_stepper.currentPosition() != 4096) {
    trigger_stepper.run();
    delay(5);
  }
}

void setNormalGait() {
  knee_stepper.setSpeed(1000);
  knee_stepper.moveTo(300);
  hip_stepper.setSpeed(1000);
  hip_stepper.moveTo(-300);
  while(knee_stepper.currentPosition() != 300 && hip_stepper.currentPosition() != -300) {
    knee_stepper.run();
    delay(5);
    hip_stepper.run();
    delay(5);
  }

  hip_stepper.setSpeed(1000);
  hip_stepper.moveTo(300); 
  knee_stepper.setSpeed(1000);
  knee_stepper.moveTo(0); 
  while(hip_stepper.currentPosition() != 300 && knee_stepper.currentPosition() != 0) {
    hip_stepper.run();
    delay(5);
    knee_stepper.run();
    delay(5);
  }
  delay(10);
}
