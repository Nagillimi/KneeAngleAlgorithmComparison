/* Gait cycle mimicking code for stepper motors
  Since the gear ratio of these motors is actually 63.68395. Each stepper takes about 240mA of 
  current at 5V in ALL operating conditions.
  
  ------------------------------------------------------------------------------------------------------------
  |    BAC 1    |     BAC 2     |    BAC 3    |    BAC 4    |   BAC 5   |   BAC 6   |   BAC 7   |   BAC 8    |
  | 1st Contact | Load Response | Mid  Stance | Term Stance | Pre Swing | 1st Swing | Mid Swing | Term Swing |
  ------------------------------------------------------------------------------------------------------------

  Notes:
  - pin assignment for Int1, Int2, Int3, Int4 is 1,3,2,4 in order on MCU.
  - HALF4WIRE is 0.9deg accuracy -> 4096 steps/rev
  - FULL4WIRE is 1.8deg accuracy -> 2048 steps/rev

*/
#include "AccelStepper.h"
#include "Wire.h"

AccelStepper knee_stepper(AccelStepper::FULL4WIRE, 8, 10, 9, 11); // 8 - 11
AccelStepper hip_stepper(AccelStepper::FULL4WIRE, 0, 2, 1, 3); // 0 - 3
AccelStepper trigger_stepper(AccelStepper::FULL4WIRE, 4, 6, 5, 7); // 4 - 7

int home_pin = 13;
const int strides = 100;
long knee_;

void setup() {
  // Begin as slave address 9:
  Wire.begin(9);
  Wire.setSDA(18);
  Wire.setSCL(19);
//  Wire.onReceive(receiveEvent);
  Wire.onRequest(requestEvent);
  
  hip_stepper.setMaxSpeed(500.0);
  knee_stepper.setMaxSpeed(500.0);
  trigger_stepper.setMaxSpeed(1000.0);

  hip_stepper.setAcceleration(200.0);
  knee_stepper.setAcceleration(200.0);
  trigger_stepper.setAcceleration(200.0);

  Serial.begin(9600);
  delay(100);
  Serial.println(
    "Stepper Motor Gait System"
    "\nMimicks the full gait cycle of one leg from stance, stride, swing"
    "\nBAC phases are tracked and recorded over Serial.\n"
    "\nPress the start button to begin...\n"
  );
  
  pinMode(home_pin, INPUT); 
  while(digitalRead(home_pin) == LOW);
  
  delay(100);
  Serial.println("Recalibration");
  
  setCalibrationPosition();
  triggerCalibration();

  Serial.println("Calibrated.\n");
  delay(500);
}

void loop() {
  runTrial(); 
  Serial.println("Trial Done."); 
  while(1);
}

void requestEvent() {
  int val = (int)(knee_);
  byte bytes[4];

  bytes[0] = (val >> 24) & 0xFF;
  bytes[1] = (val >> 16) & 0xFF;
  bytes[2] = (val >> 8) & 0xFF;
  bytes[3] = val & 0xFF;

  Wire.write(bytes, sizeof(bytes));
}

// Calibration "Homing" procedure
// Moves both arms into a mechanical stop, and sets position as zero.
// Then moves both arms to the center.

// Notes:
// - runToPosition() is dodgy, replace with run() & while loops
// - use 1024 as center if using HALF4WIRE mode, 512 for FULL4WIRE
void setCalibrationPosition() {
  // Set hip joint parameters for calibration position
  Serial.print("Moving thigh...");
  hip_stepper.setSpeed(100);
  hip_stepper.moveTo(1024);
  while(hip_stepper.currentPosition() != 1024) {
    hip_stepper.run();
    delay(4);
  }
  Serial.println("\t\tDone");
  delay(500);

  // Set knee joint parameters for calibration position
  Serial.print("Moving shank..."); 
  knee_stepper.setSpeed(100);
  knee_stepper.moveTo(1024);
  while(knee_stepper.currentPosition() != 1024) {
    knee_stepper.run();
    delay(4);
  }
  Serial.println("\t\tDone");

  // Resest current hip and knee positions as zero
  knee_stepper.setCurrentPosition(540);
  knee_stepper.setMaxSpeed(1000.0);
  knee_stepper.setAcceleration(200.0);
  hip_stepper.setCurrentPosition(540);
  hip_stepper.setMaxSpeed(1000.0);
  hip_stepper.setAcceleration(200.0);
  delay(500);

  // Move to 0
  Serial.print("Resetting both to 0...");
  knee_stepper.setSpeed(100);
  knee_stepper.moveTo(0);
  hip_stepper.setSpeed(100);
  hip_stepper.moveTo(0);
  while(knee_stepper.currentPosition() != 0 && hip_stepper.currentPosition() != 0) {
    knee_stepper.run();
    knee_ = knee_stepper.currentPosition();
    delay(5);
    hip_stepper.run();
    delay(5);
  }
  Serial.println("\tDone");
  delay(500);
}

void triggerCalibration() {
  Serial.print("Calibrating Trigger...");
  trigger_stepper.setSpeed(1000);
  trigger_stepper.moveTo(-500);
  while(trigger_stepper.currentPosition() != -500) {
    trigger_stepper.run();
    delay(4);
  }
  Serial.println("\tDone");

  // Reset trigger position as zero
  trigger_stepper.setCurrentPosition(0);
  trigger_stepper.setMaxSpeed(1000.0);
  trigger_stepper.setAcceleration(200.0);
  delay(500);
  
  Serial.print("Loading Impulse...");
  trigger_stepper.setSpeed(1000);
  trigger_stepper.moveTo(2090);
  while(trigger_stepper.currentPosition() != 2090) {
    trigger_stepper.run();
    delay(4);
  }
  Serial.println("\tDone");
  delay(500);

  Serial.print("Firing Impulse...");
  trigger_stepper.setSpeed(1000);
  trigger_stepper.moveTo(2120);
  while(trigger_stepper.currentPosition() != 2120) {
    trigger_stepper.run();
    delay(4);
  }
  Serial.println("\tDone");
  delay(500);
  
}

// Cycles through Gait Phases BAC 1 to BAC 8
void runTrial() {
  for(int i = 1; i <= strides; i++) {
      
    Serial.print("Step "); Serial.print(i);
    // BAC 1
    Serial.print(": BAC 1");
    hip_stepper.setSpeed(1000);
    hip_stepper.moveTo(-150);
    while(hip_stepper.currentPosition() != -150) {
      hip_stepper.run();
      delay(5);
    }
  
    // BAC 2
    Serial.print("..2");
    knee_stepper.setSpeed(1000);
    knee_stepper.moveTo(150); 
    hip_stepper.setSpeed(1000);
    hip_stepper.moveTo(-300);
    while(knee_stepper.currentPosition() != 150 && hip_stepper.currentPosition() != -300) {
      knee_stepper.run();
      knee_ = knee_stepper.currentPosition();
      delay(5);
      hip_stepper.run();
      delay(5);
    }
  
    // BAC 3
    Serial.print("..3");
    knee_stepper.setSpeed(1000);
    knee_stepper.moveTo(0);
    hip_stepper.setSpeed(1000);
    hip_stepper.moveTo(0);
    while(knee_stepper.currentPosition() != 0 && hip_stepper.currentPosition() != 0) {
      knee_stepper.run();
      knee_ = knee_stepper.currentPosition();
      delay(5);
      hip_stepper.run();
      delay(5);
    }
  
    // BAC 4
    Serial.print("..4");
    hip_stepper.setSpeed(1000);
    hip_stepper.moveTo(150);
    while(hip_stepper.currentPosition() != 150) {
      hip_stepper.run();
      delay(5);
    }
  
    // BAC 5
    Serial.print("..5");
    knee_stepper.setSpeed(1000);
    knee_stepper.moveTo(150);
    hip_stepper.setSpeed(1000);
    hip_stepper.moveTo(200);
    while(knee_stepper.currentPosition() != 150 && hip_stepper.currentPosition() != 200) {
      knee_stepper.run();
      knee_ = knee_stepper.currentPosition();
      delay(5);
      hip_stepper.run();
      delay(5);
    }
  
    // BAC 6
    Serial.print("..6");
    knee_stepper.setSpeed(1000);
    knee_stepper.moveTo(300);
    hip_stepper.setSpeed(1000);
    hip_stepper.moveTo(50);
    while(knee_stepper.currentPosition() != 300 && hip_stepper.currentPosition() != 50) {
      knee_stepper.run();
      knee_ = knee_stepper.currentPosition();
      delay(5);
      hip_stepper.run();
      delay(5);
    }
  
    // BAC 7
    Serial.print("..7");
    knee_stepper.setSpeed(1000);
    knee_stepper.moveTo(350);
    hip_stepper.setSpeed(1000);
    hip_stepper.moveTo(-150);
    while(knee_stepper.currentPosition() != 350 && hip_stepper.currentPosition() != -150) {
      knee_stepper.run();
      knee_ = knee_stepper.currentPosition();
      delay(5);
      hip_stepper.run();
      delay(5);
    }

    // Reset position before BAC 8, the reset
    knee_stepper.setCurrentPosition(350);
    knee_stepper.setMaxSpeed(1000.0);
    knee_stepper.setAcceleration(200.0);
    hip_stepper.setCurrentPosition(-150);
    hip_stepper.setMaxSpeed(1000.0);
    hip_stepper.setAcceleration(200.0);
  
    // BAC 8
    Serial.println("..8");
    knee_stepper.setSpeed(1000);
    knee_stepper.moveTo(0);
    hip_stepper.setSpeed(1000);
    hip_stepper.moveTo(0);
    while(knee_stepper.currentPosition() != 0 && hip_stepper.currentPosition() != 0) {
      knee_stepper.run();
      knee_ = knee_stepper.currentPosition();
      delay(5);
      hip_stepper.run();
      delay(5);
    }
  }
    
  // Resetting position
  Serial.println("Resetting Position...");
  knee_stepper.setSpeed(1000);
  knee_stepper.moveTo(0);
  hip_stepper.setSpeed(1000);
  hip_stepper.moveTo(0);
  while(knee_stepper.currentPosition() != 0 && hip_stepper.currentPosition() != 0) {
    knee_stepper.run();
    knee_ = knee_stepper.currentPosition();
    delay(5);
    hip_stepper.run();
    delay(5);
  }  
}
