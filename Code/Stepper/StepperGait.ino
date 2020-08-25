/* Gait cycle mimicking code for stepper motors
  Since the gear ratio of these motors is actually 63.68395. Each stepper takes about 240mA of 
  current at 5V in ALL operating conditions.
  
  ------------------------------------------------------------------------------------------------------------
  |    BAC 1    |     BAC 2     |    BAC 3    |    BAC 4    |   BAC 5   |   BAC 6   |   BAC 7   |   BAC 8    |
  | 1st Contact | Load Response | Mid  Stance | Term Stance | Pre Swing | 1st Swing | Mid Swing | Term Swing |
  ------------------------------------------------------------------------------------------------------------

  Notes:
  - pin assignment for Int1, Int2, Int3, Int4 is 1,3,2,4 in order on MCU.
  - HALF4WIRE is 0.09deg accuracy -> 4096 steps/rev EDIT: 4076!
  - FULL4WIRE is 0.18deg accuracy -> 2048 steps/rev EDIT: 2038!

*/
#include "AccelStepper.h"
#include "Wire.h"

// HALF4WIRE -> 4076
// FULL4WIRE -> 2038
#define STEPS_PER_REV 2038

// Custom stepper positions, all reference the CENTER_POS.
// 2038/4 = 509.5
// 2038/2 = 1019
// Note, no decimal though!!
#define CALIBRATION_POS 1019
#define CENTER_POS 509

AccelStepper knee_stepper(AccelStepper::FULL4WIRE, 8, 10, 9, 11); // 8 - 11
AccelStepper hip_stepper(AccelStepper::FULL4WIRE, 0, 2, 1, 3); // 0 - 3
AccelStepper trigger_stepper(AccelStepper::FULL4WIRE, 4, 6, 5, 7); // 4 - 7

int home_pin = 13;
const int strides = 100;
long knee_ = 0;
uint8_t gait_stage = 0, stride_num = 0;

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

// Function for whenever the Master T3.6 calls the TLC over I2C
// Sends 4 Bytes
void requestEvent() {
  // long conversion to 16 bit integer
  int16_t val = (int16_t)(knee_);

  // Encode the data packet
  byte bytes[4];  
  bytes[0] = (val >> 8) & 0xFF;
  bytes[1] = val & 0xFF;
  bytes[2] = gait_stage;
  bytes[3] = stride_num;

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
  knee_stepper.setCurrentPosition(CALIBRATION_POS);
  knee_stepper.setMaxSpeed(1000.0);
  knee_stepper.setAcceleration(200.0);
  hip_stepper.setCurrentPosition(CALIBRATION_POS);
  hip_stepper.setMaxSpeed(1000.0);
  hip_stepper.setAcceleration(200.0);
  delay(500);

  // Move to 0
  Serial.print("Resetting both to 0...");
  knee_stepper.setSpeed(100);
  knee_stepper.moveTo(CENTER_POS);
  hip_stepper.setSpeed(100);
  hip_stepper.moveTo(CENTER_POS);
  while(knee_stepper.currentPosition() != CENTER_POS && hip_stepper.currentPosition() != CENTER_POS) {
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
  trigger_stepper.moveTo(-2000);
  while(trigger_stepper.currentPosition() != -2000) {
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
    stride_num = i;
    
    Serial.print("Step "); Serial.print(i);
    // BAC 1
    Serial.print(": BAC 1");
    gait_stage = 1;
    hip_stepper.setSpeed(1000);
    hip_stepper.moveTo(CENTER_POS-150);
    while(hip_stepper.currentPosition() != CENTER_POS-150) {
      hip_stepper.run();
      delay(5);
    }
  
    // BAC 2
    Serial.print("..2");
    gait_stage = 2;
    knee_stepper.setSpeed(1000);
    knee_stepper.moveTo(CENTER_POS+150); 
    hip_stepper.setSpeed(1000);
    hip_stepper.moveTo(CENTER_POS-300);
    while(knee_stepper.currentPosition() != CENTER_POS+150 && hip_stepper.currentPosition() != CENTER_POS-300) {
      knee_stepper.run();
      knee_ = knee_stepper.currentPosition();
      delay(5);
      hip_stepper.run();
      delay(5);
    }
  
    // BAC 3
    Serial.print("..3");
    gait_stage = 3;
    knee_stepper.setSpeed(1000);
    knee_stepper.moveTo(CENTER_POS);
    hip_stepper.setSpeed(1000);
    hip_stepper.moveTo(CENTER_POS);
    while(knee_stepper.currentPosition() != CENTER_POS && hip_stepper.currentPosition() != CENTER_POS) {
      knee_stepper.run();
      knee_ = knee_stepper.currentPosition();
      delay(5);
      hip_stepper.run();
      delay(5);
    }
  
    // BAC 4
    Serial.print("..4");
    gait_stage = 4;
    hip_stepper.setSpeed(1000);
    hip_stepper.moveTo(CENTER_POS+150);
    while(hip_stepper.currentPosition() != CENTER_POS+150) {
      hip_stepper.run();
      delay(5);
    }
  
    // BAC 5
    Serial.print("..5");
    gait_stage = 5;
    knee_stepper.setSpeed(1000);
    knee_stepper.moveTo(CENTER_POS+150);
    hip_stepper.setSpeed(1000);
    hip_stepper.moveTo(CENTER_POS+200);
    while(knee_stepper.currentPosition() != CENTER_POS+150 && hip_stepper.currentPosition() != CENTER_POS+200) {
      knee_stepper.run();
      knee_ = knee_stepper.currentPosition();
      delay(5);
      hip_stepper.run();
      delay(5);
    }
  
    // BAC 6
    Serial.print("..6");
    gait_stage = 6;
    knee_stepper.setSpeed(1000);
    knee_stepper.moveTo(CENTER_POS+300);
    hip_stepper.setSpeed(1000);
    hip_stepper.moveTo(CENTER_POS+50);
    while(knee_stepper.currentPosition() != CENTER_POS+300 && hip_stepper.currentPosition() != CENTER_POS+50) {
      knee_stepper.run();
      knee_ = knee_stepper.currentPosition();
      delay(5);
      hip_stepper.run();
      delay(5);
    }
  
    // BAC 7
    Serial.print("..7");
    gait_stage = 7;
    knee_stepper.setSpeed(1000);
    knee_stepper.moveTo(CENTER_POS+350);
    hip_stepper.setSpeed(1000);
    hip_stepper.moveTo(CENTER_POS-150);
    while(knee_stepper.currentPosition() != CENTER_POS+350 && hip_stepper.currentPosition() != CENTER_POS-150) {
      knee_stepper.run();
      knee_ = knee_stepper.currentPosition();
      delay(5);
      hip_stepper.run();
      delay(5);
    }

    // Reset position before BAC 8, the reset
    knee_stepper.setCurrentPosition(CENTER_POS+350);
    knee_stepper.setMaxSpeed(1000.0);
    knee_stepper.setAcceleration(200.0);
    hip_stepper.setCurrentPosition(CENTER_POS-150);
    hip_stepper.setMaxSpeed(1000.0);
    hip_stepper.setAcceleration(200.0);
  
    // BAC 8
    Serial.println("..8");
    gait_stage = 8;
    knee_stepper.setSpeed(1000);
    knee_stepper.moveTo(CENTER_POS);
    hip_stepper.setSpeed(1000);
    hip_stepper.moveTo(CENTER_POS);
    while(knee_stepper.currentPosition() != CENTER_POS && hip_stepper.currentPosition() != CENTER_POS) {
      knee_stepper.run();
      knee_ = knee_stepper.currentPosition();
      delay(5);
      hip_stepper.run();
      delay(5);
    }
  }
    
  // Resetting position
  Serial.println("Resetting Position...");
  gait_stage = 255;
  knee_stepper.setSpeed(1000);
  knee_stepper.moveTo(CENTER_POS);
  hip_stepper.setSpeed(1000);
  hip_stepper.moveTo(CENTER_POS);
  while(knee_stepper.currentPosition() != CENTER_POS && hip_stepper.currentPosition() != CENTER_POS) {
    knee_stepper.run();
    knee_ = knee_stepper.currentPosition();
    delay(5);
    hip_stepper.run();
    delay(5);
  }  
}
