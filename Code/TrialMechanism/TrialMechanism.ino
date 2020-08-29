/* Gait cycle mimicking code for stepper motors
  Since the gear ratio of these motors is actually 63.68395. Each stepper takes about 240mA of 
  current at 5V in ALL operating conditions.
  
  ------------------------------------------------------------------------------------------------------------
  |    BAC 1    |     BAC 2     |    BAC 3    |    BAC 4    |   BAC 5   |   BAC 6   |   BAC 7   |   BAC 8    |
  | 1st Contact | Load Response | Mid  Stance | Term Stance | Pre Swing | 1st Swing | Mid Swing | Term Swing |
  ------HS-------------------------------------------------------TO-------------------------------------------

  Notes:
  - pin assignment for Int1, Int2, Int3, Int4 is 1,3,2,4 in order on MCU.
  - HALF4WIRE is 0.09deg accuracy -> 4096 steps/rev EDIT: 4076!
  - FULL4WIRE is 0.18deg accuracy -> 2048 steps/rev EDIT: 2038!

*/
#include "AccelStepper.h"
#include "Wire.h"

// Number of gait cycles with impulse interruption
#define NUM_TRIALS 2
#define GAIT_IMPULSE_CYCLES 1
#define BETWEEN_TRIAL_DELAY 30000 // 30s

// Custom stepper parameters, all reference the CENTER_POS. No decimals!
#define CALIBRATION_POS 1019
#define HIP_CENTER_POS 480
#define KNEE_CENTER_POS 450
#define IMPULSE_CAL_POS -2200
#define IMPULSE_RESET_POS 1500
#define IMPULSE_LOAD_POS 2115 // 2100 with 1 elastic
#define IMPULSE_FIRE_POS 2140 // 2115 with 1 elastic
#define DELAY_SPEED 4

int firing_delay_usec = DELAY_SPEED * 2000 / 3;

AccelStepper knee_stepper(AccelStepper::FULL4WIRE, 8, 10, 9, 11); // 8 - 11
AccelStepper hip_stepper(AccelStepper::FULL4WIRE, 0, 2, 1, 3); // 0 - 3
AccelStepper trigger_stepper(AccelStepper::FULL4WIRE, 4, 6, 5, 7); // 4 - 7

int home_pin = 13, numTrials = 50;
const int init_strides = 5;
long hip_ = 0, knee_ = 0;
// Gait stage byte is the state tracking byte between MCUs
byte gait_stage = 0x00, impulse_hit = 0;

void setup() {
  // Begin as slave address 9:
  Wire.begin(9);
  Wire.setSDA(18);
  Wire.setSCL(19);
  Wire.onRequest(requestEvent);
  
  hip_stepper.setMaxSpeed(1000.0);
  knee_stepper.setMaxSpeed(1000.0);
  trigger_stepper.setMaxSpeed(1000.0);

  hip_stepper.setAcceleration(400.0);
  knee_stepper.setAcceleration(400.0);
  trigger_stepper.setAcceleration(400.0);

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
  impulseCalibration();
  Serial.println("Calibrated.\n");
  delay(500);
}

void loop() {
  for(int i = 0; i < NUM_TRIALS; i++) {  
    Serial.println("Running Trial...");
    
    // Sends Start flag to T3.6
    gait_stage = 0x09;
   
    runTrial(); 
    Serial.println("\tTrial Done.");

    // Sends done flag to T3.6
    gait_stage = 0x0A; 
    delay(4); // 4000usec
    gait_stage = 0x0B;
  
    // delay between trials
//    delay(BETWEEN_TRIAL_DELAY);

    // Collect the number of trials set
    i++;
    setCalibrationPosition();
    impulseCalibration();
  }
  while(1);
}

// Function for whenever the Master T3.6 calls the TLC over I2C
// Sends 4 Bytes
void requestEvent() {
  // long conversion to 16 bit integer
  int16_t hip = (int16_t)(hip_);
  int16_t knee = (int16_t)(knee_);

  // Encode the data packet
  byte bytes[6];  
  bytes[0] = (hip >> 8) & 0xFF;
  bytes[1] = hip & 0xFF;
  bytes[2] = (knee >> 8) & 0xFF;
  bytes[3] = knee & 0xFF;
  bytes[4] = gait_stage;
  bytes[5] = impulse_hit;

  Wire.write(bytes, sizeof(bytes));
}

// Calibration "Homing" procedure
// Moves both arms into a mechanical stop, and sets position as zero.
// Then moves both arms to the center.

// Notes:
// - runToPosition() is dodgy, replace with run() & while loops
// - use 1024 as center if using HALF4WIRE mode, 512 for FULL4WIRE
void setCalibrationPosition() {
  // Make it easier for the hip stepper to move since the 28BYJ-48
  // have little torque
  knee_stepper.setSpeed(1000);
  knee_stepper.move(400);
  while(knee_stepper.currentPosition() != 400) {
    knee_stepper.run();
    delay(DELAY_SPEED);
  }
  
  // Set hip joint parameters for calibration position
  Serial.print("Moving thigh...");
  hip_stepper.setSpeed(1000);
  hip_stepper.move(1500);
  while(hip_stepper.currentPosition() != 1500) {
    hip_stepper.run();
    delay(DELAY_SPEED);
  }
  hip_stepper.setSpeed(1000);
  hip_stepper.move(1000);
  while(hip_stepper.currentPosition() != 2500) {
    hip_stepper.run();
    delay(DELAY_SPEED);
  }
  Serial.println("\t\tDone");
  delay(500);

  // Set knee joint parameters for calibration position
  Serial.print("Moving shank..."); 
  knee_stepper.setSpeed(1000);
  knee_stepper.move(1000);
  while(knee_stepper.currentPosition() != 1400) {
    knee_stepper.run();
    delay(DELAY_SPEED);
  }
  Serial.println("\t\tDone");

  // Resest current hip and knee positions as zero
  knee_stepper.setCurrentPosition(CALIBRATION_POS);
  knee_stepper.setMaxSpeed(1000.0);
  knee_stepper.setAcceleration(400.0);
  hip_stepper.setCurrentPosition(CALIBRATION_POS);
  hip_stepper.setMaxSpeed(1000.0);
  hip_stepper.setAcceleration(400.0);
  delay(500);

  // Move to 0
  Serial.print("Resetting both to 0...");
  hip_stepper.setSpeed(1000);
  hip_stepper.moveTo(HIP_CENTER_POS);
  while(hip_stepper.currentPosition() != HIP_CENTER_POS) {
    hip_stepper.run();
    hip_ = hip_stepper.currentPosition();
    delay(DELAY_SPEED);
  }
  knee_stepper.setSpeed(1000);
  knee_stepper.moveTo(KNEE_CENTER_POS);
  while(knee_stepper.currentPosition() != KNEE_CENTER_POS) {
    knee_stepper.run();
    delay(DELAY_SPEED);
  }
  Serial.println("\tDone");
  delay(500);
}

void impulseCalibration() {
  Serial.print("Resetting Impulse...");
  // Reset stepper to calibration position
  trigger_stepper.setSpeed(1000);
  trigger_stepper.moveTo(IMPULSE_CAL_POS);
  while(trigger_stepper.currentPosition() != IMPULSE_CAL_POS) {
    trigger_stepper.run();
    delay(DELAY_SPEED);
  }
  trigger_stepper.setCurrentPosition(0);
  trigger_stepper.setMaxSpeed(1000.0);
  trigger_stepper.setAcceleration(400.0);
  delay(500);
  Serial.println("\tDone");
  delay(500);

  Serial.print("Loading Impulse...");
  impulseLoad();
  Serial.println("\tDone");
  delay(500);

  Serial.print("Firing Impulse...");
  impulseFire();
  Serial.println("\tDone");
  delay(500);
}

void impulseReset() {
  // Reset stepper to start position
  trigger_stepper.setSpeed(1000);
  trigger_stepper.moveTo(IMPULSE_RESET_POS);
  while(trigger_stepper.currentPosition() != IMPULSE_RESET_POS) {
    trigger_stepper.run();
    delay(DELAY_SPEED);
  }
  trigger_stepper.setCurrentPosition(0);
  trigger_stepper.setMaxSpeed(1000.0);
  trigger_stepper.setAcceleration(400.0);
  delay(500);
}

// Resets the impulse trigger to a loaded position
void impulseLoad() {  
  trigger_stepper.setSpeed(1000);
  trigger_stepper.moveTo(IMPULSE_LOAD_POS);
  while(trigger_stepper.currentPosition() != IMPULSE_LOAD_POS) {
    trigger_stepper.run();
    delay(DELAY_SPEED);
  }
}

// Fires the trigger
// Delays are the DELAY_SPEED/3 in usec to sync up perfectly
void impulseFire() {
  trigger_stepper.setSpeed(1000);
  trigger_stepper.moveTo(IMPULSE_FIRE_POS);
  while(trigger_stepper.currentPosition() != IMPULSE_FIRE_POS) {
    // Set the impulse variable
    if(trigger_stepper.distanceToGo() == 5) // Roughly
      impulse_hit = 1;
    else
      impulse_hit = 0;
    // Use DELAY_SPEED-1 to make it super close to the movement already 8~9msec
    trigger_stepper.run();
    delayMicroseconds(firing_delay_usec);
    // run other steppers to keep it synchronous
    hip_stepper.run();
    delayMicroseconds(firing_delay_usec);
    knee_stepper.run();
    delayMicroseconds(firing_delay_usec);
  }
}

// Cycles through Gait Phases BAC 1 to BAC 8
// Note: For proper multi-stepper function, align delta hip & knee step values together
void runTrial() {
  // Lift impulse arm up for less friction
  impulseReset();
  impulseLoad();
  
  // First leg lift to BAC 1
  hip_stepper.setSpeed(1000);
  hip_stepper.moveTo(HIP_CENTER_POS-250); 
  while(hip_stepper.currentPosition() != HIP_CENTER_POS-250) {
    hip_stepper.run();
    hip_ = hip_stepper.currentPosition();
    delay(DELAY_SPEED);
  }
  //---------------------------------------------------------------------------------------------------
  // Start initial strides without impulse
//  for(int i = 1; i <= init_strides; i++) {
    // Movement from BAC 1 - 4
    gait_stage = 0x01;
    hip_stepper.setSpeed(1000);
    hip_stepper.moveTo(HIP_CENTER_POS+250); // from -250
    knee_stepper.setSpeed(1000);
    knee_stepper.moveTo(KNEE_CENTER_POS+125); // from 0
    while(hip_stepper.currentPosition() != HIP_CENTER_POS+250) {
      knee_stepper.run();
      knee_ = knee_stepper.currentPosition();
      delay(DELAY_SPEED);
      if(knee_stepper.distanceToGo() == 0) {
        gait_stage = 0x02;
        knee_stepper.moveTo(KNEE_CENTER_POS);
      }
      hip_stepper.run();
      hip_ = hip_stepper.currentPosition();
      delay(DELAY_SPEED);
      if(hip_stepper.currentPosition() == HIP_CENTER_POS)
          gait_stage = 0x03;
      if(hip_stepper.distanceToGo() == 100)
          gait_stage = 0x04;
    }

    // Movement from BAC 5 - 8
    gait_stage = 0x05;
    hip_stepper.setSpeed(1000);
    hip_stepper.moveTo(HIP_CENTER_POS-250); // from +250
    knee_stepper.setSpeed(1000);
    knee_stepper.moveTo(KNEE_CENTER_POS+250); // from 0
    while(hip_stepper.currentPosition() != HIP_CENTER_POS-250) { // 500 steps
      if(knee_stepper.distanceToGo() == 0) {
        knee_stepper.moveTo(KNEE_CENTER_POS); // from +250
      }
      if(hip_stepper.currentPosition() == HIP_CENTER_POS)
        gait_stage = 0x06;
      if(hip_stepper.currentPosition() == HIP_CENTER_POS-50)
        gait_stage = 0x07;
      if(hip_stepper.currentPosition() == HIP_CENTER_POS-175)
        gait_stage = 0x08;
      hip_stepper.run();
      hip_ = hip_stepper.currentPosition();
      delay(DELAY_SPEED);
      knee_stepper.run();
      knee_ = knee_stepper.currentPosition();
      delay(DELAY_SPEED);
    }
//  }
  //---------------------------------------------------------------------------------------------------
  // Start strides with impulse interruptions.
  // Could repeat this section with another loop
  for(int i = 0; i < GAIT_IMPULSE_CYCLES; i++) {
    for(int impulseEvent = 1; impulseEvent <= 8; impulseEvent++) {   
      // Movement from BAC 1 - 4
      gait_stage = 0x01;
      hip_stepper.setSpeed(1000);
      hip_stepper.moveTo(HIP_CENTER_POS+250); // from -250
      knee_stepper.setSpeed(1000);
      knee_stepper.moveTo(KNEE_CENTER_POS+125); // from 0
      while(hip_stepper.currentPosition() != HIP_CENTER_POS+250) {
        if(knee_stepper.distanceToGo() == 0) {
          gait_stage = 0x02;
          knee_stepper.moveTo(KNEE_CENTER_POS);
        }
        if(hip_stepper.currentPosition() == HIP_CENTER_POS)
            gait_stage = 0x03;
        if(hip_stepper.distanceToGo() == 100)
            gait_stage = 0x04;
            
        hip_stepper.run();
        hip_ = hip_stepper.currentPosition();
        delay(DELAY_SPEED);
        knee_stepper.run();
        knee_ = knee_stepper.currentPosition();
        delay(DELAY_SPEED);
        
  
        // Impulse detection fires.
        // Fires one step into the specified gait stage
        if(gait_stage == 0x01 && impulseEvent == 1)
          impulseFire();
        else if(gait_stage == 0x02 && impulseEvent == 2)
          impulseFire();
        else if(gait_stage == 0x03 && impulseEvent == 3)
          impulseFire();
        else if(gait_stage == 0x04 && impulseEvent == 4)
          impulseFire();
      }
  
      // Movement from BAC 5 - 8
      gait_stage = 0x05;
      hip_stepper.setSpeed(1000);
      hip_stepper.moveTo(HIP_CENTER_POS-250); // from +250
      knee_stepper.setSpeed(1000);
      knee_stepper.moveTo(KNEE_CENTER_POS+250); // from 0
      while(hip_stepper.currentPosition() != HIP_CENTER_POS-250) { // 500 steps
        if(knee_stepper.currentPosition() == KNEE_CENTER_POS+250)
          knee_stepper.moveTo(KNEE_CENTER_POS); // from +250
        if(hip_stepper.currentPosition() == HIP_CENTER_POS)
          gait_stage = 0x06;
        if(hip_stepper.currentPosition() == HIP_CENTER_POS-50)
          gait_stage = 0x07;
        if(hip_stepper.currentPosition() == HIP_CENTER_POS-175)
          gait_stage = 0x08;
        hip_stepper.run();
        hip_ = hip_stepper.currentPosition();
        delay(DELAY_SPEED);
        knee_stepper.run();
        knee_ = knee_stepper.currentPosition();
        delay(DELAY_SPEED);
  
        // Impulse detection fires.
        // Fires one step into the specified gait stage
        if(gait_stage == 0x05 && impulseEvent == 5)
          impulseFire();
        else if(gait_stage == 0x06 && impulseEvent == 6)
          impulseFire();
        else if(gait_stage == 0x07 && impulseEvent == 7)
          impulseFire();
        else if(gait_stage == 0x08 && impulseEvent == 8)
          impulseFire();
      }
      // Reset the impulse trigger after every stride 
      impulseReset();
      impulseLoad();
    }
  }    
  //---------------------------------------------------------------------------------------------------
  // Resetting position separately
  Serial.println("Resetting Position...");
  // Signifies done to T3.6
  hip_stepper.setSpeed(1000);
  hip_stepper.moveTo(HIP_CENTER_POS);
  while(hip_stepper.currentPosition() != HIP_CENTER_POS) {
    hip_stepper.run();
    hip_ = hip_stepper.currentPosition();
    delay(DELAY_SPEED);
  }  
  knee_stepper.setSpeed(1000);
  knee_stepper.moveTo(KNEE_CENTER_POS);
  while(knee_stepper.currentPosition() != KNEE_CENTER_POS) {
    knee_stepper.run();
    knee_ = knee_stepper.currentPosition();
    delay(DELAY_SPEED);
  } 
}
