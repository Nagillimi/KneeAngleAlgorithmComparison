// MPU6050 test for impulse arm forces
// Will need to store to SD at 400 Hz to get decent data

#include "Wire.h" // This library allows you to communicate with I2C devices.

const int MPU_ADDR_ONE = 0x68; // I2C address of the MPU-6050. If AD0 pin is set to HIGH, the I2C address will be 0x69.

int val, temperature;
int cal = 12, test = 13;
double accelCalX, accelCalY, accelCalZ, x, y, z, accelerometer_x, accelerometer_y, accelerometer_z, gyro_x, gyro_y, gyro_z, gForceX, gForceY, gForceZ, roll, pitch, yaw;
double totalX = 0.0, totalY = 0.0, totalZ = 0.0, accelErrorX = 0.0, accelErrorY = 0.0, accelErrorZ = 0.0;

//  int16_t temperature;

void setup() {
  Serial.begin(115200);
  Wire.begin();
  Wire.beginTransmission(MPU_ADDR_ONE); // Begins a transmission to the I2C slave (GY-521 board)
  Wire.write(0x6B); // PWR_MGMT_1 register
  Wire.write(0); // set to zero (wakes up the MPU-6050)
  Wire.endTransmission(true);
  pinMode(cal, INPUT); pinMode(test, OUTPUT);

  calibration();
}

void processAccelData() {
  gForceX = (accelerometer_x / 1670.13) - accelErrorX; // 9.81 / 16384    to get newtons
  gForceY = (accelerometer_y / 1670.13) - accelErrorY;
  gForceZ = (accelerometer_z / 1670.13) - accelErrorZ;
}

void calibration() {
  //    Serial.println("Set Accelerometer flat");

  //    for(int i = 0; i < 500; i++) { //delay animation
  //      if(i % 100 == 0)
  //        Serial.print(".");
  //      delay(5);
  //    }
  //    Serial.println();

  //    Serial.println("Reading Accelerometer");

  delay(100); //for any extra movements from pushing the button

  // Set to zero again incase it's the second time calibrating
  accelErrorX = 0.0;
  accelErrorY = 0.0;
  accelErrorZ = 0.0;
  totalX = 0.0;
  totalY = 0.0;
  totalZ = 0.0;

  for (int i = 0; i < 100; i++) { //Read accel for 300 cycles
    Wire.beginTransmission(MPU_ADDR_ONE);
    Wire.write(0x3B);
    Wire.endTransmission(false);
    Wire.requestFrom(MPU_ADDR_ONE, 3 * 2, true);

    accelerometer_x = Wire.read() << 8 | Wire.read();
    accelerometer_y = Wire.read() << 8 | Wire.read();
    accelerometer_z = Wire.read() << 8 | Wire.read();

    totalX += accelerometer_x;
    totalY += accelerometer_y;
    totalZ += accelerometer_z;

    if (i % 100 == 0)
      //        Serial.print(".");
      delay(40);
  }
  accelCalX = totalX / (100.0 * 1670.13);
  accelCalY = totalY / (100.0 * 1670.13);
  accelCalZ = totalZ / (100.0 * 1670.13);

  //    Serial.println(); Serial.println("Recorded Data for X,Y,Z:");
  //    Serial.print(accelCalX); Serial.print(",  "); Serial.print(accelCalY); Serial.print(",  "); Serial.println(accelCalZ);

  // Error calc based on flat accelerometer
  accelErrorX = accelCalX;
  accelErrorY = accelCalY;
  accelErrorZ = accelCalZ - 9.81;

  //    Serial.print(accelErrorX); Serial.print(",  "); Serial.print(accelErrorY); Serial.print(",  "); Serial.println(accelErrorZ);
}

void calculations() {
  roll = atan(gForceX / gForceZ) * 180.0 / M_PI;
  pitch = atan(gForceZ / gForceY) * 180.0 / M_PI;
  
  if(gForceZ <= 0.0 && gForceX >= 0.0)
    roll += 180.0;
  else if(gForceZ <= 0.0 && gForceX <= 0.0)
    roll -= 180.0;
 
    
}

void loop() {

  val = digitalRead(cal);

  // Sense for Switch
  if (val == HIGH) {
    digitalWrite(test, HIGH);
    calibration();
    digitalWrite(test, LOW);
  }

  Wire.beginTransmission(MPU_ADDR_ONE);
  Wire.write(0x3B); // starting with register 0x3B (ACCEL_XOUT_H) [MPU-6000 and MPU-6050 Register Map and Descriptions Revision 4.2, p.40]
  Wire.endTransmission(false); // the parameter indicates that the Arduino will send a restart. As a result, the connection is kept active.
  Wire.requestFrom(MPU_ADDR_ONE, 7 * 2, true); // request a total of 7*2=14 registers

  // "Wire.read()<<8 | Wire.read();" means two registers are read and stored in the same variable
  accelerometer_x = Wire.read() << 8 | Wire.read(); // reading registers: 0x3B (ACCEL_XOUT_H) and 0x3C (ACCEL_XOUT_L)
  accelerometer_y = Wire.read() << 8 | Wire.read(); // reading registers: 0x3D (ACCEL_YOUT_H) and 0x3E (ACCEL_YOUT_L)
  accelerometer_z = Wire.read() << 8 | Wire.read(); // reading registers: 0x3F (ACCEL_ZOUT_H) and 0x40 (ACCEL_ZOUT_L)
  temperature = Wire.read() << 8 | Wire.read(); // reading registers: 0x41 (TEMP_OUT_H) and 0x42 (TEMP_OUT_L)
  gyro_x = Wire.read() << 8 | Wire.read(); // reading registers: 0x43 (GYRO_XOUT_H) and 0x44 (GYRO_XOUT_L)
  gyro_y = Wire.read() << 8 | Wire.read(); // reading registers: 0x45 (GYRO_YOUT_H) and 0x46 (GYRO_YOUT_L)
  gyro_z = Wire.read() << 8 | Wire.read(); // reading registers: 0x47 (GYRO_ZOUT_H) and 0x48 (GYRO_ZOUT_L)

  processAccelData();
  calculations();
  Serial.print(millis()); Serial.print("\t");
  Serial.print(gForceX); Serial.print("\t");
  Serial.print(gForceY); Serial.print("\t");
  Serial.println(gForceZ);
//  Serial.print("\t");
//  Serial.print(roll);
//  Serial.print(", ");
//  Serial.println(pitch);

  //    Serial.print(" | tmp = "); Serial.print(temperature/340.00+36.53);
  //    Serial.print(" | gX = "); Serial.print(convert_int16_t_to_str(gyro_x));
  //    Serial.print(" | gY = "); Serial.print(convert_int16_t_to_str(gyro_y));
  //    Serial.print(" | gZ = "); Serial.print(convert_int16_t_to_str(gyro_z));
  //    Serial.print("     "); Serial.print(num);
  //    Serial.println();

  delay(30);
}
