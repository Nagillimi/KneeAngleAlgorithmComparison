// Avoid IDE problems by defining struct in septate .h file.
// Pad record so size is a power of two for best write performance.
/*
  Size of the total logged dataset in bits:
 | PACKET # | TIME | DIFF |FSR_HEEL | FSR_TOE | IMU_1 | IMU_2 | IMU_3 |
 |    32    |  32  |  32  |    16    |    16   |  240  |  240  |  240  |
 = 816 bits = 102 Bytes
 Note:
 - Should pad until 128 Bytes for best logging performance.
*/
#ifndef Parameters_h
#define Parameters_h
#include "MYUM7SPI.h"
#include "Wire.h"

MYUM7SPI imu1(15); // cs pin 1
MYUM7SPI imu2(20); // cs pin 2

#define UM7_MOSI_PIN 11
#define UM7_MISO_PIN 12
#define UM7_SCK_PIN 13
#define IMPULSE_PIN 33

int16_t hip_stepper_ = 0, knee_stepper_ = 0;
byte gait_stage_ = 0;
byte impulse_hit_ = 0;

int numTrials = 2;
bool startTrial = false, doneTrial = false, halt = false;

// Collection of data custom for application
struct data_t {
  uint32_t t;
  float gx_1;
  float gy_1;
  float gz_1;
  float ax_1;
  float ay_1;
  float az_1;
  float gx_2;
  float gy_2;
  float gz_2;
  float ax_2;
  float ay_2;
  float az_2;
  int16_t hip_stepper;
  int16_t knee_stepper;
  float knee_angle;
  uint8_t gait_stage; // Also logs when trial is finished. ie = 10.
  uint8_t impulse_hit;

  // data_t is now perfectly 128 Bytes
//  uint32_t whitespace;
};
// Variable for keeping track of transfer #'s for rate synchronization
uint64_t count = 0;

void getI2Cdata() {
  Wire.requestFrom(9, 6); // address, howManyBytes
  hip_stepper_ = (int16_t)(Wire.read() << 8) | Wire.read();
  knee_stepper_ = (int16_t)(Wire.read() << 8) | Wire.read();
  gait_stage_ = (byte)Wire.read();
  impulse_hit_ = (byte)Wire.read();

  if(gait_stage_ == 10) {
    doneTrial = true;
    startTrial = false;
  }
}

float calcKneeAngle() {
  // y = mx + b... lol
  if(!knee_stepper_)
    return -1;
  // Zero knee angle is set to 450 in StepperGait.ino
  float angle = (360.0/2038.0) * knee_stepper_ - 450.0*(360.0/2038.0);
  return angle;
}
//-------------------------------PARAMETERS-------------------------------------
// You may modify the log file name.  
// Digits before the dot are file versions, don't edit them.
char binName[] = "Trial_00.bin";
//------------------------------------------------------------------------------
// This example was designed for exFAT but will support FAT16/FAT32.
// Note: Uno will not support SD_FAT_TYPE = 3.
// SD_FAT_TYPE = 0 for SdFat/File as defined in SdFatConfig.h,
// 1 for FAT16/FAT32, 2 for exFAT, 3 for FAT16/FAT32 and exFAT.
#define SD_FAT_TYPE 2

#if SD_FAT_TYPE == 0
typedef SdFat sd_t;
typedef File file_t;
#elif SD_FAT_TYPE == 1
typedef SdFat32 sd_t;
typedef File32 file_t;
#elif SD_FAT_TYPE == 2
typedef SdExFat sd_t;
typedef ExFile file_t;
#elif SD_FAT_TYPE == 3
typedef SdFs sd_t;
typedef FsFile file_t;
#else  // SD_FAT_TYPE
#error Invalid SD_FAT_TYPE
#endif  // SD_FAT_TYPE
//------------------------------------------------------------------------------
// Interval between data records in microseconds.
// Try 250 with Teensy 3.6, Due, or STM32.
// Try 2000 with AVR boards, = 500Hz
// Try 4000 with SAMD Zero boards, = 250Hz
const uint16_t LOG_INTERVAL_USEC = 4000;
// Use to compare timestamps for missed packets
const uint16_t MAX_INTERVAL_USEC = 6000;
//------------------------------------------------------------------------------
// Initial time before logging starts, set once logging has begun
// And total log time of session, used to print to csv file once
// converted.
uint32_t t0, log_time;
// Init the time delta to track missed packets
uint32_t delta = 0;

// LED to light if overruns occur, define if you have one setup
#define ERROR_LED_PIN -1
//------------------------------------------------------------------------------
// SDCARD_SS_PIN is defined for the built-in SD on some boards.
#ifndef SDCARD_SS_PIN
const uint8_t SD_CS_PIN = SS;
#else  // SDCARD_SS_PIN
// Assume built-in SD is used.
const uint8_t SD_CS_PIN = SDCARD_SS_PIN;
#endif  // SDCARD_SS_PIN
//------------------------------------------------------------------------------
// FIFO SIZE - 512 byte sectors.  Modify for your board.
#ifdef __AVR_ATmega328P__
// Use 512 bytes for 328 boards.
#define FIFO_SIZE_SECTORS 1
#elif defined(__AVR__)
// Use 2 KiB for other AVR boards.
#define FIFO_SIZE_SECTORS 4
#else  // __AVR_ATmega328P__
// Use 8 KiB for non-AVR boards.
//#define FIFO_SIZE_SECTORS 16
// Use 8 KiB for Teensy 3.6
#define FIFO_SIZE_SECTORS 16
#endif  // __AVR_ATmega328P__
//------------------------------------------------------------------------------
// Preallocate 1GiB file.
const uint32_t PREALLOCATE_SIZE_MiB = 1024UL;
// Conversion to 64b variable to match the library param
const uint64_t PREALLOCATE_SIZE  =  (uint64_t)PREALLOCATE_SIZE_MiB << 20;
//------------------------------------------------------------------------------
// Select the appropriate SPI configuration. 
// ENABLE_DEDICATED_SPI default is true for Teensy boards, change this in
// SdFatConfig.h to zero if you want a (slower) shared SPI bus.
#if ENABLE_DEDICATED_SPI
#define SD_CONFIG SdSpiConfig(SD_CS_PIN, DEDICATED_SPI, SD_SCK_MHZ(50))
#else  // ENABLE_DEDICATED_SPI
// Shared SPI bus, MAY need to alter the 50MHz depending on if it's already declared
#define SD_CONFIG SdSpiConfig(SD_CS_PIN, SHARED_SPI, SD_SCK_MHZ(50))
#endif  // ENABLE_DEDICATED_SPI
//------------------------------------------------------------------------------
// Save SRAM if 328.
#ifdef __AVR_ATmega328P__
#include "MinimumSerial.h"
MinimumSerial MinSerial;
#define Serial MinSerial
#endif  // __AVR_ATmega328P__

// Boolean used to track whether or not you're just testing the sensors. Won't print
// the "Missed packet(s)" everytime when testing, otherwise printed in data logging.
bool test = false;

// Max length of file name including zero byte.
#define FILE_NAME_DIM 40

// Max number of records to buffer while SD is busy. Should alter factors to result
// in an integer! (Faster writes)
const size_t FIFO_DIM = 512 * FIFO_SIZE_SECTORS / sizeof(data_t);

// Create single sd type
sd_t sd;

// Create two filetypes
file_t binFile;
file_t csvFile;

#endif  // Parameters_h
