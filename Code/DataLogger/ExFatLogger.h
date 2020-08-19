// Avoid IDE problems by defining struct in septate .h file.
// Pad record so size is a power of two for best write performance.
/*
  Size of the total logged dataset in bits:
 | PACKET # | TIME | FSR_HEEL | FSR_TOE | IMU_1 | IMU_2 | IMU_3 |
 |    32    |  32  |    16    |    16   |  240  |  240  |  240  |
 = 816 bits = 102 Bytes
 Note:
 - Should pad until 128 Bytes for best logging performance.
*/
#ifndef ExFatLogger_h
#define ExFatLogger_h

#include "MYUM7SPI.h"

MYUM7SPI imu1(34); // cs pin 1
MYUM7SPI imu2(33); // cs pin 2

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
};
#endif  // ExFatLogger_h
