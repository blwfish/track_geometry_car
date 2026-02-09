#ifndef IMU_H
#define IMU_H

#include "config.h"

// Initialize MPU-6050(s): wake from sleep, set ranges, DLPF, sample rate.
// IMU #1 at 0x68 is required — returns false if WHO_AM_I fails.
// IMU #2 at 0x69 is optional — detected and initialized if present.
bool imuInit();

// Read all present IMUs into a single sample struct.
// IMU #1 fields are always filled; IMU #2 fields are zero if absent.
// Timestamps with millis(). Returns true on successful I2C read of IMU #1.
bool imuReadSample(imu_sample_t *sample);

// Read WHO_AM_I register from primary IMU (should return 0x68).
uint8_t imuWhoAmI();

// How many IMUs were detected during imuInit() (1 or 2).
uint8_t imuGetCount();

// Convenience: true if second IMU is present.
bool imuHasSecond();

// Unit conversion helpers
float imuAccelG(int16_t raw);
float imuGyroDPS(int16_t raw);
float imuTemperatureC(int16_t raw);

#endif // IMU_H
