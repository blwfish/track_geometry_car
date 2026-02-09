#ifndef IMU_H
#define IMU_H

#include "config.h"

// Initialize MPU-6050: wake from sleep, set ranges, DLPF, sample rate.
// Returns true if WHO_AM_I check passes.
bool imuInit();

// Read 14 bytes from MPU-6050 (accel XYZ + temp + gyro XYZ).
// Fills sample struct with raw values and timestamps with millis().
// Returns true on successful I2C read.
bool imuReadSample(imu_sample_t *sample);

// Read WHO_AM_I register (should return 0x68).
uint8_t imuWhoAmI();

// Unit conversion helpers
float imuAccelG(int16_t raw);
float imuGyroDPS(int16_t raw);
float imuTemperatureC(int16_t raw);

#endif // IMU_H
