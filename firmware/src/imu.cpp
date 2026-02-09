#include "imu.h"
#include <Wire.h>

// ===== Low-level I2C helpers =====

static void imuWriteRegister(uint8_t reg, uint8_t value) {
    Wire.beginTransmission(MPU6050_ADDR);
    Wire.write(reg);
    Wire.write(value);
    Wire.endTransmission(true);
}

static uint8_t imuReadRegister(uint8_t reg) {
    Wire.beginTransmission(MPU6050_ADDR);
    Wire.write(reg);
    Wire.endTransmission(false);  // restart
    Wire.requestFrom((uint8_t)MPU6050_ADDR, (uint8_t)1);
    return Wire.available() ? Wire.read() : 0xFF;
}

static int16_t readBigEndian16() {
    int16_t val = (int16_t)(Wire.read() << 8);
    val |= Wire.read();
    return val;
}

// ===== Public API =====

bool imuInit() {
    // Wake from sleep, select PLL with X-axis gyro reference
    imuWriteRegister(MPU6050_REG_PWR_MGMT_1, MPU6050_CLOCK_PLL_XGYRO);
    delay(10);  // Allow PLL to stabilize

    // Verify device identity
    uint8_t whoami = imuWhoAmI();
    if (whoami != MPU6050_WHO_AM_I_EXPECTED) {
        Serial.printf("[IMU] WHO_AM_I returned 0x%02X, expected 0x%02X\n",
                      whoami, MPU6050_WHO_AM_I_EXPECTED);
        return false;
    }

    // Set sample rate: 1000Hz / (9+1) = 100Hz
    imuWriteRegister(MPU6050_REG_SMPLRT_DIV, MPU6050_SMPLRT_DIV_100HZ);

    // Set DLPF to ~20Hz bandwidth
    imuWriteRegister(MPU6050_REG_CONFIG, MPU6050_DLPF_20HZ);

    // Set accelerometer range to +-2g
    imuWriteRegister(MPU6050_REG_ACCEL_CONFIG, MPU6050_ACCEL_RANGE_2G);

    // Set gyroscope range to +-250 deg/s
    imuWriteRegister(MPU6050_REG_GYRO_CONFIG, MPU6050_GYRO_RANGE_250);

    Serial.println("[IMU] MPU-6050 initialized: +-2g, +-250dps, 100Hz, DLPF 20Hz");
    return true;
}

bool imuReadSample(imu_sample_t *sample) {
    // Burst read 14 bytes starting at ACCEL_XOUT_H:
    // accel_x(2) + accel_y(2) + accel_z(2) + temp(2) + gyro_x(2) + gyro_y(2) + gyro_z(2)
    Wire.beginTransmission(MPU6050_ADDR);
    Wire.write(MPU6050_REG_ACCEL_XOUT_H);
    Wire.endTransmission(false);  // restart, keep bus active

    uint8_t bytesReceived = Wire.requestFrom((uint8_t)MPU6050_ADDR, (uint8_t)14);
    if (bytesReceived != 14) {
        return false;
    }

    sample->timestamp_ms = millis();
    sample->accel_x     = readBigEndian16();
    sample->accel_y     = readBigEndian16();
    sample->accel_z     = readBigEndian16();
    sample->temperature = readBigEndian16();
    sample->gyro_x      = readBigEndian16();
    sample->gyro_y      = readBigEndian16();
    sample->gyro_z      = readBigEndian16();

    return true;
}

uint8_t imuWhoAmI() {
    return imuReadRegister(MPU6050_REG_WHO_AM_I);
}

float imuAccelG(int16_t raw) {
    return (float)raw / ACCEL_LSB_PER_G;
}

float imuGyroDPS(int16_t raw) {
    return (float)raw / GYRO_LSB_PER_DPS;
}

float imuTemperatureC(int16_t raw) {
    return ((float)raw / TEMP_LSB_PER_DEG) + TEMP_OFFSET_DEG;
}
