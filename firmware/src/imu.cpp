#include "imu.h"
#include <Wire.h>

// ===== Dual IMU state =====
static bool imu2Present = false;
static uint8_t detectedImuCount = 1;

// ===== Low-level I2C helpers =====

static void imuWriteReg(uint8_t addr, uint8_t reg, uint8_t value) {
    Wire.beginTransmission(addr);
    Wire.write(reg);
    Wire.write(value);
    Wire.endTransmission(true);
}

static uint8_t imuReadReg(uint8_t addr, uint8_t reg) {
    Wire.beginTransmission(addr);
    Wire.write(reg);
    Wire.endTransmission(false);  // restart
    Wire.requestFrom(addr, (uint8_t)1);
    return Wire.available() ? Wire.read() : 0xFF;
}

static int16_t readBigEndian16() {
    int16_t val = (int16_t)(Wire.read() << 8);
    val |= Wire.read();
    return val;
}

// Configure a single MPU-6050 at the given address.
// Returns true if WHO_AM_I matches.
static bool imuInitOne(uint8_t addr) {
    // Wake from sleep, select PLL with X-axis gyro reference
    imuWriteReg(addr, MPU6050_REG_PWR_MGMT_1, MPU6050_CLOCK_PLL_XGYRO);
    delay(10);  // Allow PLL to stabilize

    // Verify device identity
    uint8_t whoami = imuReadReg(addr, MPU6050_REG_WHO_AM_I);
    if (whoami != MPU6050_WHO_AM_I_EXPECTED) {
        Serial.printf("[IMU] 0x%02X: WHO_AM_I=0x%02X (expected 0x%02X)\n",
                      addr, whoami, MPU6050_WHO_AM_I_EXPECTED);
        return false;
    }

    imuWriteReg(addr, MPU6050_REG_SMPLRT_DIV, MPU6050_SMPLRT_DIV_100HZ);
    imuWriteReg(addr, MPU6050_REG_CONFIG, MPU6050_DLPF_20HZ);
    imuWriteReg(addr, MPU6050_REG_ACCEL_CONFIG, MPU6050_ACCEL_RANGE_2G);
    imuWriteReg(addr, MPU6050_REG_GYRO_CONFIG, MPU6050_GYRO_RANGE_250);

    Serial.printf("[IMU] 0x%02X: initialized (+-2g, +-250dps, 100Hz, DLPF 20Hz)\n", addr);
    return true;
}

// Burst-read 14 bytes (accel+temp+gyro) from one MPU-6050.
static bool imuReadOne(uint8_t addr,
                       int16_t *ax, int16_t *ay, int16_t *az,
                       int16_t *gx, int16_t *gy, int16_t *gz,
                       int16_t *temp) {
    Wire.beginTransmission(addr);
    Wire.write(MPU6050_REG_ACCEL_XOUT_H);
    Wire.endTransmission(false);

    uint8_t bytesReceived = Wire.requestFrom(addr, (uint8_t)14);
    if (bytesReceived != 14) return false;

    *ax   = readBigEndian16();
    *ay   = readBigEndian16();
    *az   = readBigEndian16();
    *temp = readBigEndian16();
    *gx   = readBigEndian16();
    *gy   = readBigEndian16();
    *gz   = readBigEndian16();

    return true;
}

// ===== Public API =====

bool imuInit() {
    // IMU #1 at 0x68 is required
    if (!imuInitOne(MPU6050_ADDR)) {
        Serial.println("[IMU] Primary IMU (0x68) init FAILED");
        return false;
    }

    // IMU #2 at 0x69 is optional
    if (imuInitOne(MPU6050_ADDR_2)) {
        imu2Present = true;
        detectedImuCount = 2;
        Serial.println("[IMU] Dual IMU mode: 2 sensors detected");
    } else {
        imu2Present = false;
        detectedImuCount = 1;
        Serial.println("[IMU] Single IMU mode: only primary sensor present");
    }

    return true;
}

bool imuReadSample(imu_sample_t *sample) {
    // Zero the entire struct so IMU2 fields are clean when absent
    memset(sample, 0, sizeof(imu_sample_t));

    int16_t temp;
    bool ok = imuReadOne(MPU6050_ADDR,
                         &sample->accel_x, &sample->accel_y, &sample->accel_z,
                         &sample->gyro_x, &sample->gyro_y, &sample->gyro_z,
                         &temp);
    if (!ok) return false;

    sample->timestamp_ms = millis();
    sample->temperature = temp;

    // Read IMU #2 if present — failure is non-fatal (fields stay zero)
    if (imu2Present) {
        int16_t temp2;  // discarded — both sensors at same ambient temp
        imuReadOne(MPU6050_ADDR_2,
                   &sample->accel_x2, &sample->accel_y2, &sample->accel_z2,
                   &sample->gyro_x2, &sample->gyro_y2, &sample->gyro_z2,
                   &temp2);
    }

    return true;
}

uint8_t imuWhoAmI() {
    return imuReadReg(MPU6050_ADDR, MPU6050_REG_WHO_AM_I);
}

uint8_t imuGetCount() {
    return detectedImuCount;
}

bool imuHasSecond() {
    return imu2Present;
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
