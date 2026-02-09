#ifndef CONFIG_H
#define CONFIG_H

#include <Arduino.h>

// ===== VERSION INFORMATION =====
#define FIRMWARE_VERSION_BASE "0.3.0"

#ifndef BUILD_GIT_HASH
#define BUILD_GIT_HASH "dev"
#endif
#ifndef BUILD_TIME
#define BUILD_TIME __DATE__ " " __TIME__
#endif

#define FIRMWARE_VERSION FIRMWARE_VERSION_BASE "-" BUILD_GIT_HASH

// ===== I2C CONFIGURATION =====
// ESP32-WROOM default I2C pins (differs from ESP32-C3 which uses GPIO 8/9)
#define I2C_SDA_PIN         21
#define I2C_SCL_PIN         22
#define I2C_CLOCK_HZ        400000   // 400 kHz Fast Mode

// ===== I2C DEVICE ADDRESSES =====
#define MPU6050_ADDR        0x68
#define OLED_ADDR           0x3C
// #define INA219_ADDR      0x40     // Phase 2: track power monitor

// ===== MPU-6050 REGISTERS =====
#define MPU6050_REG_SMPLRT_DIV      0x19
#define MPU6050_REG_CONFIG          0x1A
#define MPU6050_REG_GYRO_CONFIG     0x1B
#define MPU6050_REG_ACCEL_CONFIG    0x1C
#define MPU6050_REG_ACCEL_XOUT_H   0x3B   // Start of 14-byte burst read
#define MPU6050_REG_TEMP_OUT_H      0x41
#define MPU6050_REG_PWR_MGMT_1     0x6B
#define MPU6050_REG_WHO_AM_I       0x75

// MPU-6050 configuration values
#define MPU6050_ACCEL_RANGE_2G      0x00   // +-2g  (16384 LSB/g)
#define MPU6050_GYRO_RANGE_250      0x00   // +-250 deg/s (131 LSB/deg/s)
#define MPU6050_DLPF_20HZ           0x04   // ~20 Hz bandwidth, ~8.5ms delay
#define MPU6050_SMPLRT_DIV_100HZ    0x09   // 1000/(9+1) = 100 Hz
#define MPU6050_CLOCK_PLL_XGYRO     0x01   // PLL with X-axis gyro reference
#define MPU6050_WHO_AM_I_EXPECTED   0x68

// Conversion factors
#define ACCEL_LSB_PER_G     16384.0f
#define GYRO_LSB_PER_DPS    131.0f
#define TEMP_LSB_PER_DEG    340.0f
#define TEMP_OFFSET_DEG     36.53f

// ===== WIFI CONFIGURATION =====
#define WIFI_AP_SSID        "GeometryCar"
#define WIFI_AP_CHANNEL     1
#define WIFI_AP_MAX_CONN    4

// ===== WEBSOCKET CONFIGURATION =====
#define WS_PATH             "/ws"
#define WS_SEND_INTERVAL_MS     100     // 100 ms = 10 Hz raw sample batches
#define WS_SUMMARY_INTERVAL_MS  1000    // 1000 ms = 1 Hz summary frames
#define WS_CLEANUP_INTERVAL_MS  2000    // 2 sec = cleanup dead connections
#define WS_SAMPLE_BATCH_SIZE    10      // Samples per WebSocket frame

// WebSocket frame types (server -> client)
#define WS_FRAME_RAW_SAMPLES    0x01
#define WS_FRAME_SUMMARY        0x02
#define WS_FRAME_REC_STATUS     0x03    // Recording state notification

// WebSocket command types (client -> server)
#define WS_CMD_START_RECORDING  0x10
#define WS_CMD_STOP_RECORDING   0x11
#define WS_CMD_CALIBRATE        0x12

// Wire format: 18 bytes per sample (no struct padding)
#define WS_SAMPLE_WIRE_SIZE     18

// ===== TIMING CONSTANTS =====
#define IMU_SAMPLE_INTERVAL_US      10000   // 10 ms = 100 Hz
#define OLED_UPDATE_INTERVAL_MS     200     // 200 ms = 5 Hz
#define SERIAL_OUTPUT_INTERVAL_MS   100     // 100 ms = 10 Hz
#define STATS_REPORT_INTERVAL_MS    10000   // 10 sec = sample rate reporting
#define SUMMARY_INTERVAL_MS         1000    // 1 sec = 1 Hz summary computation

// ===== RING BUFFER =====
#define RING_BUFFER_SIZE    1024            // 1024 samples = 10.24 sec at 100Hz

// ===== DATA STRUCTURES =====

struct imu_sample_t {
    uint32_t timestamp_ms;
    int16_t accel_x;
    int16_t accel_y;
    int16_t accel_z;
    int16_t gyro_x;
    int16_t gyro_y;
    int16_t gyro_z;
    int16_t temperature;
};

struct summary_1s_t {
    uint32_t timestamp_ms;
    float accel_x_rms;
    float accel_y_rms;
    float accel_z_rms;
    float accel_x_peak;
    float accel_y_peak;
    float accel_z_peak;
    float gyro_x_rms;
    float gyro_y_rms;
    float gyro_z_rms;
    float gyro_x_mean;
    float gyro_y_mean;
    float gyro_z_mean;
    float temperature;
    uint32_t sample_count;
    float sample_rate;
};

// ===== FLASH LOGGING =====
#define SURVEY_DIR              "/surveys"
#define SURVEY_HEADER_SIZE      64
#define SURVEY_SAMPLE_SIZE      18      // Same as WS_SAMPLE_WIRE_SIZE (no padding)
#define SURVEY_MAGIC            "GEOM"
#define SURVEY_VERSION          1       // 1=IMU only, 2=IMU+INA219 (future)
#define FLASH_MIN_FREE_BYTES    32768   // Reserve 32KB for web UI + overhead

struct __attribute__((packed)) survey_header_t {
    char     magic[4];           // "GEOM"
    uint8_t  version;            // SURVEY_VERSION
    uint8_t  sample_size;        // SURVEY_SAMPLE_SIZE (18)
    uint16_t sample_rate_hz;     // 100
    uint8_t  accel_range_g;      // 2 (±2g)
    uint8_t  gyro_range_dps;     // 250 (stored as uint8_t, 250 fits)
    uint8_t  reserved1[2];
    uint32_t start_time_ms;      // millis() at recording start
    char     car_id[16];         // "GeometryCar\0..."
    uint8_t  reserved2[32];      // Pad to 64 bytes total
};

#endif // CONFIG_H
