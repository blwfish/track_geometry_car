// Native unit tests for summary module.
// Run with: pio test -e native
//
// Summary depends on ring_buffer and imu conversion functions.
// ring_buffer.cpp is included directly (pure logic, no hardware).
// imu conversion functions are reimplemented here to avoid pulling
// in Wire.h from imu.cpp.

#include <unity.h>
#include <math.h>
#include "config.h"
#include "ring_buffer.h"
#include "summary.h"

// Mock millis storage
uint32_t _mock_millis = 0;

// Include source directly — PlatformIO native doesn't link project src
#include "../../src/ring_buffer.cpp"
#include "../../src/summary.cpp"

// ===== IMU conversion stubs (same formulas as imu.cpp) =====
// These must be provided because summary.cpp calls them, but we
// can't link imu.cpp because it pulls in Wire.h.

float imuAccelG(int16_t raw) {
    return (float)raw / ACCEL_LSB_PER_G;
}

float imuGyroDPS(int16_t raw) {
    return (float)raw / GYRO_LSB_PER_DPS;
}

float imuTemperatureC(int16_t raw) {
    return ((float)raw / TEMP_LSB_PER_DEG) + TEMP_OFFSET_DEG;
}

// ===== Helpers =====

static void fillConstant(int16_t ax, int16_t ay, int16_t az,
                          int16_t gx, int16_t gy, int16_t gz,
                          int16_t temp, uint32_t count) {
    ringBufferInit();
    for (uint32_t i = 0; i < count; i++) {
        imu_sample_t s = {};
        s.timestamp_ms = i * 10;
        s.accel_x = ax;
        s.accel_y = ay;
        s.accel_z = az;
        s.gyro_x = gx;
        s.gyro_y = gy;
        s.gyro_z = gz;
        s.temperature = temp;
        ringBufferPush(&s);
    }
}

// ===== Tests =====

void test_summary_too_few_samples() {
    ringBufferInit();
    // Push only 5 samples (minimum is 10)
    for (int i = 0; i < 5; i++) {
        imu_sample_t s = {};
        ringBufferPush(&s);
    }

    summary_1s_t sum;
    _mock_millis = 1000;
    TEST_ASSERT_FALSE(summaryCompute(&sum, 5, 100.0f));
}

void test_summary_empty_buffer() {
    ringBufferInit();
    summary_1s_t sum;
    TEST_ASSERT_FALSE(summaryCompute(&sum, 0, 100.0f));
}

void test_summary_constant_accel() {
    // 1g on Z axis, 0 on others — like sitting on a bench
    // 16384 LSB = 1.0g
    fillConstant(0, 0, 16384, 0, 0, 0, 0, 100);

    summary_1s_t sum;
    _mock_millis = 5000;
    TEST_ASSERT_TRUE(summaryCompute(&sum, 100, 100.0f));

    // RMS of constant 1.0g = 1.0g
    TEST_ASSERT_FLOAT_WITHIN(0.001f, 1.0f, sum.accel_z_rms);
    // RMS of constant 0 = 0
    TEST_ASSERT_FLOAT_WITHIN(0.001f, 0.0f, sum.accel_x_rms);
    TEST_ASSERT_FLOAT_WITHIN(0.001f, 0.0f, sum.accel_y_rms);

    // Peak of constant 1.0g = 1.0g
    TEST_ASSERT_FLOAT_WITHIN(0.001f, 1.0f, sum.accel_z_peak);
    TEST_ASSERT_FLOAT_WITHIN(0.001f, 0.0f, sum.accel_x_peak);

    // Gyro should all be zero
    TEST_ASSERT_FLOAT_WITHIN(0.01f, 0.0f, sum.gyro_x_rms);
    TEST_ASSERT_FLOAT_WITHIN(0.01f, 0.0f, sum.gyro_y_rms);
    TEST_ASSERT_FLOAT_WITHIN(0.01f, 0.0f, sum.gyro_z_rms);
    TEST_ASSERT_FLOAT_WITHIN(0.01f, 0.0f, sum.gyro_x_mean);
    TEST_ASSERT_FLOAT_WITHIN(0.01f, 0.0f, sum.gyro_y_mean);
    TEST_ASSERT_FLOAT_WITHIN(0.01f, 0.0f, sum.gyro_z_mean);
}

void test_summary_constant_gyro() {
    // 131 LSB = 1.0 deg/s
    fillConstant(0, 0, 0, 131, -131, 262, 0, 100);

    summary_1s_t sum;
    _mock_millis = 5000;
    TEST_ASSERT_TRUE(summaryCompute(&sum, 100, 100.0f));

    // Gyro RMS of constant signal = abs(signal)
    TEST_ASSERT_FLOAT_WITHIN(0.05f, 1.0f, sum.gyro_x_rms);
    TEST_ASSERT_FLOAT_WITHIN(0.05f, 1.0f, sum.gyro_y_rms);
    TEST_ASSERT_FLOAT_WITHIN(0.05f, 2.0f, sum.gyro_z_rms);

    // Gyro mean
    TEST_ASSERT_FLOAT_WITHIN(0.05f, 1.0f, sum.gyro_x_mean);
    TEST_ASSERT_FLOAT_WITHIN(0.05f, -1.0f, sum.gyro_y_mean);
    TEST_ASSERT_FLOAT_WITHIN(0.05f, 2.0f, sum.gyro_z_mean);
}

void test_summary_peak_detection() {
    ringBufferInit();

    // Push 99 samples at 0g, then 1 sample at 0.5g on X
    for (int i = 0; i < 99; i++) {
        imu_sample_t s = {};
        s.timestamp_ms = i * 10;
        ringBufferPush(&s);
    }
    imu_sample_t spike = {};
    spike.timestamp_ms = 990;
    spike.accel_x = 8192;  // 0.5g
    ringBufferPush(&spike);

    summary_1s_t sum;
    _mock_millis = 1000;
    TEST_ASSERT_TRUE(summaryCompute(&sum, 100, 100.0f));

    // Peak should capture the 0.5g spike
    TEST_ASSERT_FLOAT_WITHIN(0.01f, 0.5f, sum.accel_x_peak);

    // RMS should be much lower than peak (only 1 sample out of 100)
    TEST_ASSERT_TRUE(sum.accel_x_rms < sum.accel_x_peak);
    TEST_ASSERT_TRUE(sum.accel_x_rms < 0.1f);
}

void test_summary_negative_peak() {
    ringBufferInit();

    // Push 99 zeros, then one -0.5g sample
    for (int i = 0; i < 99; i++) {
        imu_sample_t s = {};
        ringBufferPush(&s);
    }
    imu_sample_t s = {};
    s.accel_y = -8192;  // -0.5g
    ringBufferPush(&s);

    summary_1s_t sum;
    _mock_millis = 1000;
    TEST_ASSERT_TRUE(summaryCompute(&sum, 100, 100.0f));

    // Peak uses fabsf, so negative values still register
    TEST_ASSERT_FLOAT_WITHIN(0.01f, 0.5f, sum.accel_y_peak);
}

void test_summary_sample_count_and_rate() {
    fillConstant(0, 0, 16384, 0, 0, 0, 0, 50);

    summary_1s_t sum;
    _mock_millis = 2000;
    TEST_ASSERT_TRUE(summaryCompute(&sum, 12345, 99.5f));

    TEST_ASSERT_EQUAL_UINT32(12345, sum.sample_count);
    TEST_ASSERT_FLOAT_WITHIN(0.1f, 99.5f, sum.sample_rate);
}

void test_summary_temperature() {
    // Temperature raw value: (T_degC - 36.53) * 340
    // For 25°C: (25 - 36.53) * 340 = -3920.2 ≈ -3920
    fillConstant(0, 0, 0, 0, 0, 0, -3920, 100);

    summary_1s_t sum;
    _mock_millis = 1000;
    TEST_ASSERT_TRUE(summaryCompute(&sum, 100, 100.0f));

    TEST_ASSERT_FLOAT_WITHIN(0.1f, 25.0f, sum.temperature);
}

void test_summary_uses_recent_100_only() {
    ringBufferInit();

    // Push 200 samples: first 100 at 1g, next 100 at 0.5g
    for (int i = 0; i < 100; i++) {
        imu_sample_t s = {};
        s.accel_z = 16384;  // 1g
        ringBufferPush(&s);
    }
    for (int i = 0; i < 100; i++) {
        imu_sample_t s = {};
        s.accel_z = 8192;   // 0.5g
        ringBufferPush(&s);
    }

    summary_1s_t sum;
    _mock_millis = 2000;
    TEST_ASSERT_TRUE(summaryCompute(&sum, 200, 100.0f));

    // Summary should only see the recent 100 samples (0.5g), not the older 1g ones
    TEST_ASSERT_FLOAT_WITHIN(0.01f, 0.5f, sum.accel_z_rms);
    TEST_ASSERT_FLOAT_WITHIN(0.01f, 0.5f, sum.accel_z_peak);
}

// ===== Runner =====

int main(int argc, char **argv) {
    UNITY_BEGIN();
    RUN_TEST(test_summary_too_few_samples);
    RUN_TEST(test_summary_empty_buffer);
    RUN_TEST(test_summary_constant_accel);
    RUN_TEST(test_summary_constant_gyro);
    RUN_TEST(test_summary_peak_detection);
    RUN_TEST(test_summary_negative_peak);
    RUN_TEST(test_summary_sample_count_and_rate);
    RUN_TEST(test_summary_temperature);
    RUN_TEST(test_summary_uses_recent_100_only);
    return UNITY_END();
}
