// Native unit tests for ring_buffer module.
// Run with: pio test -e native

#include <unity.h>
#include "config.h"
#include "ring_buffer.h"

// Mock millis storage (required by Arduino.h mock)
uint32_t _mock_millis = 0;

// Include source directly — PlatformIO native doesn't link project src
#include "../../src/ring_buffer.cpp"

// ===== Helper =====

static imu_sample_t makeSample(uint32_t ts, int16_t ax) {
    imu_sample_t s = {};
    s.timestamp_ms = ts;
    s.accel_x = ax;
    return s;
}

// ===== Tests =====

void test_init_empty() {
    ringBufferInit();
    TEST_ASSERT_EQUAL_UINT32(0, ringBufferCount());
    TEST_ASSERT_NULL(ringBufferGetRecent(0));
}

void test_push_one() {
    ringBufferInit();
    imu_sample_t s = makeSample(100, 1000);
    ringBufferPush(&s);

    TEST_ASSERT_EQUAL_UINT32(1, ringBufferCount());

    const imu_sample_t *got = ringBufferGetRecent(0);
    TEST_ASSERT_NOT_NULL(got);
    TEST_ASSERT_EQUAL_UINT32(100, got->timestamp_ms);
    TEST_ASSERT_EQUAL_INT16(1000, got->accel_x);
}

void test_push_multiple_ordering() {
    ringBufferInit();
    for (uint32_t i = 0; i < 5; i++) {
        imu_sample_t s = makeSample(i * 10, (int16_t)i);
        ringBufferPush(&s);
    }

    TEST_ASSERT_EQUAL_UINT32(5, ringBufferCount());

    // recency=0 is most recent (timestamp 40)
    TEST_ASSERT_EQUAL_UINT32(40, ringBufferGetRecent(0)->timestamp_ms);
    // recency=4 is oldest (timestamp 0)
    TEST_ASSERT_EQUAL_UINT32(0, ringBufferGetRecent(4)->timestamp_ms);
    // recency=5 is out of range
    TEST_ASSERT_NULL(ringBufferGetRecent(5));
}

void test_wraparound() {
    ringBufferInit();

    // Fill the entire buffer
    for (uint32_t i = 0; i < RING_BUFFER_SIZE; i++) {
        imu_sample_t s = makeSample(i, (int16_t)(i & 0x7FFF));
        ringBufferPush(&s);
    }
    TEST_ASSERT_EQUAL_UINT32(RING_BUFFER_SIZE, ringBufferCount());

    // Most recent should be the last one pushed
    TEST_ASSERT_EQUAL_UINT32(RING_BUFFER_SIZE - 1, ringBufferGetRecent(0)->timestamp_ms);
    // Oldest should be the first one pushed
    TEST_ASSERT_EQUAL_UINT32(0, ringBufferGetRecent(RING_BUFFER_SIZE - 1)->timestamp_ms);
}

void test_overwrite_oldest() {
    ringBufferInit();

    // Fill buffer completely
    for (uint32_t i = 0; i < RING_BUFFER_SIZE; i++) {
        imu_sample_t s = makeSample(i, 0);
        ringBufferPush(&s);
    }

    // Push one more — should overwrite the oldest (timestamp 0)
    imu_sample_t extra = makeSample(99999, 42);
    ringBufferPush(&extra);

    // Count should still be RING_BUFFER_SIZE (not RING_BUFFER_SIZE + 1)
    TEST_ASSERT_EQUAL_UINT32(RING_BUFFER_SIZE, ringBufferCount());

    // Most recent should be the new one
    TEST_ASSERT_EQUAL_UINT32(99999, ringBufferGetRecent(0)->timestamp_ms);
    TEST_ASSERT_EQUAL_INT16(42, ringBufferGetRecent(0)->accel_x);

    // Oldest should now be timestamp 1 (timestamp 0 was overwritten)
    TEST_ASSERT_EQUAL_UINT32(1, ringBufferGetRecent(RING_BUFFER_SIZE - 1)->timestamp_ms);
}

void test_copy_recent_basic() {
    ringBufferInit();

    for (uint32_t i = 0; i < 10; i++) {
        imu_sample_t s = makeSample(i * 100, (int16_t)i);
        ringBufferPush(&s);
    }

    // Copy last 5 samples
    imu_sample_t dest[5];
    uint32_t copied = ringBufferCopyRecent(dest, 5);
    TEST_ASSERT_EQUAL_UINT32(5, copied);

    // dest should be oldest-to-newest: timestamps 500, 600, 700, 800, 900
    TEST_ASSERT_EQUAL_UINT32(500, dest[0].timestamp_ms);
    TEST_ASSERT_EQUAL_UINT32(900, dest[4].timestamp_ms);
}

void test_copy_recent_more_than_available() {
    ringBufferInit();

    // Only push 3 samples
    for (uint32_t i = 0; i < 3; i++) {
        imu_sample_t s = makeSample(i, 0);
        ringBufferPush(&s);
    }

    // Request 10 — should only get 3
    imu_sample_t dest[10];
    uint32_t copied = ringBufferCopyRecent(dest, 10);
    TEST_ASSERT_EQUAL_UINT32(3, copied);
    TEST_ASSERT_EQUAL_UINT32(0, dest[0].timestamp_ms);
    TEST_ASSERT_EQUAL_UINT32(2, dest[2].timestamp_ms);
}

void test_copy_recent_after_wraparound() {
    ringBufferInit();

    // Push more than buffer size
    for (uint32_t i = 0; i < RING_BUFFER_SIZE + 50; i++) {
        imu_sample_t s = makeSample(i, 0);
        ringBufferPush(&s);
    }

    // Copy last 10
    imu_sample_t dest[10];
    uint32_t copied = ringBufferCopyRecent(dest, 10);
    TEST_ASSERT_EQUAL_UINT32(10, copied);

    // Should be the 10 most recent, oldest first
    uint32_t expected_start = RING_BUFFER_SIZE + 50 - 10;
    for (uint32_t i = 0; i < 10; i++) {
        TEST_ASSERT_EQUAL_UINT32(expected_start + i, dest[i].timestamp_ms);
    }
}

void test_copy_recent_empty() {
    ringBufferInit();
    imu_sample_t dest[5];
    uint32_t copied = ringBufferCopyRecent(dest, 5);
    TEST_ASSERT_EQUAL_UINT32(0, copied);
}

void test_reinit_clears() {
    ringBufferInit();

    imu_sample_t s = makeSample(42, 42);
    ringBufferPush(&s);
    TEST_ASSERT_EQUAL_UINT32(1, ringBufferCount());

    ringBufferInit();
    TEST_ASSERT_EQUAL_UINT32(0, ringBufferCount());
    TEST_ASSERT_NULL(ringBufferGetRecent(0));
}

void test_all_fields_preserved() {
    ringBufferInit();

    imu_sample_t s = {};
    s.timestamp_ms = 12345;
    s.accel_x = -100;
    s.accel_y = 200;
    s.accel_z = -300;
    s.gyro_x = 400;
    s.gyro_y = -500;
    s.gyro_z = 600;
    s.temperature = 3400;
    ringBufferPush(&s);

    const imu_sample_t *got = ringBufferGetRecent(0);
    TEST_ASSERT_EQUAL_UINT32(12345, got->timestamp_ms);
    TEST_ASSERT_EQUAL_INT16(-100, got->accel_x);
    TEST_ASSERT_EQUAL_INT16(200, got->accel_y);
    TEST_ASSERT_EQUAL_INT16(-300, got->accel_z);
    TEST_ASSERT_EQUAL_INT16(400, got->gyro_x);
    TEST_ASSERT_EQUAL_INT16(-500, got->gyro_y);
    TEST_ASSERT_EQUAL_INT16(600, got->gyro_z);
    TEST_ASSERT_EQUAL_INT16(3400, got->temperature);
}

// ===== Runner =====

int main(int argc, char **argv) {
    UNITY_BEGIN();
    RUN_TEST(test_init_empty);
    RUN_TEST(test_push_one);
    RUN_TEST(test_push_multiple_ordering);
    RUN_TEST(test_wraparound);
    RUN_TEST(test_overwrite_oldest);
    RUN_TEST(test_copy_recent_basic);
    RUN_TEST(test_copy_recent_more_than_available);
    RUN_TEST(test_copy_recent_after_wraparound);
    RUN_TEST(test_copy_recent_empty);
    RUN_TEST(test_reinit_clears);
    RUN_TEST(test_all_fields_preserved);
    return UNITY_END();
}
