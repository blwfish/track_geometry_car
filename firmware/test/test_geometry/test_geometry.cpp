// Native unit tests for geometry module.
// Run with: pio test -e native
//
// geometry.cpp is pure math with no hardware dependencies.

#include <unity.h>
#include <math.h>
#include "geometry.h"

// Mock millis storage (required by Arduino.h mock, not used here)
uint32_t _mock_millis = 0;

// Include source directly — PlatformIO native doesn't link project src
#include "../../src/geometry.cpp"

// ===== Radius computation tests =====

void test_radius_known_curve() {
    // At 10 scale mph (51.4 mm/s) with 2°/s yaw rate:
    // R = 51.4 / (2 * π/180) = 51.4 / 0.03491 = 1472.7 mm = 58.0"
    float speed = geometryScaleToModelSpeed(10.0f);
    float radius = geometryRadiusMm(2.0f, speed);
    float radiusIn = geometryMmToInches(radius);
    TEST_ASSERT_FLOAT_WITHIN(0.5f, 58.0f, radiusIn);
}

void test_radius_tight_curve() {
    // 18" radius curve — typical HO minimum
    // At 10 scale mph, what yaw rate gives 18"?
    // 18" = 457.2 mm, R = v/ω → ω = v/R
    float speed = geometryScaleToModelSpeed(10.0f);
    float expectedYawRad = speed / 457.2f;
    float expectedYawDps = expectedYawRad * 180.0f / M_PI;

    float radius = geometryRadiusMm(expectedYawDps, speed);
    float radiusIn = geometryMmToInches(radius);
    TEST_ASSERT_FLOAT_WITHIN(0.1f, 18.0f, radiusIn);
}

void test_radius_large_curve() {
    // 140" radius — typical large layout curve
    float speed = geometryScaleToModelSpeed(10.0f);
    float expectedYawRad = speed / (140.0f * 25.4f);
    float expectedYawDps = expectedYawRad * 180.0f / M_PI;

    float radius = geometryRadiusMm(expectedYawDps, speed);
    float radiusIn = geometryMmToInches(radius);
    TEST_ASSERT_FLOAT_WITHIN(1.0f, 140.0f, radiusIn);
}

void test_radius_zero_yaw_returns_zero() {
    float radius = geometryRadiusMm(0.0f, 58.0f);
    TEST_ASSERT_FLOAT_WITHIN(0.001f, 0.0f, radius);
}

void test_radius_negative_yaw_returns_zero() {
    float radius = geometryRadiusMm(-1.0f, 58.0f);
    TEST_ASSERT_FLOAT_WITHIN(0.001f, 0.0f, radius);
}

void test_radius_very_small_yaw() {
    // Just above zero — should give very large radius, not infinity
    float radius = geometryRadiusMm(0.001f, 58.0f);
    TEST_ASSERT_TRUE(radius > 0.0f);
    TEST_ASSERT_TRUE(isfinite(radius));
}

void test_radius_proportional_to_speed() {
    // Double the speed → double the radius at same yaw rate
    float r1 = geometryRadiusMm(5.0f, 50.0f);
    float r2 = geometryRadiusMm(5.0f, 100.0f);
    TEST_ASSERT_FLOAT_WITHIN(0.01f, 2.0f, r2 / r1);
}

void test_radius_inverse_to_yaw() {
    // Double the yaw rate → half the radius at same speed
    float r1 = geometryRadiusMm(2.0f, 58.0f);
    float r2 = geometryRadiusMm(4.0f, 58.0f);
    TEST_ASSERT_FLOAT_WITHIN(0.01f, 0.5f, r2 / r1);
}

// ===== Unit conversion tests =====

void test_mm_to_inches() {
    TEST_ASSERT_FLOAT_WITHIN(0.001f, 1.0f, geometryMmToInches(25.4f));
    TEST_ASSERT_FLOAT_WITHIN(0.001f, 0.0f, geometryMmToInches(0.0f));
    TEST_ASSERT_FLOAT_WITHIN(0.01f, 12.0f, geometryMmToInches(304.8f));
}

void test_scale_speed_conversion() {
    // 10 scale mph in HO (1:87)
    // 10 mph = 4470.4 mm/s full scale
    // 4470.4 / 87 = 51.384 mm/s model
    float speed = geometryScaleToModelSpeed(10.0f);
    TEST_ASSERT_FLOAT_WITHIN(0.1f, 51.4f, speed);
}

void test_scale_speed_zero() {
    TEST_ASSERT_FLOAT_WITHIN(0.001f, 0.0f, geometryScaleToModelSpeed(0.0f));
}

void test_scale_speed_proportional() {
    float s1 = geometryScaleToModelSpeed(10.0f);
    float s2 = geometryScaleToModelSpeed(20.0f);
    TEST_ASSERT_FLOAT_WITHIN(0.01f, 2.0f, s2 / s1);
}

// ===== SNR tests =====

void test_snr_at_noise_floor() {
    // Yaw rate exactly at noise floor → SNR = 1
    float snr = geometrySNR(GEOM_GYRO_NOISE_FLOOR_DPS);
    TEST_ASSERT_FLOAT_WITHIN(0.001f, 1.0f, snr);
}

void test_snr_good_measurement() {
    // 1.0 °/s yaw → SNR = 1.0/0.05 = 20
    float snr = geometrySNR(1.0f);
    TEST_ASSERT_FLOAT_WITHIN(0.001f, 20.0f, snr);
}

void test_snr_zero_yaw() {
    float snr = geometrySNR(0.0f);
    TEST_ASSERT_FLOAT_WITHIN(0.001f, 0.0f, snr);
}

void test_snr_classify_good() {
    TEST_ASSERT_EQUAL(SNR_GOOD, geometryClassifySNR(20.0f));
    TEST_ASSERT_EQUAL(SNR_GOOD, geometryClassifySNR(100.0f));
}

void test_snr_classify_marginal() {
    TEST_ASSERT_EQUAL(SNR_MARGINAL, geometryClassifySNR(10.0f));
    TEST_ASSERT_EQUAL(SNR_MARGINAL, geometryClassifySNR(19.9f));
}

void test_snr_classify_poor() {
    TEST_ASSERT_EQUAL(SNR_POOR, geometryClassifySNR(9.9f));
    TEST_ASSERT_EQUAL(SNR_POOR, geometryClassifySNR(0.0f));
}

// ===== Track classification tests =====

void test_track_straight() {
    TEST_ASSERT_EQUAL(TRACK_STRAIGHT, geometryClassifyTrack(0.0f));
    TEST_ASSERT_EQUAL(TRACK_STRAIGHT, geometryClassifyTrack(0.1f));
    TEST_ASSERT_EQUAL(TRACK_STRAIGHT, geometryClassifyTrack(-0.1f));
    TEST_ASSERT_EQUAL(TRACK_STRAIGHT, geometryClassifyTrack(0.29f));
    TEST_ASSERT_EQUAL(TRACK_STRAIGHT, geometryClassifyTrack(-0.29f));
}

void test_track_curve_left() {
    // Positive gz = counterclockwise from above = left curve
    TEST_ASSERT_EQUAL(TRACK_CURVE_LEFT, geometryClassifyTrack(0.5f));
    TEST_ASSERT_EQUAL(TRACK_CURVE_LEFT, geometryClassifyTrack(5.0f));
}

void test_track_curve_right() {
    TEST_ASSERT_EQUAL(TRACK_CURVE_RIGHT, geometryClassifyTrack(-0.5f));
    TEST_ASSERT_EQUAL(TRACK_CURVE_RIGHT, geometryClassifyTrack(-5.0f));
}

void test_track_at_threshold() {
    // At exactly 0.3 °/s — below threshold, still straight
    TEST_ASSERT_EQUAL(TRACK_STRAIGHT, geometryClassifyTrack(0.29999f));
    // Just above threshold — curve
    TEST_ASSERT_EQUAL(TRACK_CURVE_LEFT, geometryClassifyTrack(0.31f));
    TEST_ASSERT_EQUAL(TRACK_CURVE_RIGHT, geometryClassifyTrack(-0.31f));
}

// ===== Ride quality tests =====

void test_ride_smooth() {
    TEST_ASSERT_EQUAL(RIDE_SMOOTH, geometryClassifyRide(1.0f));
    TEST_ASSERT_EQUAL(RIDE_SMOOTH, geometryClassifyRide(1.005f));
    TEST_ASSERT_EQUAL(RIDE_SMOOTH, geometryClassifyRide(1.0099f));
}

void test_ride_fair() {
    TEST_ASSERT_EQUAL(RIDE_FAIR, geometryClassifyRide(1.01f));
    TEST_ASSERT_EQUAL(RIDE_FAIR, geometryClassifyRide(1.02f));
    TEST_ASSERT_EQUAL(RIDE_FAIR, geometryClassifyRide(1.0299f));
}

void test_ride_rough() {
    TEST_ASSERT_EQUAL(RIDE_ROUGH, geometryClassifyRide(1.03f));
    TEST_ASSERT_EQUAL(RIDE_ROUGH, geometryClassifyRide(1.05f));
    TEST_ASSERT_EQUAL(RIDE_ROUGH, geometryClassifyRide(1.10f));
}

void test_ride_no_gravity() {
    // In theory: 0g RMS means no vibration at all — smooth
    TEST_ASSERT_EQUAL(RIDE_SMOOTH, geometryClassifyRide(0.0f));
}

// ===== Sparkline scaling tests =====

void test_sparkline_scale_basic() {
    float values[] = {0.5f, 1.0f, 1.5f, 2.0f};
    float mn, mx, range;
    geometrySparklineScale(values, 4, 0.01f, &mn, &mx, &range);
    TEST_ASSERT_FLOAT_WITHIN(0.001f, 0.5f, mn);
    TEST_ASSERT_FLOAT_WITHIN(0.001f, 2.0f, mx);
    TEST_ASSERT_FLOAT_WITHIN(0.001f, 1.5f, range);
}

void test_sparkline_scale_constant() {
    // All values identical — should expand to minimum range
    float values[] = {1.0f, 1.0f, 1.0f, 1.0f};
    float mn, mx, range;
    geometrySparklineScale(values, 4, 0.01f, &mn, &mx, &range);
    TEST_ASSERT_FLOAT_WITHIN(0.001f, 0.01f, range);
    // Min/max should be centered around 1.0
    TEST_ASSERT_FLOAT_WITHIN(0.001f, 0.995f, mn);
    TEST_ASSERT_FLOAT_WITHIN(0.001f, 1.005f, mx);
}

void test_sparkline_scale_single_value() {
    float values[] = {0.8f};
    float mn, mx, range;
    geometrySparklineScale(values, 1, 0.01f, &mn, &mx, &range);
    TEST_ASSERT_FLOAT_WITHIN(0.001f, 0.01f, range);
}

void test_sparkline_scale_empty() {
    float mn, mx, range;
    geometrySparklineScale(nullptr, 0, 0.01f, &mn, &mx, &range);
    TEST_ASSERT_FLOAT_WITHIN(0.001f, 0.0f, mn);
    TEST_ASSERT_FLOAT_WITHIN(0.001f, 0.0f, mx);
    TEST_ASSERT_FLOAT_WITHIN(0.001f, 0.01f, range);
}

void test_sparkline_scale_negative_values() {
    float values[] = {-1.5f, -1.0f, -0.5f};
    float mn, mx, range;
    geometrySparklineScale(values, 3, 0.01f, &mn, &mx, &range);
    TEST_ASSERT_FLOAT_WITHIN(0.001f, -1.5f, mn);
    TEST_ASSERT_FLOAT_WITHIN(0.001f, -0.5f, mx);
    TEST_ASSERT_FLOAT_WITHIN(0.001f, 1.0f, range);
}

void test_sparkline_scale_min_range_guard() {
    // Range smaller than minRange should be expanded
    float values[] = {1.0f, 1.002f};
    float mn, mx, range;
    geometrySparklineScale(values, 2, 0.01f, &mn, &mx, &range);
    TEST_ASSERT_FLOAT_WITHIN(0.001f, 0.01f, range);
    TEST_ASSERT_TRUE(mn < 1.0f);
    TEST_ASSERT_TRUE(mx > 1.002f);
}

// ===== Sparkline pixel mapping tests =====

void test_sparkline_pixel_bottom() {
    // Value at min → bottom pixel
    uint8_t y = geometrySparklinePixel(0.0f, 0.0f, 1.0f, 50, 14);
    TEST_ASSERT_EQUAL_UINT8(63, y);  // yTop + height - 1
}

void test_sparkline_pixel_top() {
    // Value at max → top pixel
    uint8_t y = geometrySparklinePixel(1.0f, 0.0f, 1.0f, 50, 14);
    TEST_ASSERT_EQUAL_UINT8(50, y);  // yTop
}

void test_sparkline_pixel_middle() {
    // Value at midpoint → middle pixel
    uint8_t y = geometrySparklinePixel(0.5f, 0.0f, 1.0f, 50, 14);
    // Midpoint: 50 + 13 - (0.5 * 13) = 50 + 13 - 6 = 57
    TEST_ASSERT_EQUAL_UINT8(57, y);
}

void test_sparkline_pixel_clamp_below() {
    // Value below min → clamped to bottom
    uint8_t y = geometrySparklinePixel(-0.5f, 0.0f, 1.0f, 50, 14);
    TEST_ASSERT_EQUAL_UINT8(63, y);
}

void test_sparkline_pixel_clamp_above() {
    // Value above max → clamped to top
    uint8_t y = geometrySparklinePixel(1.5f, 0.0f, 1.0f, 50, 14);
    TEST_ASSERT_EQUAL_UINT8(50, y);
}

// ===== Runner =====

int main(int argc, char **argv) {
    UNITY_BEGIN();

    // Radius computation
    RUN_TEST(test_radius_known_curve);
    RUN_TEST(test_radius_tight_curve);
    RUN_TEST(test_radius_large_curve);
    RUN_TEST(test_radius_zero_yaw_returns_zero);
    RUN_TEST(test_radius_negative_yaw_returns_zero);
    RUN_TEST(test_radius_very_small_yaw);
    RUN_TEST(test_radius_proportional_to_speed);
    RUN_TEST(test_radius_inverse_to_yaw);

    // Unit conversions
    RUN_TEST(test_mm_to_inches);
    RUN_TEST(test_scale_speed_conversion);
    RUN_TEST(test_scale_speed_zero);
    RUN_TEST(test_scale_speed_proportional);

    // SNR
    RUN_TEST(test_snr_at_noise_floor);
    RUN_TEST(test_snr_good_measurement);
    RUN_TEST(test_snr_zero_yaw);
    RUN_TEST(test_snr_classify_good);
    RUN_TEST(test_snr_classify_marginal);
    RUN_TEST(test_snr_classify_poor);

    // Track classification
    RUN_TEST(test_track_straight);
    RUN_TEST(test_track_curve_left);
    RUN_TEST(test_track_curve_right);
    RUN_TEST(test_track_at_threshold);

    // Ride quality
    RUN_TEST(test_ride_smooth);
    RUN_TEST(test_ride_fair);
    RUN_TEST(test_ride_rough);
    RUN_TEST(test_ride_no_gravity);

    // Sparkline scaling
    RUN_TEST(test_sparkline_scale_basic);
    RUN_TEST(test_sparkline_scale_constant);
    RUN_TEST(test_sparkline_scale_single_value);
    RUN_TEST(test_sparkline_scale_empty);
    RUN_TEST(test_sparkline_scale_negative_values);
    RUN_TEST(test_sparkline_scale_min_range_guard);

    // Sparkline pixel mapping
    RUN_TEST(test_sparkline_pixel_bottom);
    RUN_TEST(test_sparkline_pixel_top);
    RUN_TEST(test_sparkline_pixel_middle);
    RUN_TEST(test_sparkline_pixel_clamp_below);
    RUN_TEST(test_sparkline_pixel_clamp_above);

    return UNITY_END();
}
