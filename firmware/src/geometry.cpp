#include "geometry.h"
#include <math.h>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

float geometryRadiusMm(float yawRateDps, float speedMms) {
    if (yawRateDps <= 0.0f) return 0.0f;
    float yawRad = yawRateDps * (float)(M_PI / 180.0);
    return speedMms / yawRad;
}

float geometryMmToInches(float mm) {
    return mm / 25.4f;
}

float geometrySNR(float yawRateDps) {
    return yawRateDps / GEOM_GYRO_NOISE_FLOOR_DPS;
}

snr_quality_t geometryClassifySNR(float snr) {
    if (snr >= GEOM_GOOD_SNR) return SNR_GOOD;
    if (snr >= GEOM_MIN_SNR)  return SNR_MARGINAL;
    return SNR_POOR;
}

track_state_t geometryClassifyTrack(float gzMeanDps) {
    float absYaw = fabsf(gzMeanDps);
    if (absYaw < GEOM_CURVE_THRESHOLD_DPS) return TRACK_STRAIGHT;
    return (gzMeanDps > 0.0f) ? TRACK_CURVE_LEFT : TRACK_CURVE_RIGHT;
}

ride_quality_t geometryClassifyRide(float accelZrms) {
    if (accelZrms < GEOM_SMOOTH_THRESHOLD_G) return RIDE_SMOOTH;
    if (accelZrms < GEOM_FAIR_THRESHOLD_G)   return RIDE_FAIR;
    return RIDE_ROUGH;
}

float geometryScaleToModelSpeed(float scaleMph) {
    // 1 mph = 447.04 mm/s, HO scale = 1:87
    return scaleMph * 447.04f / 87.0f;
}

void geometrySparklineScale(const float *values, uint32_t count,
                            float minRange,
                            float *outMin, float *outMax, float *outRange) {
    if (count == 0) {
        *outMin = 0.0f;
        *outMax = 0.0f;
        *outRange = minRange;
        return;
    }

    float mn = values[0], mx = values[0];
    for (uint32_t i = 1; i < count; i++) {
        if (values[i] < mn) mn = values[i];
        if (values[i] > mx) mx = values[i];
    }

    float range = mx - mn;
    if (range < minRange) {
        float mid = (mn + mx) / 2.0f;
        mn = mid - minRange / 2.0f;
        mx = mid + minRange / 2.0f;
        range = minRange;
    }

    *outMin = mn;
    *outMax = mx;
    *outRange = range;
}

uint8_t geometrySparklinePixel(float value, float min, float range,
                               uint8_t yTop, uint8_t height) {
    float normalized = (value - min) / range;  // 0.0 to 1.0
    if (normalized < 0.0f) normalized = 0.0f;
    if (normalized > 1.0f) normalized = 1.0f;
    return yTop + height - 1 - (uint8_t)(normalized * (height - 1));
}
