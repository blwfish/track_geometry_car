#ifndef GEOMETRY_H
#define GEOMETRY_H

#include <stdint.h>

// ===== Track Geometry Computation =====
// Pure math functions for curve detection, radius estimation,
// ride quality classification, and SNR assessment.
// No hardware dependencies — fully testable natively.

// Geometry detection thresholds
#define GEOM_CURVE_THRESHOLD_DPS   0.3f    // Below this mean yaw = straight
#define GEOM_GYRO_NOISE_FLOOR_DPS  0.05f   // MPU-6050 noise floor (~20Hz DLPF)
#define GEOM_GOOD_SNR              20.0f   // SNR for confident measurement
#define GEOM_MIN_SNR               10.0f   // SNR for marginal measurement

// Ride quality thresholds (vertical accel RMS including 1g gravity)
#define GEOM_SMOOTH_THRESHOLD_G    1.01f   // Below = smooth
#define GEOM_FAIR_THRESHOLD_G      1.03f   // Below = fair, above = rough

// Track state classification
enum track_state_t {
    TRACK_STRAIGHT,
    TRACK_CURVE_LEFT,
    TRACK_CURVE_RIGHT
};

// Ride quality classification
enum ride_quality_t {
    RIDE_SMOOTH,
    RIDE_FAIR,
    RIDE_ROUGH
};

// SNR quality classification
enum snr_quality_t {
    SNR_GOOD,       // >= GOOD_SNR — confident measurement
    SNR_MARGINAL,   // >= MIN_SNR  — usable but noisy
    SNR_POOR        // < MIN_SNR   — unreliable, increase speed
};

// Compute curve radius in millimeters from yaw rate and speed.
// yawRateDps: absolute mean yaw rate in deg/s (must be > 0)
// speedMms:   model speed in mm/s
// Returns radius in mm. Returns 0 if yawRateDps <= 0.
float geometryRadiusMm(float yawRateDps, float speedMms);

// Convert radius from mm to inches.
float geometryMmToInches(float mm);

// Compute signal-to-noise ratio for a yaw rate measurement.
// yawRateDps: absolute mean yaw rate in deg/s
// Returns SNR (dimensionless).
float geometrySNR(float yawRateDps);

// Classify SNR quality.
snr_quality_t geometryClassifySNR(float snr);

// Determine track state from signed mean yaw rate (gz_mean from summary).
// Positive gz = left curve, negative gz = right curve (right-hand rule).
track_state_t geometryClassifyTrack(float gzMeanDps);

// Classify ride quality from vertical acceleration RMS.
// accelZrms includes gravity (~1.0g baseline).
ride_quality_t geometryClassifyRide(float accelZrms);

// Convert scale speed in mph to model speed in mm/s for HO (1:87).
// scaleMph: speed in scale miles per hour
float geometryScaleToModelSpeed(float scaleMph);

// Sparkline auto-scaling: compute min, max, range from an array of floats.
// Enforces minimum range of minRange to avoid division by zero.
// Writes results to *outMin, *outMax, *outRange.
void geometrySparklineScale(const float *values, uint32_t count,
                            float minRange,
                            float *outMin, float *outMax, float *outRange);

// Map a value to a pixel Y coordinate within a sparkline band.
// value: the data value
// min, range: from geometrySparklineScale()
// yTop: top pixel of sparkline area
// height: pixel height of sparkline area
// Returns the Y pixel coordinate (higher Y = lower on screen).
uint8_t geometrySparklinePixel(float value, float min, float range,
                               uint8_t yTop, uint8_t height);

#endif // GEOMETRY_H
