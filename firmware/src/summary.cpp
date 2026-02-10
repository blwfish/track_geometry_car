#include "summary.h"
#include "imu.h"
#include "ring_buffer.h"
#include <math.h>

// Internal: compute summary from an array of samples (most-recent-first order).
static bool computeFromSamples(summary_1s_t *summary, const imu_sample_t *samples,
                               uint32_t n, uint32_t totalSamples, float sampleRate) {
    if (n < 10) return false;

    // Accumulators — single pass, IMU #1
    float ax_sum = 0, ay_sum = 0, az_sum = 0;
    float ax_sum2 = 0, ay_sum2 = 0, az_sum2 = 0;
    float gx_sum2 = 0, gy_sum2 = 0, gz_sum2 = 0;
    float gx_sum = 0, gy_sum = 0, gz_sum = 0;
    float ax_peak = 0, ay_peak = 0, az_peak = 0;
    float temp_sum = 0;

    // Accumulators — IMU #2
    float ax2_sum2 = 0, ay2_sum2 = 0, az2_sum2 = 0;
    float gx2_sum2 = 0, gy2_sum2 = 0, gz2_sum2 = 0;
    float gx2_sum = 0, gy2_sum = 0, gz2_sum = 0;
    float ax2_peak = 0, ay2_peak = 0, az2_peak = 0;

    for (uint32_t i = 0; i < n; i++) {
        const imu_sample_t *s = &samples[i];

        // IMU #1
        float ax = imuAccelG(s->accel_x);
        float ay = imuAccelG(s->accel_y);
        float az = imuAccelG(s->accel_z);
        float gx = imuGyroDPS(s->gyro_x);
        float gy = imuGyroDPS(s->gyro_y);
        float gz = imuGyroDPS(s->gyro_z);

        ax_sum += ax;
        ay_sum += ay;
        az_sum += az;

        ax_sum2 += ax * ax;
        ay_sum2 += ay * ay;
        az_sum2 += az * az;

        gx_sum2 += gx * gx;
        gy_sum2 += gy * gy;
        gz_sum2 += gz * gz;

        gx_sum += gx;
        gy_sum += gy;
        gz_sum += gz;

        float axa = fabsf(ax);
        float aya = fabsf(ay);
        float aza = fabsf(az);
        if (axa > ax_peak) ax_peak = axa;
        if (aya > ay_peak) ay_peak = aya;
        if (aza > az_peak) az_peak = aza;

        temp_sum += imuTemperatureC(s->temperature);

        // IMU #2
        float ax2 = imuAccelG(s->accel_x2);
        float ay2 = imuAccelG(s->accel_y2);
        float az2 = imuAccelG(s->accel_z2);
        float gx2 = imuGyroDPS(s->gyro_x2);
        float gy2 = imuGyroDPS(s->gyro_y2);
        float gz2 = imuGyroDPS(s->gyro_z2);

        ax2_sum2 += ax2 * ax2;
        ay2_sum2 += ay2 * ay2;
        az2_sum2 += az2 * az2;

        gx2_sum2 += gx2 * gx2;
        gy2_sum2 += gy2 * gy2;
        gz2_sum2 += gz2 * gz2;

        gx2_sum += gx2;
        gy2_sum += gy2;
        gz2_sum += gz2;

        float ax2a = fabsf(ax2);
        float ay2a = fabsf(ay2);
        float az2a = fabsf(az2);
        if (ax2a > ax2_peak) ax2_peak = ax2a;
        if (ay2a > ay2_peak) ay2_peak = ay2a;
        if (az2a > az2_peak) az2_peak = az2a;
    }

    float fn = (float)n;
    summary->timestamp_ms = millis();

    // IMU #1
    summary->accel_x_rms = sqrtf(ax_sum2 / fn);
    summary->accel_y_rms = sqrtf(ay_sum2 / fn);
    summary->accel_z_rms = sqrtf(az_sum2 / fn);
    summary->accel_x_peak = ax_peak;
    summary->accel_y_peak = ay_peak;
    summary->accel_z_peak = az_peak;
    summary->gyro_x_rms = sqrtf(gx_sum2 / fn);
    summary->gyro_y_rms = sqrtf(gy_sum2 / fn);
    summary->gyro_z_rms = sqrtf(gz_sum2 / fn);
    summary->gyro_x_mean = gx_sum / fn;
    summary->gyro_y_mean = gy_sum / fn;
    summary->gyro_z_mean = gz_sum / fn;
    summary->temperature = temp_sum / fn;
    summary->sample_count = totalSamples;
    summary->sample_rate = sampleRate;

    // IMU #2 (zeros when single-IMU since raw values are zero)
    summary->accel_x2_rms = sqrtf(ax2_sum2 / fn);
    summary->accel_y2_rms = sqrtf(ay2_sum2 / fn);
    summary->accel_z2_rms = sqrtf(az2_sum2 / fn);
    summary->accel_x2_peak = ax2_peak;
    summary->accel_y2_peak = ay2_peak;
    summary->accel_z2_peak = az2_peak;
    summary->gyro_x2_rms = sqrtf(gx2_sum2 / fn);
    summary->gyro_y2_rms = sqrtf(gy2_sum2 / fn);
    summary->gyro_z2_rms = sqrtf(gz2_sum2 / fn);
    summary->gyro_x2_mean = gx2_sum / fn;
    summary->gyro_y2_mean = gy2_sum / fn;
    summary->gyro_z2_mean = gz2_sum / fn;

    // Accel means (for grade/tilt computation)
    summary->accel_x_mean = ax_sum / fn;
    summary->accel_y_mean = ay_sum / fn;
    summary->accel_z_mean = az_sum / fn;

    return true;
}

bool summaryCompute(summary_1s_t *summary, uint32_t totalSamples, float sampleRate) {
    uint32_t available = ringBufferCount();
    if (available < 10) return false;

    uint32_t n = (available < 100) ? available : 100;

    // Copy samples from ring buffer into local array
    imu_sample_t localBuf[100];
    uint32_t copied = ringBufferCopyRecent(localBuf, n);
    if (copied < 10) return false;

    return computeFromSamples(summary, localBuf, copied, totalSamples, sampleRate);
}

bool summaryComputeFromArray(summary_1s_t *summary, const imu_sample_t *samples,
                             uint32_t sampleCount, uint32_t totalSamples, float sampleRate) {
    return computeFromSamples(summary, samples, sampleCount, totalSamples, sampleRate);
}
