#include "summary.h"
#include "imu.h"
#include "ring_buffer.h"
#include <math.h>

bool summaryCompute(summary_1s_t *summary, uint32_t totalSamples, float sampleRate) {
    uint32_t available = ringBufferCount();
    if (available < 10) return false;

    uint32_t n = (available < 100) ? available : 100;

    // Accumulators — single pass
    float ax_sum2 = 0, ay_sum2 = 0, az_sum2 = 0;
    float gx_sum2 = 0, gy_sum2 = 0, gz_sum2 = 0;
    float gx_sum = 0, gy_sum = 0, gz_sum = 0;
    float ax_peak = 0, ay_peak = 0, az_peak = 0;
    float temp_sum = 0;

    for (uint32_t i = 0; i < n; i++) {
        const imu_sample_t *s = ringBufferGetRecent(i);
        if (!s) continue;

        float ax = imuAccelG(s->accel_x);
        float ay = imuAccelG(s->accel_y);
        float az = imuAccelG(s->accel_z);
        float gx = imuGyroDPS(s->gyro_x);
        float gy = imuGyroDPS(s->gyro_y);
        float gz = imuGyroDPS(s->gyro_z);

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
    }

    float fn = (float)n;
    summary->timestamp_ms = millis();
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

    return true;
}
