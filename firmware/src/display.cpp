#include "display.h"
#include "geometry.h"
#include "imu.h"
#include "ring_buffer.h"
#include "wifi_manager.h"
#include <U8g2lib.h>
#include <Wire.h>
#include <math.h>

// Full-buffer constructor for SSD1306/SSD1315 128x64 I2C OLED.
// SSD1315 is register-compatible with SSD1306; U8g2 drives it natively.
static U8G2_SSD1306_128X64_NONAME_F_HW_I2C u8g2(U8G2_R0, /* reset=*/ U8X8_PIN_NONE);

// Dual-color OLED layout: yellow band = rows 0-15, blue band = rows 16-63
static const uint8_t YELLOW_ZONE_HEIGHT = 16;  // Top 16 rows are yellow
static const uint8_t BLUE_ZONE_TOP      = 16;  // Blue starts at row 16

// Sparkline dimensions (in blue zone)
static const uint8_t SPARKLINE_Y      = 50;   // Top of sparkline area
static const uint8_t SPARKLINE_HEIGHT = 14;   // Pixels tall
static const uint8_t SPARKLINE_WIDTH  = 128;  // Full display width

// Default survey speed for OLED radius display (mm/s).
// 10 scale mph in HO (1:87) ≈ 51.4 mm/s model speed.
// Browser dashboard lets the user override this.
static const float DEFAULT_SPEED_MMS = geometryScaleToModelSpeed(10.0f);

bool displayInit() {
    bool ok = u8g2.begin();
    if (ok) {
        u8g2.setContrast(128);
    }
    return ok;
}

void displayStartup(const char *version) {
    u8g2.clearBuffer();
    u8g2.setFont(u8g2_font_6x10_tf);
    u8g2.drawStr(0, 10, "Track Geometry Car");
    u8g2.setFont(u8g2_font_5x7_tf);
    u8g2.drawStr(0, 22, version);
    u8g2.drawStr(0, 32, BUILD_TIME);
    u8g2.drawStr(0, 46, "Initializing...");
    u8g2.sendBuffer();
}

void displayError(const char *message) {
    u8g2.clearBuffer();
    u8g2.setFont(u8g2_font_6x10_tf);
    u8g2.drawStr(0, 12, "ERROR:");
    u8g2.setFont(u8g2_font_5x7_tf);
    u8g2.drawStr(0, 28, message);
    u8g2.sendBuffer();
}

void displayWiFiInfo(const char *ssid, const char *ip, const char *mode,
                     const char *extra) {
    u8g2.clearBuffer();
    u8g2.setFont(u8g2_font_6x10_tf);
    u8g2.drawStr(0, 10, "Track Geometry Car");
    u8g2.setFont(u8g2_font_5x7_tf);
    char line[28];
    snprintf(line, sizeof(line), "%s: %s", mode, ssid);
    u8g2.drawStr(0, 24, line);
    if (extra && extra[0]) {
        u8g2.drawStr(0, 34, extra);
    }
    snprintf(line, sizeof(line), "IP: %s", ip);
    u8g2.drawStr(0, 48, line);
    u8g2.sendBuffer();
}

void displayStatus(const char *line1, const char *line2) {
    u8g2.clearBuffer();
    u8g2.setFont(u8g2_font_6x10_tf);
    u8g2.drawStr(0, 10, "Track Geometry Car");
    u8g2.setFont(u8g2_font_5x7_tf);
    u8g2.drawStr(0, 32, line1);
    if (line2) {
        u8g2.drawStr(0, 44, line2);
    }
    u8g2.sendBuffer();
}

void displayUpdate(const imu_sample_t *latest, uint32_t totalSamples,
                   uint32_t droppedSamples, float samplesPerSec,
                   uint8_t wifiClients, bool recording,
                   uint32_t recElapsedSec,
                   const summary_1s_t *summary) {
    char line[28];  // 128px / 5px per char = 25 chars + margin

    u8g2.clearBuffer();
    u8g2.setFont(u8g2_font_5x7_tf);

    // === YELLOW ZONE (rows 0-15): Status line ===
    if (recording) {
        uint32_t m = recElapsedSec / 60;
        uint32_t s = recElapsedSec % 60;
        if (wifiGetMode() == GC_WIFI_STA) {
            snprintf(line, sizeof(line), "REC %lu:%02lu %ddB",
                     (unsigned long)m, (unsigned long)s, (int)wifiGetRSSI());
        } else {
            snprintf(line, sizeof(line), "REC %lu:%02lu W:%u",
                     (unsigned long)m, (unsigned long)s, wifiClients);
        }
    } else {
        if (wifiGetMode() == GC_WIFI_STA) {
            snprintf(line, sizeof(line), "%.0fHz %luS %ddB",
                     samplesPerSec, (unsigned long)totalSamples,
                     (int)wifiGetRSSI());
        } else {
            snprintf(line, sizeof(line), "%.0fHz %luS W:%u",
                     samplesPerSec, (unsigned long)totalSamples,
                     wifiClients);
        }
    }
    u8g2.drawStr(0, 7, line);

    // Show "2×" indicator at right edge when dual IMU is active
    if (imuHasSecond()) {
        u8g2.drawStr(128 - 10, 7, "2x");  // 2 chars × 5px = 10px
    }

    // === BLUE ZONE (rows 16-63): Data ===

    // Line 2: Accelerometer (g)
    float ax = imuAccelG(latest->accel_x);
    float ay = imuAccelG(latest->accel_y);
    float az = imuAccelG(latest->accel_z);
    snprintf(line, sizeof(line), "A %+.2f %+.2f %+.2f", ax, ay, az);
    u8g2.drawStr(0, 24, line);

    // Line 3: Geometry info (curve/straight + speed recommendation)
    // Uses 1-second mean yaw rate from summary for stability
    if (summary != nullptr) {
        track_state_t state = geometryClassifyTrack(summary->gyro_z_mean);

        if (state == TRACK_STRAIGHT) {
            ride_quality_t ride = geometryClassifyRide(summary->accel_z_rms);
            const char *grade = (ride == RIDE_SMOOTH) ? "Smooth" :
                                (ride == RIDE_FAIR)   ? "Fair"   : "Rough";
            snprintf(line, sizeof(line), "Straight  %s", grade);
        } else {
            float absYaw = fabsf(summary->gyro_z_mean);
            float radiusMm = geometryRadiusMm(absYaw, DEFAULT_SPEED_MMS);
            float radiusIn = geometryMmToInches(radiusMm);
            float snr = geometrySNR(absYaw);
            snr_quality_t sq = geometryClassifySNR(snr);

            const char *spdHint = (sq == SNR_GOOD)     ? "OK" :
                                  (sq == SNR_MARGINAL)  ? "ok" :
                                                          "\x18SPD";
            snprintf(line, sizeof(line), "R:%.0f\" %s", radiusIn, spdHint);
        }
    } else {
        // No summary yet — show raw gyro
        float gx = imuGyroDPS(latest->gyro_x);
        float gy = imuGyroDPS(latest->gyro_y);
        float gz = imuGyroDPS(latest->gyro_z);
        snprintf(line, sizeof(line), "G %+.1f %+.1f %+.1f", gx, gy, gz);
    }
    u8g2.drawStr(0, 34, line);

    // Line 4: Temperature
    float tempC = imuTemperatureC(latest->temperature);
    snprintf(line, sizeof(line), "T:%.1fC", tempC);
    u8g2.drawStr(0, 44, line);

    // Separator line above sparkline
    u8g2.drawHLine(0, SPARKLINE_Y - 2, 128);

    // Sparkline: last 128 Z-axis accel samples
    uint32_t available = ringBufferCount();
    uint32_t sparkCount = (available < SPARKLINE_WIDTH) ? available : SPARKLINE_WIDTH;

    if (sparkCount > 1) {
        // Collect Z-accel values for auto-scaling
        float zValues[SPARKLINE_WIDTH];
        for (uint32_t i = 0; i < sparkCount; i++) {
            const imu_sample_t *s = ringBufferGetRecent(sparkCount - 1 - i);
            zValues[i] = s ? imuAccelG(s->accel_z) : 0.0f;
        }

        // Auto-scale with minimum range guard
        float zMin, zMax, range;
        geometrySparklineScale(zValues, sparkCount, 0.01f, &zMin, &zMax, &range);

        // Draw sparkline pixels
        uint8_t xOffset = SPARKLINE_WIDTH - sparkCount;
        for (uint32_t i = 0; i < sparkCount; i++) {
            uint8_t y = geometrySparklinePixel(zValues[i], zMin, range,
                                                SPARKLINE_Y, SPARKLINE_HEIGHT);
            u8g2.drawPixel(xOffset + i, y);
        }
    }

    u8g2.sendBuffer();
}
