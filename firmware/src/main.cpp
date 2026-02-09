#include <Arduino.h>
#include <Wire.h>
#include <esp_timer.h>
#include "config.h"
#include "imu.h"
#include "ring_buffer.h"
#include "display.h"
#include "wifi_manager.h"
#include "web_server.h"
#include "summary.h"

// ===== Timer counter =====
// Incremented by hardware timer callback, decremented by main loop.
// Using a counter instead of a bool ensures we don't miss samples
// when the main loop is busy (e.g. during OLED I2C writes).
static volatile uint32_t imuSamplesPending = 0;

// ===== Statistics =====
static uint32_t totalSamples   = 0;
static uint32_t droppedSamples = 0;

// Sample rate measurement
static uint32_t sampleCountAtLastReport = 0;
static unsigned long lastStatsReport     = 0;
static float measuredSamplesPerSec       = 0.0f;

// ===== WebSocket sample accumulation =====
static imu_sample_t wsSampleBatch[WS_SAMPLE_BATCH_SIZE];
static uint8_t wsSampleCount = 0;

// ===== Latest summary =====
static summary_1s_t latestSummary;

// ===== WiFi state =====
static bool wifiReady = false;

// ===== Non-blocking task timers =====
static unsigned long lastOledUpdate    = 0;
static unsigned long lastSerialOutput  = 0;
static unsigned long lastWsSend        = 0;
static unsigned long lastSummary       = 0;
static unsigned long lastWsCleanup     = 0;

// ===== Hardware timer =====
static esp_timer_handle_t imuTimer = nullptr;

static void IRAM_ATTR imuTimerCallback(void *arg) {
    imuSamplesPending++;
}

// ===== Setup =====
void setup() {
    Serial.begin(115200);
    Serial.printf("\n\n=== Track Geometry Car %s ===\n", FIRMWARE_VERSION);
    Serial.printf("Build: %s\n\n", BUILD_TIME);

    // Initialize I2C bus
    Wire.begin(I2C_SDA_PIN, I2C_SCL_PIN);
    Wire.setClock(I2C_CLOCK_HZ);
    Serial.printf("I2C initialized: SDA=%d, SCL=%d, %dHz\n",
                  I2C_SDA_PIN, I2C_SCL_PIN, I2C_CLOCK_HZ);

    // Initialize ring buffer
    ringBufferInit();

    // Initialize OLED display
    if (!displayInit()) {
        Serial.println("[WARN] OLED init failed - continuing without display");
    } else {
        displayStartup(FIRMWARE_VERSION);
        delay(1500);  // Show startup screen (acceptable in setup only)
    }

    // Initialize MPU-6050
    Serial.printf("MPU-6050 WHO_AM_I: 0x%02X\n", imuWhoAmI());
    if (!imuInit()) {
        Serial.println("[FATAL] MPU-6050 init failed!");
        displayError("MPU-6050 NOT FOUND");
        while (true) { delay(1000); }  // Halt - can't proceed without IMU
    }

    // Initialize WiFi AP (before starting timer)
    if (wifiInit()) {
        wifiReady = true;
        webServerInit();

        // Show WiFi info on OLED
        char ipBuf[16];
        wifiGetIP(ipBuf, sizeof(ipBuf));
        displayWiFiInfo(WIFI_AP_SSID, ipBuf);
        delay(2500);  // Show WiFi info screen
    } else {
        Serial.println("[WARN] WiFi init failed - continuing without WiFi");
    }

    // Start 100Hz sampling timer
    const esp_timer_create_args_t timerArgs = {
        .callback = imuTimerCallback,
        .arg = nullptr,
        .dispatch_method = ESP_TIMER_TASK,
        .name = "imu_timer"
    };
    ESP_ERROR_CHECK(esp_timer_create(&timerArgs, &imuTimer));
    ESP_ERROR_CHECK(esp_timer_start_periodic(imuTimer, IMU_SAMPLE_INTERVAL_US));

    // Print CSV header for serial output
    Serial.println("\ntimestamp_ms,ax_g,ay_g,az_g,gx_dps,gy_dps,gz_dps,temp_c");

    lastStatsReport = millis();
    Serial.println("Setup complete. Sampling at 100Hz.\n");
}

// ===== Main Loop =====
void loop() {
    unsigned long now = millis();

    // --- Captive portal DNS processing ---
    if (wifiReady) {
        wifiProcessDNS();
    }

    // --- 100Hz IMU sampling (counter-driven) ---
    // Drain all pending samples — catches up after OLED writes hold the bus
    while (imuSamplesPending > 0) {
        imuSamplesPending--;

        imu_sample_t sample;
        if (imuReadSample(&sample)) {
            ringBufferPush(&sample);
            totalSamples++;

            // Accumulate for WebSocket batch
            if (wsSampleCount < WS_SAMPLE_BATCH_SIZE) {
                wsSampleBatch[wsSampleCount++] = sample;
            }
        } else {
            droppedSamples++;
        }
    }

    // --- 10Hz WebSocket sample broadcast ---
    if (now - lastWsSend >= WS_SEND_INTERVAL_MS) {
        lastWsSend = now;

        if (wifiReady && webServerClientCount() > 0 && wsSampleCount > 0) {
            webServerSendSamples(wsSampleBatch, wsSampleCount);
        }
        wsSampleCount = 0;  // Reset regardless
    }

    // --- 10Hz Serial CSV output ---
    if (now - lastSerialOutput >= SERIAL_OUTPUT_INTERVAL_MS) {
        lastSerialOutput = now;

        const imu_sample_t *latest = ringBufferGetRecent(0);
        if (latest) {
            Serial.printf("%lu,%.4f,%.4f,%.4f,%.2f,%.2f,%.2f,%.1f\n",
                (unsigned long)latest->timestamp_ms,
                imuAccelG(latest->accel_x),
                imuAccelG(latest->accel_y),
                imuAccelG(latest->accel_z),
                imuGyroDPS(latest->gyro_x),
                imuGyroDPS(latest->gyro_y),
                imuGyroDPS(latest->gyro_z),
                imuTemperatureC(latest->temperature));
        }
    }

    // --- 1Hz Summary computation + WebSocket broadcast ---
    if (now - lastSummary >= SUMMARY_INTERVAL_MS) {
        lastSummary = now;

        if (summaryCompute(&latestSummary, totalSamples, measuredSamplesPerSec)) {
            if (wifiReady && webServerClientCount() > 0) {
                webServerSendSummary(&latestSummary);
            }
        }
    }

    // --- 5Hz OLED update ---
    if (now - lastOledUpdate >= OLED_UPDATE_INTERVAL_MS) {
        lastOledUpdate = now;

        const imu_sample_t *latest = ringBufferGetRecent(0);
        if (latest) {
            uint8_t clients = wifiReady ? wifiClientCount() : 0;
            displayUpdate(latest, totalSamples, droppedSamples,
                          measuredSamplesPerSec, clients);
        }
    }

    // --- 0.5Hz WebSocket cleanup ---
    if (wifiReady && now - lastWsCleanup >= WS_CLEANUP_INTERVAL_MS) {
        lastWsCleanup = now;
        webServerCleanup();
    }

    // --- 0.1Hz Stats report (every 10 seconds) ---
    if (now - lastStatsReport >= STATS_REPORT_INTERVAL_MS) {
        uint32_t samplesThisPeriod = totalSamples - sampleCountAtLastReport;
        float elapsed = (now - lastStatsReport) / 1000.0f;
        measuredSamplesPerSec = samplesThisPeriod / elapsed;

        Serial.printf("# Stats: %.1f Hz (%lu samples, %lu dropped, %lu total, %lu free heap, %u WS clients)\n",
            measuredSamplesPerSec,
            (unsigned long)samplesThisPeriod,
            (unsigned long)droppedSamples,
            (unsigned long)totalSamples,
            (unsigned long)ESP.getFreeHeap(),
            wifiReady ? webServerClientCount() : 0);

        sampleCountAtLastReport = totalSamples;
        lastStatsReport = now;
    }
}
