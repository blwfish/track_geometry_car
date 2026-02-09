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
#include "flash_logger.h"

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

// ===== Recording state =====
static unsigned long recordingStartTime = 0;

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

// ===== Recording helpers =====

static void handleRecordingRequest() {
    int8_t req = webServerGetRecordingRequest();
    if (req == 0) return;
    webServerClearRecordingRequest();

    if (req == 1 && !flashLoggerIsRecording()) {
        // Start recording
        if (flashLoggerStart()) {
            recordingStartTime = millis();
            webServerSendRecStatus(true, flashLoggerFilename(), 0, 0);
            Serial.printf("[REC] Started: %s\n", flashLoggerFilename());
        } else {
            Serial.println("[REC] Failed to start recording");
            webServerSendRecStatus(false, "", 0, 0);
        }
    } else if (req == -1 && flashLoggerIsRecording()) {
        // Stop recording
        flashLoggerStop();
        webServerSendRecStatus(false, flashLoggerFilename(),
                               flashLoggerBytesWritten(), flashLoggerSampleCount());
        Serial.printf("[REC] Stopped: %lu samples, %lu bytes\n",
                      (unsigned long)flashLoggerSampleCount(),
                      (unsigned long)flashLoggerBytesWritten());
    }
}

static void checkFlashFull() {
    if (flashLoggerIsRecording() && flashLoggerFlashFull()) {
        Serial.println("[REC] Flash full — auto-stopping");
        flashLoggerStop();
        webServerSendRecStatus(false, flashLoggerFilename(),
                               flashLoggerBytesWritten(), flashLoggerSampleCount());
    }
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
        delay(1500);
    }

    // Initialize MPU-6050
    Serial.printf("MPU-6050 WHO_AM_I: 0x%02X\n", imuWhoAmI());
    if (!imuInit()) {
        Serial.println("[FATAL] MPU-6050 init failed!");
        displayError("MPU-6050 NOT FOUND");
        while (true) { delay(1000); }
    }

    // Initialize flash logger (mounts LittleFS — must be before webServerInit)
    if (!flashLoggerInit()) {
        Serial.println("[WARN] Flash logger init failed - continuing without logging");
    }

    // Initialize WiFi AP
    if (wifiInit()) {
        wifiReady = true;
        webServerInit();

        char ipBuf[16];
        wifiGetIP(ipBuf, sizeof(ipBuf));
        displayWiFiInfo(WIFI_AP_SSID, ipBuf);
        delay(2500);
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

    if (imuGetCount() > 1) {
        Serial.println("\ntimestamp_ms,ax_g,ay_g,az_g,gx_dps,gy_dps,gz_dps,temp_c,ax2_g,ay2_g,az2_g,gx2_dps,gy2_dps,gz2_dps");
    } else {
        Serial.println("\ntimestamp_ms,ax_g,ay_g,az_g,gx_dps,gy_dps,gz_dps,temp_c");
    }

    lastStatsReport = millis();
    Serial.printf("Setup complete. Sampling at 100Hz. IMU count: %u\n\n", imuGetCount());
}

// ===== Main Loop =====
void loop() {
    unsigned long now = millis();

    // --- Captive portal DNS processing ---
    if (wifiReady) {
        wifiProcessDNS();
    }

    // --- Check recording requests from WebSocket ---
    if (wifiReady) {
        handleRecordingRequest();
    }

    // --- 100Hz IMU sampling (counter-driven) ---
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

    // --- 10Hz WebSocket sample broadcast + flash logging ---
    if (now - lastWsSend >= WS_SEND_INTERVAL_MS) {
        lastWsSend = now;

        if (wifiReady && webServerClientCount() > 0 && wsSampleCount > 0) {
            webServerSendSamples(wsSampleBatch, wsSampleCount);
        }

        // Write same batch to flash if recording
        if (flashLoggerIsRecording() && wsSampleCount > 0) {
            flashLoggerWriteSamples(wsSampleBatch, wsSampleCount);
            checkFlashFull();
        }

        wsSampleCount = 0;
    }

    // --- 10Hz Serial CSV output ---
    if (now - lastSerialOutput >= SERIAL_OUTPUT_INTERVAL_MS) {
        lastSerialOutput = now;

        const imu_sample_t *latest = ringBufferGetRecent(0);
        if (latest) {
            Serial.printf("%lu,%.4f,%.4f,%.4f,%.2f,%.2f,%.2f,%.1f",
                (unsigned long)latest->timestamp_ms,
                imuAccelG(latest->accel_x),
                imuAccelG(latest->accel_y),
                imuAccelG(latest->accel_z),
                imuGyroDPS(latest->gyro_x),
                imuGyroDPS(latest->gyro_y),
                imuGyroDPS(latest->gyro_z),
                imuTemperatureC(latest->temperature));
            if (imuGetCount() > 1) {
                Serial.printf(",%.4f,%.4f,%.4f,%.2f,%.2f,%.2f",
                    imuAccelG(latest->accel_x2),
                    imuAccelG(latest->accel_y2),
                    imuAccelG(latest->accel_z2),
                    imuGyroDPS(latest->gyro_x2),
                    imuGyroDPS(latest->gyro_y2),
                    imuGyroDPS(latest->gyro_z2));
            }
            Serial.println();
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
            bool rec = flashLoggerIsRecording();
            uint32_t recElapsed = rec ? (now - recordingStartTime) / 1000 : 0;
            // Pass latest summary for geometry display (curve/straight indicator)
            const summary_1s_t *sumPtr = (latestSummary.sample_count > 0) ? &latestSummary : nullptr;
            displayUpdate(latest, totalSamples, droppedSamples,
                          measuredSamplesPerSec, clients, rec, recElapsed, sumPtr);
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

        Serial.printf("# Stats: %.1f Hz (%lu samples, %lu dropped, %lu total, %lu free heap",
            measuredSamplesPerSec,
            (unsigned long)samplesThisPeriod,
            (unsigned long)droppedSamples,
            (unsigned long)totalSamples,
            (unsigned long)ESP.getFreeHeap());

        if (flashLoggerIsRecording()) {
            Serial.printf(", REC %lu samples, %lu KB flash free",
                (unsigned long)flashLoggerSampleCount(),
                (unsigned long)(flashLoggerFlashFree() / 1024));
        }

        Serial.printf(", %u WS clients)\n",
            wifiReady ? webServerClientCount() : 0);

        sampleCountAtLastReport = totalSamples;
        lastStatsReport = now;
    }
}
