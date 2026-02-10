#include <Arduino.h>
#include <Wire.h>
#include "config.h"
#include "imu.h"
#include "ring_buffer.h"
#include "display.h"
#include "wifi_manager.h"
#include "web_server.h"
#include "summary.h"
#include "flash_logger.h"

#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <freertos/semphr.h>

#if CONFIG_FREERTOS_UNICORE
#include <esp_timer.h>
#endif

// ===== Dual-core synchronization =====
// I2C mutex: arbitrates bus between IMU task (core 0) and OLED (core 1)
static SemaphoreHandle_t i2cMutex = NULL;

// Ring buffer spinlock: protects push (core 0) and read (core 1)
static portMUX_TYPE ringSpinlock = portMUX_INITIALIZER_UNLOCKED;

// ===== IMU task handle =====
#if !CONFIG_FREERTOS_UNICORE
static TaskHandle_t imuTaskHandle = NULL;
#else
// Single-core fallback: timer + counter (original architecture)
static esp_timer_handle_t imuTimer = nullptr;
static volatile uint32_t imuSamplesPending = 0;
static void IRAM_ATTR imuTimerCallback(void *arg) {
    imuSamplesPending++;
}
#endif

// ===== Statistics =====
static volatile uint32_t totalSamples   = 0;
static volatile uint32_t droppedSamples = 0;

// Sample rate measurement
static uint32_t sampleCountAtLastReport = 0;
static unsigned long lastStatsReport     = 0;
static float measuredSamplesPerSec       = 0.0f;

// ===== WebSocket double-buffer =====
// IMU task writes to wsWriteBuf[], main loop reads from wsReadBuf[].
// When wsWriteBuf is full, pointers swap and wsBatchReady signals main loop.
static imu_sample_t wsBufA[WS_SAMPLE_BATCH_SIZE];
static imu_sample_t wsBufB[WS_SAMPLE_BATCH_SIZE];
static imu_sample_t *wsWriteBuf = wsBufA;     // IMU task writes here
static imu_sample_t *wsReadBuf  = wsBufB;     // Main loop reads from here
static volatile uint8_t wsReadCount = 0;       // Samples in read buffer
static volatile bool wsBatchReady = false;     // Signal to main loop
static uint8_t wsWriteCount = 0;               // Samples in write buffer (task-local)

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

static void handleCalibrateRequest() {
    if (!webServerGetCalibrateRequest()) return;
    webServerClearCalibrateRequest();

    Serial.println("[CAL] Re-calibrating gyro (WebSocket request)...");

#if !CONFIG_FREERTOS_UNICORE
    // Dual-core: suspend IMU task so we can use I2C from this core
    vTaskSuspend(imuTaskHandle);
#else
    // Single-core: stop timer
    esp_timer_stop(imuTimer);
#endif

    displayStatus("Calibrating gyro...", "Keep car stationary");
    if (imuCalibrate(500)) {
        Serial.println("[CAL] Re-calibration complete");
    } else {
        Serial.println("[CAL] Re-calibration failed");
    }

#if !CONFIG_FREERTOS_UNICORE
    vTaskResume(imuTaskHandle);
#else
    esp_timer_start_periodic(imuTimer, IMU_SAMPLE_INTERVAL_US);
#endif
}

static void checkFlashFull() {
    if (flashLoggerIsRecording() && flashLoggerFlashFull()) {
        Serial.println("[REC] Flash full — auto-stopping");
        flashLoggerStop();
        webServerSendRecStatus(false, flashLoggerFilename(),
                               flashLoggerBytesWritten(), flashLoggerSampleCount());
    }
}

// ===== IMU Task (dual-core only) =====
#if !CONFIG_FREERTOS_UNICORE

static void imuTaskFunc(void *param) {
    TickType_t lastWake = xTaskGetTickCount();

    for (;;) {
        // Sleep until next 10ms boundary — more consistent than vTaskDelay()
        vTaskDelayUntil(&lastWake, pdMS_TO_TICKS(10));

        imu_sample_t sample;
        bool ok = false;

        // Take I2C mutex (wait for OLED to finish if it's mid-transfer)
        if (xSemaphoreTake(i2cMutex, pdMS_TO_TICKS(I2C_MUTEX_TIMEOUT_MS))) {
            ok = imuReadSample(&sample);
            xSemaphoreGive(i2cMutex);
        }

        if (ok) {
            // Push to ring buffer under spinlock
            taskENTER_CRITICAL(&ringSpinlock);
            ringBufferPush(&sample);
            taskEXIT_CRITICAL(&ringSpinlock);
            totalSamples++;

            // Accumulate into write buffer
            wsWriteBuf[wsWriteCount++] = sample;

            // When batch is full, swap double-buffers
            if (wsWriteCount >= WS_SAMPLE_BATCH_SIZE) {
                imu_sample_t *tmp = wsReadBuf;
                wsReadBuf = wsWriteBuf;
                wsReadCount = wsWriteCount;
                wsWriteBuf = tmp;
                wsWriteCount = 0;
                wsBatchReady = true;
            }
        } else {
            droppedSamples++;
        }
    }
}

#endif // !CONFIG_FREERTOS_UNICORE

// ===== Setup =====
void setup() {
    Serial.begin(115200);
    Serial.printf("\n\n=== Track Geometry Car %s ===\n", FIRMWARE_VERSION);
    Serial.printf("Build: %s\n\n", BUILD_TIME);

#if CONFIG_FREERTOS_UNICORE
    Serial.println("Running in SINGLE-CORE mode (timer-based sampling)");
#else
    Serial.println("Running in DUAL-CORE mode (dedicated IMU task on core 0)");
#endif

    // Initialize I2C bus
    Wire.begin(I2C_SDA_PIN, I2C_SCL_PIN);
    Wire.setClock(I2C_CLOCK_HZ);
    Serial.printf("I2C initialized: SDA=%d, SCL=%d, %dHz\n",
                  I2C_SDA_PIN, I2C_SCL_PIN, I2C_CLOCK_HZ);

    // Create synchronization primitives
    i2cMutex = xSemaphoreCreateMutex();

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

    // Gyro calibration — car must be stationary for ~5 seconds
    displayStatus("Calibrating gyro...", "Keep car stationary");
    if (!imuCalibrate(500)) {
        Serial.println("[WARN] Gyro calibration failed — continuing without offsets");
    }

    // Initialize flash logger (mounts LittleFS — must be before webServerInit)
    if (!flashLoggerInit()) {
        Serial.println("[WARN] Flash logger init failed - continuing without logging");
    }

    // Initialize WiFi (STA with AP fallback)
    if (wifiInit()) {
        wifiReady = true;
        webServerInit();

        char ipBuf[16];
        wifiGetIP(ipBuf, sizeof(ipBuf));
        if (wifiGetMode() == GC_WIFI_STA) {
            char extra[28];
            snprintf(extra, sizeof(extra), "RSSI: %d dBm", (int)wifiGetRSSI());
            displayWiFiInfo(wifiGetSSID(), ipBuf, "STA", extra);
        } else {
            displayWiFiInfo(wifiGetSSID(), ipBuf, "AP", "(open - no password)");
        }
        delay(2500);
    } else {
        Serial.println("[WARN] WiFi init failed - continuing without WiFi");
    }

    // Set up thread-safe display access (must be after displayInit, before IMU task)
    displaySetI2CMutex(i2cMutex);
    displaySetRingSpinlock(&ringSpinlock);

    // Start IMU sampling
#if !CONFIG_FREERTOS_UNICORE
    // Dual-core: dedicated FreeRTOS task on core 0
    xTaskCreatePinnedToCore(
        imuTaskFunc,
        "IMU",
        IMU_TASK_STACK_SIZE,
        NULL,
        IMU_TASK_PRIORITY,
        &imuTaskHandle,
        IMU_TASK_CORE
    );
    Serial.printf("IMU task started on core %d (priority %d, stack %d)\n",
                  IMU_TASK_CORE, IMU_TASK_PRIORITY, IMU_TASK_STACK_SIZE);
#else
    // Single-core: hardware timer + counter
    const esp_timer_create_args_t timerArgs = {
        .callback = imuTimerCallback,
        .arg = nullptr,
        .dispatch_method = ESP_TIMER_TASK,
        .name = "imu_timer"
    };
    ESP_ERROR_CHECK(esp_timer_create(&timerArgs, &imuTimer));
    ESP_ERROR_CHECK(esp_timer_start_periodic(imuTimer, IMU_SAMPLE_INTERVAL_US));
    Serial.println("IMU timer started (single-core mode, 100Hz)");
#endif

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

    // --- Check calibrate requests from WebSocket ---
    if (wifiReady) {
        handleCalibrateRequest();
    }

#if CONFIG_FREERTOS_UNICORE
    // --- Single-core: 100Hz IMU sampling (counter-driven) ---
    while (imuSamplesPending > 0) {
        imuSamplesPending--;

        imu_sample_t sample;
        if (imuReadSample(&sample)) {
            taskENTER_CRITICAL(&ringSpinlock);
            ringBufferPush(&sample);
            taskEXIT_CRITICAL(&ringSpinlock);
            totalSamples++;

            // Accumulate into write buffer
            wsWriteBuf[wsWriteCount++] = sample;

            // When batch is full, swap double-buffers
            if (wsWriteCount >= WS_SAMPLE_BATCH_SIZE) {
                imu_sample_t *tmp = wsReadBuf;
                wsReadBuf = wsWriteBuf;
                wsReadCount = wsWriteCount;
                wsWriteBuf = tmp;
                wsWriteCount = 0;
                wsBatchReady = true;
            }
        } else {
            droppedSamples++;
        }
    }
#endif

    // --- WebSocket sample broadcast + flash logging ---
    // Dual-core: triggered by double-buffer swap (wsBatchReady)
    // Single-core: triggered by time interval (same as before)
#if CONFIG_FREERTOS_UNICORE
    if (now - lastWsSend >= WS_SEND_INTERVAL_MS) {
        lastWsSend = now;
#endif

    if (wsBatchReady) {
        wsBatchReady = false;

        // Capture batch pointer and count (stable after flag clear)
        imu_sample_t *batch = wsReadBuf;
        uint8_t count = wsReadCount;

        if (wifiReady && webServerClientCount() > 0 && count > 0) {
            webServerSendSamples(batch, count);
        }

        // Write same batch to flash if recording
        if (flashLoggerIsRecording() && count > 0) {
            flashLoggerWriteSamples(batch, count);
            checkFlashFull();
        }
    }

#if CONFIG_FREERTOS_UNICORE
    }
#endif

    // --- 10Hz Serial CSV output ---
    if (now - lastSerialOutput >= SERIAL_OUTPUT_INTERVAL_MS) {
        lastSerialOutput = now;

        imu_sample_t latestCopy;
        bool haveLatest = false;
        taskENTER_CRITICAL(&ringSpinlock);
        const imu_sample_t *latest = ringBufferGetRecent(0);
        if (latest) {
            latestCopy = *latest;
            haveLatest = true;
        }
        taskEXIT_CRITICAL(&ringSpinlock);

        if (haveLatest) {
            Serial.printf("%lu,%.4f,%.4f,%.4f,%.2f,%.2f,%.2f,%.1f",
                (unsigned long)latestCopy.timestamp_ms,
                imuAccelG(latestCopy.accel_x),
                imuAccelG(latestCopy.accel_y),
                imuAccelG(latestCopy.accel_z),
                imuGyroDPS(latestCopy.gyro_x),
                imuGyroDPS(latestCopy.gyro_y),
                imuGyroDPS(latestCopy.gyro_z),
                imuTemperatureC(latestCopy.temperature));
            if (imuGetCount() > 1) {
                Serial.printf(",%.4f,%.4f,%.4f,%.2f,%.2f,%.2f",
                    imuAccelG(latestCopy.accel_x2),
                    imuAccelG(latestCopy.accel_y2),
                    imuAccelG(latestCopy.accel_z2),
                    imuGyroDPS(latestCopy.gyro_x2),
                    imuGyroDPS(latestCopy.gyro_y2),
                    imuGyroDPS(latestCopy.gyro_z2));
            }
            Serial.println();
        }
    }

    // --- 1Hz Summary computation + WebSocket broadcast ---
    if (now - lastSummary >= SUMMARY_INTERVAL_MS) {
        lastSummary = now;

        // Copy samples under spinlock, compute outside
        imu_sample_t summaryBuf[100];
        uint32_t copied;
        taskENTER_CRITICAL(&ringSpinlock);
        copied = ringBufferCopyRecent(summaryBuf, 100);
        taskEXIT_CRITICAL(&ringSpinlock);

        if (summaryComputeFromArray(&latestSummary, summaryBuf, copied,
                                     totalSamples, measuredSamplesPerSec)) {
            if (wifiReady && webServerClientCount() > 0) {
                webServerSendSummary(&latestSummary);
            }
        }
    }

    // --- 5Hz OLED update ---
    if (now - lastOledUpdate >= OLED_UPDATE_INTERVAL_MS) {
        lastOledUpdate = now;

        // Copy latest sample under spinlock
        imu_sample_t latestCopy;
        bool haveLatest = false;
        taskENTER_CRITICAL(&ringSpinlock);
        const imu_sample_t *latest = ringBufferGetRecent(0);
        if (latest) {
            latestCopy = *latest;
            haveLatest = true;
        }
        taskEXIT_CRITICAL(&ringSpinlock);

        if (haveLatest) {
            uint8_t clients = wifiReady ? wifiClientCount() : 0;
            bool rec = flashLoggerIsRecording();
            uint32_t recElapsed = rec ? (now - recordingStartTime) / 1000 : 0;
            // Pass latest summary for geometry display (curve/straight indicator)
            const summary_1s_t *sumPtr = (latestSummary.sample_count > 0) ? &latestSummary : nullptr;
            displayUpdate(&latestCopy, totalSamples, droppedSamples,
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
