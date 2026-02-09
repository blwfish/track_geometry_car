#include "flash_logger.h"
#include <LittleFS.h>

// ===== State =====
static bool recording = false;
static File recordingFile;
static char currentFilename[32] = {0};      // e.g. "/surveys/survey_0001.bin"
static uint32_t bytesWritten = 0;
static uint32_t samplesWritten = 0;
static uint16_t nextFileNumber = 1;

// ===== Internal helpers =====

// Scan /surveys directory to find the highest existing file number,
// so we pick up where we left off after a reboot.
static void scanForNextFileNumber() {
    File dir = LittleFS.open(SURVEY_DIR);
    if (!dir || !dir.isDirectory()) {
        nextFileNumber = 1;
        return;
    }

    uint16_t highest = 0;
    File entry;
    while ((entry = dir.openNextFile())) {
        const char *name = entry.name();
        // Parse "survey_NNNN.bin" — name from openNextFile() may or may not
        // include the leading path depending on LittleFS version
        const char *p = strstr(name, "survey_");
        if (p) {
            uint16_t num = (uint16_t)atoi(p + 7);  // skip "survey_"
            if (num > highest) highest = num;
        }
        entry.close();
    }
    dir.close();
    nextFileNumber = highest + 1;
}

// Write the 64-byte binary header to the current file.
static bool writeHeader() {
    survey_header_t hdr = {};
    memcpy(hdr.magic, SURVEY_MAGIC, 4);
    hdr.version = SURVEY_VERSION;
    hdr.sample_size = SURVEY_SAMPLE_SIZE;
    hdr.sample_rate_hz = 100;
    hdr.accel_range_g = 2;
    hdr.gyro_range_dps = 250;
    hdr.start_time_ms = millis();
    strncpy(hdr.car_id, "GeometryCar", sizeof(hdr.car_id));

    size_t written = recordingFile.write((const uint8_t *)&hdr, sizeof(hdr));
    return written == sizeof(hdr);
}

// ===== Public API =====

bool flashLoggerInit() {
    if (!LittleFS.begin(true)) {  // true = format on first use
        Serial.println("[FLASH] LittleFS mount failed!");
        return false;
    }

    Serial.printf("[FLASH] LittleFS mounted: %lu KB total, %lu KB used\n",
                  (unsigned long)(LittleFS.totalBytes() / 1024),
                  (unsigned long)(LittleFS.usedBytes() / 1024));

    // Create surveys directory if it doesn't exist
    if (!LittleFS.exists(SURVEY_DIR)) {
        LittleFS.mkdir(SURVEY_DIR);
        Serial.println("[FLASH] Created /surveys directory");
    }

    scanForNextFileNumber();
    Serial.printf("[FLASH] Next survey number: %u\n", nextFileNumber);
    return true;
}

bool flashLoggerStart() {
    if (recording) return false;  // Already recording

    // Check free space before starting
    if (flashLoggerFlashFull()) {
        Serial.println("[FLASH] Cannot start recording: flash full");
        return false;
    }

    // Generate filename
    snprintf(currentFilename, sizeof(currentFilename),
             SURVEY_DIR "/survey_%04u.bin", nextFileNumber);

    recordingFile = LittleFS.open(currentFilename, "w");
    if (!recordingFile) {
        Serial.printf("[FLASH] Failed to open %s for writing\n", currentFilename);
        return false;
    }

    // Write header
    if (!writeHeader()) {
        Serial.println("[FLASH] Failed to write header");
        recordingFile.close();
        LittleFS.remove(currentFilename);
        return false;
    }

    bytesWritten = SURVEY_HEADER_SIZE;
    samplesWritten = 0;
    recording = true;
    nextFileNumber++;

    Serial.printf("[FLASH] Recording started: %s\n", currentFilename);
    return true;
}

void flashLoggerWriteSamples(const imu_sample_t *samples, uint8_t count) {
    if (!recording || count == 0) return;

    // Serialize field-by-field, same layout as WebSocket wire format.
    // 18 bytes per sample: timestamp(4) + accel_xyz(6) + gyro_xyz(6) + temp(2)
    uint8_t buf[WS_SAMPLE_BATCH_SIZE * SURVEY_SAMPLE_SIZE];  // max 180 bytes

    for (uint8_t i = 0; i < count; i++) {
        size_t off = i * SURVEY_SAMPLE_SIZE;
        const imu_sample_t *s = &samples[i];

        memcpy(&buf[off + 0],  &s->timestamp_ms, 4);
        memcpy(&buf[off + 4],  &s->accel_x, 2);
        memcpy(&buf[off + 6],  &s->accel_y, 2);
        memcpy(&buf[off + 8],  &s->accel_z, 2);
        memcpy(&buf[off + 10], &s->gyro_x, 2);
        memcpy(&buf[off + 12], &s->gyro_y, 2);
        memcpy(&buf[off + 14], &s->gyro_z, 2);
        memcpy(&buf[off + 16], &s->temperature, 2);
    }

    size_t totalBytes = count * SURVEY_SAMPLE_SIZE;
    size_t written = recordingFile.write(buf, totalBytes);
    bytesWritten += written;
    samplesWritten += count;

    if (written != totalBytes) {
        Serial.printf("[FLASH] Write error: expected %u, wrote %u\n",
                      (unsigned)totalBytes, (unsigned)written);
    }
}

bool flashLoggerStop() {
    if (!recording) return false;

    recordingFile.flush();
    recordingFile.close();
    recording = false;

    Serial.printf("[FLASH] Recording stopped: %s (%lu samples, %lu bytes)\n",
                  currentFilename, (unsigned long)samplesWritten,
                  (unsigned long)bytesWritten);
    return true;
}

bool flashLoggerIsRecording() {
    return recording;
}

const char* flashLoggerFilename() {
    return currentFilename;
}

uint32_t flashLoggerBytesWritten() {
    return bytesWritten;
}

uint32_t flashLoggerSampleCount() {
    return samplesWritten;
}

uint32_t flashLoggerFlashFree() {
    return LittleFS.totalBytes() - LittleFS.usedBytes();
}

uint32_t flashLoggerFlashTotal() {
    return LittleFS.totalBytes();
}

bool flashLoggerFlashFull() {
    return flashLoggerFlashFree() < FLASH_MIN_FREE_BYTES;
}

uint8_t flashLoggerFileCount() {
    uint8_t count = 0;
    File dir = LittleFS.open(SURVEY_DIR);
    if (!dir || !dir.isDirectory()) return 0;

    File entry;
    while ((entry = dir.openNextFile())) {
        if (!entry.isDirectory()) count++;
        entry.close();
    }
    dir.close();
    return count;
}

bool flashLoggerDelete(const char *filename) {
    char path[48];
    // If filename already has the directory prefix, use as-is
    if (strncmp(filename, SURVEY_DIR, strlen(SURVEY_DIR)) == 0) {
        strncpy(path, filename, sizeof(path));
    } else {
        snprintf(path, sizeof(path), SURVEY_DIR "/%s", filename);
    }
    path[sizeof(path) - 1] = '\0';

    if (!LittleFS.exists(path)) return false;

    bool ok = LittleFS.remove(path);
    if (ok) {
        Serial.printf("[FLASH] Deleted %s\n", path);
    }
    return ok;
}
