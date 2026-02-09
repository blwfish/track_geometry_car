#ifndef FLASH_LOGGER_H
#define FLASH_LOGGER_H

#include "config.h"

// Initialize LittleFS and survey directory. Call before webServerInit().
// Returns true if LittleFS mounted successfully.
bool flashLoggerInit();

// Start recording: open new file, write header.
// Returns true on success.
bool flashLoggerStart();

// Write a batch of samples to the current recording file.
// Uses field-by-field serialization (18 bytes/sample, no struct padding).
// No-op if not recording.
void flashLoggerWriteSamples(const imu_sample_t *samples, uint8_t count);

// Stop recording: close file.
// Returns true on success, false if not recording.
bool flashLoggerStop();

// Query recording state
bool flashLoggerIsRecording();
const char* flashLoggerFilename();     // Current or last recording filename
uint32_t flashLoggerBytesWritten();    // Bytes written to current recording
uint32_t flashLoggerSampleCount();     // Samples written to current recording

// Flash storage info
uint32_t flashLoggerFlashFree();
uint32_t flashLoggerFlashTotal();

// Returns true if flash is too full to continue recording
bool flashLoggerFlashFull();

// File management
uint8_t flashLoggerFileCount();                    // Number of saved survey files
bool flashLoggerDelete(const char *filename);      // Delete a survey file

#endif // FLASH_LOGGER_H
