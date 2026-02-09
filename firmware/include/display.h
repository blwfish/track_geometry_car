#ifndef DISPLAY_H
#define DISPLAY_H

#include "config.h"

// Initialize the OLED display. Returns true on success.
bool displayInit();

// Show startup screen with firmware version. Call once during setup().
void displayStartup(const char *version);

// Show an error message (large font, centered). Blocks display until next call.
void displayError(const char *message);

// Update the live display with current IMU data and stats.
// Called at OLED_UPDATE_INTERVAL_MS rate from main loop.
void displayUpdate(const imu_sample_t *latest, uint32_t totalSamples,
                   uint32_t droppedSamples, float samplesPerSec);

#endif // DISPLAY_H
