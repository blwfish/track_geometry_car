#ifndef DISPLAY_H
#define DISPLAY_H

#include "config.h"

// Initialize the OLED display. Returns true on success.
bool displayInit();

// Show startup screen with firmware version. Call once during setup().
void displayStartup(const char *version);

// Show an error message (large font, centered). Blocks display until next call.
void displayError(const char *message);

// Show WiFi AP info screen. Call during setup after WiFi init.
void displayWiFiInfo(const char *ssid, const char *ip);

// Update the live display with current IMU data and stats.
// Called at OLED_UPDATE_INTERVAL_MS rate from main loop.
// When recording is true, the yellow zone shows REC indicator + elapsed time.
// summary may be nullptr if no summary is available yet.
void displayUpdate(const imu_sample_t *latest, uint32_t totalSamples,
                   uint32_t droppedSamples, float samplesPerSec,
                   uint8_t wifiClients, bool recording = false,
                   uint32_t recElapsedSec = 0,
                   const summary_1s_t *summary = nullptr);

#endif // DISPLAY_H
