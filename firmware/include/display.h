#ifndef DISPLAY_H
#define DISPLAY_H

#include "config.h"

// Initialize the OLED display. Returns true on success.
bool displayInit();

// Show startup screen with firmware version. Call once during setup().
void displayStartup(const char *version);

// Show an error message (large font, centered). Blocks display until next call.
void displayError(const char *message);

// Show WiFi info screen. Call during setup after WiFi init.
// mode: "AP" or "STA"; extra: "(open - no password)" or RSSI string
void displayWiFiInfo(const char *ssid, const char *ip, const char *mode = "AP",
                     const char *extra = "(open - no password)");

// Show a status message centered on the display.
// Used for "Joining <network>..." or "Calibrating gyro..." etc.
void displayStatus(const char *line1, const char *line2 = nullptr);

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
