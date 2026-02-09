#ifndef WEB_SERVER_H
#define WEB_SERVER_H

#include "config.h"

// Initialize AsyncWebServer, WebSocket, and LittleFS.
// Must be called after wifiInit().
void webServerInit();

// Broadcast a batch of raw IMU samples to connected WebSocket clients.
// Serializes field-by-field (18 bytes per sample on wire, no struct padding).
void webServerSendSamples(const imu_sample_t *samples, uint8_t count);

// Broadcast a 1-second summary to connected WebSocket clients.
void webServerSendSummary(const summary_1s_t *summary);

// Get the number of connected WebSocket clients.
uint8_t webServerClientCount();

// Clean up disconnected WebSocket clients. Call every ~2 seconds.
void webServerCleanup();

#endif // WEB_SERVER_H
