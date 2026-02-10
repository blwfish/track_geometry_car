#ifndef WEB_SERVER_H
#define WEB_SERVER_H

#include "config.h"

// Initialize AsyncWebServer, WebSocket, and HTTP API routes.
// LittleFS must already be mounted (by flashLoggerInit()).
// Must be called after wifiInit().
void webServerInit();

// Broadcast a batch of raw IMU samples to connected WebSocket clients.
// Serializes field-by-field (18 bytes per sample on wire, no struct padding).
void webServerSendSamples(const imu_sample_t *samples, uint8_t count);

// Broadcast a 1-second summary to connected WebSocket clients.
void webServerSendSummary(const summary_1s_t *summary);

// Broadcast recording status change to connected WebSocket clients.
// Frame type 0x03: [0x03][recording:u8][filename_len:u8][filename][bytes:u32][samples:u32]
void webServerSendRecStatus(bool recording, const char *filename,
                            uint32_t bytes, uint32_t samples);

// Get the number of connected WebSocket clients.
uint8_t webServerClientCount();

// Clean up disconnected WebSocket clients. Call every ~2 seconds.
void webServerCleanup();

// Recording request flag: set by WebSocket command handler (async context),
// polled by main loop (synchronous context).
// Returns: 1 = start requested, -1 = stop requested, 0 = no request
int8_t webServerGetRecordingRequest();
void webServerClearRecordingRequest();

// Calibration request flag: set by WS_CMD_CALIBRATE, polled by main loop.
bool webServerGetCalibrateRequest();
void webServerClearCalibrateRequest();

#endif // WEB_SERVER_H
