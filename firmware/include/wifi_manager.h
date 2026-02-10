#ifndef WIFI_MANAGER_H
#define WIFI_MANAGER_H

#include "config.h"

// WiFi operating mode (prefixed to avoid conflict with ESP-IDF wifi_mode_t)
enum gc_wifi_mode_t {
    GC_WIFI_AP,
    GC_WIFI_STA
};

// Initialize WiFi.
// Checks NVS for saved STA credentials. If found, attempts STA connection
// with WIFI_STA_CONNECT_TIMEOUT_MS timeout. On failure (or no credentials),
// falls back to open AP mode with captive portal DNS.
// Returns true on success (either mode).
bool wifiInit();

// Get the current WiFi operating mode.
gc_wifi_mode_t wifiGetMode();

// Process captive portal DNS requests. Call from loop().
// Only processes in AP mode; no-op in STA mode.
void wifiProcessDNS();

// Get the number of connected WiFi clients (AP mode) or 1 if STA connected.
uint8_t wifiClientCount();

// Get the current IP address as a string (AP IP or STA IP).
const char* wifiGetIP(char *buf, size_t bufLen);

// Get the SSID we're connected to (STA mode) or AP SSID.
const char* wifiGetSSID();

// Get RSSI in dBm (STA mode only, returns 0 in AP mode).
int8_t wifiGetRSSI();

// Trigger an async WiFi network scan. Results available via wifiGetScanResults().
// Returns true if scan was started.
bool wifiStartScan();

// Check if a scan is in progress.
bool wifiScanInProgress();

// Check if scan results are available (scan completed with results).
bool wifiScanResultsReady();

// Get scan results as a JSON string. Caller must call wifiStartScan() first
// and wait for wifiScanResultsReady() to return true.
// Returns JSON array: [{ssid, rssi, open}, ...]
// Consumes results (subsequent calls return "[]" until next scan).
String wifiGetScanResultsJSON();

// Save STA credentials to NVS and reboot into STA mode.
bool wifiConfigureSTA(const char *ssid, const char *password);

// Clear saved STA credentials from NVS and reboot into AP mode.
void wifiClearSTA();

#endif // WIFI_MANAGER_H
