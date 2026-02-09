#ifndef WIFI_MANAGER_H
#define WIFI_MANAGER_H

#include "config.h"

// Initialize WiFi in open AP mode (no password).
// Sets up DNS server for captive portal.
// Returns true on success.
bool wifiInit();

// Process captive portal DNS requests. Call from loop().
void wifiProcessDNS();

// Get the number of connected WiFi clients.
uint8_t wifiClientCount();

// Get the AP IP address as a string.
const char* wifiGetIP(char *buf, size_t bufLen);

#endif // WIFI_MANAGER_H
