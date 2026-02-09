#include "wifi_manager.h"
#include <WiFi.h>
#include <DNSServer.h>

static DNSServer dnsServer;
static const uint16_t DNS_PORT = 53;

bool wifiInit() {
    WiFi.mode(WIFI_AP);

    // Open AP — no password needed for a geometry car
    bool ok = WiFi.softAP(WIFI_AP_SSID, NULL, WIFI_AP_CHANNEL, false, WIFI_AP_MAX_CONN);
    if (!ok) {
        Serial.println("[WiFi] AP start FAILED");
        return false;
    }

    delay(100);  // Let AP stabilize

    // Start DNS server for captive portal — resolve all domains to our IP
    dnsServer.start(DNS_PORT, "*", WiFi.softAPIP());

    Serial.printf("[WiFi] AP started: SSID=%s (open), IP=%s\n",
                  WIFI_AP_SSID, WiFi.softAPIP().toString().c_str());
    Serial.println("[WiFi] Captive portal DNS active");
    return true;
}

void wifiProcessDNS() {
    dnsServer.processNextRequest();
}

uint8_t wifiClientCount() {
    return WiFi.softAPgetStationNum();
}

const char* wifiGetIP(char *buf, size_t bufLen) {
    strncpy(buf, WiFi.softAPIP().toString().c_str(), bufLen - 1);
    buf[bufLen - 1] = '\0';
    return buf;
}
