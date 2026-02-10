#include "wifi_manager.h"
#include <WiFi.h>
#include <DNSServer.h>
#include <Preferences.h>

static DNSServer dnsServer;
static const uint16_t DNS_PORT = 53;
static gc_wifi_mode_t currentMode = GC_WIFI_AP;
static char staSSID[33] = {0};  // Current SSID (AP or STA)

// ===== NVS helpers =====

static bool nvsLoadCredentials(char *ssid, size_t ssidLen, char *pass, size_t passLen) {
    Preferences prefs;
    if (!prefs.begin(WIFI_NVS_NAMESPACE, true)) return false;

    String s = prefs.getString(WIFI_NVS_KEY_SSID, "");
    String p = prefs.getString(WIFI_NVS_KEY_PASS, "");
    prefs.end();

    if (s.length() == 0) return false;

    strncpy(ssid, s.c_str(), ssidLen - 1);
    ssid[ssidLen - 1] = '\0';
    strncpy(pass, p.c_str(), passLen - 1);
    pass[passLen - 1] = '\0';
    return true;
}

static bool nvsSaveCredentials(const char *ssid, const char *password) {
    Preferences prefs;
    if (!prefs.begin(WIFI_NVS_NAMESPACE, false)) return false;

    prefs.putString(WIFI_NVS_KEY_SSID, ssid);
    prefs.putString(WIFI_NVS_KEY_PASS, password);
    prefs.end();
    return true;
}

static void nvsClearCredentials() {
    Preferences prefs;
    if (!prefs.begin(WIFI_NVS_NAMESPACE, false)) return;
    prefs.remove(WIFI_NVS_KEY_SSID);
    prefs.remove(WIFI_NVS_KEY_PASS);
    prefs.end();
}

// ===== STA connection attempt =====

static bool trySTAConnect(const char *ssid, const char *password) {
    Serial.printf("[WiFi] Attempting STA connection to '%s'...\n", ssid);

    WiFi.mode(WIFI_STA);
    WiFi.begin(ssid, password);

    unsigned long start = millis();
    while (WiFi.status() != WL_CONNECTED) {
        if (millis() - start > WIFI_STA_CONNECT_TIMEOUT_MS) {
            Serial.printf("[WiFi] STA connection timeout after %dms\n",
                          WIFI_STA_CONNECT_TIMEOUT_MS);
            WiFi.disconnect(true);
            return false;
        }
        delay(100);
    }

    Serial.printf("[WiFi] STA connected: SSID=%s, IP=%s, RSSI=%d dBm\n",
                  ssid, WiFi.localIP().toString().c_str(), WiFi.RSSI());
    return true;
}

// ===== AP setup =====

static bool startAP() {
    WiFi.mode(WIFI_AP_STA);  // AP+STA: AP serves clients, STA enables scanning

    bool ok = WiFi.softAP(WIFI_AP_SSID, NULL, WIFI_AP_CHANNEL, false, WIFI_AP_MAX_CONN);
    if (!ok) {
        Serial.println("[WiFi] AP start FAILED");
        return false;
    }

    delay(100);  // Let AP stabilize

    dnsServer.start(DNS_PORT, "*", WiFi.softAPIP());

    Serial.printf("[WiFi] AP started: SSID=%s (open), IP=%s\n",
                  WIFI_AP_SSID, WiFi.softAPIP().toString().c_str());
    Serial.println("[WiFi] Captive portal DNS active");
    return true;
}

// ===== Public API =====

bool wifiInit() {
    char ssid[33] = {0};
    char pass[65] = {0};

    if (nvsLoadCredentials(ssid, sizeof(ssid), pass, sizeof(pass))) {
        if (trySTAConnect(ssid, pass)) {
            currentMode = GC_WIFI_STA;
            strncpy(staSSID, ssid, sizeof(staSSID) - 1);
            return true;
        }
        Serial.println("[WiFi] STA failed, falling back to AP mode");
    }

    // No credentials or STA failed — start AP
    currentMode = GC_WIFI_AP;
    strncpy(staSSID, WIFI_AP_SSID, sizeof(staSSID) - 1);
    return startAP();
}

gc_wifi_mode_t wifiGetMode() {
    return currentMode;
}

void wifiProcessDNS() {
    if (currentMode == GC_WIFI_AP) {
        dnsServer.processNextRequest();
    }
}

uint8_t wifiClientCount() {
    if (currentMode == GC_WIFI_STA) {
        return WiFi.status() == WL_CONNECTED ? 1 : 0;
    }
    return WiFi.softAPgetStationNum();
}

const char* wifiGetIP(char *buf, size_t bufLen) {
    if (currentMode == GC_WIFI_STA) {
        strncpy(buf, WiFi.localIP().toString().c_str(), bufLen - 1);
    } else {
        strncpy(buf, WiFi.softAPIP().toString().c_str(), bufLen - 1);
    }
    buf[bufLen - 1] = '\0';
    return buf;
}

const char* wifiGetSSID() {
    return staSSID;
}

int8_t wifiGetRSSI() {
    if (currentMode == GC_WIFI_STA) {
        return (int8_t)WiFi.RSSI();
    }
    return 0;
}

bool wifiStartScan() {
    // WiFi.scanNetworks(async=true, show_hidden=false)
    int result = WiFi.scanNetworks(true, false);
    return result == WIFI_SCAN_RUNNING || result >= 0;
}

bool wifiScanInProgress() {
    return WiFi.scanComplete() == WIFI_SCAN_RUNNING;
}

bool wifiScanResultsReady() {
    return WiFi.scanComplete() >= 0;
}

String wifiGetScanResultsJSON() {
    int n = WiFi.scanComplete();
    if (n < 0) return "[]";

    String json = "[";
    for (int i = 0; i < n; i++) {
        if (i > 0) json += ",";
        json += "{\"ssid\":\"";
        // Escape any quotes in SSID
        String ssid = WiFi.SSID(i);
        ssid.replace("\"", "\\\"");
        json += ssid;
        json += "\",\"rssi\":";
        json += String(WiFi.RSSI(i));
        json += ",\"open\":";
        json += (WiFi.encryptionType(i) == WIFI_AUTH_OPEN) ? "true" : "false";
        json += "}";
    }
    json += "]";

    WiFi.scanDelete();
    return json;
}

bool wifiConfigureSTA(const char *ssid, const char *password) {
    if (!nvsSaveCredentials(ssid, password)) {
        Serial.println("[WiFi] Failed to save STA credentials");
        return false;
    }
    Serial.printf("[WiFi] STA credentials saved for '%s', rebooting...\n", ssid);
    delay(500);
    ESP.restart();
    return true;  // Never reached
}

void wifiClearSTA() {
    nvsClearCredentials();
    Serial.println("[WiFi] STA credentials cleared, rebooting into AP mode...");
    delay(500);
    ESP.restart();
}
