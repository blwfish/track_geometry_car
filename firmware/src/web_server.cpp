#include "web_server.h"
#include "wifi_manager.h"
#include "flash_logger.h"
#include <ESPAsyncWebServer.h>
#include <ArduinoJson.h>
#include <LittleFS.h>

static AsyncWebServer server(80);
static AsyncWebSocket ws(WS_PATH);

// Recording request flag: set from async WS handler, polled from main loop.
// volatile because it's written from the async WebSocket callback context
// and read from the main loop.
static volatile int8_t recordingRequest = 0;  // 1=start, -1=stop, 0=none
static volatile bool calibrateRequest = false;

// ===== WebSocket event handler =====

static void onWsEvent(AsyncWebSocket *server, AsyncWebSocketClient *client,
                      AwsEventType type, void *arg, uint8_t *data, size_t len) {
    switch (type) {
        case WS_EVT_CONNECT:
            Serial.printf("[WS] Client #%u connected from %s\n",
                          client->id(), client->remoteIP().toString().c_str());
            break;
        case WS_EVT_DISCONNECT:
            Serial.printf("[WS] Client #%u disconnected\n", client->id());
            break;
        case WS_EVT_DATA: {
            if (len >= 1) {
                uint8_t cmd = data[0];
                switch (cmd) {
                    case WS_CMD_START_RECORDING:
                        Serial.println("[WS] Command: Start recording");
                        recordingRequest = 1;
                        break;
                    case WS_CMD_STOP_RECORDING:
                        Serial.println("[WS] Command: Stop recording");
                        recordingRequest = -1;
                        break;
                    case WS_CMD_CALIBRATE:
                        Serial.println("[WS] Command: Calibrate");
                        calibrateRequest = true;
                        break;
                    default:
                        Serial.printf("[WS] Unknown command: 0x%02X\n", cmd);
                        break;
                }
            }
            break;
        }
        case WS_EVT_ERROR:
            Serial.printf("[WS] Client #%u error\n", client->id());
            break;
        case WS_EVT_PONG:
            break;
    }
}

// ===== Captive portal redirect (AP mode only) =====

static String getRedirectURL() {
    char ip[16];
    wifiGetIP(ip, sizeof(ip));
    return String("http://") + ip + "/";
}

static void handleCaptivePortal(AsyncWebServerRequest *request) {
    // In STA mode, no captive portal — just serve the page
    if (wifiGetMode() == GC_WIFI_STA) {
        request->send(LittleFS, "/index.html", "text/html");
        return;
    }

    char ip[16];
    wifiGetIP(ip, sizeof(ip));

    if (request->host() == "connectivitycheck.gstatic.com" ||
        request->host() == "clients3.google.com" ||
        request->host() == "captive.apple.com" ||
        request->host() == "www.msftconnecttest.com") {
        request->redirect(getRedirectURL());
        return;
    }
    if (request->host() != ip) {
        request->redirect(getRedirectURL());
        return;
    }
    request->send(LittleFS, "/index.html", "text/html");
}

// ===== Survey API handlers =====

// GET /api/surveys — list saved survey files as JSON array
static void handleListSurveys(AsyncWebServerRequest *request) {
    String json = "[";
    File dir = LittleFS.open(SURVEY_DIR);
    if (dir && dir.isDirectory()) {
        bool first = true;
        File entry;
        while ((entry = dir.openNextFile())) {
            if (!entry.isDirectory()) {
                size_t sz = entry.size();
                uint32_t samples = (sz > SURVEY_HEADER_SIZE)
                    ? (sz - SURVEY_HEADER_SIZE) / SURVEY_SAMPLE_SIZE : 0;
                float duration = samples / 100.0f;

                if (!first) json += ",";
                first = false;
                json += "{\"name\":\"";
                json += entry.name();
                json += "\",\"size\":";
                json += String(sz);
                json += ",\"samples\":";
                json += String(samples);
                json += ",\"duration_sec\":";
                json += String(duration, 1);
                json += "}";
            }
            entry.close();
        }
        dir.close();
    }
    json += "]";
    request->send(200, "application/json", json);
}

// GET /api/survey/* — download a survey file
static void handleDownloadSurvey(AsyncWebServerRequest *request) {
    // URL is /api/survey/survey_0001.bin — extract filename after last /
    String url = request->url();
    int lastSlash = url.lastIndexOf('/');
    if (lastSlash < 0) {
        request->send(404, "text/plain", "Not found");
        return;
    }
    String filename = url.substring(lastSlash + 1);
    String path = String(SURVEY_DIR) + "/" + filename;

    if (!LittleFS.exists(path)) {
        request->send(404, "text/plain", "Not found");
        return;
    }

    AsyncWebServerResponse *response = request->beginResponse(
        LittleFS, path, "application/octet-stream");
    response->addHeader("Content-Disposition",
        "attachment; filename=\"" + filename + "\"");
    request->send(response);
}

// DELETE /api/survey/* — delete a survey file
static void handleDeleteSurvey(AsyncWebServerRequest *request) {
    String url = request->url();
    int lastSlash = url.lastIndexOf('/');
    if (lastSlash < 0) {
        request->send(404, "text/plain", "Not found");
        return;
    }
    String filename = url.substring(lastSlash + 1);

    if (flashLoggerDelete(filename.c_str())) {
        request->send(200, "text/plain", "OK");
    } else {
        request->send(404, "text/plain", "Not found");
    }
}

// GET /api/status — recording status + flash info
static void handleStatus(AsyncWebServerRequest *request) {
    String json = "{";
    json += "\"recording\":";
    json += flashLoggerIsRecording() ? "true" : "false";
    json += ",\"filename\":\"";
    json += flashLoggerFilename();
    json += "\",\"bytes_written\":";
    json += String(flashLoggerBytesWritten());
    json += ",\"samples_written\":";
    json += String(flashLoggerSampleCount());
    json += ",\"flash_free\":";
    json += String(flashLoggerFlashFree());
    json += ",\"flash_total\":";
    json += String(flashLoggerFlashTotal());
    json += ",\"file_count\":";
    json += String(flashLoggerFileCount());
    json += "}";
    request->send(200, "application/json", json);
}

// ===== WiFi config API handlers =====

// GET /api/wifi/status — current WiFi state
static void handleWifiStatus(AsyncWebServerRequest *request) {
    char ip[16];
    wifiGetIP(ip, sizeof(ip));

    String json = "{\"mode\":\"";
    json += (wifiGetMode() == GC_WIFI_STA) ? "sta" : "ap";
    json += "\",\"ssid\":\"";
    json += wifiGetSSID();
    json += "\",\"ip\":\"";
    json += ip;
    json += "\",\"rssi\":";
    json += String(wifiGetRSSI());
    json += "}";
    request->send(200, "application/json", json);
}

// GET /api/wifi/scan — trigger scan and return results
static void handleWifiScan(AsyncWebServerRequest *request) {
    if (wifiScanInProgress()) {
        request->send(202, "application/json", "{\"status\":\"scanning\"}");
        return;
    }

    if (wifiScanResultsReady()) {
        request->send(200, "application/json", wifiGetScanResultsJSON());
        return;
    }

    // Start a new scan
    wifiStartScan();
    request->send(202, "application/json", "{\"status\":\"scanning\"}");
}

// POST /api/wifi/connect — save credentials and reboot
static void handleWifiConnect(AsyncWebServerRequest *request) {
    // Body handled in onRequestBody
    request->send(400, "text/plain", "Missing body");
}

static void handleWifiConnectBody(AsyncWebServerRequest *request, uint8_t *data,
                                   size_t len, size_t index, size_t total) {
    if (index != 0) return;  // Only handle first chunk

    JsonDocument doc;
    DeserializationError err = deserializeJson(doc, data, len);
    if (err) {
        request->send(400, "text/plain", "Invalid JSON");
        return;
    }

    const char *ssid = doc["ssid"];
    const char *password = doc["password"] | "";

    if (!ssid || strlen(ssid) == 0) {
        request->send(400, "text/plain", "Missing SSID");
        return;
    }

    request->send(200, "application/json", "{\"status\":\"rebooting\"}");

    // Delay slightly to let the response send, then save + reboot
    // wifiConfigureSTA will reboot the ESP32
    delay(200);
    wifiConfigureSTA(ssid, password);
}

// POST /api/wifi/disconnect — clear credentials and reboot to AP
static void handleWifiDisconnect(AsyncWebServerRequest *request) {
    request->send(200, "application/json", "{\"status\":\"rebooting\"}");
    delay(200);
    wifiClearSTA();
}

// ===== Public API =====

void webServerInit() {
    // Note: LittleFS must already be mounted by flashLoggerInit()

    // WebSocket handler
    ws.onEvent(onWsEvent);
    server.addHandler(&ws);

    // Serve static files from LittleFS
    server.serveStatic("/", LittleFS, "/").setDefaultFile("index.html");

    // Survey API routes (must be registered BEFORE captive portal catch-all)
    server.on("/api/surveys", HTTP_GET, handleListSurveys);
    server.on("^\\/api\\/survey\\/(.+)$", HTTP_GET, handleDownloadSurvey);
    server.on("^\\/api\\/survey\\/(.+)$", HTTP_DELETE, handleDeleteSurvey);
    server.on("/api/status", HTTP_GET, handleStatus);

    // WiFi config API routes
    server.on("/api/wifi/status", HTTP_GET, handleWifiStatus);
    server.on("/api/wifi/scan", HTTP_GET, handleWifiScan);
    server.on("/api/wifi/connect", HTTP_POST, handleWifiConnect, NULL, handleWifiConnectBody);
    server.on("/api/wifi/disconnect", HTTP_POST, handleWifiDisconnect);

    // Captive portal: catch-all for unknown hosts
    server.onNotFound(handleCaptivePortal);

    // Android generates a 204 check
    server.on("/generate_204", HTTP_GET, [](AsyncWebServerRequest *request) {
        request->redirect(getRedirectURL());
    });

    // Apple captive portal check
    server.on("/hotspot-detect.html", HTTP_GET, [](AsyncWebServerRequest *request) {
        request->redirect(getRedirectURL());
    });

    server.begin();
    Serial.println("[HTTP] Web server started on port 80");
}

// Send only to clients that aren't backed up.
static void wsSendToHealthyClients(uint8_t *buf, size_t len) {
    for (auto *client : ws.getClients()) {
        if (client->status() == WS_CONNECTED && client->canSend()) {
            client->binary(buf, len);
        }
    }
}

void webServerSendSamples(const imu_sample_t *samples, uint8_t count) {
    if (count == 0 || ws.count() == 0) return;

    size_t frameSize = 2 + count * WS_SAMPLE_WIRE_SIZE;
    uint8_t buf[2 + WS_SAMPLE_BATCH_SIZE * WS_SAMPLE_WIRE_SIZE];

    buf[0] = WS_FRAME_RAW_SAMPLES;
    buf[1] = count;

    for (uint8_t i = 0; i < count; i++) {
        size_t off = 2 + i * WS_SAMPLE_WIRE_SIZE;
        const imu_sample_t *s = &samples[i];

        memcpy(&buf[off + 0],  &s->timestamp_ms, 4);
        memcpy(&buf[off + 4],  &s->accel_x, 2);
        memcpy(&buf[off + 6],  &s->accel_y, 2);
        memcpy(&buf[off + 8],  &s->accel_z, 2);
        memcpy(&buf[off + 10], &s->gyro_x, 2);
        memcpy(&buf[off + 12], &s->gyro_y, 2);
        memcpy(&buf[off + 14], &s->gyro_z, 2);
        memcpy(&buf[off + 16], &s->temperature, 2);
        // IMU #2 fields (zero when single-IMU)
        memcpy(&buf[off + 18], &s->accel_x2, 2);
        memcpy(&buf[off + 20], &s->accel_y2, 2);
        memcpy(&buf[off + 22], &s->accel_z2, 2);
        memcpy(&buf[off + 24], &s->gyro_x2, 2);
        memcpy(&buf[off + 26], &s->gyro_y2, 2);
        memcpy(&buf[off + 28], &s->gyro_z2, 2);
    }

    wsSendToHealthyClients(buf, frameSize);
}

void webServerSendSummary(const summary_1s_t *summary) {
    if (ws.count() == 0) return;

    uint8_t buf[1 + sizeof(summary_1s_t)];
    buf[0] = WS_FRAME_SUMMARY;
    memcpy(&buf[1], summary, sizeof(summary_1s_t));

    wsSendToHealthyClients(buf, sizeof(buf));
}

void webServerSendRecStatus(bool recording, const char *filename,
                            uint32_t bytes, uint32_t samples) {
    if (ws.count() == 0) return;

    uint8_t fnLen = filename ? strlen(filename) : 0;
    // Frame: [type:1][recording:1][fnLen:1][filename:N][bytes:4][samples:4]
    size_t frameSize = 3 + fnLen + 8;
    uint8_t buf[64];  // plenty for any filename

    buf[0] = WS_FRAME_REC_STATUS;
    buf[1] = recording ? 1 : 0;
    buf[2] = fnLen;
    if (fnLen > 0) memcpy(&buf[3], filename, fnLen);
    memcpy(&buf[3 + fnLen], &bytes, 4);
    memcpy(&buf[3 + fnLen + 4], &samples, 4);

    wsSendToHealthyClients(buf, frameSize);
}

uint8_t webServerClientCount() {
    return ws.count();
}

void webServerCleanup() {
    ws.cleanupClients();
}

int8_t webServerGetRecordingRequest() {
    return recordingRequest;
}

void webServerClearRecordingRequest() {
    recordingRequest = 0;
}

bool webServerGetCalibrateRequest() {
    return calibrateRequest;
}

void webServerClearCalibrateRequest() {
    calibrateRequest = false;
}
