#include "web_server.h"
#include <ESPAsyncWebServer.h>
#include <LittleFS.h>

static AsyncWebServer server(80);
static AsyncWebSocket ws(WS_PATH);

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
            // Binary commands from client
            if (len >= 1) {
                uint8_t cmd = data[0];
                switch (cmd) {
                    case WS_CMD_START_RECORDING:
                        Serial.println("[WS] Command: Start recording");
                        break;
                    case WS_CMD_STOP_RECORDING:
                        Serial.println("[WS] Command: Stop recording");
                        break;
                    case WS_CMD_CALIBRATE:
                        Serial.println("[WS] Command: Calibrate");
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

// ===== Captive portal redirect =====

static void handleCaptivePortal(AsyncWebServerRequest *request) {
    // Android captive portal detection
    if (request->host() == "connectivitycheck.gstatic.com" ||
        request->host() == "clients3.google.com") {
        request->redirect("http://192.168.4.1/");
        return;
    }
    // Apple captive portal detection
    if (request->host() == "captive.apple.com") {
        request->redirect("http://192.168.4.1/");
        return;
    }
    // Windows captive portal detection
    if (request->host() == "www.msftconnecttest.com") {
        request->redirect("http://192.168.4.1/");
        return;
    }
    // Generic: if host is not our IP, redirect
    if (request->host() != "192.168.4.1") {
        request->redirect("http://192.168.4.1/");
        return;
    }
    // If we get here, serve from LittleFS
    request->send(LittleFS, "/index.html", "text/html");
}

// ===== Public API =====

void webServerInit() {
    // Initialize LittleFS (format on first use)
    if (!LittleFS.begin(true)) {
        Serial.println("[FS] LittleFS mount failed!");
        return;
    }
    Serial.println("[FS] LittleFS mounted");

    // WebSocket handler
    ws.onEvent(onWsEvent);
    server.addHandler(&ws);

    // Serve static files from LittleFS
    server.serveStatic("/", LittleFS, "/").setDefaultFile("index.html");

    // Captive portal: catch-all for unknown hosts
    server.onNotFound(handleCaptivePortal);

    // Android generates a 204 check
    server.on("/generate_204", HTTP_GET, [](AsyncWebServerRequest *request) {
        request->redirect("http://192.168.4.1/");
    });

    // Apple captive portal check
    server.on("/hotspot-detect.html", HTTP_GET, [](AsyncWebServerRequest *request) {
        request->redirect("http://192.168.4.1/");
    });

    server.begin();
    Serial.println("[HTTP] Web server started on port 80");
}

// Send only to clients that aren't backed up.
// Prevents "Too many messages queued" errors when a client disconnects
// ungracefully and the TCP buffer fills before cleanup reaps the client.
static void wsSendToHealthyClients(uint8_t *buf, size_t len) {
    for (auto *client : ws.getClients()) {
        if (client->status() == WS_CONNECTED && client->canSend()) {
            client->binary(buf, len);
        }
    }
}

void webServerSendSamples(const imu_sample_t *samples, uint8_t count) {
    if (count == 0 || ws.count() == 0) return;

    // Serialize field-by-field: 18 bytes per sample on the wire (no struct padding)
    size_t frameSize = 2 + count * WS_SAMPLE_WIRE_SIZE;
    uint8_t buf[2 + WS_SAMPLE_BATCH_SIZE * WS_SAMPLE_WIRE_SIZE];  // max 182 bytes

    buf[0] = WS_FRAME_RAW_SAMPLES;
    buf[1] = count;

    for (uint8_t i = 0; i < count; i++) {
        size_t off = 2 + i * WS_SAMPLE_WIRE_SIZE;
        const imu_sample_t *s = &samples[i];

        // Little-endian (native on ESP32) — copy field by field
        memcpy(&buf[off + 0],  &s->timestamp_ms, 4);
        memcpy(&buf[off + 4],  &s->accel_x, 2);
        memcpy(&buf[off + 6],  &s->accel_y, 2);
        memcpy(&buf[off + 8],  &s->accel_z, 2);
        memcpy(&buf[off + 10], &s->gyro_x, 2);
        memcpy(&buf[off + 12], &s->gyro_y, 2);
        memcpy(&buf[off + 14], &s->gyro_z, 2);
        memcpy(&buf[off + 16], &s->temperature, 2);
    }

    wsSendToHealthyClients(buf, frameSize);
}

void webServerSendSummary(const summary_1s_t *summary) {
    if (ws.count() == 0) return;

    // Frame type + all summary fields serialized in order
    uint8_t buf[1 + sizeof(summary_1s_t)];
    buf[0] = WS_FRAME_SUMMARY;
    memcpy(&buf[1], summary, sizeof(summary_1s_t));

    wsSendToHealthyClients(buf, sizeof(buf));
}

uint8_t webServerClientCount() {
    return ws.count();
}

void webServerCleanup() {
    ws.cleanupClients();
}
