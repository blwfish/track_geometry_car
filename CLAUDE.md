# Track Geometry Car

Instrumented HO model railroad car that surveys track quality using a 6-axis IMU,
streaming live data over WiFi to a phone browser.

## Project Structure

```
firmware/           PlatformIO project (ESP32, Arduino framework)
  include/          Header files (config.h, module interfaces)
  src/              Implementation (.cpp files)
  data/             LittleFS web UI (index.html)
  test/             Unit tests (native + embedded)
docs/               Specifications and architecture documentation
```

## Build Commands

All PlatformIO commands run from `firmware/`:

```bash
# Compile firmware
~/.platformio/penv/bin/pio run

# Flash firmware to ESP32
~/.platformio/penv/bin/pio run -t upload

# Upload web UI to LittleFS
~/.platformio/penv/bin/pio run -t uploadfs

# Serial monitor (115200 baud)
~/.platformio/penv/bin/pio device monitor

# Run native unit tests (no hardware needed)
~/.platformio/penv/bin/pio test -e native

# Run on-target tests (requires ESP32 connected)
~/.platformio/penv/bin/pio test -e esp32dev
```

## Hardware

- **MCU**: ESP32-WROOM-32 devkit (Xtensa LX6, not C3/RISC-V)
- **IMU**: MPU-6050 (GY-521 breakout), I2C address 0x68
- **OLED**: SSD1315 0.96" 128x64, I2C address 0x3C (uses SSD1306 driver)
- **I2C pins**: SDA=GPIO21, SCL=GPIO22, 400kHz

Pin definitions are in `firmware/include/config.h`. When switching to ESP32-C3,
change `I2C_SDA_PIN`/`I2C_SCL_PIN` to GPIO 8/9 and board to `esp32-c3-devkitm-1`.

## Architecture

Modules are independent with clean interfaces (no cross-includes between peers):

| Module | Files | Hardware? | Testable natively? |
|--------|-------|-----------|--------------------|
| config | config.h | No | N/A (definitions only) |
| ring_buffer | ring_buffer.h/cpp | No | Yes |
| summary | summary.h/cpp | No* | Yes* |
| imu | imu.h/cpp | Yes (I2C) | No |
| display | display.h/cpp | Yes (I2C) | No |
| wifi_manager | wifi_manager.h/cpp | Yes (WiFi) | No |
| web_server | web_server.h/cpp | Yes (WiFi) | No |

*summary.cpp calls `imuAccelG()`/`imuGyroDPS()` and `ringBufferGetRecent()` —
these are mockable for native tests.

## Key Design Decisions

- **100Hz sampling via esp_timer** with `volatile uint32_t` counter (not bool flag —
  bool loses samples when OLED I2C holds the bus)
- **Struct padding**: `imu_sample_t` is 18 logical bytes but 20 in memory due to
  alignment. WebSocket serialization is field-by-field, not memcpy.
- **`summary_1s_t`** is all 4-byte aligned fields, safe to memcpy for WebSocket.
- **Dual-color OLED**: SSD1315 has yellow rows 0-15, blue rows 16-63. Text layout
  accounts for this.
- **Captive portal**: DNSServer resolves all domains to 192.168.4.1, triggers
  auto-open on phones.
- **WebSocket backpressure**: `canSend()` check per client prevents queue overflow
  when clients disconnect ungracefully.

## WiFi / Testing the Web UI

Power on the ESP32 → connect phone/laptop to "GeometryCar" open WiFi →
captive portal auto-opens dashboard. If not, navigate to http://192.168.4.1.

## Implementation Status

- [x] Phase 1: IMU + OLED + ring buffer + 100Hz sampling
- [x] Phase 2: WiFi AP + WebSocket + browser dashboard
- [ ] Phase 3: INA219 power monitor (hardware arriving)
- [ ] Phase 4: Flash logging & download
- [ ] Phase 5: Calibration & NVS storage
- [ ] Phase 6: Analysis script (Python)
- [ ] Phase 7: Validation & refinement
