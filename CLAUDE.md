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
analysis/           Python post-processing (analyze_survey.py)
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

## Analysis Commands

Run from `analysis/` using the venv:

```bash
# Set up venv (first time only)
python3 -m venv .venv && .venv/bin/pip install numpy matplotlib

# Analyze a binary survey file from flash
.venv/bin/python analyze_survey.py <survey.bin>

# Analyze a browser-captured JSON file
.venv/bin/python analyze_survey.py <survey.json>

# Custom speed/thresholds
.venv/bin/python analyze_survey.py survey.bin --speed 15 --section-len 500

# Before/after comparison
.venv/bin/python analyze_survey.py after.bin --compare before.bin

# Generate synthetic test data
.venv/bin/python generate_test_data.py
```

## Hardware

- **MCU**: ESP32-WROOM-32 devkit (Xtensa LX6, not C3/RISC-V)
- **IMU**: MPU-6050 (GY-521 breakout), I2C address 0x68 (primary, required)
- **IMU #2**: MPU-6050 at 0x69 (AD0=HIGH, optional — auto-detected at boot)
- **OLED**: SSD1315 0.96" 128x64, I2C address 0x3C (uses SSD1306 driver)
- **I2C pins**: SDA=GPIO21, SCL=GPIO22, 400kHz

Pin definitions are in `firmware/include/config.h`. When switching to ESP32-C3,
change `I2C_SDA_PIN`/`I2C_SCL_PIN` to GPIO 8/9 and board to `esp32-c3-devkitm-1`.

## Architecture

Modules are independent with clean interfaces (no cross-includes between peers):

| Module | Files | Hardware? | Testable natively? |
|--------|-------|-----------|--------------------|
| config | config.h | No | N/A (definitions only) |
| ring_buffer | ring_buffer.h/cpp | No | Yes (11 tests) |
| summary | summary.h/cpp | No* | Yes* (20 tests) |
| geometry | geometry.h/cpp | No | Yes (37 tests) |
| imu | imu.h/cpp | Yes (I2C) | No |
| display | display.h/cpp | Yes (I2C) | No |
| wifi_manager | wifi_manager.h/cpp | Yes (WiFi) | No |
| web_server | web_server.h/cpp | Yes (WiFi) | No |
| flash_logger | flash_logger.h/cpp | Yes (LittleFS) | No |

*summary.cpp calls `imuAccelG()`/`imuGyroDPS()` and `ringBufferGetRecent()` —
these are mockable for native tests.

geometry.cpp contains pure math extracted from display.cpp: curve radius computation,
track state classification, ride quality grading, SNR assessment, speed conversion,
and sparkline auto-scaling. All functions are hardware-independent.

## Key Design Decisions

- **100Hz sampling via esp_timer** with `volatile uint32_t` counter (not bool flag —
  bool loses samples when OLED I2C holds the bus)
- **Struct padding**: `imu_sample_t` is 30 logical bytes (18 IMU1 + 12 IMU2) but
  larger in memory due to alignment. WebSocket/flash serialization is field-by-field.
- **Dual IMU**: IMU #2 at 0x69 is auto-detected. When absent, all IMU2 fields are
  zero-filled (memset in imuReadSample). Wire format is always 30 bytes/sample;
  survey files use version 1 (single) or 2 (dual) in the header.
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
- [x] Phase 4: Flash logging & download (LittleFS, ~14 min capacity)
- [ ] Phase 4b: Micro-SD card logging (SdFat, FAT32, format-on-first-use)
- [ ] Phase 5: Calibration & NVS storage
- [x] Phase 6: Analysis script (Python) — `analysis/analyze_survey.py`
- [ ] Phase 7: Validation & refinement
- [ ] Future: MQTT throttle control (autonomous survey speed via JMRI)
- [ ] Future: Clearance detector (pivoting arm + hall sensor)
- [x] Dual IMU support (MPU-6050 x2, 0x68 + 0x69, graceful single-IMU fallback)
- [ ] Future: ESP32-CAM wheelset camera (standalone, MJPEG stream)
- [ ] Future: 3D printed PRR F43 depressed-center flat car body

## Related Projects

- **test-and-calibration-track** — Automated loco speed calibration with vibration
  analysis. Identifies the quietest locomotive in the fleet for geometry survey duty
  (least mechanical noise coupling through couplers/rails into the IMU). Also provides
  calibrated speed tables so the geometry car knows its true velocity from the
  commanded speed step, enabling accurate spatial measurement without an on-car
  speed sensor.
- **esp32-config** — Shared MQTT infrastructure and ESP32 patterns.

See `docs/FUTURE_PLANS.md` for detailed notes on all future ideas.
