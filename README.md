# Track Geometry Car

An instrumented HO scale model railroad car that surveys track quality in real time.
Place it on the track, connect your phone to its WiFi, and get a live strip chart of
every bump, kink, twist, and rough spot on your layout.

## What It Measures

| Defect | How It Shows Up |
|--------|----------------|
| Rail joints & kinks | Vertical acceleration spikes |
| Low spots & dips | Sine-wave dips in vertical profile |
| Rough track / bad ballast | Elevated noise floor over a section |
| Gauge & alignment problems | Lateral acceleration anomalies |
| Twist (cross-level error) | Roll rate deviation — top derailment cause |
| Turnout frog impacts | Characteristic vertical + lateral bump |
| Grade changes | Sustained pitch angle |
| Curve profile | Yaw rate reveals radius, kinks, easements |

## Hardware (~$8)

- ESP32-WROOM-32 devkit
- MPU-6050 6-axis IMU (GY-521 breakout)
- SSD1315 0.96" OLED display
- All connected via I2C (SDA=GPIO21, SCL=GPIO22)

Future additions: INA219 for track voltage/current monitoring, DCC power supply
circuit for self-contained operation.

## Quick Start

### Build & Flash

Requires [PlatformIO](https://platformio.org/).

```bash
cd firmware

# Compile
pio run

# Flash firmware
pio run -t upload

# Upload web UI
pio run -t uploadfs

# Serial monitor
pio device monitor
```

### Use It

1. Power on the ESP32 (USB or track power)
2. Connect your phone to the **GeometryCar** WiFi (open, no password)
3. The dashboard opens automatically via captive portal
4. Place the car on the track and push or tow it around
5. Watch the live strip chart — bumps and defects show up immediately
6. Hit **Start Recording** to capture a survey, **Save** to download as JSON

### Run Tests

```bash
cd firmware
pio test -e native     # Host-side unit tests (no hardware needed)
pio test -e esp32dev   # On-target integration tests
```

## Browser Dashboard

The phone-first dashboard features:
- **Dual-axis strip chart** — accelerometer (g, left axis) and gyroscope (deg/s, right axis)
- **Toggleable traces** — tap to show/hide individual channels
- **Numeric readouts** — RMS, peak, and gyro values updated every second
- **Survey recording** — start/stop/save to JSON for post-processing
- **Responsive layout** — works in portrait, landscape, and desktop browsers

## Project Structure

```
firmware/
  include/        config.h, module headers
  src/            Module implementations
  data/           Web UI (LittleFS)
  test/           Unit tests
docs/
  TRACK_GEOMETRY_CAR_SPEC.md    Full specification
  ARCHITECTURE.md               Firmware architecture
```

## Architecture

The firmware is organized as independent modules with clean interfaces:

- **imu** — MPU-6050 driver, 100Hz burst reads via I2C
- **ring_buffer** — 1024-sample circular buffer (10.24 seconds)
- **summary** — 1Hz RMS/peak/mean statistics from ring buffer
- **display** — OLED status display with sparkline
- **wifi_manager** — Open AP + captive portal via DNS
- **web_server** — AsyncWebServer + binary WebSocket streaming

See [docs/ARCHITECTURE.md](docs/ARCHITECTURE.md) for details.

## License

MIT
