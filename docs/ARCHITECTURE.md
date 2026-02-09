# Firmware Architecture

## Overview

The firmware runs on an ESP32-WROOM-32 (Arduino framework via PlatformIO) and does
three things simultaneously: samples a 6-axis IMU at 100Hz, displays live status on
an OLED, and streams data over WiFi/WebSocket to a phone browser.

All I2C devices (MPU-6050, SSD1315 OLED) share one bus at 400kHz. There is no RTOS
task splitting — everything runs in a single-threaded cooperative loop driven by
timing checks.

## Module Dependency Graph

```
main.cpp
  |
  +-- imu.h/cpp            MPU-6050 driver (I2C)
  +-- ring_buffer.h/cpp    Circular sample storage
  +-- summary.h/cpp        1Hz statistics (reads ring_buffer, uses imu conversions)
  +-- display.h/cpp        OLED driver (U8g2, I2C)
  +-- wifi_manager.h/cpp   WiFi AP + captive portal DNS
  +-- web_server.h/cpp     AsyncWebServer + WebSocket
  +-- config.h             Shared constants, structs, pin definitions
```

No module includes another peer module's header (except summary, which uses
ring_buffer and imu conversion helpers). All coordination happens in main.cpp.

## Data Flow

```
                 esp_timer ISR (100Hz)
                      |
                      v
              imuSamplesPending++  (volatile counter)
                      |
                      v
              loop() drains counter
                      |
            +---------+---------+
            |                   |
            v                   v
      ringBufferPush()    wsSampleBatch[]
            |                   |
     +------+------+           |  (every 100ms, batch of 10)
     |             |           v
  summary      display    webServerSendSamples()
  (1Hz)        (5Hz)          |
     |             |           v
     v             v      WebSocket binary frame (type 0x01)
  webServerSendSummary()      |
     |                        v
     v                   Browser: strip chart + recording
  WebSocket binary frame (type 0x02)
     |
     v
  Browser: numeric readouts
```

## Timing Budget

The main loop runs continuously. Each iteration:

| Task | Frequency | Duration | Notes |
|------|-----------|----------|-------|
| IMU read | 100 Hz | ~0.5 ms | I2C burst read 14 bytes |
| Ring buffer push | 100 Hz | <0.01 ms | Array copy |
| WebSocket batch send | 10 Hz | ~0.2 ms | 182 bytes per frame |
| Summary compute | 1 Hz | ~0.3 ms | 100 samples, single pass |
| OLED update | 5 Hz | ~3 ms | U8g2 sendBuffer() holds I2C bus |
| Serial CSV | 10 Hz | ~0.1 ms | Debug output |
| DNS process | every loop | <0.01 ms | Captive portal |
| WS cleanup | 0.5 Hz | <0.1 ms | Reap dead clients |

Total CPU utilization is well under 50%. The main constraint is the shared I2C bus:
the OLED's 3ms sendBuffer() blocks IMU reads, which is why the sampling uses a
counter (not a boolean flag) — the counter accumulates during the OLED write and the
loop drains all pending reads afterward.

## Sample Rate Integrity

### The Problem

The original design used `volatile bool imuSampleFlag`. The timer ISR set it to true,
and the loop checked and cleared it. But when the OLED sendBuffer() held the I2C bus
for ~3ms, the timer could fire multiple times while the flag was already true. Each
extra firing was lost — the bool can't count. Result: 85Hz instead of 100Hz.

### The Fix

Changed to `volatile uint32_t imuSamplesPending`. The ISR increments it. The loop
uses a `while (pending > 0)` drain loop that reads one IMU sample per decrement.
After a 3ms OLED write, the counter reaches ~3, and the loop immediately catches up
by reading 3 samples back-to-back. No samples lost.

## WebSocket Protocol

### Binary Frames (Server to Client)

**Type 0x01 — Raw Sample Batch:**
```
Byte 0:     0x01 (frame type)
Byte 1:     count (number of samples, typically 10)
Bytes 2+:   count * 18 bytes, each sample:
              [0..3]   uint32_t timestamp_ms (little-endian)
              [4..5]   int16_t  accel_x
              [6..7]   int16_t  accel_y
              [8..9]   int16_t  accel_z
              [10..11]  int16_t  gyro_x
              [12..13]  int16_t  gyro_y
              [14..15]  int16_t  gyro_z
              [16..17]  int16_t  temperature
```

Note: samples are serialized field-by-field (18 bytes on wire), NOT via memcpy of
`imu_sample_t` (which is 20 bytes in memory due to compiler padding for alignment).

**Type 0x02 — 1-Second Summary:**
```
Byte 0:     0x02 (frame type)
Bytes 1+:   memcpy of summary_1s_t (all fields are 4-byte aligned, no padding issues)
              uint32_t timestamp_ms
              float    accel_x_rms, accel_y_rms, accel_z_rms
              float    accel_x_peak, accel_y_peak, accel_z_peak
              float    gyro_x_rms, gyro_y_rms, gyro_z_rms
              float    gyro_x_mean, gyro_y_mean, gyro_z_mean
              float    temperature
              uint32_t sample_count
              float    sample_rate
```

### Commands (Client to Server)

| Byte | Command |
|------|---------|
| 0x10 | Start recording |
| 0x11 | Stop recording |
| 0x12 | Calibrate |

## Captive Portal

The WiFi AP runs a DNS server that resolves ALL domain lookups to 192.168.4.1.
When a phone connects to the open "GeometryCar" network, the OS performs captive
portal detection:

- **Android**: requests `connectivitycheck.gstatic.com` or `clients3.google.com`
- **Apple**: requests `captive.apple.com`
- **Windows**: requests `www.msftconnecttest.com`

The web server intercepts these with HTTP 302 redirects to `http://192.168.4.1/`.
A catch-all handler redirects any non-local host. The result: the browser opens
automatically without the user typing any URL.

## Browser Dashboard

Single-page app served from LittleFS (`data/index.html`). Features:

- **WebSocket connection** with auto-reconnect (2 second retry)
- **Dual-axis strip chart** (Canvas 2D): left axis = accel (g), right axis = gyro (deg/s), independently auto-scaled
- **Trace toggles**: tap to show/hide any of the 6 channels
- **Numeric readouts**: RMS, peak (accel), RMS/mean (gyro), temp, sample rate
- **Recording**: browser-side accumulation of raw samples, saved as JSON blob
- **Responsive**: portrait phone, landscape phone (compact layout), desktop (3-column readouts)
- **Orientation handling**: listens for both `resize` and `orientationchange` events

Data is parsed from binary WebSocket frames using `DataView` for endian-correct
access. Chart uses `Float32Array` circular buffers for performance. Subsampling
limits the number of canvas line segments to screen width.

## Memory Usage

As of Phase 2 (v0.2.0):
- **RAM**: 20.8% (68KB of 328KB)
- **Flash**: 67.3% (882KB of 1311KB)

Major RAM consumers:
- Ring buffer: 1024 * 20 bytes = ~20KB
- AsyncWebServer/WebSocket buffers: ~15KB
- WiFi stack: ~20KB

Flash has ample room for LittleFS data partition and future phases.
