# Track Geometry Car

## Overview

A self-contained instrumented car that surveys the track as it is towed or pushed around the layout, measuring vertical profile, lateral alignment, track cant, and twist using a 6-axis IMU (accelerometer + gyroscope). The car streams live data over WiFi to any browser (phone, tablet, or desktop), and the browser captures the full survey for post-processing into a track quality map.

This is the track-side complement to the [Locomotive Speed Calibration System](SPEED_CALIBRATION_SPEC.md) — that spec gets locomotives ready, this one gets the *track* ready.

**What it finds:**
- **Rail joints and kinks** — vertical spikes at specific locations
- **Low spots and dips** — sine-wave depressions in vertical profile
- **Rough track / bad ballast** — elevated vertical noise floor over a section
- **Gauge and alignment problems** — lateral acceleration anomalies
- **Twist (cross-level error)** — one rail higher than the other; the most common cause of derailments
- **Turnout frog impacts** — characteristic vertical + lateral bump signature
- **Grade changes** — sustained longitudinal tilt
- **Curvature profile** — radius measurement from yaw rate, kinks within curves, inconsistent easements
- **Tight spots** — sections where drag increases (dirty track, tight curves, binding)
- **Missing feeders** — voltage sag in sections far from feeder drops
- **Dead frogs** — power dropouts at insulated frog turnouts
- **Bad rail joiners** — step changes in voltage at joint locations
- **Undersized bus wire** — gradual voltage droop across long runs

## Design Principles

- Completely self-contained — no layout WiFi or MQTT dependency
- Browser-as-display — live streaming via WiFi + WebSocket to any browser (phone, tablet, desktop)
- Zero configuration — place on track, open browser, go
- Tiny hardware — must fit inside a standard HO rolling stock shell
- Passive survey — car is towed at constant speed, no active locomotion
- Data logging — browser captures full survey; on-board flash provides backup
- Repeatable — run the same route twice, get comparable data for before/after verification

---

## Hardware

### IMU Sensor

**MPU-6050** (GY-521 breakout or bare chip):

| Parameter | Value |
|-----------|-------|
| Axes | 3-axis accelerometer + 3-axis gyroscope |
| Interface | I2C (address 0x68 default) |
| Accelerometer range | ±2g (best for track geometry: 16,384 LSB/g) |
| Gyro range | ±250°/s (best for model railroad speeds: 131 LSB/°/s) |
| Sample rate | 100 Hz (configurable via SMPLRT_DIV register) |
| Power | ~3.9 mA (all 6 axes active) |
| GY-521 breakout size | 20 × 16 × 3 mm |
| Bare chip size | 4 × 4 × 0.9 mm |

The ±2g accelerometer range provides maximum sensitivity. Model railroad track defects produce accelerations well under 1g — even a severe kink at speed is unlikely to exceed ±2g. The ±250°/s gyro range is similarly appropriate; the car isn't spinning, just gently rocking.

The MPU-6050 also includes a built-in temperature sensor, useful for compensating gyro drift on longer surveys.

### Microcontroller

**ESP32-C3 Super Mini:**

| Parameter | Value |
|-----------|-------|
| Dimensions | 22.5 × 18 × ~4 mm |
| CPU | RISC-V single core, 160 MHz |
| RAM | 400 KB SRAM |
| Flash | 4 MB |
| WiFi | 802.11 b/g/n, SoftAP mode supported |
| I2C | GPIO 8 (SDA), GPIO 9 (SCL) |
| USB | USB-C (native, for charging/programming) |
| GPIO | 11 available |

The ESP32-C3 is chosen over the full ESP32 for its much smaller physical size. The single RISC-V core at 160 MHz is more than sufficient for 100 Hz I2C reads, FFT/statistics, and WebSocket streaming. The full ESP32 DevKit (~55 × 28 mm) simply won't fit in HO rolling stock.

**Why not ESP32-S3?** The S3 is physically larger in most module formats and has capabilities (dual core, USB OTG, vector instructions) that aren't needed here. The C3 is the smallest, cheapest, and most power-efficient option that still has WiFi.

### Power Supply

Most model railroad layouts run DCC, which provides 14-18V AC on the rails at all times. This is more than enough to power the geometry car indefinitely — no battery to charge, no runtime limit, no power switch needed.

**Primary: DCC Track Power**

A full-wave bridge rectifier converts DCC track voltage to DC, then a small buck or linear regulator steps it down to 3.3V for the ESP32-C3.

| Component | Purpose | Size | Cost |
|-----------|---------|------|------|
| DB107 bridge rectifier | DCC AC → DC (~12-16V DC) | 4 × 4 mm | $0.05 |
| AMS1117-3.3 linear regulator | 12-16V DC → 3.3V | SOT-223 (6.5 × 3.5 mm) | $0.10 |
| 1000µF 25V electrolytic cap | Keep-alive through dead spots | 10 × 13 mm | $0.15 |
| 10µF ceramic cap | Output filter | 0805 | $0.02 |

**Keep-alive capacitor:** A 1000µF capacitor on the rectified DC rail (before the regulator) stores enough energy to ride through dirty track, insulated frogs, and short dead spots. At 100 mA draw from ~14V, a 1000µF cap provides roughly 100ms of hold-up time — enough to coast across a standard HO turnout frog (~25mm ≈ 250ms at 20 scale mph, but both trucks provide pickup so at least one set of wheels is usually live). The layouts that need this car the most tend to be the ones with the worst electrical, so this is essential rather than optional.

**Why linear regulator, not buck converter?** At ~100 mA draw, the AMS1117 dissipates ~1.2W from 16V input. That's warm but within spec (the SOT-223 package handles it fine). A buck converter would be more efficient but physically larger and more complex. For 100 mA, the linear regulator wins on simplicity and size.

**Pickup:** Wheels pick up DCC power through the existing truck wipers — no modification to the car's trucks beyond ensuring clean contact. Both trucks should be wired (one rail per truck) for reliable pickup through turnouts and dirty spots.

**Power budget:**

| Component | Current |
|-----------|---------|
| ESP32-C3 (CPU + WiFi active) | ~80 mA |
| MPU-6050 (all axes) | ~4 mA |
| SSD1306 OLED (active) | ~10 mA |
| INA219 power monitor | ~1 mA |
| Regulator overhead | ~5 mA |
| **Total** | **~100 mA** |

With DCC track power, runtime is unlimited. The car runs as long as it's on powered track.

### Track Power Monitor

**INA219 current/voltage sensor (I2C, address 0x40):**

| Parameter | Value |
|-----------|-------|
| Interface | I2C (shares bus with MPU-6050 and OLED) |
| Voltage measurement | 0–26V DC, 4 mV resolution |
| Current measurement | ±3.2A, 0.8 mA resolution (with 0.1Ω shunt) |
| Additional GPIO required | **None** — shares I2C bus |
| Power | ~1 mA |
| Breakout size | 25 × 12 mm |
| Cost | ~$1 |

The INA219 sits on the rectified DC rail between the bridge rectifier and the regulator, measuring both bus voltage and current draw. It shares the I2C bus with the MPU-6050 (0x68) and SSD1306 (0x3C) — three devices, one bus, zero extra pins.

**What it measures:**

- **Track voltage (DC after rectifier):** Healthy DCC is 14-16V DC. Drops below ~12V indicate inadequate feeders, excessive bus wire resistance, or too many devices drawing power on that district. A voltage profile across the layout is a direct map of electrical infrastructure quality.
- **Voltage drops at specific locations:** A sudden dip at a particular spot on the layout means the feeder distribution is weak there. Correlate with position data and you know exactly where to add feeders.
- **Dead spots:** Voltage dropping to zero (rescued by the keep-alive cap) reveals insulated frogs, dirty rail, and gaps in the feeder network. The cap keeps the car running, but the INA219 records the dropout.
- **Current draw:** The car's own ~100 mA draw is near-constant, so current variations primarily reflect contact resistance. High-resistance joints or dirty wheels show up as current fluctuations.

**Sampling:** Read the INA219 at 10 Hz (every 100ms) alongside the IMU summary data. The INA219 is fast enough for this — its default conversion time is 532µs. The I2C read takes ~0.5ms, interleaved between the 10ms IMU reads with no contention.

**Firmware addition:**

```c
// Track power monitor config (added to summary_1s_t)
float track_voltage_mean;     // Average rectified DCC voltage
float track_voltage_min;      // Minimum voltage (worst dropout this second)
float track_current_mean;     // Average current draw (mA)
uint8_t power_dropouts;       // Count of voltage dips below threshold this second
```

**Alternative: LiPo Battery (for DC layouts)**

On DC layouts, track voltage varies with throttle setting and may be zero when the operator isn't running a train. A small LiPo battery provides independent power:

| Battery | Dimensions | Capacity | Est. Runtime |
|---------|-----------|----------|-------------|
| 401030 | 4 × 10 × 30 mm | 100 mAh | ~1 hour |
| 501230 | 5 × 12 × 30 mm | 150 mAh | ~1.5 hours |
| 601248 | 6 × 12 × 48 mm | 280 mAh | ~2.5 hours |

With battery, add a TP4056 charger module ($0.30) and a small slide switch for power on/off. Charge via USB-C when the car body is removed.

**Note:** On a DCC layout, the car powers up the instant it touches the rails and shuts down when lifted off. No switch needed — track power IS the power switch.

### Physical Packaging

The car should be a piece of rolling stock that blends in on the layout — not a science experiment on wheels. Good candidates:

- **40' flatcar** — open deck provides easy access; hide electronics under a removable load (lumber, crates, tarp)
- **Boxcar** — fully enclosed, protects electronics, but harder to access
- **Caboose / cabin car** — sits at end of train, has windows for a status LED to be visible

**Mounting considerations:**
- MPU-6050 should be mounted rigidly to the car frame, centered between the trucks, oriented with X along the track, Y lateral, Z vertical
- Both trucks wired for power pickup (one rail each) — solder to existing truck wipers
- Bridge rectifier and regulator mounted on small perfboard with ESP32-C3 and IMU
- USB-C port accessible by removing the car body for programming
- No wires hanging out — everything internal
- For DC/battery variant: add slide switch (SPST) accessible from outside, secure battery with foam tape

### Car Quality

The survey car itself must have **good trucks and wheels** — free-rolling, no flat spots, clean bearings. Any defect in the car's own running gear appears in the data as track defects. Metal wheelsets with free-spinning axles are preferred over plastic.

Consider running the car on a known-good section of track first to establish a baseline vibration signature for the car itself. This "self-noise floor" can be subtracted from survey data.

### On-board Display

**SSD1306 OLED, 0.91" or 0.96", 128×32 or 128×64, I2C:**

| Parameter | Value |
|-----------|-------|
| Size (0.91") | 12 × 38 mm |
| Size (0.96") | 12 × 35 mm (128×64) |
| Interface | I2C (address 0x3C) |
| Power | ~10 mA |
| Additional GPIO required | **None** — shares I2C bus with MPU-6050 |

The OLED makes the car self-documenting and usable without a phone connected. It shares the I2C bus with the MPU-6050 and INA219 (three devices, three addresses: IMU at 0x68, OLED at 0x3C, power monitor at 0x40), so zero additional pins are needed.

**Display modes:**

*Startup / idle (AP mode):*
```
┌──────────────────┐
│AP: GeometryCar   │
│Pass: trackwork   │
│→ 192.168.4.1     │
│Ready             │
└──────────────────┘
```

*Startup / idle (station mode):*
```
┌──────────────────┐
│WiFi: LayoutNet ✓ │
│geometrycar.local │
│→ 192.168.1.47    │
│Ready             │
└──────────────────┘
```

*Recording / live:*
```
┌──────────────────┐
│●REC  3:42 14.6V  │
│▁▂▃▅▇▅▃▂▁▂▃▁▂▁▂▃ │
│⚠ DIP  0.18g     │
│R=22" grade 1.2%  │
└──────────────────┘
```

The top line shows recording status, elapsed time, and live track voltage. The sparkline on the second line is a scrolling mini-chart of vertical acceleration — glanceable roughness indicator. The warning line flashes briefly when an event exceeds threshold (including voltage dropouts). Curve radius and grade update in real time.

**Mounting:** For a flatcar, the OLED faces up as part of the visible "load" on the deck. For a boxcar or caboose, position it behind a window opening.

### A/B End Orientation

The car has a defined A end and B end, marked with a small sticker or paint dot on the underframe. The MPU-6050 is mounted with its X axis aligned to the track centerline, with **positive X pointing toward the A end**.

This establishes a consistent sign convention:
- **Positive yaw (gyro Z):** Car is curving to the **left** when traveling A-end first
- **Negative yaw:** Curving to the **right** when traveling A-end first
- **Positive lateral accel (Y):** Force toward the **right** side of the car
- **Positive pitch (gyro Y):** Car is pitching **nose up** (ascending grade, A-end first)

**Auto-detection:** If the user forgets which end is which, the firmware can auto-detect orientation on the first curve encountered — the sign of the yaw rate combined with the lateral acceleration direction resolves the ambiguity. The web UI and OLED show the detected orientation.

**Why this matters:** When comparing two surveys, or when publishing results for others to analyze ("Dave's curves vs. Hal's curves"), the data needs a consistent frame of reference. Left vs. right curves, ascending vs. descending grades, and the direction of twist all depend on knowing which way the car was traveling. The A/B convention makes every survey directly comparable.

---

## Firmware

### Architecture

```
┌─────────────────────────────────────────────┐
│  ESP32-C3 Super Mini                        │
│                                             │
│  ┌──────────┐  ┌──────────────┐             │
│  │ MPU-6050 │──│ I2C Bus      │             │
│  │ (100 Hz) │  │ (shared)     │             │
│  └──────────┘  └──┬───────┬───┘             │
│  ┌──────────┐     │       │                 │
│  │ SSD1306  │─────┘  ┌────┴─────┐           │
│  │ OLED     │        │ INA219   │           │
│  └──────────┘        │ V+I mon  │           │
│                      └──────────┘           │
│                ┌──────────────┐              │
│                │ Sample Buffer│              │
│                │ (ring buffer)│              │
│                └──┬────────┬──┘              │
│                   │        │                 │
│         ┌─────────┴┐ ┌────┴─────────┐       │
│         │ WebSocket│ │ Flash Logger │       │
│         │ Streamer │ │ (LittleFS)   │       │
│         └──────────┘ └──────────────┘       │
│                                              │
│  ┌──────────────────┐                        │
│  │ WiFi AP/Station  │                        │
│  │ AsyncWebServer   │                        │
│  │ mDNS + WebSocket │                        │
│  └──────────────────┘                        │
│                                              │
│  ┌──────────────────┐                        │
│  │ DCC Track Power  │                        │
│  │ Bridge + LDO     │                        │
│  │ 1000µF keep-alive│                        │
│  └──────────────────┘                        │
└──────────────────────────────────────────────┘
```

### Startup Sequence

1. Power on (car placed on DCC-powered track) → initialize I2C, MPU-6050, INA219
2. Configure MPU-6050: ±2g accel, ±250°/s gyro, 100 Hz sample rate, DLPF ~20 Hz
3. Check boot button: if held, force AP mode; otherwise read WiFi mode from NVS
4. Start WiFi in configured mode:
   - **AP mode:** SoftAP `GeometryCar` / `trackwork`; IP 192.168.4.1
   - **Station mode:** Connect to stored SSID; start mDNS as `geometrycar.local`
   - If station mode fails to connect within 10 seconds, fall back to AP mode
5. Display WiFi mode, SSID, and IP address on OLED
6. Start AsyncWebServer on port 80
7. Serve web UI from LittleFS
8. Begin sampling IMU at 100 Hz via timer interrupt, INA219 at 10 Hz
9. Wait for WebSocket client connection or recording start command

### IMU Sampling

**Timer-driven at 100 Hz:**
- Hardware timer fires every 10 ms
- ISR sets a flag; main loop reads MPU-6050 via I2C (I2C is not ISR-safe)
- Each read takes ~1 ms at 400 kHz I2C clock (14 bytes: 6 accel + 2 temp + 6 gyro)
- Store in ring buffer with timestamp (`millis()`)

**Sample data structure:**

```c
struct imu_sample_t {
    uint32_t timestamp_ms;    // millis() at sample time
    int16_t accel_x;          // Raw accelerometer values
    int16_t accel_y;          // (divide by 16384 for g's at ±2g range)
    int16_t accel_z;
    int16_t gyro_x;           // Raw gyroscope values
    int16_t gyro_y;           // (divide by 131 for °/s at ±250°/s range)
    int16_t gyro_z;
    int16_t temperature;      // Raw temp (for drift compensation)
};  // 16 bytes per sample
```

**Ring buffer:** 1024 samples = 16 KB. At 100 Hz this holds 10.2 seconds, giving plenty of margin for WebSocket transmission jitter.

### Data Processing (On-chip)

**Real-time (per-sample):**
- Apply calibration offsets (stored in flash, set during initial calibration)
- Convert raw values to physical units (g, °/s)
- Compute running statistics: mean, RMS, peak for each axis over 1-second windows

**Per-second summary (streamed to browser):**

```c
struct summary_1s_t {
    uint32_t timestamp_ms;
    float accel_z_rms;        // Vertical roughness indicator
    float accel_y_rms;        // Lateral roughness indicator
    float accel_x_mean;       // Longitudinal grade indicator
    float gyro_x_rms;         // Roll (twist) indicator
    float gyro_y_rms;         // Pitch rate
    float gyro_z_mean;        // Yaw rate — mean indicates steady curve
    float gyro_z_rms;         // Yaw rate — RMS indicates curvature variation
    float accel_z_peak;       // Worst vertical event in this second
    float accel_y_peak;       // Worst lateral event in this second
    float curve_radius_mm;    // Computed: speed / yaw_rate (0 = straight)
    float grade_percent;      // Computed: from pitch angle
    float temperature;        // For drift tracking
    float track_voltage_mean; // Rectified DCC voltage (average this second)
    float track_voltage_min;  // Worst voltage dip this second
    float track_current_mean; // Current draw in mA (contact quality indicator)
    uint8_t power_dropouts;   // Voltage dips below threshold this second
};
```

This 1-second summary is what gets streamed to the browser for the live display. The full 100 Hz raw data is logged to flash for detailed post-processing.

**Curvature computation (on-chip):**

When gyro Z (yaw rate) exceeds a threshold indicating the car is in a curve, compute the radius in real time:

```
radius_mm = speed_mm_s / (gyro_z_rad_s)
          = speed_mm_s / (gyro_z_deg_s × π / 180)
```

At 20 scale mph in HO (~100 mm/s), typical model railroad curves produce:

| Curve Radius | Yaw Rate |
|-------------|----------|
| 457 mm (18") | 12.5°/s |
| 559 mm (22") | 10.2°/s |
| 686 mm (27") | 8.4°/s |
| 914 mm (36") | 6.3°/s |

All well within the MPU-6050's resolution at ±250°/s. The lateral acceleration in the curve provides an independent cross-check: `a_lateral = v²/r`. When the yaw-derived and accel-derived radii agree, the measurement is reliable.

What's most useful isn't the radius per se — it's **curvature consistency**. A properly laid curve has smooth, constant yaw rate. Spikes or steps in yaw rate within a curve indicate kinks — places where the rail bends sharply instead of following a smooth arc. These are derailment sources that are very hard to see by eye but obvious in the gyro trace.

**Event detection (on-chip):**
- Vertical spike > threshold → flag as "joint/kink"
- Lateral spike > threshold → flag as "alignment defect"
- Roll rate sustained > threshold → flag as "twist/cross-level"
- Yaw rate spike within a steady curve → flag as "curve kink"
- Track voltage < threshold → flag as "voltage drop" (dead spot, dirty rail, missing feeder)
- Track voltage = 0 (cap sustaining) → flag as "power dropout"
- These events trigger a brief highlight in the live browser display and flash on the OLED

### Flash Logging

**Format:** Binary, written to LittleFS.

Each survey run creates a file: `/survey_YYYYMMDD_HHMMSS.bin`

File structure:
```
[Header: 64 bytes]
  magic: "GEOM"
  version: 2
  sample_rate_hz: 100
  accel_range: 2  (±2g)
  gyro_range: 250 (±250°/s)
  start_time: (millis)
  car_id: (string, 16 chars)

[Samples: 20 bytes each]
  imu_sample_t records + electrical data, sequential
    uint32_t timestamp_ms
    int16_t  accel_x, accel_y, accel_z
    int16_t  gyro_x, gyro_y, gyro_z
    int16_t  temperature
    uint16_t track_voltage_mv    // Rectified DCC voltage in millivolts
    uint16_t track_current_ma    // Current draw in milliamps
```

**Storage capacity:** The ESP32-C3 has 4 MB flash. After firmware (~1.5 MB) and web UI assets (~200 KB), approximately 2 MB is available for data. At 20 bytes/sample × 100 Hz = 2.0 KB/s = 120 KB/min:
- 2 MB holds ~17 minutes of continuous recording
- Sufficient for most home layouts in a single pass
- For larger layouts, the browser captures the WebSocket stream directly to the phone/tablet/laptop — no flash limit
- The on-board flash log serves as a backup in case the browser disconnects mid-survey

**Two recording paths (simultaneous):**
1. **Browser capture (primary):** The web UI JavaScript accumulates the raw WebSocket stream in memory and offers a "Save Survey" download when stopped. No storage limit beyond the browser device's memory/disk. Works identically in AP mode (phone) and station mode (desktop).
2. **Flash log (backup):** The car simultaneously writes to LittleFS. If the browser disconnects or crashes mid-survey, the on-board log preserves the data up to the 17-minute flash limit. Downloadable via HTTP GET from the web UI's file manager.

Post-processing: transfer the saved `.bin` file to a PC (directly, via cloud storage, AirDrop, etc.), then run `analyze_survey.py` to produce the JSON report and visualizations.

---

## WiFi and Web Interface

### WiFi Modes

The car supports two WiFi modes, selectable via the web UI settings page (stored in NVS) or by holding the boot button during power-up:

**Mode 1: Standalone AP (default)**

```c
WiFi.softAP("GeometryCar", "trackwork");
// IP: 192.168.4.1
// Channel: 1 (default)
// Max connections: 2
```

The car creates its own network — no dependency on layout WiFi, home router, or internet connectivity. Walk up, connect phone, open browser. Works in a basement, a garage, a club layout, anywhere. Best for: club visits, quick surveys, lending to a friend.

**Mode 2: Station (join layout WiFi)**

```c
WiFi.begin(ssid, password);  // Credentials stored in NVS
MDNS.begin("geometrycar");   // Discoverable as geometrycar.local
```

The car joins the layout's existing WiFi network and announces itself via mDNS. Any computer on the network can connect to `http://geometrycar.local`. Best for: large layouts, long surveys, real-time analysis on a full-sized screen.

**Why station mode matters for large layouts:** The car's on-board flash holds ~17 minutes of data. A large layout that takes 30-45 minutes per lap exceeds that. In station mode, any browser on the layout network connects to `http://geometrycar.local` — same web UI, same WebSocket stream, same everything. The browser captures the data to the device it's running on (phone, tablet, laptop), with no storage limit. A 45-minute survey at ~2 KB/s is only ~5.4 MB.

The browser is the universal client in both modes — the only difference is how the browser reaches the car (direct AP vs. layout network). No special software needed on any device.

**Station mode benefits:**
- **Unlimited recording length** — the browser saves the stream locally; no flash constraint
- **Big-screen dashboard** — strip charts on a desktop monitor, multiple traces at full resolution
- **mDNS discovery** — no need to find the car's IP address; `geometrycar.local` just works
- **NTP time** — with internet access, the car can get real time for survey filenames
- **Easy file transfer** — saved survey files on a phone can go to iCloud, Google Drive, etc. for later analysis on a PC

**Mode switching:**
- Default: AP mode (works out of the box, no configuration needed)
- Configure station mode via the settings page when connected in AP mode (enter SSID + password)
- Hold boot button during power-up to force AP mode (recovery if station mode credentials are wrong)
- OLED shows the active mode and IP address on startup

### Web UI

A single-page application served from LittleFS. The browser (phone in AP mode, or desktop in station mode) gets:

**Live Dashboard:**
- Scrolling strip chart showing all 6 axes (or selectable subset)
- Vertical (Z) acceleration is the primary trace — shows bumps and dips
- Lateral (Y) acceleration is the secondary trace — shows alignment issues
- Roll (gyro X) shows twist
- Yaw (gyro Z) shows curvature — steady deflection = curve, spike = kink
- Computed curve radius displayed as numeric readout when in a curve
- Computed grade percentage displayed when on a slope
- Color-coded event markers when thresholds are exceeded
- Current RMS values as numeric readouts (updated 1/sec)
- Recording status indicator (recording / idle / flash full)

**Controls:**
- **Start Recording** — begins logging to flash AND capturing the WebSocket stream in the browser
- **Stop Recording** — ends logging, shows file size and duration, offers "Save Survey" to download the browser-captured data
- **Saved Surveys** — lists on-board flash logs; download any previous survey as backup
- **Calibrate** — places car on known-level track, captures zero offsets
- **Settings** — threshold adjustments, sample rate, car ID, WiFi mode (AP/station), station SSID + password

**Implementation:**
- HTML + JavaScript served from LittleFS (~50-100 KB total)
- WebSocket connection for real-time data push
- Chart rendering via HTML5 Canvas (no heavy libraries needed for strip charts)
- Binary WebSocket frames for efficiency (20 bytes/sample or ~60-byte summaries)

### WebSocket Protocol

**Server → Client (real-time streaming):**

At 100 Hz, the server sends binary frames containing batches of samples. To reduce WebSocket overhead, batch 10 samples per frame (10 Hz frame rate, 200 bytes/frame):

```
Frame type 0x01: Raw samples (batch)
  [1 byte: frame type]
  [1 byte: sample count]
  [N × 20 bytes: sample records (IMU + electrical)]
```

At 1-second intervals, send a summary frame:

```
Frame type 0x02: 1-second summary
  [1 byte: frame type]
  [36 bytes: summary_1s_t]
```

**Client → Server (commands):**

```
Frame type 0x10: Start recording
Frame type 0x11: Stop recording
Frame type 0x12: Request calibration
Frame type 0x13: Download survey file (returns HTTP URL)
```

### Data Rates

| Stream | Rate | Notes |
|--------|------|-------|
| Raw samples | ~2.0 KB/s | 100 Hz × 20 bytes, batched |
| Summary | ~60 bytes/s | 1 Hz × ~60 bytes (incl. electrical) |
| Total WiFi throughput | ~2.1 KB/s | Trivial for 802.11n |

For comparison, the ESP32-C3 can sustain several hundred KB/s over WiFi. The geometry car's data rate is about 1% of capacity. Latency will be imperceptible.

---

## Calibration

### Zero-Offset Calibration

Before first use (and periodically thereafter), the car must be calibrated to remove sensor offsets:

1. Place the car on a known-flat, known-level section of track
2. Press "Calibrate" in the web UI
3. The firmware samples 500 readings (~5 seconds) and averages them
4. The expected values on level track are: accel_x=0, accel_y=0, accel_z=+1g, gyro_x/y/z=0
5. Offsets are computed and stored in flash (NVS)
6. Applied to all subsequent readings

**Stored calibration:**
```c
struct calibration_t {
    int16_t accel_offset[3];   // Raw offset to subtract
    int16_t gyro_offset[3];    // Raw offset to subtract
    float temperature_ref;     // Temperature at calibration time
    uint32_t timestamp;        // When calibration was performed
};
```

### Temperature Compensation

The MPU-6050 gyroscope drifts with temperature (~±20°/s across the full temperature range, but much less over a few degrees). For a survey that takes 5-10 minutes in a stable indoor environment, drift is minimal. However, if the car was just powered on (heating from sleep), the first minute of data may show some drift.

Mitigation: record temperature at each sample; post-processing can apply a linear drift correction if needed.

### Track Speed Reference

The car has no odometer — it doesn't know its own speed or position along the track. It records data vs. time, not vs. distance. To map defects to specific locations:

**Option 1: Known constant speed**
Tow the car at a known, constant speed (e.g., using a calibrated locomotive from the speed calibration system). Speed × time = distance.

**Option 2: RFID waypoints**
If the layout has RDM6300 RFID readers at known locations (from the loco identification project), the car can carry an RFID tag. Each reader detection timestamps the car's passage at a known point, allowing the survey to be position-registered.

**Option 3: Manual markers**
Press a button in the browser at known landmarks (turnouts, crossings, block boundaries). These insert event markers into the data stream for later alignment.

**Option 4: Turnout detection**
Turnout frogs produce a very distinctive signature (sharp lateral + vertical impulse). These can be automatically detected in the data and matched to the layout's turnout map for automatic position registration.

---

## Post-Processing

### Survey Analysis Script (`analyze_survey.py`)

A Python script that reads the binary log file and produces a track quality report.

```
Usage: analyze_survey.py <survey_file.bin> [options]

Options:
  --speed <mph>        Assumed constant scale speed (default: 20)
  --scale <factor>     Scale factor (default: 87.1 for HO)
  --output <dir>       Output directory for reports (default: ./report)
  --markers <file>     Optional marker file for position registration
  --threshold-vert <g> Vertical event threshold (default: 0.15g)
  --threshold-lat <g>  Lateral event threshold (default: 0.10g)
  --threshold-twist <°/s> Twist rate threshold (default: 5.0)
```

### Output Report

**Track quality summary:**
```json
{
  "survey_id": "20260206_143000",
  "duration_sec": 480,
  "distance_estimated_mm": 96000,
  "overall_grade": "B",
  "sections": [
    {
      "start_mm": 0,
      "end_mm": 5000,
      "type": "straight",
      "vertical_rms_g": 0.02,
      "lateral_rms_g": 0.01,
      "twist_rms_deg_s": 0.8,
      "track_voltage_mean": 14.8,
      "track_voltage_min": 14.2,
      "power_dropouts": 0,
      "grade": "A",
      "events": []
    },
    {
      "start_mm": 5000,
      "end_mm": 10000,
      "type": "curve",
      "curve_radius_mm": 559,
      "curve_radius_inches": 22.0,
      "curvature_consistency": 0.92,
      "vertical_rms_g": 0.08,
      "lateral_rms_g": 0.04,
      "twist_rms_deg_s": 3.2,
      "track_voltage_mean": 13.1,
      "track_voltage_min": 10.8,
      "power_dropouts": 1,
      "grade_percent": 0.0,
      "grade": "C",
      "events": [
        {
          "type": "curve_kink",
          "position_mm": 7200,
          "severity": "moderate",
          "yaw_rate_deviation_deg_s": 4.5,
          "description": "Sharp curvature change within curve — kink in rail"
        },
        {
          "type": "twist",
          "position_mm": 8500,
          "severity": "minor",
          "roll_rate_peak_deg_s": 4.1,
          "description": "Cross-level variation over ~50mm"
        }
      ]
    }
  ],
  "curves": [
    {
      "start_mm": 5000,
      "end_mm": 10000,
      "direction": "left",
      "avg_radius_mm": 559,
      "min_radius_mm": 480,
      "max_radius_mm": 610,
      "consistency": 0.92,
      "has_easement": false,
      "kinks": 1,
      "note": "22\" curve, moderately consistent, one kink detected at 7200mm"
    }
  ],
  "grades": [
    {
      "start_mm": 15000,
      "end_mm": 22000,
      "avg_grade_percent": 2.1,
      "max_grade_percent": 2.8,
      "direction": "ascending",
      "note": "2.1% grade over 7000mm (8\" rise)"
    }
  ],
  "electrical": {
    "voltage_mean": 14.2,
    "voltage_min": 10.8,
    "voltage_max": 15.1,
    "total_dropouts": 3,
    "dropout_locations_mm": [8200, 23500, 41000],
    "weak_spots": [
      {
        "start_mm": 8000,
        "end_mm": 8500,
        "voltage_min": 10.8,
        "probable_cause": "Insulated frog — no feeder on frog rails",
        "severity": "moderate"
      },
      {
        "start_mm": 40000,
        "end_mm": 42000,
        "voltage_min": 11.5,
        "probable_cause": "Long feeder run — voltage drop under load",
        "severity": "minor"
      }
    ],
    "electrical_grade": "B",
    "note": "3 weak spots detected; worst at turnout frog (8200mm)"
  },
  "event_summary": {
    "total_events": 16,
    "joints_kinks": 4,
    "curve_kinks": 1,
    "alignment_defects": 2,
    "twist_faults": 3,
    "turnout_impacts": 3,
    "voltage_drops": 2,
    "power_dropouts": 1
  },
  "grading": {
    "A": "Excellent — smooth, well-maintained track",
    "B": "Good — minor imperfections, unlikely to cause issues",
    "C": "Fair — noticeable roughness, may cause issues with sensitive equipment",
    "D": "Poor — significant defects, derailment risk",
    "F": "Failing — immediate attention required"
  }
}
```

**Visualizations (generated by script):**
- **Strip chart:** Full-survey plot of Z-accel (vertical) and Y-accel (lateral) vs. estimated position, with event markers
- **Curvature plot:** Yaw rate (or computed radius) vs. position — shows where curves are, how consistent they are, and where kinks occur
- **Grade profile:** Pitch angle (or grade %) vs. position — shows the elevation profile of the layout
- **Voltage profile:** Track voltage vs. position — instantly shows where feeders are weak, where frogs cause dropouts, and where bus resistance is too high
- **Heat map:** Track plan overlay (if available) color-coded by section grade
- **Histogram:** Distribution of vertical/lateral RMS values across all sections
- **Before/after comparison:** Overlay two surveys to show improvement after track work

### Defect Classification

The analysis script classifies events by their signature shape:

| Defect | Vertical (Z) | Lateral (Y) | Roll (gyro X) | Yaw (gyro Z) | Duration |
|--------|-------------|-------------|----------------|---------------|----------|
| **Rail joint** | Sharp spike (±) | Minimal | Minimal | Minimal | <50ms |
| **Kink (straight)** | Sharp spike | May have lateral | Brief roll | Brief yaw spike | <50ms |
| **Kink (in curve)** | Minimal | Lateral spike | Brief roll | Spike in steady yaw | <50ms |
| **Low spot / dip** | Smooth sine | Minimal | Minimal | Minimal | 100-500ms |
| **Rough ballast** | Elevated noise | Elevated noise | Elevated noise | Elevated noise | Sustained |
| **Gauge problem** | Minimal | Wobble | Roll wobble | Minimal | Sustained |
| **Twist (cross-level)** | Minimal | Minimal | Sustained roll rate | Minimal | 200ms+ |
| **Turnout frog** | Sharp spike | Sharp lateral spike | Brief roll | Brief yaw step | <100ms |
| **Smooth curve** | Minimal | Sustained lateral G | Sustained roll (cant) | Steady yaw rate | Sustained |
| **Uneven curve** | Minimal | Varying lateral | Varying roll | Varying yaw rate | Sustained |
| **Flat spot in curve** | Minimal | Lateral step | Roll step | Yaw drops to ~0 | 100-500ms |
| **Curve entry/exit** | Minimal | Ramp in lateral | Ramp in roll | Ramp in yaw | Transition |
| **Grade change** | Sustained pitch | Minimal | Minimal | Minimal | Transition |

Electrical defects (from INA219):

| Defect | Voltage | Current | Typical Cause | Duration |
|--------|---------|---------|---------------|----------|
| **Missing feeder** | Gradual sag (>2V below nominal) | Normal | Long rail run from nearest feeder | Sustained |
| **Insulated frog** | Sharp dropout (may hit 0V) | Drops to 0 | Unfed frog rails on power-routing turnout | 100-500ms |
| **Dirty rail** | Noisy/intermittent voltage | Erratic | Oxidation, contamination, poor wheel contact | Variable |
| **Bad rail joiner** | Step change in voltage | Step in current | Corroded or loose rail joiner adding resistance | At joint |
| **Bus wire resistance** | Steady sag across section | Normal | Undersized bus wire or long branch run | Sustained |

---

## Benchmarking and Comparison

### What Makes Good Track

The geometry car produces objective, quantitative data about track quality. Over time, this creates a vocabulary for discussing trackwork that goes beyond "it runs nice":

| Metric | Excellent | Good | Fair | Poor |
|--------|-----------|------|------|------|
| Vertical RMS (straight) | <0.01g | 0.01–0.03g | 0.03–0.08g | >0.08g |
| Lateral RMS (straight) | <0.005g | 0.005–0.02g | 0.02–0.05g | >0.05g |
| Curvature consistency | >0.95 | 0.90–0.95 | 0.80–0.90 | <0.80 |
| Max twist rate | <1°/s | 1–3°/s | 3–6°/s | >6°/s |
| Turnout frog impact | <0.10g | 0.10–0.20g | 0.20–0.35g | >0.35g |
| Voltage stability (min/mean) | >0.95 | 0.90–0.95 | 0.80–0.90 | <0.80 |
| Power dropouts per lap | 0 | 1–2 | 3–5 | >5 |

*Note: These thresholds are initial estimates. Real-world calibration will refine them. Part of the value of the project is establishing these norms empirically.*

### Comparing Layouts

Because the output format is standardized JSON with consistent units and a defined A/B end convention, surveys from different layouts are directly comparable. This enables:

- **"What does Dave do differently?"** — compare curve consistency, grade compensation, and easement profiles between two layouts side by side
- **Best practices from data** — "layouts with curvature consistency >0.93 and easements report zero curve-related derailments"
- **Club layout maintenance** — track quality surveys before and after work sessions, with quantified improvement
- **Design validation** — does the new helix actually hold the 2% grade you designed, or does it sag to 2.8% in the middle?
- **Build quality over time** — survey during construction, catch problems before scenery makes them unfixable

### Sharing Results

The JSON output and binary survey files are portable. A community could maintain a shared database of anonymized track quality data to establish norms across many layouts, scales, and construction techniques. The ~$8 electronics cost makes it feasible for any club to build one.

---

## Bill of Materials

| Qty | Part | Purpose | Est. Cost |
|-----|------|---------|-----------|
| 1 | ESP32-C3 Super Mini | Controller + WiFi AP | $3 |
| 1 | MPU-6050 (GY-521 breakout) | 6-axis IMU | $1 |
| 1 | SSD1306 0.91" OLED (I2C) | On-board display | $2 |
| 1 | INA219 breakout (I2C) | Track voltage/current monitor | $1 |
| 1 | DB107 bridge rectifier | DCC AC → DC | $0.05 |
| 1 | AMS1117-3.3 regulator | 3.3V regulation | $0.10 |
| 1 | 1000µF 25V electrolytic cap | Keep-alive through dead spots | $0.15 |
| 1 | 10µF ceramic cap | Output filter | $0.02 |
| 1 | HO flatcar or boxcar | Rolling stock shell | $10-20 (or existing) |
| - | Wire, foam tape, heat shrink | Assembly | $1 |
| | | **Total (electronics only)** | **~$8** |
| | | **Total (with new car)** | **~$28** |
|  |  |  |  |
| **Optional (DC layout battery variant):** | | | |
| 1 | LiPo battery (601248 or similar) | Power (~280 mAh) | $3 |
| 1 | TP4056 LiPo charger module | USB charging | $0.30 |
| 1 | Slide switch (SPST) | Power on/off | $0.10 |

---

## Implementation Plan

### Phase 1: Hardware Build & Proof of Concept
- Assemble ESP32-C3 + MPU-6050 + INA219 on breadboard (no car shell yet)
- Build DCC power supply circuit (bridge rectifier + 1000µF cap + AMS1117)
- Implement I2C communication with MPU-6050 at 100 Hz and INA219 at 10 Hz
- Verify IMU and voltage/current data on serial monitor
- Push the breadboard along the track by hand, observe raw data
- **Estimated effort:** 3-4 hours

### Phase 2: WiFi & Live Streaming
- Implement WiFi SoftAP mode (default) and station mode (join layout WiFi)
- Implement mDNS advertisement (`geometrycar.local`) for station mode
- Implement mode switching: NVS-stored credentials, boot button override for AP mode
- Set up AsyncWebServer + WebSocket
- Serve minimal HTML page with live numeric readouts
- Stream 1-second summaries over WebSocket
- Test on phone browser (AP mode) and desktop browser (station mode)
- **Estimated effort:** 5-7 hours

### Phase 3: Web UI Dashboard & OLED Display
- Build strip chart display using HTML5 Canvas
- Implement scrolling multi-axis trace display
- Add curvature readout and grade display
- Add track voltage trace and dropout markers
- Add event detection with visual markers (including electrical events)
- Implement browser-side stream capture (accumulate WebSocket data in JavaScript, offer Save/download)
- Add recording start/stop controls (triggers both browser capture and flash logging)
- Add numeric RMS readouts
- Polish for phone screen sizes (responsive layout)
- Implement SSD1306 OLED driver on shared I2C bus
- Display WiFi credentials, recording status, sparkline, events, curve radius, grade, and voltage on OLED
- Implement A/B end auto-detection from first curve encountered
- **Estimated effort:** 9-12 hours

### Phase 4: Flash Logging & Download
- Implement binary logging to LittleFS as backup (simultaneous with browser capture)
- Add file management (list saved surveys, delete, download)
- Implement HTTP download endpoint for flash-stored survey files
- Test dual recording: browser capture + flash backup, verify both produce identical data
- **Estimated effort:** 3-4 hours

### Phase 5: Calibration & Packaging
- Implement zero-offset calibration routine
- Store calibration in NVS
- Install electronics in car shell
- Mount MPU-6050 rigidly, aligned to track axes
- Wire truck pickups through bridge rectifier and regulator
- Verify operation as complete rolling stock
- **Estimated effort:** 3-4 hours

### Phase 6: Analysis Script
- Write `analyze_survey.py` post-processing script
- Implement binary file reader (v2 format with electrical data)
- Implement section-by-section grading
- Implement event detection and classification (mechanical + electrical)
- Implement curvature computation (radius from yaw rate, consistency scoring)
- Implement grade computation (from pitch angle)
- Implement electrical analysis (voltage profile, dropout detection, weak spot identification)
- Generate strip chart, curvature plot, grade profile, and voltage profile visualizations
- Generate JSON report with curves, grades, and electrical arrays
- Test with real survey data from the layout
- **Estimated effort:** 9-12 hours

### Phase 7: Validation & Refinement
- Survey a known section of track with intentional defects
- Verify that the system detects: joints, low spots, twist, turnout impacts, curve kinks
- Adjust thresholds based on real-world data
- Run before/after comparison: fix a known defect, re-survey, verify improvement
- Document the car's self-noise floor on known-good track
- **Estimated effort:** 3-4 hours

**Total estimated effort:** 37-53 hours

---

## Success Criteria

- [ ] IMU samples reliably at 100 Hz with no dropped samples
- [ ] WiFi AP connects within 5 seconds; phone browser loads dashboard within 3 seconds
- [ ] Live strip chart updates smoothly at ≥10 fps in browser
- [ ] Recording captures ≥10 minutes of continuous data without flash overflow
- [ ] Survey file downloads to browser without corruption
- [ ] Zero-offset calibration reduces static noise to <0.01g RMS on level track
- [ ] System detects a deliberately placed rail joint or shimmed rail (known defect)
- [ ] Turnout frog passages produce a clearly identifiable signature in the data
- [ ] Curve radius measurement agrees with physical measurement (tape measure) within ±10%
- [ ] Curvature trace clearly distinguishes a smooth curve from one with a deliberate kink
- [ ] Grade measurement detects a known slope (e.g., shimmed track end) within ±0.5%
- [ ] Before/after surveys show measurable improvement when a known defect is corrected
- [ ] Station mode connects to layout WiFi and is reachable at `geometrycar.local` within 10 seconds
- [ ] Browser-captured survey of >17 minutes saved without data loss (exceeds on-board flash capacity)
- [ ] Boot button override reliably forces AP mode when station credentials are misconfigured
- [ ] Car powers up reliably within 2 seconds of being placed on DCC-powered track
- [ ] Keep-alive capacitor sustains operation across insulated frogs and short dead spots without reboot
- [ ] Track voltage measurement agrees with multimeter reading within ±0.5V
- [ ] Voltage profile clearly identifies a deliberately disconnected feeder section
- [ ] Power dropout events are logged with position when the car crosses a dead frog
- [ ] OLED displays WiFi credentials on power-up without browser connection
- [ ] OLED shows real-time event alerts visible while watching the car on the layout
- [ ] A/B end orientation detected automatically on first curve
- [ ] Electronics fit inside an HO rolling stock shell with no external wires

---

## Future Enhancements

- **RFID tag on the car** — automatic position registration when passing layout RFID readers
- **GPS-style position tracking** — dead reckoning from accelerometer integration (limited accuracy, but combined with RFID waypoints could be useful)
- **Automated turnout detection** — identify turnout passages by their unique vertical+lateral impulse signature and auto-register position against the layout's turnout map
- **Coupler force estimation** — sustained longitudinal acceleration indicates drag; correlate with track location to find dirty track or binding curves
- **Multi-car survey** — two geometry cars at different points in a train to measure track twist more precisely (differential roll between cars)
- **Historical trending** — re-survey monthly, track degradation over time, predict maintenance needs
- **Integration with layout CTC/dispatch** — publish defect locations to JMRI as "slow orders" on specific blocks
- **Sound analysis add-on** — add a MEMS mic to the car to capture wheel/rail noise, complementing the vibration data with an acoustic signature
- **N scale version** — ESP32-C3 Super Mini and bare MPU-6050 chip could fit in N scale (1:160) rolling stock with a custom PCB
