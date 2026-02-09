# Future Plans & Ideas

Collected ideas for future phases, hardware upgrades, and car design.
Items here are aspirational — not all will be implemented.

## Hardware Upgrades

### Phase 4b: Micro-SD Card Logging
- **Status**: Parts on order (HiLetgo micro-SD SPI module)
- **Wiring**: VSPI — MISO=GPIO19, MOSI=GPIO23, SCK=GPIO18, CS=GPIO5
- **Library**: SdFat (supports FAT32 + exFAT, can format cards on-device)
- **Capacity**: Effectively unlimited (64GB card = ~411 days at 1.8 KB/s)
- **Software approach**: Swap LittleFS calls in `flash_logger.cpp` for SD equivalents.
  Same API, same binary format, same HTTP endpoints. Web UI shows "SD" instead of "Flash".
- **Format on first use**: Detect unreadable card → offer "Format SD" button in web UI.
  SdFat can format any card to FAT32 regardless of size.
- **Card access**: Slot in roof of 3D printed car body for hot-swap without disassembly.

### Dual IMU (MPU-6050 x2)
- **Purpose**: Differential measurement enables true track geometry extraction
  - Twist/warp: difference in roll between front and rear
  - Curvature: difference in yaw rate
  - Speed estimation: cross-correlate bump signatures between IMUs,
    time delay ÷ known baseline distance = velocity
  - Separates car body dynamics from actual track geometry
- **Wiring**: Both on same I2C bus (GPIO 21/22)
  - IMU #1 at front truck bolster: AD0=LOW → address 0x68
  - IMU #2 at rear truck bolster: AD0=HIGH → address 0x69
  - Both mounted to car body (not truck frame), as low as possible
- **Baseline distance**: ~170mm on a 62' HO car (truck center to truck center)
- **Firmware changes**: Read both IMUs in the 100Hz timer callback,
  expand `imu_sample_t` to include both sensor sets. Bump survey version to 3.
- **I2C bandwidth**: 14 bytes × 2 sensors × 100Hz = 2800 bytes/s at 400kHz —
  well within budget (400kHz I2C can sustain ~40KB/s)
- **Dashboard**: Add second set of traces (fA.x, fA.y, etc. for front;
  rA.x, rA.y for rear) or computed differential channels

### ESP32-CAM Wheelset Camera
- **Purpose**: Visual correlation of wheel/rail interaction with IMU data
- **Approach**: Standalone ESP32-CAM module (OV2640, 2MP, ~$5)
  - Own WiFi station connecting to GeometryCar AP, or own AP on different channel
  - Streams MJPEG at 10-15fps via built-in web server example
  - Independent of geometry car firmware — no code changes needed
- **Mounting**: Wide-angle (120°+) lens looking straight down at front truck
  from ~15mm above. Covers full HO gauge width (~16mm) plus rail heads.
  Watch end axle, not center axle (center has lateral play for curves).
- **Lighting**: 2x white SMD LEDs flanking camera, angled into wheel well
- **Dashboard integration**: Embed as `<img>` tag or PiP window in existing
  dashboard, pointing to ESP32-CAM's stream URL
- **Future**: Frame-by-frame correlation with IMU timestamps in Phase 6
  analysis script. "At this IMU spike, here's what the wheel was doing."

### INA219 Power Monitor (Phase 3)
- **Status**: Hardware arriving
- **Purpose**: Track power quality — voltage drops, dirty track detection
- **Address**: 0x40 on same I2C bus
- **Correlates with**: IMU data for identifying electrical dead spots,
  dirty rail segments, turnout frog gaps

## Car Design

### Prototype: PRR F43 Depressed-Center Flat Car
- **Why this car**:
  - 62' long → ~170mm truck-to-truck baseline at HO, good for differential IMU
  - Low depressed center well → IMUs mount low near rail plane
  - Two 6-wheel Buckeye trucks → good tracking, faithful geometry transmission
  - Plausible MOW car — looks right in a work train with instrumentation on deck
  - PRR drawing available for reference dimensions
- **3D printed body** — custom designed, not a kit bash
  - Depressed well floor: IMU mounting pockets/standoffs
  - Raised end platforms: ESP32, SD card module, OLED, electronics stack vertical
  - Roof slot over SD card for hot-swap access
  - Snap-fit or screw-down roof for wiring access
  - Kadee #5 compatible coupler pockets
- **Avoid**: Multi-truck articulated cars (Schnabel, centipede flats) —
  too many wheels averaging out track input, acts as mechanical low-pass filter

### Truck Design
- **6-wheel trucks** (Buckeye pattern for PRR F43)
- **3D printed in resin** — stiffness is a tunable variable:
  - Hard/rigid resin (standard tough): maximum sensitivity, IMU sees everything
  - Flexible resin (Siraya Tenacious, Resione F80): simulates real suspension,
    filters high-frequency content, shows what rolling stock actually experiences
  - Blended ratios: tunable stiffness between extremes
- **Experimental value**: Run same track section with different truck resin
  hardnesses, compare frequency response in analysis script. Rigid trucks
  show more high-freq content (joints, frogs); flexible trucks emphasize
  longer wavelength geometry (curves, grades, dips).
- **Potential NMRA clinic material**: "How truck compliance filters track input"

### Sensor Placement (Dual IMU Configuration)
```
  [OLED + ESP32 + SD]          [Camera + LEDs]
  ┌─────────────────┐          ┌─────────────────┐
  │  Raised End #1   │          │  Raised End #2   │
  │                  │          │                  │
  ├──────────────────┤          ├──────────────────┤
  │     IMU #2       │          │     IMU #1       │
  │    (0x69)        │ Low Well │    (0x68)        │
  │  Near rear truck │──────────│ Near front truck │
  ├──────────────────┤          ├──────────────────┤
  │  ══╤══╤══╤══     │          │  ══╤══╤══╤══     │
  │  6-wheel truck   │          │  6-wheel truck   │
  └──────────────────┘          └──────────────────┘
       Rear                          Front
                                (direction of travel)
```

## Software / Analysis

### Phase 6: Python Analysis Script
- Parse binary survey files (64-byte header + 18-byte samples)
- Time-series plots of all channels
- FFT / spectral analysis to identify resonant frequencies
- Dual-IMU differential: compute twist, curvature, estimated speed
- Cross-correlation for speed-from-bump-delay
- Distance-domain conversion (using estimated speed)
- Overlay multiple runs for before/after track work comparison
- Heat map: color-coded track quality along the layout
- Export to CSV for external tools

### Dashboard Enhancements
- Computed channels (twist, curvature) when dual IMU is present
- Camera PiP window when ESP32-CAM is detected on network
- Track power overlay when INA219 is active
- Run comparison mode: overlay current run against a saved baseline
