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

### Clearance Detector (Pivoting Arm)
- **Inspiration**: B&O CE15 clearance car at B&O Railroad Museum — a large
  spoked wheel/arm assembly on the roof that detects overhead obstructions
  by physical contact during a clearance survey run
- **Purpose**: Detect clearance infringements — both overhead (low bridges,
  tunnel portals, signal bridges, scenery) and underbody (raised spikes,
  track debris, roadbed intrusions that snag Kadee glad hands, brake rigging,
  uncoupling pins, or other underbody detail parts)
- **Overhead concept**: A thin vertical arm (brass wire or 0.015" piano wire)
  mounted on a pivot at the car roof, extending upward to the HO loading gauge
  limit (~22mm above rail head for NMRA S-7). Spring-return or gravity
  self-centering. When the arm contacts an obstruction, it deflects the pivot.
- **Underbody concept**: A lightweight wire skid or feeler hanging below the
  car between the trucks, set to the minimum underbody clearance height
  (just above railhead — matching the lowest hanging detail parts like Kadee
  glad hands at ~3mm above rail). Pivots upward on contact with an obstruction.
  Same hall sensor detection as overhead arms.
  - Simpler alternative: a thin spring-steel strip running longitudinally
    between the trucks at glad-hand height. Any upward deflection closes a
    contact or triggers a hall sensor. Acts like a continuous feeler rather
    than a point detector.
  - Can also detect ballast/debris piled too high between the rails (common
    after ballast work on the prototype, and on model railroads after
    scenicking)
- **Detection options** (in order of preference):
  1. **Hall effect sensor + magnet**: Tiny A3144 or SS49E hall sensor at the
     pivot base, small neodymium magnet on the arm. Deflection moves the magnet
     away from the sensor → digital or analog signal. No mechanical switch to
     wear out. ~$0.50.
  2. **Microswitch**: Omron D2F or similar subminiature lever switch at the
     pivot. Reliable, but has a minimum actuation force that may be too high
     for delicate scenery.
  3. **Optical interrupter**: Slotted photosensor at pivot, flag on arm.
     Clean digital signal, no contact wear, but bulkier.
- **Arm design considerations**:
  - Flexible enough to not damage scenery on contact
  - Stiff enough to return to rest position reliably
  - Overhead: multiple arms at different heights for an envelope profile
    (e.g., arms at 20mm, 21mm, 22mm = coarse clearance binning)
  - Overhead: or a single adjustable-height arm, run multiple passes
  - Underbody: single feeler at glad-hand height is sufficient — anything
    that snags a glad hand is a problem regardless of exact height
  - Lateral arms possible too (horizontal wires at gauge + clearance width)
    for side clearance of platforms, tunnel walls, signal masts
  - Full envelope: overhead + lateral + underbody = complete loading gauge
    survey in one pass
- **Firmware**:
  - One GPIO pin per arm (digital input, interrupt on FALLING edge)
  - Don't need to sample at 100Hz in `imu_sample_t` — contact events are
    sparse, so log them as timestamped events in a separate section of the
    survey file (event type + timestamp + which arm)
  - Or: add a single `uint8_t clearance_flags` bitmask to `imu_sample_t`
    (1 bit per arm, up to 8 arms) — costs only 1 extra byte per sample,
    simpler than a separate event log, and analysis script can extract edges
  - Debounce in firmware: 10-20ms ignore window after trigger (arm bounces)
- **Survey format**: If using the bitmask approach, bump `SURVEY_SAMPLE_SIZE`
  by 1 byte and add `num_clearance_arms` + `arm_height_mm[]` to the header's
  reserved bytes. If using event log, append events after the sample data
  with a separate index.
- **Analysis script**:
  - Plot clearance events on the position strip chart (vertical red lines)
  - Cross-reference with vertical curve and grade data — overhead clearance
    is tighter at sag vertical curves (car rides higher), underbody clearance
    is tighter at crest vertical curves (car droops between trucks)
  - Report: list of clearance infringements with position, which arm(s) hit,
    estimated clearance range based on which arms triggered and which didn't
  - Compare runs: did fixing that bridge abutment actually improve clearance?
- **Mounting on PRR F43**:
  - Overhead: arm assembly on the depressed well section (lowest point of car)
    gives the most conservative measurement. Looks prototypical as MOW
    clearance survey equipment. 3D print a small housing/bearing block that
    press-fits onto the car body.
  - Underbody: feeler strip mounts to underframe between trucks, inside the
    rail gauge. The F43's depressed center well has generous vertical space
    between the floor and railhead — feeler hangs from the well floor.
  - Both look appropriate on a MoW clearance/geometry car.
- **Calibration**: Set arm height with a gauge block on a known-good section
  of track. Mark the arm at the NMRA loading gauge height. No electronic
  calibration needed — it's purely mechanical.

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

### Quantitative Change Detection (Run-to-Run Differencing)
- **Purpose**: Detect and quantify how track geometry changes over time —
  thermal expansion, humidity warping, ballast settlement, benchwork movement.
  Real railroads trend repeated geometry measurements to predict where
  maintenance is needed before it becomes a problem. Same concept at HO scale.
- **Spatial alignment** — the key problem:
  - Two runs of the same track won't have matching timestamps or positions
    (speed varies, start point differs). Must align spatially, not temporally.
  - **Landmark cross-correlation**: Rail joints, turnout frogs, and other sharp
    IMU signatures are fingerprints. Detect these in both runs, match them,
    then warp/stretch the position axis between matched landmarks to align.
    This is analogous to surveying "chain adjustment."
  - **Dual IMU advantage**: Speed-from-bump-correlation gives true velocity
    at every sample, so position estimates are much more accurate. Less
    warping correction needed, and the correction that remains is more reliable.
  - **Fiducial markers** (optional): Small magnets glued under the track at
    known positions, detected by a hall sensor on the car. Gives absolute
    position fixes. Cheap, invisible, and eliminates cumulative drift entirely.
    Could share the clearance detector's hall sensor with a second channel.
- **Differenced channels** — once aligned, subtract run B from run A:
  - **Lateral shift**: Change in mean yaw rate through a curve → Δradius →
    track moved laterally. Report in mm of lateral displacement.
  - **Vertical settlement**: Change in integrated pitch (grade profile) →
    track rose or sank. A dip that deepens 0.5mm between runs = ballast
    settling or benchwork sag.
  - **Twist development**: Change in roll rate → one rail rising relative to
    the other (cross-level change). Early indicator of ballast problems or
    roadbed moisture.
  - **Curve migration**: If a curve's centroid position shifts between runs,
    the whole curve slid along the roadbed (rail creep).
  - **Joint opening**: If a rail joint's IMU spike widens or shifts position,
    the rail gap is changing (thermal expansion/contraction).
- **Trend analysis** (3+ runs over time):
  - Per-section trend: "this curve has been tightening 0.1"/month since October"
  - Rate-of-change alerts: flag sections where degradation is accelerating
  - Seasonal patterns: correlate with temperature/humidity if logged
  - Maintenance effectiveness: quantify improvement from specific track work
- **Analysis script implementation** (`--diff` mode):
  - Input: two or more survey files of the same track, in chronological order
  - Step 1: Detect landmarks (joints, frogs, sharp events) in each run
  - Step 2: Match landmarks between runs using cross-correlation windows
  - Step 3: Warp position axes to align matched landmarks (piecewise linear)
  - Step 4: Interpolate both runs onto a common position grid
  - Step 5: Compute per-channel differences
  - Step 6: Report sections ranked by magnitude of change, type of change
    (lateral/vertical/twist), and direction of trend
  - Output: change report (text + JSON), difference strip chart, trend plots
    if >2 runs provided
- **Distinct from `--compare`**: The existing `--compare` flag overlays two
  runs visually on the same plots. `--diff` does quantitative subtraction
  with spatial alignment — it answers "what changed and by how much" rather
  than "here are both runs side by side."
- **Practical use cases**:
  - Seasonal survey: run in January (cold/dry) and July (hot/humid), diff
    shows where the layout moves with the seasons
  - Post-maintenance verification: survey before and after shimming/releveling,
    diff confirms the fix and quantifies improvement
  - Long-term monitoring: quarterly surveys build a trend database, identify
    problem areas before they cause derailments
  - New construction settling: frequent surveys of newly built sections to
    track initial settlement curve

### MQTT Throttle Control (Autonomous Survey Speed)
- **Purpose**: The geometry car commands its own locomotive to the optimal
  survey speed via MQTT → JMRI → DCC, ensuring consistent data quality
  without manual throttle management. The car becomes a self-driving
  survey vehicle — set the DCC address, hit start, walk away.
- **Why this matters for data quality**:
  - Curve radius accuracy depends on speed being known and appropriate.
    Too slow in broad curves = low SNR, noisy radius estimate. Too fast
    in tight curves = accel saturation. The car knows what it needs.
  - Constant speed produces evenly-spaced spatial samples. Human throttle
    control introduces speed variation that degrades position estimates.
  - With speed-from-bump-correlation (dual IMU), the car knows its actual
    speed and can correct toward the target — true closed-loop control.
  - Eliminates the biggest source of error: the assumed `--speed` value
    in the analysis script no longer needs to be a guess.
- **Infrastructure** (already in place):
  - JMRI running with MQTT connection to Mosquitto broker
  - JMRI throttle topics: `cab/{address}/throttle` (0-100 integer %),
    `cab/{address}/direction` ("FORWARD"/"REVERSE"/"STOP")
  - esp32-config pattern: broker address + prefix in NVRAM, `PubSubClient`
  - mqtt-logger captures all traffic for debugging
- **WiFi architecture**: ESP32 supports `WIFI_AP_STA` (simultaneous AP +
  Station). Phone connects to "GeometryCar" AP for dashboard, while ESP32
  connects to layout WiFi as a station for MQTT broker access.
  - Dashboard still works on the captive portal AP
  - MQTT traffic goes over the STA interface to the layout network
  - If layout WiFi is unavailable, AP-only mode still works (manual speed)
  - NVS stores: layout SSID, layout password, MQTT broker host/port,
    MQTT prefix (default: empty, matching JMRI defaults)
- **Firmware changes**:
  - Add `PubSubClient` (same library as esp32-config)
  - New module: `throttle.h/cpp` — MQTT connection, speed command publishing
  - NVS settings: DCC address, layout WiFi credentials, broker address,
    target survey speed (or "auto"), max speed limit
  - Publish to `cab/{address}/throttle` with integer 0-100
  - Publish to `cab/{address}/direction` with "FORWARD"
  - Subscribe to `cab/{address}/throttle` for feedback (confirm commanded
    speed was accepted)
- **Speed control logic** (runs at 1Hz, alongside summary computation):
  - **Manual mode**: User sets target speed in dashboard, car commands that
    speed step and holds it. The speed input in the Geometry readout group
    becomes both the display value and the commanded value.
  - **Auto mode**: Car adjusts speed based on measurement quality:
    - On straight track: target a minimum vertical accel RMS (need enough
      vibration for useful data). Default ~12 scale mph.
    - In curves: target yaw SNR ≥ 25. If SNR is low, increase speed.
      If accel_y (centripetal) exceeds 1.5g, decrease speed.
    - Rate limit: ±1 speed step per second (smooth acceleration, no jerking)
    - Max speed limit: configurable, default 20 scale mph
    - Stop on loss of MQTT connection (safety)
  - **With dual IMU**: Use measured speed (bump correlation) as feedback
    instead of assumed speed. True closed-loop: commanded speed → measured
    speed → error → correction. PID or simple proportional control.
- **Dashboard additions**:
  - Collapsible "Throttle" panel (like file manager):
    - DCC address input (number, 1-9999)
    - Layout WiFi SSID + password fields (stored in NVS)
    - MQTT broker host:port
    - Mode selector: Off / Manual / Auto
    - Target speed (manual mode) or speed range (auto mode)
    - Current commanded speed readout
    - MQTT connection status indicator
    - Emergency stop button (large, red, always visible when throttle active)
  - Speed input in Geometry group becomes read-only when throttle is active
    (car knows its own speed, no need to guess)
- **Safety**:
  - Emergency stop on: MQTT disconnect, WiFi disconnect, any firmware crash
    (watchdog), dashboard disconnect with no reconnect in 5 seconds
  - E-stop sends speed 0 + publishes "STOP" direction
  - Never exceed max speed limit regardless of auto-mode calculation
  - Dashboard E-stop button sends both MQTT stop and WebSocket stop command
  - On startup, throttle is always OFF — requires explicit enable
  - Speed ramp-up from zero on start (don't jump to target immediately)
- **Survey workflow with throttle control**:
  1. Place loco + geometry car on track
  2. Open dashboard, set DCC address, enable throttle (manual or auto)
  3. Hit "Start Recording"
  4. Car accelerates to survey speed, begins logging
  5. In auto mode: car adjusts speed through curves for optimal measurement
  6. User hits "Stop Recording" — car decelerates to stop
  7. Download survey file — speed data is embedded (no `--speed` guess needed)
- **Survey file enhancement**: When throttle is active, log commanded speed
  in each summary (1Hz). Analysis script can use actual commanded speed per
  section instead of a single `--speed` value for the entire run. If dual
  IMU is present, log measured speed too — comparison of commanded vs measured
  reveals locomotive speed table accuracy.
- **Integration with test-and-calibration-track**: The calibration track
  produces accurate speed tables for each locomotive. The geometry car can
  load a loco's speed table from NVS or MQTT and convert DCC speed steps
  to actual model speed for precise position calculation — even without
  the dual IMU bump-correlation method.

### Dashboard Enhancements
- Computed channels (twist, curvature) when dual IMU is present
- Camera PiP window when ESP32-CAM is detected on network
- Track power overlay when INA219 is active
- Run comparison mode: overlay current run against a saved baseline
