# Track Geometry Car — PCB Design

Custom PCB to replace the breadboard/jumper wire prototype, sized to fit
inside an HO scale flat car (~55' prototype, 31mm between stake pockets).

## Status: v0.1 — Initial Design

This is the starting point. Expect changes before fabrication.

## Board Dimensions

```
Length:  76mm  (~55' HO scale with clearance margin)
Width:   31mm  (between stake pockets)
Layers:  2     (F.Cu + B.Cu)
Thickness: 1.6mm (standard)
```

## Component Placement (A-end → B-end)

```
A END                                                    B END
┌──────────────────────────────────────────────────────────────┐
│ USB-C │▓▓▓ANTENNA▓▓│   ESP32-WROOM-32   │OLED│    │VREG│TRK│
│       │▓▓KEEPOUT▓▓▓│                     │HDR │    │    │PWR│
│       │            │      [IMU1]         │    │    │    │   │
│  ○ A-truck pin                    [INA]  │    │  ○ B-truck  │
│       │            │                     │    │[IMU2] │RECT│
│       │            │                     │ SD │1000uF│    │   │
│       │            │                     │CARD│      │    │   │
└──────────────────────────────────────────────────────────────┘
         ←11mm→                              ←11mm→
                    ←———— 54mm truck spacing ————→
```

## Key Layout Decisions

1. **ESP32 antenna at A-end** — Keepout zone (no copper on either layer)
   under the antenna section. WiFi signal radiates toward the open end.

2. **IMU1 at board center** — Best location for whole-car geometry
   measurement. Centered between trucks.

3. **IMU2 over B-truck** — Differential measurement between truck and
   mid-car for twist/warp detection.

4. **Power supply at B-end** — Regulator, rectifier, and bulk cap are
   clustered away from the ESP32 antenna and IMUs (EMI).

5. **USB-C at A-end edge** — Accessible for programming without
   disassembly. Edge-mounted.

6. **OLED on header** — 4-pin 2.54mm header along top edge. The OLED
   module sits on top of the PCB, facing up.

7. **SD card on bottom edge** — Card inserts from the side.

8. **Truck pin holes** — 2mm non-plated through-holes at bolster centers
   for king pin mounting.

## Schematic — Block Diagram

```
                    ┌──────────────┐
  Track ──→ DB107 ──→ 1000µF ──→ AMS1117-3.3 ──→ 3V3 rail
  (DCC)    bridge    keep-alive     LDO
            rect.     capacitor

                         3V3 rail
                           │
            ┌──────────────┼──────────────┐
            │              │              │
     ┌──────┴──────┐  ┌───┴───┐  ┌───────┴───────┐
     │ ESP32-WROOM │  │ OLED  │  │ MPU-6050 x2   │
     │    -32      │  │0.96"  │  │ 0x68 + 0x69   │
     │             │  │ I2C   │  │ I2C, 100Hz    │
     │ GPIO21=SDA ─┼──┤ SDA   │──┤ SDA           │
     │ GPIO22=SCL ─┼──┤ SCL   │──┤ SCL           │
     │             │  └───────┘  └───────────────┘
     │ GPIO18=SCK ─┼──→ SD Card (SPI)
     │ GPIO19=MISO─┼──→   "
     │ GPIO23=MOSI─┼──→   "        ┌─────────┐
     │ GPIO5=CS   ─┼──→   "        │ INA219  │
     │             │               │ 0x40    │
     │ GPIO3=RXD  ─┼──→ USB-C D-  │ I2C     │──→ Track V/I
     │ GPIO1=TXD  ─┼──→ USB-C D+  └─────────┘
     └─────────────┘
```

## I2C Bus

All I2C devices share one bus (GPIO21 SDA, GPIO22 SCL, 400kHz):

| Device          | Address | Pull-ups  |
|-----------------|---------|-----------|
| MPU-6050 #1     | 0x68    | 4.7kΩ shared |
| MPU-6050 #2     | 0x69    | (same bus)   |
| SSD1315 OLED    | 0x3C    | (same bus)   |
| INA219          | 0x40    | (same bus)   |

## Bill of Materials (BOM)

| Ref | Component | Package | Qty | Notes |
|-----|-----------|---------|-----|-------|
| U2  | ESP32-WROOM-32 | Module | 1 | WiFi MCU |
| U3  | MPU-6050 | QFN-24 4x4mm | 1 | Primary IMU, AD0=GND |
| U4  | MPU-6050 | QFN-24 4x4mm | 1 | Secondary IMU, AD0=3V3 |
| U5  | INA219 | MSOP-8 | 1 | Track power monitor |
| U1  | AMS1117-3.3 | SOT-223 | 1 | 3.3V LDO regulator |
| D1  | DB107 | DIP-4 / ABS | 1 | Bridge rectifier |
| J1  | Track power | 2-pin header | 1 | DCC input |
| J2  | OLED header | 4-pin 2.54mm | 1 | GND/VCC/SCL/SDA |
| J3  | USB-C receptacle | 16P SMD | 1 | Programming + power |
| J4  | Micro SD socket | SMD | 1 | Extended logging |
| C1  | 1000µF/25V | Electrolytic 10x10.5mm | 1 | Keep-alive |
| C2  | 10µF | 0805 ceramic | 1 | Regulator input |
| C3  | 22µF | 0805 ceramic | 1 | Regulator output |
| C4-C8 | 100nF | 0402 ceramic | 5 | Bypass caps |
| R1-R2 | 10kΩ | 0402 | 2 | EN + GPIO0 pull-ups |
| R3-R4 | 4.7kΩ | 0402 | 2 | I2C pull-ups |
| R5  | 0.1Ω | 2512 | 1 | INA219 shunt |
| R6  | 5.1kΩ | 0402 | 1 | USB-C CC pull-down |

## Opening in KiCad

```bash
open /Volumes/Files/claude/track_geometry_car/pcb/track_geometry_car.kicad_pro
```

Or from KiCad: File → Open Project → navigate to `pcb/track_geometry_car.kicad_pro`

## What Needs Work

- [ ] Wire up actual nets in schematic (currently uses global labels, needs wires)
- [ ] Import netlist into PCB editor
- [ ] Place actual component footprints (currently just placement guides)
- [ ] Route traces
- [ ] Add ground pour on both layers
- [ ] Run DRC (Design Rule Check)
- [ ] Verify antenna keepout is correct for ESP32-WROOM-32 datasheet
- [ ] Add fiducial marks for JLCPCB assembly
- [ ] Generate Gerber files for fabrication
- [ ] Confirm truck pin hole diameter matches actual HO truck king pins

## Fabrication Target

- **JLCPCB** 2-layer, 1.6mm, HASL finish
- **Assembly**: JLCPCB SMT service for QFN and small passives
- **Cost estimate**: ~$25-40 for 5 assembled boards
