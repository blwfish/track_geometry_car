#!/usr/bin/env python3
"""
Generate synthetic survey files for testing analyze_survey.py.

Creates a binary .bin file matching the firmware's flash logger format,
simulating a survey run with straights, curves, grades, and defects.
Also creates a matching browser JSON file for cross-format verification.

The simulated layout (all distances in model mm at HO scale):
  0-5000mm     Straight, clean track
  5000-7000mm  Left curve, 22" radius, with one kink at 6000mm
  7000-9000mm  Straight, one rail joint at 8000mm
  ~9000mm      Sag vertical curve (grade entry)
  9000-10000mm 2% ascending grade
  ~10000mm     Sharp crest vertical curve (grade exit)
  10000-12000mm Straight, turnout frog at 11000mm
  ~11500mm     Clearance spike (obstruction on right side)
  12000-14000mm Right curve, 18" radius, with spiral easements
  14000-16000mm Straight, rough section (elevated noise)
"""

import json
import struct
import sys
from pathlib import Path

import numpy as np

# Constants matching firmware
SURVEY_HEADER_SIZE = 64
SURVEY_SAMPLE_SIZE = 18
ACCEL_LSB_PER_G = 16384.0
GYRO_LSB_PER_DPS = 131.0
TEMP_LSB_PER_DEG = 340.0
TEMP_OFFSET_DEG = 36.53

SAMPLE_RATE = 100  # Hz
TOTAL_DISTANCE_MM = 16000
SPEED_SCALE_MPH = 20
SCALE = 87.1  # HO

# Convert speed to model mm/ms
SPEED_MM_PER_S = (SPEED_SCALE_MPH * 447.04) / SCALE
SPEED_MM_PER_MS = SPEED_MM_PER_S / 1000

TOTAL_TIME_MS = int(TOTAL_DISTANCE_MM / SPEED_MM_PER_MS)
N_SAMPLES = int(TOTAL_TIME_MS / (1000 / SAMPLE_RATE))


def make_header() -> bytes:
    """Create a 64-byte binary header matching survey_header_t."""
    buf = bytearray(64)
    buf[0:4] = b"GEOM"
    buf[4] = 1                          # version
    buf[5] = SURVEY_SAMPLE_SIZE         # sample_size
    struct.pack_into("<H", buf, 6, SAMPLE_RATE)  # sample_rate_hz
    buf[8] = 2                          # accel_range_g
    buf[9] = 250                        # gyro_range_dps (fits in uint8)
    struct.pack_into("<I", buf, 12, 0)  # start_time_ms
    car_id = b"GeometryCar\x00"
    buf[16:16 + len(car_id)] = car_id
    return bytes(buf)


def position_to_time_ms(pos_mm):
    """Convert position in mm to time in ms."""
    return pos_mm / SPEED_MM_PER_MS


def position_to_sample_idx(pos_mm):
    """Convert position in mm to sample index."""
    return int(position_to_time_ms(pos_mm) * SAMPLE_RATE / 1000)


def generate_samples():
    """Generate synthetic IMU data with known features."""
    rng = np.random.default_rng(42)

    timestamps = np.arange(N_SAMPLES) * (1000 // SAMPLE_RATE)  # ms
    positions = timestamps * SPEED_MM_PER_MS

    # Base signals: gravity on Z, small noise everywhere
    noise_floor = 0.003  # g RMS for "clean" track
    accel_x = rng.normal(0, noise_floor, N_SAMPLES)      # longitudinal
    accel_y = rng.normal(0, noise_floor, N_SAMPLES)      # lateral
    accel_z = 1.0 + rng.normal(0, noise_floor, N_SAMPLES)  # vertical (1g gravity)
    gyro_x = rng.normal(0, 0.3, N_SAMPLES)               # roll
    gyro_y = rng.normal(0, 0.3, N_SAMPLES)               # pitch
    gyro_z = rng.normal(0, 0.3, N_SAMPLES)               # yaw

    temperature = 25.0 + rng.normal(0, 0.1, N_SAMPLES)   # ~25°C

    # --- Layout features ---

    # 1. Left curve at 5000-7000mm (22" radius = 558.8mm)
    # Yaw rate for curve: ω = v/r in rad/s, convert to °/s
    radius_mm = 22.0 * 25.4  # 558.8mm
    yaw_rate_dps = np.degrees(SPEED_MM_PER_S / radius_mm)  # ~10.5 °/s
    s1, s2 = position_to_sample_idx(5000), position_to_sample_idx(7000)
    gyro_z[s1:s2] += yaw_rate_dps
    accel_y[s1:s2] += 0.02  # slight lateral in curve

    # Kink in curve at 6000mm — brief yaw spike
    kink_idx = position_to_sample_idx(6000)
    gyro_z[kink_idx:kink_idx + 3] += 8.0
    accel_y[kink_idx:kink_idx + 3] += 0.12

    # 2. Rail joint at 8000mm — sharp vertical spike
    joint_idx = position_to_sample_idx(8000)
    accel_z[joint_idx] += 0.25
    accel_z[joint_idx + 1] -= 0.15

    # 3. Grade 2% ascending at 9000-10000mm with vertical curves at transitions
    # 2% grade → sin(θ) ≈ 0.02g on X axis
    s1, s2 = position_to_sample_idx(9000), position_to_sample_idx(10000)
    accel_x[s1:s2] += 0.02

    # 3a. Sag (bottom of grade) at 9000mm — gradual pitch transition
    # Transition over ~200ms (20 samples). Pitch rate = grade_change / time
    # Going from 0% to 2%: pitch changes by arcsin(0.02) ≈ 1.15°
    # Over 200ms → ~5.7°/s pitch rate
    sag_idx = position_to_sample_idx(9000)
    sag_len = 20  # 200ms transition
    sag_half = sag_len // 2
    # Smooth ramp up then down (half-sine envelope)
    sag_profile = np.sin(np.linspace(0, np.pi, sag_len))
    peak_pitch_sag = 5.7  # °/s
    gyro_y[sag_idx - sag_half:sag_idx - sag_half + sag_len] += peak_pitch_sag * sag_profile

    # 3b. Sharp crest (top of grade) at 10000mm — abrupt pitch transition
    # Sharper than the sag: over ~100ms (10 samples), more severe
    # Going from 2% to 0%: pitch rate ~11.5°/s (twice as sharp)
    crest_idx = position_to_sample_idx(10000)
    crest_len = 10  # 100ms transition — sharper!
    crest_half = crest_len // 2
    crest_profile = np.sin(np.linspace(0, np.pi, crest_len))
    peak_pitch_crest = -11.5  # °/s (negative = nose dropping = crest)
    gyro_y[crest_idx - crest_half:crest_idx - crest_half + crest_len] += peak_pitch_crest * crest_profile

    # 4. Turnout frog at 11000mm — vertical + lateral spike
    frog_idx = position_to_sample_idx(11000)
    accel_z[frog_idx] += 0.30
    accel_z[frog_idx + 1] -= 0.20
    accel_y[frog_idx] += 0.15
    gyro_z[frog_idx:frog_idx + 3] += 3.0

    # 5. Right curve at 12000-14000mm (18" radius) WITH spiral easements
    radius_mm2 = 18.0 * 25.4  # 457.2mm
    yaw_rate_dps2 = np.degrees(SPEED_MM_PER_S / radius_mm2)
    s1, s2 = position_to_sample_idx(12000), position_to_sample_idx(14000)
    # Instead of abrupt step, create entry and exit spirals
    curve_samples = s2 - s1
    easement_samples = curve_samples // 5  # ~20% of curve = spiral at each end
    body_start = s1 + easement_samples
    body_end = s2 - easement_samples
    # Entry spiral: linear ramp from 0 to full yaw rate
    entry_ramp = np.linspace(0, 1, easement_samples)
    gyro_z[s1:body_start] -= yaw_rate_dps2 * entry_ramp
    # Full curve body
    gyro_z[body_start:body_end] -= yaw_rate_dps2
    # Exit spiral: linear ramp from full back to 0
    exit_ramp = np.linspace(1, 0, easement_samples)
    gyro_z[body_end:s2] -= yaw_rate_dps2 * exit_ramp
    # Lateral accel follows the same pattern
    accel_y[s1:body_start] -= 0.03 * entry_ramp
    accel_y[body_start:body_end] -= 0.03
    accel_y[body_end:s2] -= 0.03 * exit_ramp

    # 5a. Clearance spike at 11500mm — tunnel portal brush on right side
    # Single sharp lateral impulse, one direction only, no vertical/roll/yaw
    clearance_idx = position_to_sample_idx(11500)
    accel_y[clearance_idx] -= 0.18      # pushed left (obstruction on right)
    accel_y[clearance_idx + 1] -= 0.08  # very short impulse

    # 6. Rough section at 14000-16000mm — elevated noise
    s1, s2 = position_to_sample_idx(14000), position_to_sample_idx(16000)
    accel_z[s1:s2] += rng.normal(0, 0.04, s2 - s1)
    accel_y[s1:s2] += rng.normal(0, 0.03, s2 - s1)
    gyro_x[s1:s2] += rng.normal(0, 2.0, s2 - s1)

    # Add a twist fault at 15000mm
    twist_idx = position_to_sample_idx(15000)
    twist_len = 30  # 300ms at 100Hz
    gyro_x[twist_idx:twist_idx + twist_len] += 7.0

    return timestamps, accel_x, accel_y, accel_z, gyro_x, gyro_y, gyro_z, temperature


def write_binary(filepath: Path, timestamps, ax, ay, az, gx, gy, gz, temp):
    """Write a binary .bin file matching flash_logger format."""
    header = make_header()

    # Convert physical units back to raw int16
    raw_ax = np.clip(ax * ACCEL_LSB_PER_G, -32768, 32767).astype(np.int16)
    raw_ay = np.clip(ay * ACCEL_LSB_PER_G, -32768, 32767).astype(np.int16)
    raw_az = np.clip(az * ACCEL_LSB_PER_G, -32768, 32767).astype(np.int16)
    raw_gx = np.clip(gx * GYRO_LSB_PER_DPS, -32768, 32767).astype(np.int16)
    raw_gy = np.clip(gy * GYRO_LSB_PER_DPS, -32768, 32767).astype(np.int16)
    raw_gz = np.clip(gz * GYRO_LSB_PER_DPS, -32768, 32767).astype(np.int16)
    raw_temp = np.clip((temp - TEMP_OFFSET_DEG) * TEMP_LSB_PER_DEG, -32768, 32767).astype(np.int16)

    with open(filepath, "wb") as f:
        f.write(header)
        for i in range(len(timestamps)):
            f.write(struct.pack("<I", int(timestamps[i])))
            f.write(struct.pack("<h", raw_ax[i]))
            f.write(struct.pack("<h", raw_ay[i]))
            f.write(struct.pack("<h", raw_az[i]))
            f.write(struct.pack("<h", raw_gx[i]))
            f.write(struct.pack("<h", raw_gy[i]))
            f.write(struct.pack("<h", raw_gz[i]))
            f.write(struct.pack("<h", raw_temp[i]))

    n = len(timestamps)
    file_size = SURVEY_HEADER_SIZE + n * SURVEY_SAMPLE_SIZE
    print(f"Wrote {filepath} ({n} samples, {file_size} bytes)")


def write_json(filepath: Path, timestamps, ax, ay, az, gx, gy, gz, temp):
    """Write a browser-format JSON file."""
    samples = []
    for i in range(len(timestamps)):
        samples.append({
            "t": int(timestamps[i]),
            "ax": round(float(ax[i]), 6),
            "ay": round(float(ay[i]), 6),
            "az": round(float(az[i]), 6),
            "gx": round(float(gx[i]), 4),
            "gy": round(float(gy[i]), 4),
            "gz": round(float(gz[i]), 4),
            "temp": round(float(temp[i]), 1),
        })

    survey = {
        "version": 1,
        "car": "GeometryCar",
        "start_time": "2026-02-09T12:00:00.000Z",
        "duration_sec": float(timestamps[-1]) / 1000,
        "sample_rate_hz": SAMPLE_RATE,
        "accel_range_g": 2,
        "gyro_range_dps": 250,
        "samples": samples,
    }

    with open(filepath, "w") as f:
        json.dump(survey, f)

    print(f"Wrote {filepath} ({len(samples)} samples, {filepath.stat().st_size} bytes)")


def main():
    output_dir = Path(__file__).parent / "test_data"
    output_dir.mkdir(exist_ok=True)

    print("Generating synthetic survey data...")
    print(f"  Speed: {SPEED_SCALE_MPH} scale mph = {SPEED_MM_PER_S:.1f} mm/s model")
    print(f"  Distance: {TOTAL_DISTANCE_MM} mm ({TOTAL_DISTANCE_MM / 304.8:.1f} ft)")
    print(f"  Duration: {TOTAL_TIME_MS / 1000:.1f}s = {N_SAMPLES} samples")
    print()

    timestamps, ax, ay, az, gx, gy, gz, temp = generate_samples()

    write_binary(output_dir / "test_survey.bin", timestamps, ax, ay, az, gx, gy, gz, temp)
    write_json(output_dir / "test_survey.json", timestamps, ax, ay, az, gx, gy, gz, temp)

    print()
    print("Test data features:")
    print("  0-5000mm     Clean straight")
    print("  5000-7000mm  Left curve 22\" (abrupt entry/exit), kink at 6000mm")
    print("  7000-9000mm  Straight, rail joint at 8000mm")
    print("  9000mm       Sag vertical curve (grade entry, ~5.7°/s pitch)")
    print("  9000-10000mm 2% ascending grade")
    print("  10000mm      Sharp crest vertical curve (grade exit, ~11.5°/s pitch)")
    print("  10000-12000mm Straight, turnout frog at 11000mm")
    print("  11500mm      Clearance spike (obstruction on right side)")
    print("  12000-14000mm Right curve 18\" (with spiral easements)")
    print("  14000-16000mm Rough straight, twist at 15000mm")


if __name__ == "__main__":
    main()
