#!/usr/bin/env python3
"""
Track Geometry Car — Survey Analysis Script

Reads binary (.bin) or browser-captured JSON (.json) survey files from the
geometry car and produces a track quality report with visualizations.

Usage:
    python analyze_survey.py <survey_file> [options]

Options:
    --speed <mph>            Assumed constant scale speed (default: 20)
    --scale <factor>         Model scale factor (default: 87.1 for HO)
    --output <dir>           Output directory for reports (default: ./report)
    --threshold-vert <g>     Vertical event threshold (default: 0.15)
    --threshold-lat <g>      Lateral event threshold (default: 0.10)
    --threshold-twist <dps>  Twist rate threshold (default: 5.0)
    --section-len <mm>       Section length for grading (default: 5000)
    --no-plots               Skip generating plot images
    --compare <file>         Overlay a second survey for before/after comparison
"""

import argparse
import json
import struct
import sys
from dataclasses import dataclass, field
from pathlib import Path

import numpy as np

# ===== Constants matching firmware config.h =====

SURVEY_MAGIC = b"GEOM"
SURVEY_HEADER_SIZE = 64
SURVEY_SAMPLE_SIZE_V1 = 18  # bytes per sample, version 1 (single IMU)
SURVEY_SAMPLE_SIZE_V2 = 30  # bytes per sample, version 2 (dual IMU)

ACCEL_LSB_PER_G = 16384.0
GYRO_LSB_PER_DPS = 131.0
TEMP_LSB_PER_DEG = 340.0
TEMP_OFFSET_DEG = 36.53

# Default thresholds from spec (will be refined with real data)
DEFAULT_SPEED_MPH = 20
DEFAULT_SCALE = 87.1  # HO
DEFAULT_SECTION_LEN_MM = 1000

# Grading thresholds (from spec table)
GRADE_THRESHOLDS = {
    "vert_rms": [0.01, 0.03, 0.08],       # A/B, B/C, C/D boundaries (g)
    "lat_rms": [0.005, 0.02, 0.05],        # A/B, B/C, C/D boundaries (g)
    "twist_max": [1.0, 3.0, 6.0],          # A/B, B/C, C/D boundaries (°/s)
    "curvature_consistency": [0.95, 0.90, 0.80],  # A/B, B/C, C/D (higher=better)
}


# ===== Data Structures =====

@dataclass
class SurveyHeader:
    magic: str
    version: int
    sample_size: int
    sample_rate_hz: int
    accel_range_g: int
    gyro_range_dps: int
    start_time_ms: int
    car_id: str


@dataclass
class SurveyData:
    """Holds a fully parsed survey with physical-unit arrays."""
    header: SurveyHeader
    source_file: str
    timestamp_ms: np.ndarray   # uint32
    accel_x: np.ndarray        # float, in g
    accel_y: np.ndarray        # float, in g
    accel_z: np.ndarray        # float, in g
    gyro_x: np.ndarray         # float, in °/s
    gyro_y: np.ndarray         # float, in °/s
    gyro_z: np.ndarray         # float, in °/s
    temperature: np.ndarray    # float, in °C
    position_mm: np.ndarray = field(default=None)  # computed from speed × time
    # IMU #2 (None when single IMU)
    accel_x2: np.ndarray = field(default=None)
    accel_y2: np.ndarray = field(default=None)
    accel_z2: np.ndarray = field(default=None)
    gyro_x2: np.ndarray = field(default=None)
    gyro_y2: np.ndarray = field(default=None)
    gyro_z2: np.ndarray = field(default=None)

    @property
    def has_imu2(self) -> bool:
        return self.accel_x2 is not None


@dataclass
class Event:
    type: str
    position_mm: float
    time_ms: float
    severity: str  # "minor", "moderate", "severe"
    description: str
    details: dict = field(default_factory=dict)


@dataclass
class Section:
    start_mm: float
    end_mm: float
    type: str  # "straight", "curve", "grade"
    grade: str  # A-F
    vertical_rms_g: float
    lateral_rms_g: float
    twist_rms_dps: float
    events: list = field(default_factory=list)
    # Curve-specific
    curve_radius_mm: float = None
    curve_direction: str = None
    curvature_consistency: float = None


@dataclass
class TransitionInfo:
    """Describes entry or exit transition (easement/spiral) of a curve."""
    present: bool
    length_mm: float           # length of the transition zone
    rate_dps_per_mm: float     # rate of curvature change (°/s per mm)
    type: str                  # "abrupt", "short", "spiral"


@dataclass
class CurveInfo:
    start_mm: float
    end_mm: float
    direction: str
    avg_radius_mm: float
    min_radius_mm: float
    max_radius_mm: float
    consistency: float
    entry_transition: TransitionInfo
    exit_transition: TransitionInfo
    kinks: int
    note: str


@dataclass
class GradeInfo:
    start_mm: float
    end_mm: float
    avg_grade_percent: float
    max_grade_percent: float
    direction: str
    note: str


@dataclass
class VerticalCurveInfo:
    position_mm: float
    type: str              # "crest" or "sag"
    radius_mm: float       # vertical curve radius (smaller = sharper)
    peak_pitch_rate_dps: float
    grade_before_pct: float
    grade_after_pct: float
    severity: str          # "minor", "moderate", "severe"
    note: str


# ===== Parsing =====

def parse_binary(filepath: Path) -> SurveyData:
    """Parse a binary .bin survey file from flash logger."""
    data = filepath.read_bytes()

    if len(data) < SURVEY_HEADER_SIZE:
        raise ValueError(f"File too small ({len(data)} bytes, need {SURVEY_HEADER_SIZE} for header)")

    # Parse 64-byte header (packed struct matching survey_header_t)
    magic = data[0:4]
    if magic != SURVEY_MAGIC:
        raise ValueError(f"Bad magic: {magic!r} (expected {SURVEY_MAGIC!r})")

    version = data[4]
    sample_size = data[5]
    sample_rate_hz = struct.unpack_from("<H", data, 6)[0]
    accel_range_g = data[8]
    gyro_range_dps = data[9]
    # bytes 10-11: reserved
    start_time_ms = struct.unpack_from("<I", data, 12)[0]
    car_id = data[16:32].split(b"\x00")[0].decode("ascii", errors="replace")
    # bytes 32-63: reserved

    header = SurveyHeader(
        magic=magic.decode("ascii"),
        version=version,
        sample_size=sample_size,
        sample_rate_hz=sample_rate_hz,
        accel_range_g=accel_range_g,
        gyro_range_dps=gyro_range_dps,
        start_time_ms=start_time_ms,
        car_id=car_id,
    )

    # Determine sample size from header (v1=18, v2=30; also honor sample_size field)
    if sample_size > 0:
        actual_sample_size = sample_size
    elif version >= 2:
        actual_sample_size = SURVEY_SAMPLE_SIZE_V2
    else:
        actual_sample_size = SURVEY_SAMPLE_SIZE_V1

    dual_imu = actual_sample_size >= SURVEY_SAMPLE_SIZE_V2

    # Parse samples
    sample_data = data[SURVEY_HEADER_SIZE:]
    n_samples = len(sample_data) // actual_sample_size
    if n_samples == 0:
        raise ValueError("No samples in file")

    remainder = len(sample_data) % actual_sample_size
    if remainder != 0:
        print(f"Warning: {remainder} trailing bytes ignored", file=sys.stderr)

    timestamps = np.empty(n_samples, dtype=np.uint32)
    raw_accel = np.empty((n_samples, 3), dtype=np.int16)
    raw_gyro = np.empty((n_samples, 3), dtype=np.int16)
    raw_temp = np.empty(n_samples, dtype=np.int16)

    if dual_imu:
        raw_accel2 = np.empty((n_samples, 3), dtype=np.int16)
        raw_gyro2 = np.empty((n_samples, 3), dtype=np.int16)
        # V2 layout: timestamp(4) + accel(6) + gyro(6) + temp(2) + accel2(6) + gyro2(6) = 30
        sample_fmt = "<I3h3hh3h3h"
        assert struct.calcsize(sample_fmt) == SURVEY_SAMPLE_SIZE_V2
    else:
        raw_accel2 = raw_gyro2 = None
        # V1 layout: timestamp(4) + accel(6) + gyro(6) + temp(2) = 18
        sample_fmt = "<I3h3hh"
        assert struct.calcsize(sample_fmt) == SURVEY_SAMPLE_SIZE_V1

    for i in range(n_samples):
        offset = SURVEY_HEADER_SIZE + i * actual_sample_size
        vals = struct.unpack_from(sample_fmt, data, offset)
        timestamps[i] = vals[0]
        raw_accel[i] = vals[1:4]
        raw_gyro[i] = vals[4:7]
        raw_temp[i] = vals[7]
        if dual_imu:
            raw_accel2[i] = vals[8:11]
            raw_gyro2[i] = vals[11:14]

    # Convert to physical units
    accel = raw_accel.astype(np.float64) / ACCEL_LSB_PER_G
    gyro = raw_gyro.astype(np.float64) / GYRO_LSB_PER_DPS
    temp = raw_temp.astype(np.float64) / TEMP_LSB_PER_DEG + TEMP_OFFSET_DEG

    imu2_kwargs = {}
    if dual_imu:
        accel2 = raw_accel2.astype(np.float64) / ACCEL_LSB_PER_G
        gyro2 = raw_gyro2.astype(np.float64) / GYRO_LSB_PER_DPS
        imu2_kwargs = dict(
            accel_x2=accel2[:, 0], accel_y2=accel2[:, 1], accel_z2=accel2[:, 2],
            gyro_x2=gyro2[:, 0], gyro_y2=gyro2[:, 1], gyro_z2=gyro2[:, 2],
        )

    imu_label = "dual IMU" if dual_imu else "single IMU"
    print(f"Parsed {n_samples} samples from binary file "
          f"(v{version}, {imu_label}, {header.sample_rate_hz} Hz, "
          f"{n_samples / header.sample_rate_hz:.1f}s)")

    return SurveyData(
        header=header,
        source_file=str(filepath),
        timestamp_ms=timestamps,
        accel_x=accel[:, 0],
        accel_y=accel[:, 1],
        accel_z=accel[:, 2],
        gyro_x=gyro[:, 0],
        gyro_y=gyro[:, 1],
        gyro_z=gyro[:, 2],
        temperature=temp,
        **imu2_kwargs,
    )


def parse_json(filepath: Path) -> SurveyData:
    """Parse a browser-captured JSON survey file."""
    with open(filepath) as f:
        obj = json.load(f)

    samples = obj["samples"]
    n = len(samples)
    if n == 0:
        raise ValueError("No samples in JSON file")

    header = SurveyHeader(
        magic="JSON",
        version=obj.get("version", 1),
        sample_size=0,
        sample_rate_hz=obj.get("sample_rate_hz", 100),
        accel_range_g=obj.get("accel_range_g", 2),
        gyro_range_dps=obj.get("gyro_range_dps", 250),
        start_time_ms=0,
        car_id=obj.get("car", "GeometryCar"),
    )

    # Browser JSON stores values already in physical units (g, °/s, °C)
    timestamps = np.array([s["ts"] for s in samples], dtype=np.uint32)
    accel_x = np.array([s["ax"] for s in samples], dtype=np.float64)
    accel_y = np.array([s["ay"] for s in samples], dtype=np.float64)
    accel_z = np.array([s["az"] for s in samples], dtype=np.float64)
    gyro_x = np.array([s["gx"] for s in samples], dtype=np.float64)
    gyro_y = np.array([s["gy"] for s in samples], dtype=np.float64)
    gyro_z = np.array([s["gz"] for s in samples], dtype=np.float64)
    temperature = np.array([s["temp"] for s in samples], dtype=np.float64)

    # IMU #2 fields (present in version 2 JSON from dual-IMU firmware)
    imu2_kwargs = {}
    if "ax2" in samples[0]:
        imu2_kwargs = dict(
            accel_x2=np.array([s["ax2"] for s in samples], dtype=np.float64),
            accel_y2=np.array([s["ay2"] for s in samples], dtype=np.float64),
            accel_z2=np.array([s["az2"] for s in samples], dtype=np.float64),
            gyro_x2=np.array([s["gx2"] for s in samples], dtype=np.float64),
            gyro_y2=np.array([s["gy2"] for s in samples], dtype=np.float64),
            gyro_z2=np.array([s["gz2"] for s in samples], dtype=np.float64),
        )

    imu_label = "dual IMU" if imu2_kwargs else "single IMU"
    print(f"Parsed {n} samples from JSON file "
          f"({imu_label}, {header.sample_rate_hz} Hz, {n / header.sample_rate_hz:.1f}s)")

    return SurveyData(
        header=header,
        source_file=str(filepath),
        timestamp_ms=timestamps,
        accel_x=accel_x,
        accel_y=accel_y,
        accel_z=accel_z,
        gyro_x=gyro_x,
        gyro_y=gyro_y,
        gyro_z=gyro_z,
        temperature=temperature,
        **imu2_kwargs,
    )


def load_survey(filepath: Path) -> SurveyData:
    """Load a survey file, auto-detecting format from extension."""
    if filepath.suffix == ".json":
        return parse_json(filepath)
    elif filepath.suffix == ".bin":
        return parse_binary(filepath)
    else:
        raise ValueError(f"Unknown file extension: {filepath.suffix} (expected .bin or .json)")


# ===== Position Estimation =====

def compute_positions(survey: SurveyData, speed_mph: float, scale: float):
    """Convert timestamps to estimated positions using constant speed assumption.

    Speed is given in scale mph. Convert:
      scale_mph → prototype_mph → prototype_mm_per_ms → scale_mm_per_ms
    """
    # Convert scale speed to model mm/ms
    # 1 mph = 447.04 mm/s (prototype)
    prototype_mm_per_s = speed_mph * 447.04
    model_mm_per_s = prototype_mm_per_s / scale
    model_mm_per_ms = model_mm_per_s / 1000.0

    t = survey.timestamp_ms.astype(np.float64)
    t_relative = t - t[0]
    survey.position_mm = t_relative * model_mm_per_ms


# ===== Section Detection =====

def detect_sections(survey: SurveyData, section_len_mm: float,
                    thresholds: dict) -> list[Section]:
    """Divide the survey into fixed-length sections and classify each."""
    pos = survey.position_mm
    total_len = pos[-1] - pos[0]
    sections = []

    start = pos[0]
    while start < pos[-1]:
        end = min(start + section_len_mm, pos[-1])
        mask = (pos >= start) & (pos < end)
        if np.sum(mask) < 10:  # skip very short trailing sections
            break

        az = survey.accel_z[mask]
        ay = survey.accel_y[mask]
        gx = survey.gyro_x[mask]
        gz = survey.gyro_z[mask]

        # Remove gravity from Z-accel (static Z ≈ 1g on level track)
        az_dynamic = az - np.mean(az)
        # Remove DC offset from lateral (should be ~0 on straight, nonzero in curves)
        ay_dynamic = ay - np.mean(ay)

        vert_rms = float(np.sqrt(np.mean(az_dynamic**2)))
        lat_rms = float(np.sqrt(np.mean(ay_dynamic**2)))
        twist_rms = float(np.sqrt(np.mean(gx**2)))

        # Classify section type from gyro Z (yaw rate)
        mean_yaw = float(np.mean(gz))
        # Classify from accel X (pitch → grade)
        mean_pitch_g = float(np.mean(survey.accel_x[mask]))

        if abs(mean_yaw) > 2.0:  # °/s — sustained yaw → curve
            section_type = "curve"
        elif abs(mean_pitch_g) > 0.015:  # ~0.9° pitch → ~1.5% grade
            section_type = "grade"
        else:
            section_type = "straight"

        grade = compute_section_grade(vert_rms, lat_rms, twist_rms, section_type)

        sec = Section(
            start_mm=float(start),
            end_mm=float(end),
            type=section_type,
            grade=grade,
            vertical_rms_g=vert_rms,
            lateral_rms_g=lat_rms,
            twist_rms_dps=twist_rms,
        )

        if section_type == "curve":
            # Compute curve info using the section's yaw data
            mean_yr = float(mean_yaw)
            if abs(mean_yr) > 0.1:
                # radius = speed / yaw_rate (in consistent units)
                # speed in mm/s, yaw in rad/s → radius in mm
                pos_section = pos[mask]
                speed_mm_s = np.mean(np.diff(pos_section)) * survey.header.sample_rate_hz
                yaw_rad_s = np.radians(mean_yr)
                if abs(yaw_rad_s) > 1e-6:
                    radius = abs(speed_mm_s / yaw_rad_s)
                    sec.curve_radius_mm = float(radius)
                sec.curve_direction = "left" if mean_yr > 0 else "right"
                # Consistency: 1 - (std/mean) of yaw rate, clamped to [0, 1]
                if abs(mean_yr) > 0.5:
                    consistency = 1.0 - min(float(np.std(gz)) / abs(mean_yr), 1.0)
                    sec.curvature_consistency = max(0.0, consistency)

        sections.append(sec)
        start = end

    return sections


def compute_section_grade(vert_rms: float, lat_rms: float,
                          twist_rms: float, section_type: str) -> str:
    """Grade a section A-F based on RMS values."""
    # Score each metric independently, take worst
    def grade_metric(value, thresholds_list):
        if value <= thresholds_list[0]:
            return "A"
        elif value <= thresholds_list[1]:
            return "B"
        elif value <= thresholds_list[2]:
            return "C"
        else:
            return "D"

    g_vert = grade_metric(vert_rms, GRADE_THRESHOLDS["vert_rms"])
    g_lat = grade_metric(lat_rms, GRADE_THRESHOLDS["lat_rms"])
    g_twist = grade_metric(twist_rms, GRADE_THRESHOLDS["twist_max"])

    # Overall = worst grade
    return max(g_vert, g_lat, g_twist)  # 'D' > 'C' > 'B' > 'A' lexicographically


# ===== Event Detection =====

def detect_events(survey: SurveyData, thresholds: dict) -> list[Event]:
    """Detect discrete track events (joints, kinks, turnout frogs, etc.)."""
    events = []
    pos = survey.position_mm
    t = survey.timestamp_ms

    # Remove gravity from Z-accel
    az = survey.accel_z - np.mean(survey.accel_z)
    ay = survey.accel_y
    gx = survey.gyro_x
    gz = survey.gyro_z

    th_vert = thresholds["vert"]
    th_lat = thresholds["lat"]
    th_twist = thresholds["twist"]

    # Find peaks in vertical acceleration (rail joints, kinks)
    vert_peaks = find_peaks_above(np.abs(az), th_vert, min_spacing_samples=10)
    for idx in vert_peaks:
        lat_val = abs(ay[idx])
        yaw_val = abs(gz[idx])

        # Classify by signature shape (from spec defect table)
        if lat_val > th_lat * 0.5 and yaw_val > 2.0:
            event_type = "turnout_frog"
            desc = f"Sharp vertical+lateral impact — probable turnout frog"
        elif lat_val > th_lat * 0.3:
            event_type = "kink"
            desc = f"Vertical spike with lateral component — kink in rail"
        else:
            event_type = "joint"
            desc = f"Vertical spike, minimal lateral — rail joint or gap"

        severity = classify_severity(abs(az[idx]), th_vert)
        events.append(Event(
            type=event_type,
            position_mm=float(pos[idx]),
            time_ms=float(t[idx]),
            severity=severity,
            description=desc,
            details={
                "vertical_g": float(az[idx]),
                "lateral_g": float(ay[idx]),
                "yaw_dps": float(gz[idx]),
            },
        ))

    # Find sustained twist (roll rate)
    twist_regions = find_sustained_above(np.abs(gx), th_twist,
                                         min_duration_samples=20)
    for start_idx, end_idx in twist_regions:
        mid = (start_idx + end_idx) // 2
        peak_roll = float(np.max(np.abs(gx[start_idx:end_idx])))
        events.append(Event(
            type="twist",
            position_mm=float(pos[mid]),
            time_ms=float(t[mid]),
            severity=classify_severity(peak_roll, th_twist),
            description=f"Cross-level variation over ~{float(pos[end_idx] - pos[start_idx]):.0f}mm",
            details={
                "roll_rate_peak_dps": peak_roll,
                "start_mm": float(pos[start_idx]),
                "end_mm": float(pos[end_idx]),
            },
        ))

    # Find lateral-only events (alignment defects or clearance spikes)
    lat_peaks = find_peaks_above(np.abs(ay), th_lat, min_spacing_samples=10)
    for idx in lat_peaks:
        # Skip if already captured as a joint/kink/frog event
        if abs(az[idx]) > th_vert * 0.5:
            continue

        # Distinguish clearance spike from alignment defect:
        # Clearance spike: single-direction impulse, very short, no roll/yaw,
        #   minimal vertical. Something physical pushed the car sideways.
        # Alignment defect: lateral displacement, may have some roll.

        # Check impulse duration: look at how many consecutive samples exceed
        # half the peak value
        half_peak = abs(ay[idx]) * 0.5
        impulse_len = 1
        for k in range(1, min(10, len(ay) - idx)):
            if abs(ay[idx + k]) > half_peak:
                impulse_len += 1
            else:
                break
        for k in range(1, min(10, idx)):
            if abs(ay[idx - k]) > half_peak:
                impulse_len += 1
            else:
                break

        # Check for roll/yaw accompaniment
        roll_present = abs(gx[idx]) > 1.5  # °/s
        yaw_present = abs(gz[idx]) > 1.5

        is_clearance = (impulse_len <= 5 and  # <50ms at 100Hz
                        not roll_present and
                        not yaw_present and
                        abs(az[idx]) < th_vert * 0.3)

        if is_clearance:
            side = "left" if ay[idx] > 0 else "right"
            events.append(Event(
                type="clearance",
                position_mm=float(pos[idx]),
                time_ms=float(t[idx]),
                severity=classify_severity(abs(ay[idx]), th_lat),
                description=f"Lateral impulse ({side} side) — possible clearance obstruction",
                details={
                    "lateral_g": float(ay[idx]),
                    "side": side,
                    "impulse_duration_ms": impulse_len * (1000 / survey.header.sample_rate_hz),
                },
            ))
        else:
            events.append(Event(
                type="alignment",
                position_mm=float(pos[idx]),
                time_ms=float(t[idx]),
                severity=classify_severity(abs(ay[idx]), th_lat),
                description=f"Lateral displacement without vertical — alignment defect",
                details={
                    "lateral_g": float(ay[idx]),
                    "vertical_g": float(az[idx]),
                },
            ))

    # Sort by position
    events.sort(key=lambda e: e.position_mm)
    return events


def find_peaks_above(signal: np.ndarray, threshold: float,
                     min_spacing_samples: int = 10) -> list[int]:
    """Find indices where signal exceeds threshold, with minimum spacing."""
    above = np.where(signal > threshold)[0]
    if len(above) == 0:
        return []

    peaks = []
    last_peak = -min_spacing_samples * 2
    i = 0
    while i < len(above):
        # Find the peak within this cluster of above-threshold samples
        cluster_start = above[i]
        cluster_end = cluster_start
        while i + 1 < len(above) and above[i + 1] - above[i] <= 2:
            i += 1
            cluster_end = above[i]

        peak_idx = cluster_start + int(np.argmax(signal[cluster_start:cluster_end + 1]))
        if peak_idx - last_peak >= min_spacing_samples:
            peaks.append(peak_idx)
            last_peak = peak_idx
        i += 1

    return peaks


def find_sustained_above(signal: np.ndarray, threshold: float,
                         min_duration_samples: int = 20) -> list[tuple[int, int]]:
    """Find regions where signal stays above threshold for minimum duration."""
    above = signal > threshold
    regions = []
    in_region = False
    start = 0

    for i in range(len(above)):
        if above[i] and not in_region:
            start = i
            in_region = True
        elif not above[i] and in_region:
            if i - start >= min_duration_samples:
                regions.append((start, i))
            in_region = False

    if in_region and len(above) - start >= min_duration_samples:
        regions.append((start, len(above)))

    return regions


def classify_severity(value: float, threshold: float) -> str:
    """Classify severity based on how far above threshold."""
    ratio = value / threshold
    if ratio < 1.5:
        return "minor"
    elif ratio < 3.0:
        return "moderate"
    else:
        return "severe"


# ===== Curve Analysis =====

def _measure_transition(yaw: np.ndarray, pos: np.ndarray,
                        full_yaw: float, speed_mm_s: float,
                        from_entry: bool) -> TransitionInfo:
    """Measure a curve entry or exit transition (easement/spiral).

    A transition is the zone where yaw rate ramps from ~0 to full curve rate
    (entry) or from full rate back to ~0 (exit). A proper spiral easement has
    a linear ramp; an abrupt entry is a step function.

    Measurement: walk inward from the curve boundary, find where yaw first
    reaches 80% of full value. The distance from the boundary to that point
    is the transition length.
    """
    n = len(yaw)
    if n < 10:
        return TransitionInfo(present=False, length_mm=0, rate_dps_per_mm=0, type="abrupt")

    threshold_low = abs(full_yaw) * 0.2   # below this = "not yet in curve"
    threshold_high = abs(full_yaw) * 0.8  # above this = "fully in curve"

    if from_entry:
        # Walk from start of curve inward
        search = np.abs(yaw)
        search_pos = pos
    else:
        # Walk from end of curve inward (reverse)
        search = np.abs(yaw[::-1])
        search_pos = pos[-1] - pos[::-1] + pos[0]  # reverse positions

    # Find first sample above threshold_high
    high_idx = None
    for k in range(n):
        if search[k] >= threshold_high:
            high_idx = k
            break

    if high_idx is None or high_idx < 2:
        # Yaw jumps to full value immediately — abrupt entry
        return TransitionInfo(present=False, length_mm=0, rate_dps_per_mm=0, type="abrupt")

    # Transition length = distance from curve boundary to where yaw reaches 80%
    trans_length_mm = float(abs(search_pos[high_idx] - search_pos[0]))

    if trans_length_mm < 1.0:
        return TransitionInfo(present=False, length_mm=0, rate_dps_per_mm=0, type="abrupt")

    # Rate of curvature change (°/s per mm of travel)
    yaw_change = float(search[high_idx] - search[0])
    rate = abs(yaw_change) / trans_length_mm

    # Classify: compare transition length to curve length
    curve_length = float(pos[-1] - pos[0])
    # A proper spiral easement is typically 10-25% of curve length
    # A "short" transition is less than 5% — just slight rounding, not a real easement
    frac = trans_length_mm / curve_length if curve_length > 0 else 0

    if frac >= 0.08:
        trans_type = "spiral"
    elif frac >= 0.02:
        trans_type = "short"
    else:
        return TransitionInfo(present=False, length_mm=0, rate_dps_per_mm=0, type="abrupt")

    return TransitionInfo(
        present=True,
        length_mm=trans_length_mm,
        rate_dps_per_mm=rate,
        type=trans_type,
    )


def analyze_curves(survey: SurveyData, sections: list[Section]) -> list[CurveInfo]:
    """Merge adjacent curve sections and analyze curve quality."""
    curves = []
    pos = survey.position_mm
    gz = survey.gyro_z

    i = 0
    while i < len(sections):
        if sections[i].type != "curve":
            i += 1
            continue

        # Merge consecutive curve sections with same direction
        curve_start = sections[i].start_mm
        curve_end = sections[i].end_mm
        direction = sections[i].curve_direction
        j = i + 1
        while j < len(sections) and sections[j].type == "curve":
            if sections[j].curve_direction == direction:
                curve_end = sections[j].end_mm
                j += 1
            else:
                break

        # Analyze the merged curve
        mask = (pos >= curve_start) & (pos < curve_end)
        yaw = gz[mask]
        if len(yaw) < 10:
            i = j
            continue

        # Compute speed for radius calculation
        pos_segment = pos[mask]
        if len(pos_segment) > 1:
            speed_mm_s = float(np.mean(np.diff(pos_segment))) * survey.header.sample_rate_hz
        else:
            speed_mm_s = 1.0

        # Preliminary mean for transition detection (uses full curve including spirals)
        prelim_mean_yr = float(np.mean(yaw))

        # Measure entry and exit transitions first, so we can trim to body
        entry_transition = _measure_transition(
            yaw, pos_segment, prelim_mean_yr, speed_mm_s, from_entry=True)
        exit_transition = _measure_transition(
            yaw, pos_segment, prelim_mean_yr, speed_mm_s, from_entry=False)

        # Trim to curve body (exclude transition zones) for radius/consistency/kinks
        body_start_mm = curve_start + entry_transition.length_mm
        body_end_mm = curve_end - exit_transition.length_mm
        body_mask = (pos >= body_start_mm) & (pos < body_end_mm)
        yaw_body = gz[body_mask]

        if len(yaw_body) >= 10:
            mean_yr = float(np.mean(yaw_body))
            std_yr = float(np.std(yaw_body))
        else:
            # Curve is all transition, no body — use full data
            mean_yr = prelim_mean_yr
            std_yr = float(np.std(yaw))
            yaw_body = yaw

        # Radius from body yaw rate (excludes spirals)
        mean_yaw_rad = np.radians(mean_yr)
        if abs(mean_yaw_rad) > 1e-6:
            avg_r = abs(speed_mm_s / mean_yaw_rad)
        else:
            avg_r = float("inf")

        yaw_body_rad = np.radians(yaw_body)
        core = np.abs(yaw_body_rad) > abs(mean_yaw_rad) * 0.5
        if np.any(core):
            radii = np.abs(speed_mm_s / yaw_body_rad[core])
            min_r = float(np.min(radii))
            max_r = float(np.max(radii))
        else:
            min_r = max_r = avg_r

        consistency = 1.0 - min(std_yr / abs(mean_yr), 1.0) if abs(mean_yr) > 0.5 else 0.0

        # Count kinks within curve body only (transition ramps aren't kinks)
        yaw_deviation = np.abs(yaw_body - mean_yr)
        kink_threshold = max(abs(mean_yr) * 0.3, 2.0)
        kinks = len(find_peaks_above(yaw_deviation, kink_threshold, min_spacing_samples=10))

        radius_in = avg_r / 25.4  # mm to inches
        parts = [f"{radius_in:.0f}\" curve"]
        parts.append(
            f"{'well' if consistency > 0.9 else 'moderately' if consistency > 0.8 else 'poorly'} "
            f"consistent")
        if kinks > 0:
            parts.append(f"{kinks} kink(s)")
        if entry_transition.present:
            parts.append(f"entry {entry_transition.type} {entry_transition.length_mm:.0f}mm")
        if exit_transition.present:
            parts.append(f"exit {exit_transition.type} {exit_transition.length_mm:.0f}mm")
        note = ", ".join(parts)

        curves.append(CurveInfo(
            start_mm=curve_start,
            end_mm=curve_end,
            direction=direction or ("left" if mean_yr > 0 else "right"),
            avg_radius_mm=avg_r,
            min_radius_mm=min_r,
            max_radius_mm=max_r,
            consistency=consistency,
            entry_transition=entry_transition,
            exit_transition=exit_transition,
            kinks=kinks,
            note=note,
        ))

        i = j

    return curves


# ===== Grade Analysis =====

def analyze_grades(survey: SurveyData, sections: list[Section]) -> list[GradeInfo]:
    """Identify and analyze grade (slope) regions."""
    grades = []
    pos = survey.position_mm
    ax = survey.accel_x  # pitch axis

    i = 0
    while i < len(sections):
        if sections[i].type != "grade":
            i += 1
            continue

        # Merge consecutive grade sections
        grade_start = sections[i].start_mm
        grade_end = sections[i].end_mm
        j = i + 1
        while j < len(sections) and sections[j].type == "grade":
            grade_end = sections[j].end_mm
            j += 1

        mask = (pos >= grade_start) & (pos < grade_end)
        pitch_g = ax[mask]

        # Convert pitch acceleration to grade percentage
        # On a slope, accel_x ≈ sin(angle) in g, and grade% ≈ tan(angle) × 100
        # For small angles sin ≈ tan, so grade% ≈ accel_x_g × 100
        avg_grade_pct = float(np.mean(pitch_g)) * 100
        max_grade_pct = float(np.max(np.abs(pitch_g))) * 100
        direction = "ascending" if avg_grade_pct > 0 else "descending"

        rise_mm = abs(avg_grade_pct / 100) * (grade_end - grade_start)
        note = (f"{abs(avg_grade_pct):.1f}% grade over {grade_end - grade_start:.0f}mm "
                f"({rise_mm / 25.4:.1f}\" rise)")

        grades.append(GradeInfo(
            start_mm=grade_start,
            end_mm=grade_end,
            avg_grade_percent=abs(avg_grade_pct),
            max_grade_percent=max_grade_pct,
            direction=direction,
            note=note,
        ))

        i = j

    return grades


# ===== Vertical Curve Detection =====

def detect_vertical_curves(survey: SurveyData, thresholds: dict) -> list[VerticalCurveInfo]:
    """Detect vertical curves (crests and sags) at grade transitions.

    A vertical curve is where the track changes pitch — the top of a hill (crest)
    or the bottom of a valley (sag). Sharp vertical curves are a derailment risk
    because long cars bridge across them and lift wheels off the rail.

    Detection method: look for sustained pitch rate (gyro_y) which indicates
    the car is rotating in pitch as it traverses the vertical curve. The vertical
    curve radius R = v / ω where v is model speed and ω is pitch rate.
    """
    vcurves = []
    pos = survey.position_mm
    gy = survey.gyro_y
    ax = survey.accel_x  # pitch axis (grade proxy)
    fs = survey.header.sample_rate_hz

    th_pitch_rate = thresholds.get("pitch_rate", 3.0)  # °/s

    # Smooth pitch rate to reduce noise (moving average, ~50ms window)
    window = max(1, int(fs * 0.05))
    if len(gy) > window:
        kernel = np.ones(window) / window
        gy_smooth = np.convolve(gy, kernel, mode="same")
    else:
        gy_smooth = gy

    # Find regions of sustained pitch rate.
    # Vertical curves can be short (~100ms for a sharp crest), so use a low
    # minimum duration. The threshold itself filters out noise.
    regions = find_sustained_above(np.abs(gy_smooth), th_pitch_rate,
                                   min_duration_samples=max(3, int(fs * 0.03)))

    for start_idx, end_idx in regions:
        mid = (start_idx + end_idx) // 2

        # Compute vertical curve radius: R = v / ω
        # Use position data to get speed in mm/s
        if end_idx - start_idx > 1:
            local_speed = float(np.mean(np.diff(pos[start_idx:end_idx]))) * fs
        else:
            local_speed = float(np.mean(np.diff(pos))) * fs

        peak_pitch_rate = float(np.max(np.abs(gy_smooth[start_idx:end_idx])))
        mean_pitch_rate = float(np.mean(gy_smooth[start_idx:end_idx]))
        pitch_rate_rad = np.radians(peak_pitch_rate)

        if pitch_rate_rad > 1e-6:
            radius = local_speed / pitch_rate_rad
        else:
            continue

        # Determine crest vs sag from pitch direction
        # Look at grade (accel_x) before and after the transition
        lookback = min(int(fs * 0.5), start_idx)  # 500ms or whatever is available
        lookahead = min(int(fs * 0.5), len(ax) - end_idx)

        grade_before = float(np.mean(ax[start_idx - lookback:start_idx])) * 100
        grade_after = float(np.mean(ax[end_idx:end_idx + lookahead])) * 100

        # Crest: grade goes from positive (ascending) to less positive/negative
        # Sag: grade goes from negative (descending) to less negative/positive
        # Or equivalently: crest has negative pitch rate (nose dropping),
        # sag has positive pitch rate (nose rising)
        if mean_pitch_rate < 0:
            vc_type = "crest"
        else:
            vc_type = "sag"

        # Severity based on radius relative to typical car length
        # An HO 60' car is ~210mm. NMRA recommends vertical curves of at least
        # 2× car length radius. Under 1× is severe.
        car_length_mm = 210  # typical HO 60' car
        if radius < car_length_mm:
            severity = "severe"
        elif radius < car_length_mm * 2:
            severity = "moderate"
        elif radius < car_length_mm * 4:
            severity = "minor"
        else:
            severity = "minor"

        radius_in = radius / 25.4
        note = (f"{vc_type.title()} vertical curve, R={radius_in:.1f}\" "
                f"({grade_before:.1f}% → {grade_after:.1f}%), "
                f"peak pitch rate {peak_pitch_rate:.1f}°/s")

        vcurves.append(VerticalCurveInfo(
            position_mm=float(pos[mid]),
            type=vc_type,
            radius_mm=float(radius),
            peak_pitch_rate_dps=peak_pitch_rate,
            grade_before_pct=grade_before,
            grade_after_pct=grade_after,
            severity=severity,
            note=note,
        ))

        # Also add as an event for the event list
    return vcurves


# ===== Spectral Analysis =====

def compute_spectrum(survey: SurveyData) -> dict:
    """Compute FFT of key channels for frequency analysis."""
    fs = survey.header.sample_rate_hz
    n = len(survey.accel_z)

    # Remove DC offset
    az = survey.accel_z - np.mean(survey.accel_z)
    ay = survey.accel_y - np.mean(survey.accel_y)

    # Apply Hann window to reduce spectral leakage
    window = np.hanning(n)
    az_windowed = az * window
    ay_windowed = ay * window

    # FFT
    freqs = np.fft.rfftfreq(n, d=1.0 / fs)
    az_fft = np.abs(np.fft.rfft(az_windowed)) * 2 / n
    ay_fft = np.abs(np.fft.rfft(ay_windowed)) * 2 / n

    # Find dominant frequencies (top 5 peaks)
    def top_peaks(spectrum, freqs, n_peaks=5, min_freq=0.5):
        mask = freqs >= min_freq
        s = spectrum[mask]
        f = freqs[mask]
        if len(s) < n_peaks:
            return []
        indices = np.argsort(s)[-n_peaks:][::-1]
        return [(float(f[i]), float(s[i])) for i in indices]

    return {
        "frequencies_hz": freqs,
        "vertical_spectrum": az_fft,
        "lateral_spectrum": ay_fft,
        "vertical_peaks": top_peaks(az_fft, freqs),
        "lateral_peaks": top_peaks(ay_fft, freqs),
    }


# ===== Report Generation =====

def overall_grade(sections: list[Section]) -> str:
    """Compute overall survey grade from section grades."""
    if not sections:
        return "N/A"
    grade_vals = {"A": 4, "B": 3, "C": 2, "D": 1, "F": 0}
    val_grades = {v: k for k, v in grade_vals.items()}
    avg = np.mean([grade_vals.get(s.grade, 0) for s in sections])
    # Round to nearest grade
    return val_grades.get(round(avg), "C")


def generate_report(survey: SurveyData, sections: list[Section],
                    events: list[Event], curves: list[CurveInfo],
                    grades: list[GradeInfo], vcurves: list[VerticalCurveInfo],
                    spectrum: dict) -> dict:
    """Build the JSON report structure matching the spec."""
    duration = float(survey.timestamp_ms[-1] - survey.timestamp_ms[0]) / 1000
    distance = float(survey.position_mm[-1] - survey.position_mm[0])

    # Count events by type
    event_counts = {}
    for e in events:
        event_counts[e.type] = event_counts.get(e.type, 0) + 1

    report = {
        "survey_file": survey.source_file,
        "format": survey.header.magic,
        "version": survey.header.version,
        "imu_count": 2 if survey.has_imu2 else 1,
        "car_id": survey.header.car_id,
        "sample_rate_hz": survey.header.sample_rate_hz,
        "duration_sec": round(duration, 1),
        "total_samples": len(survey.timestamp_ms),
        "distance_estimated_mm": round(distance, 0),
        "overall_grade": overall_grade(sections),
        "temperature_mean_c": round(float(np.mean(survey.temperature)), 1),
        "temperature_range_c": [
            round(float(np.min(survey.temperature)), 1),
            round(float(np.max(survey.temperature)), 1),
        ],
        "sections": [
            {
                "start_mm": round(s.start_mm, 0),
                "end_mm": round(s.end_mm, 0),
                "type": s.type,
                "vertical_rms_g": round(s.vertical_rms_g, 4),
                "lateral_rms_g": round(s.lateral_rms_g, 4),
                "twist_rms_deg_s": round(s.twist_rms_dps, 2),
                "grade": s.grade,
                **({"curve_radius_mm": round(s.curve_radius_mm, 0)}
                   if s.curve_radius_mm else {}),
                **({"curve_direction": s.curve_direction}
                   if s.curve_direction else {}),
                **({"curvature_consistency": round(s.curvature_consistency, 2)}
                   if s.curvature_consistency is not None else {}),
                "events": [
                    {
                        "type": e.type,
                        "position_mm": round(e.position_mm, 0),
                        "severity": e.severity,
                        "description": e.description,
                    }
                    for e in events
                    if s.start_mm <= e.position_mm < s.end_mm
                ],
            }
            for s in sections
        ],
        "curves": [
            {
                "start_mm": round(c.start_mm, 0),
                "end_mm": round(c.end_mm, 0),
                "direction": c.direction,
                "avg_radius_mm": round(c.avg_radius_mm, 0),
                "avg_radius_inches": round(c.avg_radius_mm / 25.4, 1),
                "min_radius_mm": round(c.min_radius_mm, 0),
                "max_radius_mm": round(c.max_radius_mm, 0),
                "consistency": round(c.consistency, 2),
                "entry_transition": {
                    "type": c.entry_transition.type,
                    "length_mm": round(c.entry_transition.length_mm, 0),
                    "rate_dps_per_mm": round(c.entry_transition.rate_dps_per_mm, 4),
                } if c.entry_transition.present else {"type": "abrupt"},
                "exit_transition": {
                    "type": c.exit_transition.type,
                    "length_mm": round(c.exit_transition.length_mm, 0),
                    "rate_dps_per_mm": round(c.exit_transition.rate_dps_per_mm, 4),
                } if c.exit_transition.present else {"type": "abrupt"},
                "kinks": c.kinks,
                "note": c.note,
            }
            for c in curves
        ],
        "grades": [
            {
                "start_mm": round(g.start_mm, 0),
                "end_mm": round(g.end_mm, 0),
                "avg_grade_percent": round(g.avg_grade_percent, 1),
                "max_grade_percent": round(g.max_grade_percent, 1),
                "direction": g.direction,
                "note": g.note,
            }
            for g in grades
        ],
        "vertical_curves": [
            {
                "position_mm": round(vc.position_mm, 0),
                "type": vc.type,
                "radius_mm": round(vc.radius_mm, 0),
                "radius_inches": round(vc.radius_mm / 25.4, 1),
                "peak_pitch_rate_dps": round(vc.peak_pitch_rate_dps, 1),
                "grade_before_pct": round(vc.grade_before_pct, 1),
                "grade_after_pct": round(vc.grade_after_pct, 1),
                "severity": vc.severity,
                "note": vc.note,
            }
            for vc in vcurves
        ],
        "spectral_peaks": {
            "vertical_hz": [
                {"freq_hz": round(f, 1), "amplitude_g": round(a, 4)}
                for f, a in spectrum.get("vertical_peaks", [])
            ],
            "lateral_hz": [
                {"freq_hz": round(f, 1), "amplitude_g": round(a, 4)}
                for f, a in spectrum.get("lateral_peaks", [])
            ],
        },
        "event_summary": {
            "total_events": len(events),
            **{k: v for k, v in event_counts.items()},
        },
        "grading_key": {
            "A": "Excellent — smooth, well-maintained track",
            "B": "Good — minor imperfections, unlikely to cause issues",
            "C": "Fair — noticeable roughness, may cause issues with sensitive equipment",
            "D": "Poor — significant defects, derailment risk",
        },
    }

    return report


# ===== Visualization =====

def generate_plots(survey: SurveyData, sections: list[Section],
                   events: list[Event], curves: list[CurveInfo],
                   vcurves: list[VerticalCurveInfo],
                   spectrum: dict, output_dir: Path,
                   compare: SurveyData = None):
    """Generate matplotlib visualizations."""
    try:
        import matplotlib
        matplotlib.use("Agg")
        import matplotlib.pyplot as plt
    except ImportError:
        print("Warning: matplotlib not available, skipping plots", file=sys.stderr)
        return

    pos = survey.position_mm
    pos_m = pos / 1000  # convert to meters for readability

    # --- Strip chart: vertical + lateral accel vs position ---
    fig, (ax1, ax2) = plt.subplots(2, 1, figsize=(14, 8), sharex=True)
    fig.suptitle("Track Geometry Survey — Acceleration", fontsize=14, fontweight="bold")

    az_dynamic = survey.accel_z - np.mean(survey.accel_z)
    ax1.plot(pos_m, az_dynamic, linewidth=0.4, color="#4488cc", alpha=0.8)
    ax1.set_ylabel("Vertical (Z) [g]")
    ax1.axhline(0, color="#444", linewidth=0.5)
    ax1.grid(True, alpha=0.3)

    ax2.plot(pos_m, survey.accel_y, linewidth=0.4, color="#cc4444", alpha=0.8, label="IMU1")
    ax2.set_ylabel("Lateral (Y) [g]")
    ax2.set_xlabel("Position [m]")
    ax2.axhline(0, color="#444", linewidth=0.5)
    ax2.grid(True, alpha=0.3)

    # IMU2 overlays (dashed, dimmer)
    if survey.has_imu2:
        az2_dynamic = survey.accel_z2 - np.mean(survey.accel_z2)
        ax1.plot(pos_m, az2_dynamic, linewidth=0.3, color="#44cc88", alpha=0.5,
                 linestyle="--", label="IMU2")
        ax2.plot(pos_m, survey.accel_y2, linewidth=0.3, color="#cc8888", alpha=0.5,
                 linestyle="--", label="IMU2")
        ax1.legend(fontsize=8)
        ax2.legend(fontsize=8)

    # Mark events
    for e in events:
        color = {"joint": "#888", "kink": "#ff8800", "turnout_frog": "#ff0000",
                 "twist": "#aa44ff", "alignment": "#44aaff"}.get(e.type, "#888")
        marker = {"severe": "x", "moderate": "D", "minor": "."}.get(e.severity, ".")
        for ax in (ax1, ax2):
            ax.axvline(e.position_mm / 1000, color=color, alpha=0.3, linewidth=0.5)

    # Color background by section grade
    grade_colors = {"A": "#00880020", "B": "#88880020", "C": "#ff880020", "D": "#ff000020"}
    for s in sections:
        color = grade_colors.get(s.grade, "#88888820")
        for ax in (ax1, ax2):
            ax.axvspan(s.start_mm / 1000, s.end_mm / 1000, color=color)

    if compare:
        cpos_m = compare.position_mm / 1000
        caz = compare.accel_z - np.mean(compare.accel_z)
        ax1.plot(cpos_m, caz, linewidth=0.3, color="#44cc88", alpha=0.5, label="comparison")
        ax2.plot(cpos_m, compare.accel_y, linewidth=0.3, color="#cc8844", alpha=0.5, label="comparison")
        ax1.legend(fontsize=8)

    plt.tight_layout()
    fig.savefig(output_dir / "strip_chart.png", dpi=150)
    plt.close(fig)
    print(f"  Saved strip_chart.png")

    # --- Curvature plot: yaw rate vs position ---
    fig, ax = plt.subplots(figsize=(14, 4))
    ax.plot(pos_m, survey.gyro_z, linewidth=0.5, color="#cc44cc", label="IMU1")
    ax.set_ylabel("Yaw Rate [°/s]")
    ax.set_xlabel("Position [m]")
    ax.set_title("Curvature Profile (Yaw Rate)")
    ax.axhline(0, color="#444", linewidth=0.5)
    ax.grid(True, alpha=0.3)

    if survey.has_imu2:
        ax.plot(pos_m, survey.gyro_z2, linewidth=0.3, color="#8844cc", alpha=0.5,
                linestyle="--", label="IMU2")
        ax.legend(fontsize=8)

    for c in curves:
        ax.axvspan(c.start_mm / 1000, c.end_mm / 1000, color="#cc44cc20")
        mid = (c.start_mm + c.end_mm) / 2000
        ax.annotate(f"{c.avg_radius_mm / 25.4:.0f}\"",
                    xy=(mid, 0), fontsize=7, ha="center", color="#cc44cc")

    plt.tight_layout()
    fig.savefig(output_dir / "curvature.png", dpi=150)
    plt.close(fig)
    print(f"  Saved curvature.png")

    # --- Grade profile: pitch angle vs position, with vertical curves ---
    fig, (ax_grade, ax_pitch) = plt.subplots(2, 1, figsize=(14, 6), sharex=True)
    fig.suptitle("Grade Profile", fontsize=14, fontweight="bold")

    grade_pct = survey.accel_x * 100  # approx grade % for small angles
    ax_grade.plot(pos_m, grade_pct, linewidth=0.5, color="#44aa44")
    ax_grade.set_ylabel("Grade [%]")
    ax_grade.axhline(0, color="#444", linewidth=0.5)
    ax_grade.grid(True, alpha=0.3)

    # Mark vertical curves on grade plot
    for vc in vcurves:
        color = "#ff0000" if vc.severity == "severe" else (
                "#ff8800" if vc.severity == "moderate" else "#ffcc00")
        marker = "v" if vc.type == "crest" else "^"
        ax_grade.plot(vc.position_mm / 1000, 0, marker=marker, color=color,
                      markersize=10, zorder=5)
        ax_grade.annotate(
            f"{'Crest' if vc.type == 'crest' else 'Sag'}\nR={vc.radius_mm / 25.4:.0f}\"",
            xy=(vc.position_mm / 1000, 0), fontsize=6, ha="center",
            va="top" if vc.type == "crest" else "bottom",
            color=color, fontweight="bold")

    ax_pitch.plot(pos_m, survey.gyro_y, linewidth=0.5, color="#cc8844")
    ax_pitch.set_ylabel("Pitch Rate [°/s]")
    ax_pitch.set_xlabel("Position [m]")
    ax_pitch.axhline(0, color="#444", linewidth=0.5)
    ax_pitch.grid(True, alpha=0.3)

    plt.tight_layout()
    fig.savefig(output_dir / "grade_profile.png", dpi=150)
    plt.close(fig)
    print(f"  Saved grade_profile.png")

    # --- Frequency spectrum ---
    fig, (ax1, ax2) = plt.subplots(2, 1, figsize=(10, 6), sharex=True)
    fig.suptitle("Frequency Spectrum", fontsize=14, fontweight="bold")

    freqs = spectrum["frequencies_hz"]
    mask = freqs <= 50  # only show up to Nyquist/useful range

    ax1.plot(freqs[mask], spectrum["vertical_spectrum"][mask],
             linewidth=0.5, color="#4488cc")
    ax1.set_ylabel("Vertical [g]")
    ax1.grid(True, alpha=0.3)
    ax1.set_title("Vertical (Z-axis)")

    ax2.plot(freqs[mask], spectrum["lateral_spectrum"][mask],
             linewidth=0.5, color="#cc4444")
    ax2.set_ylabel("Lateral [g]")
    ax2.set_xlabel("Frequency [Hz]")
    ax2.grid(True, alpha=0.3)
    ax2.set_title("Lateral (Y-axis)")

    plt.tight_layout()
    fig.savefig(output_dir / "spectrum.png", dpi=150)
    plt.close(fig)
    print(f"  Saved spectrum.png")

    # --- Section grade histogram ---
    fig, ax = plt.subplots(figsize=(6, 4))
    grade_list = [s.grade for s in sections]
    grade_labels = ["A", "B", "C", "D"]
    counts = [grade_list.count(g) for g in grade_labels]
    colors = ["#44cc44", "#cccc44", "#ff8844", "#ff4444"]
    ax.bar(grade_labels, counts, color=colors)
    ax.set_ylabel("Number of Sections")
    ax.set_title("Section Grade Distribution")
    ax.grid(True, alpha=0.3, axis="y")

    plt.tight_layout()
    fig.savefig(output_dir / "grade_histogram.png", dpi=150)
    plt.close(fig)
    print(f"  Saved grade_histogram.png")


# ===== CSV Export =====

def export_csv(survey: SurveyData, output_dir: Path):
    """Export survey data as CSV for external tools."""
    csv_path = output_dir / "survey_data.csv"
    header = "timestamp_ms,position_mm,accel_x_g,accel_y_g,accel_z_g,gyro_x_dps,gyro_y_dps,gyro_z_dps,temperature_c"
    if survey.has_imu2:
        header += ",accel_x2_g,accel_y2_g,accel_z2_g,gyro_x2_dps,gyro_y2_dps,gyro_z2_dps"
    header += "\n"
    with open(csv_path, "w") as f:
        f.write(header)
        for i in range(len(survey.timestamp_ms)):
            line = (f"{survey.timestamp_ms[i]},"
                    f"{survey.position_mm[i]:.1f},"
                    f"{survey.accel_x[i]:.6f},"
                    f"{survey.accel_y[i]:.6f},"
                    f"{survey.accel_z[i]:.6f},"
                    f"{survey.gyro_x[i]:.4f},"
                    f"{survey.gyro_y[i]:.4f},"
                    f"{survey.gyro_z[i]:.4f},"
                    f"{survey.temperature[i]:.1f}")
            if survey.has_imu2:
                line += (f",{survey.accel_x2[i]:.6f},"
                         f"{survey.accel_y2[i]:.6f},"
                         f"{survey.accel_z2[i]:.6f},"
                         f"{survey.gyro_x2[i]:.4f},"
                         f"{survey.gyro_y2[i]:.4f},"
                         f"{survey.gyro_z2[i]:.4f}")
            f.write(line + "\n")
    print(f"  Saved survey_data.csv ({len(survey.timestamp_ms)} rows)")


# ===== Main =====

def main():
    parser = argparse.ArgumentParser(
        description="Track Geometry Car — Survey Analysis",
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog="Reads binary (.bin) or browser-captured JSON (.json) survey files\n"
               "and produces a track quality report with visualizations.",
    )
    parser.add_argument("survey_file", help="Path to survey file (.bin or .json)")
    parser.add_argument("--speed", type=float, default=DEFAULT_SPEED_MPH,
                        help=f"Assumed constant scale speed in mph (default: {DEFAULT_SPEED_MPH})")
    parser.add_argument("--scale", type=float, default=DEFAULT_SCALE,
                        help=f"Scale factor (default: {DEFAULT_SCALE} for HO)")
    parser.add_argument("--output", type=str, default="./report",
                        help="Output directory for reports (default: ./report)")
    parser.add_argument("--threshold-vert", type=float, default=0.15,
                        help="Vertical event threshold in g (default: 0.15)")
    parser.add_argument("--threshold-lat", type=float, default=0.10,
                        help="Lateral event threshold in g (default: 0.10)")
    parser.add_argument("--threshold-twist", type=float, default=5.0,
                        help="Twist rate threshold in °/s (default: 5.0)")
    parser.add_argument("--threshold-pitch-rate", type=float, default=3.0,
                        help="Pitch rate threshold for vertical curves in °/s (default: 3.0)")
    parser.add_argument("--section-len", type=float, default=DEFAULT_SECTION_LEN_MM,
                        help=f"Section length in mm (default: {DEFAULT_SECTION_LEN_MM})")
    parser.add_argument("--no-plots", action="store_true",
                        help="Skip generating plot images")
    parser.add_argument("--compare", type=str, default=None,
                        help="Second survey file for before/after comparison overlay")

    args = parser.parse_args()

    filepath = Path(args.survey_file)
    if not filepath.exists():
        print(f"Error: file not found: {filepath}", file=sys.stderr)
        sys.exit(1)

    output_dir = Path(args.output)
    output_dir.mkdir(parents=True, exist_ok=True)

    thresholds = {
        "vert": args.threshold_vert,
        "lat": args.threshold_lat,
        "twist": args.threshold_twist,
        "pitch_rate": args.threshold_pitch_rate,
    }

    # Load survey
    print(f"Loading {filepath}...")
    survey = load_survey(filepath)

    # Compute positions
    compute_positions(survey, args.speed, args.scale)
    print(f"Estimated distance: {survey.position_mm[-1]:.0f} mm "
          f"({survey.position_mm[-1] / 25.4:.1f}\" / "
          f"{survey.position_mm[-1] / 304.8:.1f} ft) "
          f"at {args.speed} scale mph")

    # Detect sections
    print("Analyzing sections...")
    sections = detect_sections(survey, args.section_len, thresholds)
    print(f"  {len(sections)} sections: "
          f"{sum(1 for s in sections if s.type == 'straight')} straight, "
          f"{sum(1 for s in sections if s.type == 'curve')} curve, "
          f"{sum(1 for s in sections if s.type == 'grade')} grade")

    # Detect events
    print("Detecting events...")
    events = detect_events(survey, thresholds)
    print(f"  {len(events)} events detected")

    # Analyze curves
    curves = analyze_curves(survey, sections)
    print(f"  {len(curves)} curve(s) identified")

    # Analyze grades
    grades_info = analyze_grades(survey, sections)
    print(f"  {len(grades_info)} grade region(s) identified")

    # Detect vertical curves
    vcurves = detect_vertical_curves(survey, thresholds)
    if vcurves:
        print(f"  {len(vcurves)} vertical curve(s) detected")
        # Add vertical curves as events too
        for vc in vcurves:
            events.append(Event(
                type="vertical_curve",
                position_mm=vc.position_mm,
                time_ms=0,
                severity=vc.severity,
                description=vc.note,
                details={
                    "vc_type": vc.type,
                    "radius_mm": vc.radius_mm,
                    "radius_inches": round(vc.radius_mm / 25.4, 1),
                    "peak_pitch_rate_dps": vc.peak_pitch_rate_dps,
                    "grade_before_pct": vc.grade_before_pct,
                    "grade_after_pct": vc.grade_after_pct,
                },
            ))
        events.sort(key=lambda e: e.position_mm)

    # Spectral analysis
    print("Computing spectrum...")
    spectrum = compute_spectrum(survey)

    # Assign events to sections
    for s in sections:
        s.events = [e for e in events if s.start_mm <= e.position_mm < s.end_mm]

    # Generate report
    print("Generating report...")
    report = generate_report(survey, sections, events, curves, grades_info, vcurves, spectrum)
    report_path = output_dir / "report.json"
    with open(report_path, "w") as f:
        json.dump(report, f, indent=2)
    print(f"  Saved report.json")

    # Export CSV
    export_csv(survey, output_dir)

    # Generate plots
    if not args.no_plots:
        print("Generating plots...")
        compare_survey = None
        if args.compare:
            compare_path = Path(args.compare)
            if compare_path.exists():
                compare_survey = load_survey(compare_path)
                compute_positions(compare_survey, args.speed, args.scale)
            else:
                print(f"  Warning: comparison file not found: {compare_path}", file=sys.stderr)
        generate_plots(survey, sections, events, curves, vcurves, spectrum, output_dir, compare_survey)

    # Print summary to console
    print()
    print(f"{'='*60}")
    print(f"TRACK QUALITY REPORT")
    print(f"{'='*60}")
    print(f"File:     {filepath.name}")
    print(f"Duration: {report['duration_sec']}s ({report['total_samples']} samples)")
    print(f"Distance: {report['distance_estimated_mm']:.0f} mm "
          f"({report['distance_estimated_mm'] / 304.8:.1f} ft)")
    print(f"Overall:  {report['overall_grade']}")
    print()

    grade_list = [s.grade for s in sections]
    for g in ["A", "B", "C", "D"]:
        count = grade_list.count(g)
        if count > 0:
            bar = "#" * count
            print(f"  {g}: {bar} ({count})")

    if events:
        print()
        print(f"Events ({len(events)} total):")
        for etype, count in sorted(report["event_summary"].items()):
            if etype != "total_events":
                print(f"  {etype}: {count}")

    if curves:
        print()
        print("Curves:")
        for c in curves:
            parts = [f"  {c.direction:>5} {c.avg_radius_mm / 25.4:5.1f}\" "
                     f"({c.start_mm:.0f}–{c.end_mm:.0f}mm) "
                     f"consistency={c.consistency:.2f}"]
            if c.kinks > 0:
                parts.append(f"  {c.kinks} kink(s)")
            transitions = []
            if c.entry_transition.present:
                transitions.append(f"entry:{c.entry_transition.type} {c.entry_transition.length_mm:.0f}mm")
            if c.exit_transition.present:
                transitions.append(f"exit:{c.exit_transition.type} {c.exit_transition.length_mm:.0f}mm")
            if transitions:
                parts.append(f"  [{', '.join(transitions)}]")
            else:
                parts.append("  [abrupt entry/exit]")
            print("".join(parts))

    if grades_info:
        print()
        print("Grades:")
        for g in grades_info:
            print(f"  {g.direction:>10} {g.avg_grade_percent:4.1f}% "
                  f"({g.start_mm:.0f}–{g.end_mm:.0f}mm)")

    if vcurves:
        print()
        print("Vertical curves:")
        for vc in vcurves:
            severity_tag = f" [{vc.severity.upper()}]" if vc.severity != "minor" else ""
            print(f"  {vc.type:>5} R={vc.radius_mm / 25.4:5.1f}\" "
                  f"at {vc.position_mm:.0f}mm "
                  f"({vc.grade_before_pct:+.1f}% → {vc.grade_after_pct:+.1f}%) "
                  f"pitch {vc.peak_pitch_rate_dps:.1f}°/s{severity_tag}")

    print()
    print(f"Report: {report_path}")
    print(f"Output: {output_dir}/")


if __name__ == "__main__":
    main()
