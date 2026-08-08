#!/usr/bin/env python3
"""Parse UART3 0x83 telemetry and analyze track modes 3 and 4."""

from __future__ import annotations

import argparse
import csv
import struct
from dataclasses import dataclass
from pathlib import Path

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np


MAGIC = b"\xA5\x5A"
FRAME_FORMAT = "<2sBBBBBBHHIhhhH"
FRAME_SIZE = struct.calcsize(FRAME_FORMAT)
WHEEL_DIAMETER_M = 0.065
RPM_TO_MPS = np.pi * WHEEL_DIAMETER_M / 60.0
IMU_INVALID = -32768
BUTTON_NAMES = {0: "NONE", 1: "SW1", 2: "SW2", 3: "SW3", 4: "SW4"}


@dataclass
class ParseStats:
    total_bytes: int = 0
    frames: int = 0
    crc_errors: int = 0
    header_errors: int = 0
    discarded_bytes: int = 0
    sequence_gaps: int = 0


@dataclass
class MotionSegment:
    mode: int
    trial: int
    start: int
    stop: int
    moving_start: int
    moving_stop: int


def crc16_ccitt_false(data: bytes) -> int:
    crc = 0xFFFF
    for value in data:
        crc ^= value << 8
        for _ in range(8):
            crc = ((crc << 1) ^ 0x1021) & 0xFFFF if crc & 0x8000 else (crc << 1) & 0xFFFF
    return crc


def load_raw_stream(session_dir: Path) -> bytes:
    files = sorted(session_dir.glob("mcu_uart_*.bin"))
    if not files:
        raise FileNotFoundError(f"no mcu_uart_*.bin files in {session_dir}")
    return b"".join(path.read_bytes() for path in files)


def parse_stream(raw: bytes) -> tuple[dict[str, np.ndarray], ParseStats]:
    fields: dict[str, list[int]] = {
        "flags": [],
        "last_button": [],
        "active_button": [],
        "button_sequence": [],
        "sequence": [],
        "mcu_ms": [],
        "imu_accel_mm_s2": [],
        "left_rpm_x10": [],
        "right_rpm_x10": [],
    }
    stats = ParseStats(total_bytes=len(raw))
    offset = 0
    previous_sequence: int | None = None
    while offset < len(raw):
        magic_at = raw.find(MAGIC, offset)
        if magic_at < 0:
            stats.discarded_bytes += len(raw) - offset
            break
        stats.discarded_bytes += magic_at - offset
        offset = magic_at
        if offset + FRAME_SIZE > len(raw):
            break
        frame = raw[offset : offset + FRAME_SIZE]
        if frame[2:5] != bytes((0x01, 0x83, FRAME_SIZE)):
            stats.header_errors += 1
            offset += 1
            continue
        expected_crc = struct.unpack_from("<H", frame, 22)[0]
        if crc16_ccitt_false(frame[:22]) != expected_crc:
            stats.crc_errors += 1
            offset += 1
            continue
        (
            _,
            _,
            _,
            _,
            flags,
            last_button,
            active_button,
            button_sequence,
            sequence,
            mcu_ms,
            imu_accel,
            left_rpm,
            right_rpm,
            _,
        ) = struct.unpack(FRAME_FORMAT, frame)
        if previous_sequence is not None:
            delta = (sequence - previous_sequence) & 0xFFFF
            if 0 < delta < 0x8000:
                stats.sequence_gaps += delta - 1
        previous_sequence = sequence
        values = (
            flags,
            last_button,
            active_button,
            button_sequence,
            sequence,
            mcu_ms,
            imu_accel,
            left_rpm,
            right_rpm,
        )
        for name, value in zip(fields, values):
            fields[name].append(value)
        stats.frames += 1
        offset += FRAME_SIZE
    if not fields["mcu_ms"]:
        raise ValueError("no valid UART3 operational telemetry frames found")
    return {name: np.asarray(values) for name, values in fields.items()}, stats


def unwrap_mcu_time(mcu_ms: np.ndarray) -> np.ndarray:
    raw = mcu_ms.astype(np.uint64)
    delta = ((raw[1:] - raw[:-1]) & np.uint64(0xFFFFFFFF)).astype(float)
    elapsed_ms = np.r_[0.0, np.cumsum(delta)]
    return elapsed_ms / 1000.0


def moving_average(values: np.ndarray, samples: int) -> np.ndarray:
    samples = max(1, int(samples))
    if samples == 1:
        return values.astype(float, copy=True)
    kernel = np.ones(samples, dtype=float) / samples
    padded = np.pad(values.astype(float), (samples // 2, samples - 1 - samples // 2), mode="edge")
    return np.convolve(padded, kernel, mode="valid")


def fill_short_false_regions(mask: np.ndarray, max_samples: int) -> np.ndarray:
    output = mask.astype(bool, copy=True)
    edges = np.flatnonzero(np.r_[True, output[1:] != output[:-1], True])
    for start, stop in zip(edges[:-1], edges[1:]):
        if not output[start] and start > 0 and stop < output.size and stop - start <= max_samples:
            output[start:stop] = True
    return output


def true_regions(mask: np.ndarray) -> list[tuple[int, int]]:
    padded = np.r_[False, mask.astype(bool), False]
    edges = np.flatnonzero(padded[1:] != padded[:-1])
    return [(int(start), int(stop)) for start, stop in edges.reshape(-1, 2)]


def detect_motion_segments(t: np.ndarray, center_speed: np.ndarray) -> list[tuple[int, int]]:
    sample_rate = 1.0 / float(np.median(np.diff(t)))
    envelope = moving_average(np.abs(center_speed), round(0.12 * sample_rate))
    moving = envelope >= 0.025
    moving = fill_short_false_regions(moving, round(0.35 * sample_rate))
    regions = [
        (start, stop)
        for start, stop in true_regions(moving)
        if t[stop - 1] - t[start] >= 0.8
    ]
    return regions


def local_polynomial(
    t: np.ndarray,
    values: np.ndarray,
    half_window_s: float,
    degree: int = 2,
    derivative: int = 0,
) -> np.ndarray:
    output = np.full(values.shape, np.nan, dtype=float)
    finite = np.isfinite(values)
    for index, center in enumerate(t):
        selected = finite & (np.abs(t - center) <= half_window_s)
        if np.count_nonzero(selected) < degree + 2:
            continue
        x = (t[selected] - center) / half_window_s
        distance = np.abs(x)
        weights = (1.0 - distance**3) ** 3
        design = np.column_stack([x**power for power in range(degree + 1)])
        weighted_design = design * np.sqrt(weights)[:, None]
        weighted_values = values[selected] * np.sqrt(weights)
        coefficients, *_ = np.linalg.lstsq(weighted_design, weighted_values, rcond=None)
        if derivative == 0:
            output[index] = coefficients[0]
        elif derivative == 1:
            output[index] = coefficients[1] / half_window_s
        else:
            output[index] = 2.0 * coefficients[2] / (half_window_s**2)
    return output


def button_events(data: dict[str, np.ndarray], t: np.ndarray) -> list[tuple[float, int, int]]:
    changes = np.flatnonzero(data["button_sequence"][1:] != data["button_sequence"][:-1]) + 1
    return [
        (float(t[index]), int(data["last_button"][index]), int(data["button_sequence"][index]))
        for index in changes
    ]


def assign_modes(
    regions: list[tuple[int, int]],
    t: np.ndarray,
    events: list[tuple[float, int, int]],
    first_mode: int,
    explicit_modes: list[int] | None,
) -> list[MotionSegment]:
    if explicit_modes is not None:
        if len(explicit_modes) != len(regions):
            raise ValueError(
                f"--modes supplied {len(explicit_modes)} labels but "
                f"detected {len(regions)} movement segments"
            )
        modes = explicit_modes
    else:
        modes = []
        current_mode = first_mode
        previous_stop_time = float(t[0])
        for region_index, (moving_start, _) in enumerate(regions):
            if region_index > 0:
                for event_t, button, _ in events:
                    if previous_stop_time < event_t < float(t[moving_start]):
                        if button == 1:
                            current_mode = current_mode % 5 + 1
                        elif button == 2:
                            current_mode = (current_mode + 3) % 5 + 1
            modes.append(current_mode)
            previous_stop_time = float(t[regions[region_index][1] - 1])
    sample_rate = 1.0 / float(np.median(np.diff(t)))
    padding = round(0.7 * sample_rate)
    trials: dict[int, int] = {}
    segments: list[MotionSegment] = []
    for mode, (moving_start, moving_stop) in zip(modes, regions):
        trials[mode] = trials.get(mode, 0) + 1
        segments.append(
            MotionSegment(
                mode=mode,
                trial=trials[mode],
                start=max(0, moving_start - padding),
                stop=min(t.size, moving_stop + padding),
                moving_start=moving_start,
                moving_stop=moving_stop,
            )
        )
    return segments


def segment_key(segment: MotionSegment) -> tuple[int, int]:
    return segment.mode, segment.trial


def segment_label(segment: MotionSegment) -> str:
    return f"Mode {segment.mode}, trial {segment.trial}"


def imu_baseline(
    data: dict[str, np.ndarray],
    center_speed: np.ndarray,
    segment: MotionSegment,
    sample_rate: float,
) -> float:
    start = max(0, segment.moving_start - round(1.5 * sample_rate))
    stop = segment.moving_start
    valid = (
        (data["imu_accel_mm_s2"][start:stop] != IMU_INVALID)
        & (np.abs(center_speed[start:stop]) < 0.02)
    )
    candidates = data["imu_accel_mm_s2"][start:stop][valid].astype(float) / 1000.0
    if candidates.size < 10:
        global_valid = (data["imu_accel_mm_s2"] != IMU_INVALID) & (np.abs(center_speed) < 0.02)
        candidates = data["imu_accel_mm_s2"][global_valid].astype(float) / 1000.0
    return float(np.median(candidates)) if candidates.size else 0.0


def derive_segment(
    data: dict[str, np.ndarray],
    t: np.ndarray,
    center_speed: np.ndarray,
    segment: MotionSegment,
) -> dict[str, np.ndarray | float]:
    index = slice(segment.start, segment.stop)
    local_t = t[index] - t[segment.moving_start]
    sample_rate = 1.0 / float(np.median(np.diff(t)))
    baseline = imu_baseline(data, center_speed, segment, sample_rate)
    imu = data["imu_accel_mm_s2"][index].astype(float) / 1000.0
    imu[data["imu_accel_mm_s2"][index] == IMU_INVALID] = np.nan
    imu -= baseline
    speed = center_speed[index]
    encoder_accel_raw = np.gradient(moving_average(speed, round(0.05 * sample_rate)), local_t)
    encoder_speed_fit = local_polynomial(local_t, speed, 0.18, derivative=0)
    encoder_accel_fit = local_polynomial(local_t, speed, 0.18, derivative=1)
    imu_fit = local_polynomial(local_t, imu, 0.22, derivative=0)
    finite = np.isfinite(imu_fit) & np.isfinite(encoder_accel_fit)
    dynamic = finite & (
        (np.abs(encoder_accel_fit) >= 0.03) | (np.abs(imu_fit) >= 0.06)
    )
    calibration_mask = dynamic if np.count_nonzero(dynamic) >= 20 else finite
    calibration_design = np.column_stack(
        [imu_fit[calibration_mask], np.ones(np.count_nonzero(calibration_mask))]
    )
    imu_gain, imu_fit_bias = np.linalg.lstsq(
        calibration_design,
        encoder_accel_fit[calibration_mask],
        rcond=None,
    )[0]
    imu_gain = float(np.clip(imu_gain, 0.0, 2.0))
    imu_fit_bias = float(imu_fit_bias)
    imu_calibrated_fit = imu_gain * imu_fit + imu_fit_bias
    fitted_accel = 0.5 * (encoder_accel_fit + imu_calibrated_fit)
    return {
        "t": local_t,
        "left_rpm": data["left_rpm_x10"][index].astype(float) / 10.0,
        "right_rpm": data["right_rpm_x10"][index].astype(float) / 10.0,
        "left_speed": data["left_rpm_x10"][index].astype(float) / 10.0 * RPM_TO_MPS,
        "right_speed": data["right_rpm_x10"][index].astype(float) / 10.0 * RPM_TO_MPS,
        "center_speed": speed,
        "encoder_speed_fit": encoder_speed_fit,
        "imu_accel": imu,
        "encoder_accel_raw": encoder_accel_raw,
        "imu_accel_fit": imu_fit,
        "encoder_accel_fit": encoder_accel_fit,
        "imu_accel_calibrated_fit": imu_calibrated_fit,
        "fitted_accel": fitted_accel,
        "imu_baseline": baseline,
        "imu_fit_gain": imu_gain,
        "imu_fit_bias": imu_fit_bias,
    }


def focus_windows(derived: dict[str, np.ndarray | float]) -> list[tuple[float, float, str]]:
    t = np.asarray(derived["t"])
    speed = np.asarray(derived["encoder_speed_fit"])
    peak = float(np.nanmax(np.abs(speed)))
    moving = np.abs(speed) >= max(0.025, 0.15 * peak)
    regions = true_regions(moving)
    if not regions:
        return [(float(t[0]), float(t[-1]), "motion")]
    start = regions[0][0]
    stop = regions[-1][1] - 1
    return [
        (max(float(t[0]), float(t[start]) - 0.45), min(float(t[-1]), float(t[start]) + 1.5), "start acceleration"),
        (max(float(t[0]), float(t[stop]) - 1.5), min(float(t[-1]), float(t[stop]) + 0.45), "braking / stop"),
    ]


def add_button_markers(ax: plt.Axes, events: list[tuple[float, int, int]], origin: float, left: float, right: float) -> None:
    for event_t, button, sequence in events:
        local_t = event_t - origin
        if left <= local_t <= right:
            ax.axvline(local_t, color="0.25", lw=0.7, ls=":", alpha=0.7)
            ax.text(local_t, 0.97, f"{BUTTON_NAMES.get(button, button)} #{sequence}", transform=ax.get_xaxis_transform(), rotation=90, va="top", ha="right", fontsize=7)


def plot_mode(
    path: Path,
    segment: MotionSegment,
    derived: dict[str, np.ndarray | float],
    events: list[tuple[float, int, int]],
    origin: float,
) -> None:
    t = np.asarray(derived["t"])
    fig, axes = plt.subplots(3, 1, figsize=(15, 11), sharex=True, constrained_layout=True)
    axes[0].plot(t, derived["left_speed"], label="left wheel", lw=0.9, alpha=0.8)
    axes[0].plot(t, derived["right_speed"], label="right wheel", lw=0.9, alpha=0.8)
    axes[0].plot(t, derived["encoder_speed_fit"], label="vehicle center fit", lw=2.0, color="black")
    axes[0].set_ylabel("encoder speed (m/s)")
    axes[0].legend(ncol=3)

    axes[1].plot(t, derived["imu_accel"], color="tab:orange", alpha=0.25, lw=0.7, label="IMU raw, baseline removed")
    axes[1].plot(t, derived["encoder_accel_raw"], color="tab:blue", alpha=0.22, lw=0.7, label="encoder derivative")
    axes[1].plot(t, derived["imu_accel_fit"], color="tab:orange", lw=1.8, label="IMU local fit")
    axes[1].plot(t, derived["encoder_accel_fit"], color="tab:blue", lw=1.8, label="encoder local fit")
    axes[1].axhline(0.0, color="0.3", lw=0.7)
    axes[1].set_ylabel("acceleration (m/s^2)")
    axes[1].legend(ncol=2)

    axes[2].plot(t, derived["fitted_accel"], color="tab:red", lw=2.1, label="fused fitted acceleration")
    axes[2].plot(t, derived["imu_accel_calibrated_fit"], color="tab:purple", lw=1.1, ls="--", alpha=0.8, label="IMU fit calibrated to encoder")
    axes[2].fill_between(t, 0.0, derived["fitted_accel"], color="tab:red", alpha=0.18)
    axes[2].axhline(0.0, color="0.3", lw=0.7)
    axes[2].set_ylabel("fitted accel (m/s^2)")
    axes[2].set_xlabel("time from detected motion start (s)")
    axes[2].legend()
    for ax in axes:
        add_button_markers(ax, events, origin, float(t[0]), float(t[-1]))
        ax.grid(True, alpha=0.25)
    fig.suptitle(f"Track {segment_label(segment)}: encoder speed and vehicle acceleration")
    fig.savefig(path, dpi=180)
    plt.close(fig)


def plot_acceleration_focus(
    path: Path,
    segments: list[MotionSegment],
    derived_by_segment: dict[tuple[int, int], dict[str, np.ndarray | float]],
) -> None:
    fig, axes = plt.subplots(2, len(segments), figsize=(8 * len(segments), 8), constrained_layout=True)
    if len(segments) == 1:
        axes = np.asarray(axes).reshape(2, 1)
    for column, segment in enumerate(segments):
        derived = derived_by_segment[segment_key(segment)]
        t = np.asarray(derived["t"])
        for row, (left, right, label) in enumerate(focus_windows(derived)):
            mask = (t >= left) & (t <= right)
            ax = axes[row, column]
            ax.plot(t[mask], np.asarray(derived["imu_accel"])[mask], color="tab:orange", alpha=0.22, lw=0.7, label="IMU raw")
            ax.plot(t[mask], np.asarray(derived["encoder_accel_raw"])[mask], color="tab:blue", alpha=0.20, lw=0.7, label="encoder derivative")
            ax.plot(t[mask], np.asarray(derived["imu_accel_fit"])[mask], color="tab:orange", lw=2.0, label="IMU fit")
            ax.plot(t[mask], np.asarray(derived["encoder_accel_fit"])[mask], color="tab:blue", lw=2.0, label="encoder fit")
            ax.plot(t[mask], np.asarray(derived["imu_accel_calibrated_fit"])[mask], color="tab:purple", lw=1.5, ls="--", label="calibrated IMU fit")
            ax.plot(t[mask], np.asarray(derived["fitted_accel"])[mask], color="tab:red", lw=2.2, label="fused fit")
            ax.axhline(0.0, color="0.3", lw=0.7)
            ax.set_title(f"{segment_label(segment)}: {label}")
            ax.set_xlabel("time from motion start (s)")
            ax.set_ylabel("acceleration (m/s^2)")
            ax.grid(True, alpha=0.25)
            ax.legend(fontsize=8, ncol=2)
    fig.suptitle("Magnified acceleration transitions and fitted curves")
    fig.savefig(path, dpi=190)
    plt.close(fig)


def plot_normalized_fits(
    path: Path,
    segments: list[MotionSegment],
    derived_by_segment: dict[tuple[int, int], dict[str, np.ndarray | float]],
) -> None:
    fig, axes = plt.subplots(len(segments), 1, figsize=(15, 4.5 * len(segments)), constrained_layout=True)
    if len(segments) == 1:
        axes = [axes]
    for ax, segment in zip(axes, segments):
        derived = derived_by_segment[segment_key(segment)]
        t = np.asarray(derived["t"])
        speed = np.asarray(derived["encoder_speed_fit"])
        moving = np.abs(speed) >= max(0.025, 0.1 * float(np.nanmax(np.abs(speed))))
        regions = true_regions(moving)
        start, stop = (regions[0][0], regions[-1][1]) if regions else (0, t.size)
        x = (t[start:stop] - t[start]) / max(t[stop - 1] - t[start], 1e-6)
        ax.plot(x, np.asarray(derived["imu_accel_fit"])[start:stop], label="IMU fitted, baseline only", lw=1.0, alpha=0.45)
        ax.plot(x, np.asarray(derived["imu_accel_calibrated_fit"])[start:stop], label="IMU fitted, encoder-calibrated", lw=1.7, color="tab:purple")
        ax.plot(x, np.asarray(derived["encoder_accel_fit"])[start:stop], label="encoder fitted", lw=1.7)
        ax.plot(x, np.asarray(derived["fitted_accel"])[start:stop], label="fused vehicle acceleration fit", lw=2.3, color="tab:red")
        ax.axhline(0.0, color="0.3", lw=0.7)
        ax.set_ylabel("acceleration (m/s^2)")
        ax.set_xlabel("normalized movement progress")
        ax.set_title(f"{segment_label(segment)} acceleration curve fit")
        ax.grid(True, alpha=0.25)
        ax.legend(ncol=2)
    fig.savefig(path, dpi=180)
    plt.close(fig)


def write_segment_csv(path: Path, derived: dict[str, np.ndarray | float]) -> None:
    columns = [
        "t_s",
        "left_rpm",
        "right_rpm",
        "center_speed_mps",
        "encoder_speed_fit_mps",
        "imu_accel_mps2",
        "encoder_accel_raw_mps2",
        "imu_accel_fit_mps2",
        "imu_accel_calibrated_fit_mps2",
        "encoder_accel_fit_mps2",
        "fitted_accel_mps2",
    ]
    keys = [
        "t",
        "left_rpm",
        "right_rpm",
        "center_speed",
        "encoder_speed_fit",
        "imu_accel",
        "encoder_accel_raw",
        "imu_accel_fit",
        "imu_accel_calibrated_fit",
        "encoder_accel_fit",
        "fitted_accel",
    ]
    with path.open("w", newline="", encoding="utf-8") as handle:
        writer = csv.writer(handle)
        writer.writerow(columns)
        for row in zip(*(np.asarray(derived[key]) for key in keys)):
            writer.writerow(f"{float(value):.8g}" for value in row)


def write_summary(
    path: Path,
    stats: ParseStats,
    t: np.ndarray,
    events: list[tuple[float, int, int]],
    regions: list[tuple[int, int]],
    segments: list[MotionSegment],
    derived_by_segment: dict[tuple[int, int], dict[str, np.ndarray | float]],
) -> None:
    lines = [
        f"raw_bytes: {stats.total_bytes}",
        f"valid_frames: {stats.frames}",
        f"duration_s: {t[-1]:.3f}",
        f"crc_errors: {stats.crc_errors}",
        f"header_errors: {stats.header_errors}",
        f"discarded_bytes: {stats.discarded_bytes}",
        f"sequence_gaps: {stats.sequence_gaps}",
        "",
        "button_events:",
    ]
    lines.extend(f"  t={event_t:.3f}s {BUTTON_NAMES.get(button, button)} sequence={sequence}" for event_t, button, sequence in events)
    lines.append("")
    lines.append("detected_motion_segments:")
    lines.extend(f"  {index + 1}: {t[start]:.3f}..{t[stop - 1]:.3f}s duration={t[stop - 1] - t[start]:.3f}s" for index, (start, stop) in enumerate(regions))
    for segment in segments:
        derived = derived_by_segment[segment_key(segment)]
        speed = np.asarray(derived["encoder_speed_fit"])
        imu_fit = np.asarray(derived["imu_accel_fit"])
        encoder_fit = np.asarray(derived["encoder_accel_fit"])
        fused_fit = np.asarray(derived["fitted_accel"])
        finite = np.isfinite(imu_fit) & np.isfinite(encoder_fit)
        correlation = float(np.corrcoef(imu_fit[finite], encoder_fit[finite])[0, 1]) if np.count_nonzero(finite) > 2 else float("nan")
        lines.extend(
            [
                "",
                f"mode_{segment.mode}_trial_{segment.trial}:",
                f"  source_time_s: {t[segment.start]:.3f}..{t[segment.stop - 1]:.3f}",
                f"  moving_time_s: {t[segment.moving_start]:.3f}..{t[segment.moving_stop - 1]:.3f}",
                f"  imu_baseline_mps2: {float(derived['imu_baseline']):.4f}",
                f"  peak_abs_speed_mps: {float(np.nanmax(np.abs(speed))):.4f}",
                f"  peak_abs_imu_fit_mps2: {float(np.nanmax(np.abs(imu_fit))):.4f}",
                f"  peak_abs_encoder_fit_mps2: {float(np.nanmax(np.abs(encoder_fit))):.4f}",
                f"  imu_fit_to_encoder_gain: {float(derived['imu_fit_gain']):.4f}",
                f"  imu_fit_to_encoder_bias_mps2: {float(derived['imu_fit_bias']):.4f}",
                f"  imu_encoder_fit_correlation: {correlation:.4f}",
                f"  peak_fused_accel_mps2: {float(np.nanmax(fused_fit)):.4f}",
                f"  peak_fused_braking_mps2: {float(np.nanmin(fused_fit)):.4f}",
            ]
        )
    path.write_text("\n".join(lines) + "\n", encoding="utf-8")


def main() -> None:
    script_dir = Path(__file__).resolve().parent
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("session", type=Path, help="directory containing mcu_uart_*.bin")
    parser.add_argument("--output-dir", type=Path, default=script_dir / "output")
    parser.add_argument("--first-mode", type=int, default=3, choices=range(1, 6), help="mode of the first detected movement; later modes follow SW1/SW2 menu events")
    parser.add_argument("--modes", type=int, nargs="+", default=None, help="explicit label for every detected movement segment; overrides menu inference")
    args = parser.parse_args()

    raw = load_raw_stream(args.session)
    data, stats = parse_stream(raw)
    t = unwrap_mcu_time(data["mcu_ms"])
    left_speed = data["left_rpm_x10"].astype(float) / 10.0 * RPM_TO_MPS
    right_speed = data["right_rpm_x10"].astype(float) / 10.0 * RPM_TO_MPS
    center_speed = 0.5 * (left_speed + right_speed)
    regions = detect_motion_segments(t, center_speed)
    events = button_events(data, t)
    segments = assign_modes(regions, t, events, args.first_mode, args.modes)
    derived_by_segment = {
        segment_key(segment): derive_segment(data, t, center_speed, segment)
        for segment in segments
    }

    args.output_dir.mkdir(parents=True, exist_ok=True)
    for segment in segments:
        derived = derived_by_segment[segment_key(segment)]
        stem = f"mode_{segment.mode}_trial_{segment.trial}"
        plot_mode(
            args.output_dir / f"{stem}_motion.png",
            segment,
            derived,
            events,
            float(t[segment.moving_start]),
        )
        write_segment_csv(args.output_dir / f"{stem}_telemetry.csv", derived)
    plot_acceleration_focus(args.output_dir / "acceleration_focus.png", segments, derived_by_segment)
    plot_normalized_fits(args.output_dir / "acceleration_curve_fit.png", segments, derived_by_segment)
    write_summary(args.output_dir / "summary.txt", stats, t, events, regions, segments, derived_by_segment)
    print(args.output_dir.resolve())


if __name__ == "__main__":
    main()
