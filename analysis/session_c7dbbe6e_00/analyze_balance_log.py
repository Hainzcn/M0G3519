#!/usr/bin/env python3
"""Analyze balance_control_v4 telemetry for session_c7dbbe6e_00.

The script intentionally depends only on NumPy and Matplotlib so it can run
without modifying the firmware project's Python environment.
"""

from __future__ import annotations

import argparse
import csv
from collections import Counter
from pathlib import Path

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np


PHASE_NAMES = {
    0: "hold",
    1: "accel",
    2: "track",
    3: "brake",
    4: "overspeed",
    5: "edge_recovery",
    6: "capture",
}

FRICTION_NAMES = {
    0: "motion",
    1: "breakaway",
    2: "capture",
    3: "stopped",
}

CONTROL_FLAGS = {
    0x0001: "measurement_fresh",
    0x0002: "predict_only",
    0x0004: "edge_recovery",
    0x0008: "dynamics_saturated",
    0x0010: "angle_saturated",
    0x0020: "slew_saturated",
    0x0040: "hard_edge",
    0x0080: "velocity_saturated",
    0x0100: "overspeed_pullback",
    0x0200: "predictor_degraded",
    0x0400: "capture_active",
    0x0800: "breakaway_active",
    0x1000: "calibration_pending",
}

# Event boundaries determined from the state/measurement transitions in this
# session. Times are relative to the first MCU telemetry sample.
EVENTS = {
    "event_1": (4.520, 43.940),
    "event_2": (48.360, 56.871),
}

# Production currently advances the state by this pure-delay setting before
# making braking and capture decisions.
PREDICTION_HORIZON_S = 0.120


def load_csv(path: Path) -> tuple[list[str], dict[str, np.ndarray]]:
    with path.open("r", newline="", encoding="utf-8-sig") as handle:
        reader = csv.DictReader(handle)
        rows = list(reader)
        fields = list(reader.fieldnames or [])

    data: dict[str, np.ndarray] = {}
    for field in fields:
        values: list[float] = []
        for row in rows:
            raw = row[field]
            values.append(np.nan if raw in ("", "None") else float(raw))
        data[field] = np.asarray(values, dtype=float)
    return fields, data


def contiguous_regions(mask: np.ndarray) -> list[tuple[int, int]]:
    """Return inclusive-exclusive regions where mask is true."""
    padded = np.r_[False, mask.astype(bool), False]
    edges = np.flatnonzero(padded[1:] != padded[:-1])
    return [(int(start), int(stop)) for start, stop in edges.reshape(-1, 2)]


def transition_rows(values: np.ndarray) -> np.ndarray:
    return np.r_[0, np.flatnonzero(values[1:] != values[:-1]) + 1]


def bit_fraction(values: np.ndarray, bit: int, mask: np.ndarray | None = None) -> float:
    selected = values.astype(np.int64) if mask is None else values[mask].astype(np.int64)
    if selected.size == 0:
        return float("nan")
    return float(np.mean((selected & bit) != 0))


def dominant_frequency(signal: np.ndarray, sample_rate_hz: float) -> tuple[float, float]:
    finite = np.isfinite(signal)
    x = signal[finite]
    if x.size < 32:
        return float("nan"), float("nan")
    x = x - np.mean(x)
    window = np.hanning(x.size)
    spectrum = np.abs(np.fft.rfft(x * window))
    frequency = np.fft.rfftfreq(x.size, d=1.0 / sample_rate_hz)
    useful = (frequency >= 0.15) & (frequency <= 15.0)
    if not np.any(useful):
        return float("nan"), float("nan")
    local = int(np.argmax(spectrum[useful]))
    index = np.flatnonzero(useful)[local]
    amplitude = 2.0 * spectrum[index] / np.sum(window)
    return float(frequency[index]), float(amplitude)


def sign_change_count(values: np.ndarray, threshold: float) -> int:
    signs = np.zeros(values.size, dtype=np.int8)
    signs[values > threshold] = 1
    signs[values < -threshold] = -1
    signs = signs[signs != 0]
    if signs.size < 2:
        return 0
    return int(np.count_nonzero(signs[1:] != signs[:-1]))


def episode_count(mask: np.ndarray) -> int:
    return len(contiguous_regions(mask))


def max_region_duration(mask: np.ndarray, t: np.ndarray) -> float:
    durations = [t[stop - 1] - t[start] for start, stop in contiguous_regions(mask)]
    return max(durations, default=0.0)


def first_index(mask: np.ndarray) -> int | None:
    indices = np.flatnonzero(mask)
    return int(indices[0]) if indices.size else None


def estimate_tracking_lag(
    command: np.ndarray,
    feedback: np.ndarray,
    valid: np.ndarray,
    sample_rate_hz: float,
    max_lag_s: float = 0.5,
) -> tuple[float, float, float]:
    best_lag = 0
    best_rmse = float("inf")
    zero_rmse = float("nan")
    max_lag_samples = int(round(max_lag_s * sample_rate_hz))
    for lag in range(max_lag_samples + 1):
        if lag == 0:
            mask = valid & np.isfinite(command) & np.isfinite(feedback)
            error = feedback[mask] - command[mask]
        else:
            mask = (
                valid[lag:]
                & valid[:-lag]
                & np.isfinite(feedback[lag:])
                & np.isfinite(command[:-lag])
            )
            error = feedback[lag:][mask] - command[:-lag][mask]
        if error.size < 20:
            continue
        rmse = float(np.sqrt(np.mean(error * error)))
        if lag == 0:
            zero_rmse = rmse
        if rmse < best_rmse:
            best_rmse = rmse
            best_lag = lag
    return best_lag / sample_rate_hz, zero_rmse, best_rmse


def event_landmarks(
    t: np.ndarray, data: dict[str, np.ndarray], start_s: float, stop_s: float
) -> dict[str, int | None]:
    event = (t >= start_s) & (t < stop_s)
    indices = np.flatnonzero(event)
    first = int(indices[0])
    initial_sign = 1.0 if data["estimated_position_dmm"][first] >= 0.0 else -1.0
    return {
        "start": first,
        "first_4mm": first_index(event & (np.abs(data["estimated_position_dmm"]) <= 40.0)),
        "first_crossing": first_index(
            event & (initial_sign * data["estimated_position_dmm"] <= 0.0)
        ),
        "first_overspeed": first_index(event & (data["control_phase"] == 4.0)),
        "first_capture": first_index(event & (data["control_phase"] == 6.0)),
        "first_hold": first_index(
            event
            & (data["control_phase"] == 0.0)
            & (data["friction_mode"] == 3.0)
        ),
    }


def append_metric(
    rows: list[tuple[str, str, float | int | str, str]],
    scope: str,
    name: str,
    value: float | int | str,
    unit: str = "",
) -> None:
    rows.append((scope, name, value, unit))


def compute_metrics(
    t: np.ndarray, data: dict[str, np.ndarray]
) -> list[tuple[str, str, float | int | str, str]]:
    rows: list[tuple[str, str, float | int | str, str]] = []
    sample_rate = 1000.0 / float(np.median(np.diff(data["mcu_ms"])))
    active = data["state"] == 5.0
    flags = data["control_flags"].astype(np.int64)

    vision_changes = np.flatnonzero(data["vision_sequence"][1:] != data["vision_sequence"][:-1]) + 1
    vision_dt = np.diff(t[vision_changes])
    vision_dt = vision_dt[(vision_dt > 0.0) & (vision_dt < 0.2)]
    append_metric(rows, "record", "sample_rate", sample_rate, "Hz")
    append_metric(rows, "record", "telemetry_sequence_gap_sum", int(np.sum(data["sequence_gap"])), "frames")
    append_metric(rows, "record", "fault_samples", int(np.count_nonzero(data["fault"])), "samples")
    append_metric(rows, "vision", "update_interval_median", float(np.median(vision_dt)), "s")
    append_metric(rows, "vision", "update_interval_p95", float(np.percentile(vision_dt, 95)), "s")
    append_metric(rows, "vision", "update_interval_mean", float(np.mean(vision_dt)), "s")
    append_metric(rows, "vision", "mean_update_rate", float(1.0 / np.mean(vision_dt)), "Hz")
    append_metric(rows, "vision", "predict_only_fraction_active", bit_fraction(flags, 0x0002, active), "fraction")

    overspeed_active = active & (data["control_phase"] == 4.0)
    hold_active = active & (data["control_phase"] == 0.0)
    append_metric(rows, "active", "duration", float(np.count_nonzero(active)) / sample_rate, "s")
    append_metric(rows, "active", "overspeed_duration", float(np.count_nonzero(overspeed_active)) / sample_rate, "s")
    append_metric(rows, "active", "overspeed_fraction", float(np.mean(data["control_phase"][active] == 4.0)), "fraction")
    append_metric(rows, "active", "hold_duration", float(np.count_nonzero(hold_active)) / sample_rate, "s")

    post_first_hold = active & (t >= 12.220) & (t < 43.940)
    post_hold_feedforward = data["feedforward_accel_mm_s2"][post_first_hold]
    append_metric(rows, "event_1_post_hold", "duration", float(np.count_nonzero(post_first_hold)) / sample_rate, "s")
    append_metric(rows, "event_1_post_hold", "feedforward_nonzero_fraction", float(np.mean(np.abs(post_hold_feedforward) > 1.0)), "fraction")
    append_metric(rows, "event_1_post_hold", "feedforward_rms", float(np.sqrt(np.mean(post_hold_feedforward * post_hold_feedforward))) / 1000.0, "m/s^2")
    append_metric(rows, "event_1_post_hold", "feedforward_sign_changes_over_20", sign_change_count(post_hold_feedforward, 20.0), "changes")

    capture = active & (data["control_phase"] == 6.0)
    capture_entry = capture & ~np.r_[False, capture[:-1]]
    estimated_velocity = data["estimated_velocity_mm_s"][capture]
    predicted_velocity = data["predicted_velocity_mm_s"][capture]
    velocity_mismatch = estimated_velocity - predicted_velocity
    entry_estimated_velocity = data["estimated_velocity_mm_s"][capture_entry]
    entry_predicted_velocity = data["predicted_velocity_mm_s"][capture_entry]
    entry_velocity_mismatch = entry_estimated_velocity - entry_predicted_velocity
    horizon_samples = int(round(PREDICTION_HORIZON_S * sample_rate))
    entry_indices = np.flatnonzero(capture_entry)
    future_indices = entry_indices + horizon_samples
    forecast_valid = future_indices < t.size
    forecast_valid &= active[future_indices.clip(max=t.size - 1)]
    entry_indices = entry_indices[forecast_valid]
    future_indices = future_indices[forecast_valid]
    forecast_velocity = data["predicted_velocity_mm_s"][entry_indices]
    realized_velocity = data["estimated_velocity_mm_s"][future_indices]
    forecast_error = realized_velocity - forecast_velocity
    append_metric(rows, "capture", "episode_count", episode_count(capture), "episodes")
    append_metric(rows, "capture", "total_duration", float(np.count_nonzero(capture)) / sample_rate, "s")
    append_metric(rows, "capture", "estimated_abs_velocity_median", float(np.median(np.abs(estimated_velocity))), "mm/s")
    append_metric(rows, "capture", "estimated_abs_velocity_p95", float(np.percentile(np.abs(estimated_velocity), 95)), "mm/s")
    append_metric(rows, "capture", "predicted_abs_velocity_max", float(np.max(np.abs(predicted_velocity))), "mm/s")
    append_metric(rows, "capture", "estimated_minus_predicted_velocity_abs_median", float(np.median(np.abs(velocity_mismatch))), "mm/s")
    append_metric(rows, "capture", "estimated_minus_predicted_velocity_abs_p95", float(np.percentile(np.abs(velocity_mismatch), 95)), "mm/s")
    append_metric(rows, "capture", "estimated_velocity_over_4mm_s_fraction", float(np.mean(np.abs(estimated_velocity) > 4.0)), "fraction")
    append_metric(rows, "capture_entry", "count", int(np.count_nonzero(capture_entry)), "entries")
    append_metric(rows, "capture_entry", "estimated_abs_velocity_median", float(np.median(np.abs(entry_estimated_velocity))), "mm/s")
    append_metric(rows, "capture_entry", "estimated_abs_velocity_p95", float(np.percentile(np.abs(entry_estimated_velocity), 95)), "mm/s")
    append_metric(rows, "capture_entry", "predicted_abs_velocity_max", float(np.max(np.abs(entry_predicted_velocity))), "mm/s")
    append_metric(rows, "capture_entry", "estimated_minus_predicted_velocity_abs_median", float(np.median(np.abs(entry_velocity_mismatch))), "mm/s")
    append_metric(rows, "capture_entry", "estimated_velocity_over_4mm_s_fraction", float(np.mean(np.abs(entry_estimated_velocity) > 4.0)), "fraction")
    append_metric(rows, "capture_forecast", "horizon", PREDICTION_HORIZON_S, "s")
    append_metric(rows, "capture_forecast", "valid_entry_count", int(forecast_velocity.size), "entries")
    append_metric(rows, "capture_forecast", "realized_abs_velocity_median", float(np.median(np.abs(realized_velocity))), "mm/s")
    append_metric(rows, "capture_forecast", "realized_abs_velocity_p95", float(np.percentile(np.abs(realized_velocity), 95)), "mm/s")
    append_metric(rows, "capture_forecast", "predicted_to_realized_abs_error_median", float(np.median(np.abs(forecast_error))), "mm/s")
    append_metric(rows, "capture_forecast", "predicted_to_realized_abs_error_p95", float(np.percentile(np.abs(forecast_error), 95)), "mm/s")
    append_metric(rows, "capture_forecast", "realized_velocity_within_4mm_s_fraction", float(np.mean(np.abs(realized_velocity) <= 4.0)), "fraction")

    shaped = data["shaped_lever_cdeg"] / 100.0
    actual = data["actual_lever_cdeg"] / 100.0
    raw = data["raw_lever_cdeg"] / 100.0
    lag_s, zero_rmse, lagged_rmse = estimate_tracking_lag(shaped, actual, active, sample_rate)
    append_metric(rows, "actuator", "best_shaped_to_actual_lag", lag_s, "s")
    append_metric(rows, "actuator", "tracking_rmse_zero_lag", zero_rmse, "deg")
    append_metric(rows, "actuator", "tracking_rmse_best_lag", lagged_rmse, "deg")
    append_metric(rows, "actuator", "raw_to_shaped_abs_error_p95_active", float(np.percentile(np.abs(raw[active] - shaped[active]), 95)), "deg")
    append_metric(rows, "actuator", "slew_saturated_fraction_active", bit_fraction(flags, 0x0020, active), "fraction")
    motor_error = np.abs(data["motor_target_cdeg"] - data["motor_feedback_cdeg"]) / 100.0
    append_metric(rows, "actuator", "motor_target_feedback_error_p95_active", float(np.percentile(motor_error[active], 95)), "motor_deg")
    append_metric(rows, "actuator", "motor_target_feedback_error_max_active", float(np.max(motor_error[active])), "motor_deg")
    append_metric(rows, "actuator", "motor_follow_error_over_5deg_fraction_active", float(np.mean(motor_error[active] > 5.0)), "fraction")

    for event_name, (start_s, stop_s) in EVENTS.items():
        event = (t >= start_s) & (t < stop_s)
        landmarks = event_landmarks(t, data, start_s, stop_s)
        start_index = int(landmarks["start"] or 0)
        initial_position = data["estimated_position_dmm"][start_index] / 10.0
        append_metric(rows, event_name, "start_time", start_s, "s")
        append_metric(rows, event_name, "initial_position", initial_position, "mm")
        for key in ("first_4mm", "first_crossing", "first_overspeed", "first_capture", "first_hold"):
            index = landmarks[key]
            value = float("nan") if index is None else t[index] - start_s
            append_metric(rows, event_name, f"{key}_after_start", value, "s")

        first_center = landmarks["first_4mm"]
        post_start = start_s if first_center is None else t[first_center]
        post = event & (t >= post_start)
        approach = event if first_center is None else event & (t <= t[first_center])
        frequency, amplitude = dominant_frequency(
            data["estimated_position_dmm"][post] / 10.0, sample_rate
        )
        approach_indices = np.flatnonzero(approach)
        phase_transitions = int(
            np.count_nonzero(
                data["control_phase"][approach_indices[1:]]
                != data["control_phase"][approach_indices[:-1]]
            )
        )
        append_metric(rows, event_name, "approach_duration_to_first_4mm", post_start - start_s, "s")
        append_metric(rows, event_name, "approach_control_phase_transitions", phase_transitions, "transitions")
        append_metric(rows, event_name, "approach_feedforward_sign_changes_over_20", sign_change_count(data["feedforward_accel_mm_s2"][approach], 20.0), "changes")
        append_metric(rows, event_name, "approach_desired_accel_sign_changes_over_50", sign_change_count(data["desired_accel_mm_s2"][approach], 50.0), "changes")
        append_metric(rows, event_name, "approach_raw_lever_sign_changes_over_0p2deg", sign_change_count(data["raw_lever_cdeg"][approach] / 100.0, 0.2), "changes")
        append_metric(rows, event_name, "approach_slew_saturated_fraction", bit_fraction(flags, 0x0020, approach), "fraction")
        append_metric(rows, event_name, "approach_reference_position_error_rms", float(np.sqrt(np.mean(((data["reference_position_dmm"][approach] - data["estimated_position_dmm"][approach]) / 10.0) ** 2))), "mm")
        append_metric(rows, event_name, "post_first_arrival_position_mean", float(np.mean(data["estimated_position_dmm"][post] / 10.0)), "mm")
        append_metric(rows, event_name, "post_first_arrival_position_rms", float(np.sqrt(np.mean((data["estimated_position_dmm"][post] / 10.0) ** 2))), "mm")
        append_metric(rows, event_name, "post_first_arrival_position_min", float(np.min(data["estimated_position_dmm"][post] / 10.0)), "mm")
        append_metric(rows, event_name, "post_first_arrival_position_max", float(np.max(data["estimated_position_dmm"][post] / 10.0)), "mm")
        append_metric(rows, event_name, "post_first_arrival_dominant_frequency", frequency, "Hz")
        append_metric(rows, event_name, "post_first_arrival_fft_amplitude", amplitude, "mm")
        settled = event & (np.abs(data["estimated_position_dmm"]) <= 40.0) & (np.abs(data["estimated_velocity_mm_s"]) <= 4.0)
        append_metric(rows, event_name, "max_continuous_estimated_settle_time", max_region_duration(settled, t), "s")
        append_metric(rows, event_name, "overspeed_episode_count", episode_count(event & (data["control_phase"] == 4.0)), "episodes")
        append_metric(rows, event_name, "capture_episode_count", episode_count(event & (data["control_phase"] == 6.0)), "episodes")
        append_metric(rows, event_name, "hold_episode_count", episode_count(event & (data["control_phase"] == 0.0)), "episodes")
        append_metric(rows, event_name, "reference_velocity_sign_changes", sign_change_count(data["reference_velocity_mm_s"][event], 2.0), "changes")
        append_metric(rows, event_name, "desired_accel_sign_changes_over_50", sign_change_count(data["desired_accel_mm_s2"][event], 50.0), "changes")
        append_metric(rows, event_name, "overspeed_fraction", float(np.mean(data["control_phase"][event] == 4.0)), "fraction")
        append_metric(rows, event_name, "slew_saturated_fraction", bit_fraction(flags, 0x0020, event), "fraction")
        append_metric(rows, event_name, "angle_saturated_fraction", bit_fraction(flags, 0x0010, event), "fraction")
        event_lag, event_zero_rmse, event_lagged_rmse = estimate_tracking_lag(
            shaped, actual, event & active, sample_rate
        )
        append_metric(rows, event_name, "shaped_to_actual_best_alignment_lag", event_lag, "s")
        append_metric(rows, event_name, "shaped_to_actual_zero_lag_rmse", event_zero_rmse, "deg")
        append_metric(rows, event_name, "shaped_to_actual_best_lag_rmse", event_lagged_rmse, "deg")

    recovery = (t >= 43.940) & (t < 48.360)
    append_metric(rows, "long_recovery", "duration", float(t[np.flatnonzero(recovery)[-1]] - t[np.flatnonzero(recovery)[0]] + 1.0 / sample_rate), "s")
    append_metric(rows, "long_recovery", "vision_age_max", float(np.nanmax(data["vision_age_ms"][recovery])), "ms")
    recovery_delta = np.diff(data["estimated_position_dmm"][recovery] / 10.0)
    append_metric(rows, "long_recovery", "largest_estimated_position_jump", float(np.max(np.abs(recovery_delta))), "mm/sample")
    append_metric(rows, "long_recovery", "state6_fraction", float(np.mean(data["state"][recovery] == 6.0)), "fraction")

    return rows


def write_metrics(
    path: Path, rows: list[tuple[str, str, float | int | str, str]]
) -> None:
    with path.open("w", newline="", encoding="utf-8") as handle:
        writer = csv.writer(handle)
        writer.writerow(["scope", "metric", "value", "unit"])
        for scope, name, value, unit in rows:
            if isinstance(value, float):
                rendered = f"{value:.6g}"
            else:
                rendered = value
            writer.writerow([scope, name, rendered, unit])


def write_transitions(output: Path, t: np.ndarray, data: dict[str, np.ndarray]) -> None:
    keys = ("state", "control_phase", "friction_mode")
    indices = np.unique(np.concatenate([transition_rows(data[key]) for key in keys]))
    with output.open("w", newline="", encoding="utf-8") as handle:
        writer = csv.writer(handle)
        writer.writerow(
            [
                "t_s",
                "mcu_ms",
                "state",
                "control_phase",
                "phase_name",
                "friction_mode",
                "friction_name",
                "position_mm",
                "velocity_mm_s",
                "reference_position_mm",
                "reference_velocity_mm_s",
                "desired_accel_mm_s2",
                "raw_lever_deg",
                "actual_lever_deg",
                "vision_age_ms",
                "confidence",
                "control_flags",
            ]
        )
        for i in indices:
            phase = int(data["control_phase"][i])
            friction = int(data["friction_mode"][i])
            writer.writerow(
                [
                    f"{t[i]:.3f}",
                    int(data["mcu_ms"][i]),
                    int(data["state"][i]),
                    phase,
                    PHASE_NAMES.get(phase, "unknown"),
                    friction,
                    FRICTION_NAMES.get(friction, "unknown"),
                    f"{data['estimated_position_dmm'][i] / 10.0:.3f}",
                    f"{data['estimated_velocity_mm_s'][i]:.3f}",
                    f"{data['reference_position_dmm'][i] / 10.0:.3f}",
                    f"{data['reference_velocity_mm_s'][i]:.3f}",
                    f"{data['desired_accel_mm_s2'][i]:.3f}",
                    f"{data['raw_lever_cdeg'][i] / 100.0:.3f}",
                    f"{data['actual_lever_cdeg'][i] / 100.0:.3f}",
                    data["vision_age_ms"][i],
                    int(data["confidence"][i]),
                    int(data["control_flags"][i]),
                ]
            )


def plot_overview(path: Path, t: np.ndarray, data: dict[str, np.ndarray]) -> None:
    fig, axes = plt.subplots(6, 1, figsize=(16, 15), sharex=True, constrained_layout=True)
    ax = axes[0]
    ax.plot(t, data["estimated_position_dmm"] / 10.0, label="estimated", lw=1.2)
    ax.plot(t, data["predicted_position_dmm"] / 10.0, label="predicted", lw=0.8)
    ax.plot(t, data["reference_position_dmm"] / 10.0, label="reference", lw=1.0)
    ax.plot(t, data["target_position_dmm"] / 10.0, label="target", lw=1.0, ls="--")
    ax.set_ylabel("position (mm)")
    ax.legend(ncol=4, loc="upper right")

    ax = axes[1]
    ax.plot(t, data["estimated_velocity_mm_s"], label="estimated", lw=1.0)
    ax.plot(t, data["predicted_velocity_mm_s"], label="predicted", lw=0.8)
    ax.plot(t, data["reference_velocity_mm_s"], label="reference", lw=1.0)
    ax.axhline(30.0, color="0.5", ls=":", lw=0.8)
    ax.axhline(-30.0, color="0.5", ls=":", lw=0.8)
    ax.set_ylabel("velocity (mm/s)")
    ax.legend(ncol=3, loc="upper right")

    ax = axes[2]
    ax.plot(t, data["feedforward_accel_mm_s2"], label="feedforward", lw=1.0)
    ax.plot(t, data["feedback_accel_mm_s2"], label="feedback", lw=1.0)
    ax.plot(t, data["desired_accel_mm_s2"], label="desired", lw=1.2)
    ax.set_ylabel("accel (mm/s^2)")
    ax.legend(ncol=3, loc="upper right")

    ax = axes[3]
    ax.plot(t, data["raw_lever_cdeg"] / 100.0, label="raw", lw=1.0)
    ax.plot(t, data["shaped_lever_cdeg"] / 100.0, label="rate-shaped", lw=1.0)
    ax.plot(t, data["actual_lever_cdeg"] / 100.0, label="actual", lw=1.0)
    ax.set_ylabel("logical lever (deg)")
    ax.legend(ncol=3, loc="upper right")

    ax = axes[4]
    ax.step(t, data["control_phase"], where="post", label="control phase", lw=1.0)
    ax.step(t, data["friction_mode"], where="post", label="friction mode", lw=1.0)
    ax.step(t, data["state"], where="post", label="app state", lw=1.0)
    ax.set_ylabel("discrete state")
    ax.legend(ncol=3, loc="upper right")

    ax = axes[5]
    ax.plot(t, data["vision_age_ms"], label="vision age", lw=1.0)
    ax.plot(t, data["confidence"], label="confidence", lw=1.0)
    ax.set_ylabel("vision (ms / score)")
    ax.set_xlabel("time since first telemetry sample (s, MCU clock)")
    ax.legend(ncol=2, loc="upper right")
    regions = [
        (0.0, 4.520, "startup / placement", "0.7"),
        (4.520, 8.980, "first return", "tab:blue"),
        (8.980, 43.940, "sustained re-excitation", "tab:red"),
        (43.940, 48.360, "RECOVERY / vision loss", "tab:orange"),
        (48.360, t[-1], "second return", "tab:green"),
    ]
    for left, right, label, color in regions:
        for item in axes:
            item.axvspan(left, right, color=color, alpha=0.035)
        axes[0].text(
            0.5 * (left + right),
            0.98,
            label,
            transform=axes[0].get_xaxis_transform(),
            ha="center",
            va="top",
            fontsize=8,
            color=color,
        )
    for item in axes:
        item.grid(True, alpha=0.25)
    fig.suptitle("Balance control telemetry overview")
    fig.savefig(path, dpi=160)
    plt.close(fig)


def plot_event_comparison(path: Path, t: np.ndarray, data: dict[str, np.ndarray]) -> None:
    plot_windows = {
        "Event 1": (3.520, 18.520, EVENTS["event_1"][0]),
        "Recovery and Event 2": (43.400, 56.871, EVENTS["event_2"][0]),
    }
    fig, axes = plt.subplots(4, 2, figsize=(17, 13), constrained_layout=True)
    for column, (title, (left, right, active_start)) in enumerate(plot_windows.items()):
        mask = (t >= left) & (t <= right)
        x = t[mask] - active_start
        panels = axes[:, column]
        panels[0].plot(x, data["estimated_position_dmm"][mask] / 10.0, label="estimated", lw=1.3)
        panels[0].plot(x, data["reference_position_dmm"][mask] / 10.0, label="reference", lw=1.0)
        panels[0].axhspan(-4.0, 4.0, color="0.8", alpha=0.35, label="capture position band")
        panels[0].set_ylabel("position (mm)")

        panels[1].plot(x, data["estimated_velocity_mm_s"][mask], label="estimated", lw=1.0)
        panels[1].plot(x, data["predicted_velocity_mm_s"][mask], label="predicted", lw=1.0)
        panels[1].plot(x, data["reference_velocity_mm_s"][mask], label="reference", lw=1.0)
        panels[1].axhspan(-4.0, 4.0, color="0.8", alpha=0.35, label="capture velocity band")
        panels[1].set_ylabel("velocity (mm/s)")

        panels[2].plot(x, data["feedforward_accel_mm_s2"][mask], label="feedforward", lw=1.0)
        panels[2].plot(x, data["feedback_accel_mm_s2"][mask], label="feedback", lw=1.0)
        panels[2].plot(x, data["desired_accel_mm_s2"][mask], label="desired", lw=1.1)
        overspeed = data["control_phase"][mask] == 4.0
        panels[2].fill_between(x, -450.0, 450.0, where=overspeed, color="tab:red", alpha=0.08, label="overspeed mode")
        capture = data["control_phase"][mask] == 6.0
        panels[2].fill_between(x, -450.0, 450.0, where=capture, color="tab:green", alpha=0.10, label="capture mode")
        panels[2].set_ylim(-460, 460)
        panels[2].set_ylabel("accel (mm/s^2)")

        panels[3].plot(x, data["raw_lever_cdeg"][mask] / 100.0, label="raw", lw=1.0)
        panels[3].plot(x, data["shaped_lever_cdeg"][mask] / 100.0, label="rate-shaped", lw=1.0)
        panels[3].plot(x, data["actual_lever_cdeg"][mask] / 100.0, label="actual", lw=1.0)
        panels[3].set_ylabel("logical lever (deg)")
        panels[3].set_xlabel("time from ACTIVE entry (s)")

        for row, panel in enumerate(panels):
            panel.axvline(0.0, color="black", ls="--", lw=0.9)
            recovery = data["state"][mask] == 6.0
            panel.fill_between(
                x,
                panel.get_ylim()[0],
                panel.get_ylim()[1],
                where=recovery,
                color="tab:orange",
                alpha=0.045,
                label="RECOVERY" if row == 0 else None,
            )
            panel.grid(True, alpha=0.25)
            panel.legend(loc="upper right", ncol=2 if row != 1 else 3, fontsize=8)
        panels[0].set_title(title)
    fig.suptitle("Two ball-return events: return followed by repeated re-excitation")
    fig.savefig(path, dpi=170)
    plt.close(fig)


def spectrum(signal: np.ndarray, sample_rate_hz: float) -> tuple[np.ndarray, np.ndarray]:
    x = signal - np.mean(signal)
    window = np.hanning(x.size)
    amplitude = 2.0 * np.abs(np.fft.rfft(x * window)) / np.sum(window)
    frequency = np.fft.rfftfreq(x.size, d=1.0 / sample_rate_hz)
    return frequency, amplitude


def plot_oscillation(path: Path, t: np.ndarray, data: dict[str, np.ndarray]) -> None:
    sample_rate = 1000.0 / float(np.median(np.diff(data["mcu_ms"])))
    windows = {
        "Event 1 after first arrival": (8.980, 43.500),
        "Event 2 after first arrival": (52.000, 56.871),
    }
    fig, axes = plt.subplots(2, 2, figsize=(16, 9), constrained_layout=True)
    for row, (title, (left, right)) in enumerate(windows.items()):
        mask = (t >= left) & (t < right)
        position = data["estimated_position_dmm"][mask] / 10.0
        axes[row, 0].plot(t[mask], position, label="estimated position", lw=1.2)
        axes[row, 0].plot(t[mask], data["reference_position_dmm"][mask] / 10.0, label="reference", lw=0.9)
        axes[row, 0].axhspan(-4, 4, color="0.8", alpha=0.35)
        axes[row, 0].set_title(title)
        axes[row, 0].set_ylabel("position (mm)")
        axes[row, 0].set_xlabel("time (s)")
        axes[row, 0].legend(loc="upper right")
        axes[row, 0].grid(True, alpha=0.25)

        frequency, amplitude = spectrum(position, sample_rate)
        useful = (frequency >= 0.15) & (frequency <= 5.0)
        axes[row, 1].plot(frequency[useful], amplitude[useful], lw=1.2)
        peak = np.flatnonzero(useful)[np.argmax(amplitude[useful])]
        axes[row, 1].axvline(frequency[peak], color="tab:red", ls="--", label=f"peak {frequency[peak]:.2f} Hz")
        axes[row, 1].set_ylabel("FFT amplitude (mm)")
        axes[row, 1].set_xlabel("frequency (Hz)")
        axes[row, 1].legend(loc="upper right")
        axes[row, 1].grid(True, alpha=0.25)
    fig.suptitle("Post-return oscillation and dominant frequency")
    fig.savefig(path, dpi=170)
    plt.close(fig)


def plot_capture_mismatch(path: Path, t: np.ndarray, data: dict[str, np.ndarray]) -> None:
    active = data["state"] == 5.0
    capture = active & (data["control_phase"] == 6.0)
    capture_entry = capture & ~np.r_[False, capture[:-1]]
    sample_rate = 1000.0 / float(np.median(np.diff(data["mcu_ms"])))
    horizon_samples = int(round(PREDICTION_HORIZON_S * sample_rate))
    realized_future = np.full(t.size, np.nan)
    realized_future[:-horizon_samples] = data["estimated_velocity_mm_s"][horizon_samples:]
    forecast_valid = active & np.isfinite(realized_future)
    forecast_valid[:-horizon_samples] &= active[horizon_samples:]
    fig, axes = plt.subplots(1, 2, figsize=(16, 6), constrained_layout=True)

    axes[0].scatter(
        data["predicted_velocity_mm_s"][forecast_valid],
        realized_future[forecast_valid],
        s=7,
        alpha=0.12,
        label="all valid ACTIVE forecasts",
    )
    axes[0].scatter(
        data["predicted_velocity_mm_s"][capture_entry & forecast_valid],
        realized_future[capture_entry & forecast_valid],
        s=20,
        alpha=0.8,
        color="tab:red",
        label="CAPTURE entries",
    )
    axes[0].axvspan(-4, 4, color="0.7", alpha=0.2, label="capture criterion on prediction")
    axes[0].axhspan(-4, 4, color="tab:green", alpha=0.1, label="desired realized band")
    axes[0].set_xlim(-110, 110)
    axes[0].set_ylim(-130, 140)
    axes[0].set_xlabel("predicted velocity (mm/s)")
    axes[0].set_ylabel("estimated velocity 120 ms later (mm/s)")
    axes[0].set_title("120 ms prediction does not match the realized future state")
    axes[0].legend(loc="upper left", fontsize=8)
    axes[0].grid(True, alpha=0.25)

    window = (t >= 16.0) & (t <= 19.6)
    axes[1].plot(t[window], data["estimated_velocity_mm_s"][window], label="estimated now", lw=1.2)
    axes[1].plot(t[window], data["predicted_velocity_mm_s"][window], label="predicted +120 ms", lw=1.2)
    axes[1].plot(t[window], realized_future[window], label="realized +120 ms", lw=1.0, ls="--")
    local_capture = capture[window]
    axes[1].fill_between(t[window], -80, 130, where=local_capture, color="tab:red", alpha=0.14, label="CAPTURE")
    axes[1].axhspan(-4, 4, color="0.7", alpha=0.2)
    axes[1].set_ylim(-80, 130)
    axes[1].set_xlabel("time (s)")
    axes[1].set_ylabel("velocity (mm/s)")
    axes[1].set_title("Example: repeated transient capture entries")
    axes[1].legend(loc="upper right")
    axes[1].grid(True, alpha=0.25)
    fig.savefig(path, dpi=170)
    plt.close(fig)


def plot_actuator_tracking(path: Path, t: np.ndarray, data: dict[str, np.ndarray]) -> None:
    windows = {
        "First target passage": (8.3, 10.6),
        "Repeated center passages": (16.0, 19.7),
    }
    fig, axes = plt.subplots(2, 1, figsize=(15, 8), constrained_layout=True)
    for ax, (title, (left, right)) in zip(axes, windows.items()):
        mask = (t >= left) & (t <= right)
        ax.plot(t[mask], data["raw_lever_cdeg"][mask] / 100.0, label="raw control", lw=1.1)
        ax.plot(t[mask], data["shaped_lever_cdeg"][mask] / 100.0, label="rate-shaped command", lw=1.1)
        ax.step(t[mask], data["actual_lever_cdeg"][mask] / 100.0, where="post", label="100 ms motor feedback", lw=1.1)
        overspeed = data["control_phase"][mask] == 4.0
        ax.fill_between(t[mask], -3.5, 3.5, where=overspeed, color="tab:red", alpha=0.08, label="overspeed mode")
        ax.set_ylim(-3.5, 3.5)
        ax.set_ylabel("logical lever (deg)")
        ax.set_xlabel("time (s)")
        ax.set_title(title)
        ax.grid(True, alpha=0.25)
        ax.legend(loc="upper right", ncol=4, fontsize=8)
    fig.suptitle("Control reversals pass through a slew-limited, delayed actuator")
    fig.savefig(path, dpi=170)
    plt.close(fig)


def write_report(path: Path, t: np.ndarray, data: dict[str, np.ndarray]) -> None:
    dt = np.diff(data["mcu_ms"]) / 1000.0
    sample_rate = 1.0 / float(np.median(dt))
    lines: list[str] = []
    lines.append(f"samples: {t.size}")
    lines.append(f"duration_s: {t[-1] - t[0]:.3f}")
    lines.append(f"median_sample_period_ms: {1000.0 / sample_rate:.3f}")
    lines.append(f"mcu_period_min_max_ms: {np.min(dt) * 1000.0:.1f}, {np.max(dt) * 1000.0:.1f}")
    lines.append(f"sequence_gap_sum: {int(np.sum(data['sequence_gap']))}")
    lines.append(f"fault_nonzero_samples: {int(np.count_nonzero(data['fault']))}")
    lines.append("")

    for key in ("state", "control_phase", "friction_mode", "flags", "control_flags"):
        counts = Counter(data[key].astype(np.int64).tolist())
        lines.append(f"{key}_counts: {dict(sorted(counts.items()))}")

    lines.append("")
    control_flags = data["control_flags"]
    for bit, name in CONTROL_FLAGS.items():
        lines.append(f"control_flag_{name}_fraction: {bit_fraction(control_flags, bit):.4f}")

    lines.append("")
    lines.append("state_transitions:")
    for i in transition_rows(data["state"]):
        lines.append(
            "  "
            f"t={t[i]:.3f}s state={int(data['state'][i])} "
            f"pos={data['estimated_position_dmm'][i] / 10.0:.1f}mm "
            f"vel={data['estimated_velocity_mm_s'][i]:.1f}mm/s "
            f"confidence={int(data['confidence'][i])}"
        )

    lines.append("")
    lines.append("large_position_regions_abs_gt_20mm_duration_ge_0.2s:")
    for start, stop in contiguous_regions(np.abs(data["estimated_position_dmm"]) >= 200.0):
        if t[stop - 1] - t[start] < 0.2:
            continue
        segment = data["estimated_position_dmm"][start:stop] / 10.0
        lines.append(
            "  "
            f"{t[start]:.3f}..{t[stop - 1]:.3f}s duration={t[stop - 1]-t[start]:.3f}s "
            f"min={np.min(segment):.1f}mm max={np.max(segment):.1f}mm"
        )

    delta_pos = np.diff(data["estimated_position_dmm"] / 10.0)
    jump_indices = np.argsort(np.abs(delta_pos))[-20:][::-1] + 1
    lines.append("")
    lines.append("largest_single_sample_position_changes:")
    for i in jump_indices:
        lines.append(
            "  "
            f"t={t[i]:.3f}s delta={delta_pos[i-1]:+.1f}mm "
            f"pos={data['estimated_position_dmm'][i] / 10.0:.1f}mm "
            f"vision_age={data['vision_age_ms'][i]:.0f}ms confidence={data['confidence'][i]:.0f}"
        )

    full_frequency, full_amplitude = dominant_frequency(
        data["estimated_position_dmm"] / 10.0, sample_rate
    )
    lines.append("")
    lines.append(f"full_record_position_dominant_frequency_hz: {full_frequency:.3f}")
    lines.append(f"full_record_position_fft_amplitude_mm: {full_amplitude:.3f}")

    path.write_text("\n".join(lines) + "\n", encoding="utf-8")


def main() -> None:
    script_dir = Path(__file__).resolve().parent
    repo_root = script_dir.parents[1]
    default_input = repo_root.parent / "log" / "session_c7dbbe6e_00" / "balance_control_v4_telemetry.csv"
    parser = argparse.ArgumentParser()
    parser.add_argument("input", nargs="?", type=Path, default=default_input)
    parser.add_argument("--output-dir", type=Path, default=script_dir / "output")
    args = parser.parse_args()

    args.output_dir.mkdir(parents=True, exist_ok=True)
    _, data = load_csv(args.input)
    t = (data["mcu_ms"] - data["mcu_ms"][0]) / 1000.0

    write_report(args.output_dir / "summary.txt", t, data)
    write_transitions(args.output_dir / "transitions.csv", t, data)
    write_metrics(args.output_dir / "metrics.csv", compute_metrics(t, data))
    plot_overview(args.output_dir / "overview.png", t, data)
    plot_event_comparison(args.output_dir / "event_comparison.png", t, data)
    plot_oscillation(args.output_dir / "post_return_oscillation.png", t, data)
    plot_capture_mismatch(args.output_dir / "capture_predictor_mismatch.png", t, data)
    plot_actuator_tracking(args.output_dir / "actuator_tracking.png", t, data)
    print(args.output_dir.resolve())


if __name__ == "__main__":
    main()
