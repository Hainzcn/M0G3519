# Session c7dbbe6e_00 telemetry analysis

## Reproduce

From the repository root:

```powershell
python .\analysis\session_c7dbbe6e_00\analyze_balance_log.py
```

The default input is the sibling directory
`../log/session_c7dbbe6e_00/balance_control_v4_telemetry.csv`. The script uses
only NumPy and Matplotlib and writes all derived files under `output/`.

MCU time is used for dynamics because `mcu_ms` advances by exactly 10 ms per
record. `rx_host_ms` is a serial receive timestamp and spans about 0.41 s less
than the MCU clock over this recording.

## Event boundaries

Times below are seconds from the first telemetry sample on the MCU clock.

| Time | Observation |
|---:|---|
| 3.930 | First ball detection jumps to +118.6 mm while the app is waiting for vision |
| 4.080 | First short ACTIVE entry |
| 4.310 | RECOVERY after stale vision |
| 4.520 | Stable ACTIVE entry used as event 1 start, +116.0 mm |
| 8.980 | First entry into the +/-4 mm position band; velocity is still high and OVERSPEED starts |
| 35.940 | Short RECOVERY dropout |
| 36.060 | ACTIVE resumes |
| 43.940 | Long RECOVERY / vision-loss interval begins |
| 47.840-48.320 | Reacquisition produces nonphysical position discontinuities |
| 48.360 | ACTIVE resumes, used as event 2 start, +107.6 mm |

The long recovery interval is excluded from event-2 control metrics. Its
largest one-sample estimated-position jump is 145.7 mm, so it must not be used
for plant identification.

## Main quantitative evidence

- The first +/-4 mm entries occur 4.46 s and 4.34 s after the two ACTIVE
  starts. The longest simultaneous `|estimated position| <= 4 mm` and
  `|estimated velocity| <= 4 mm/s` runs are only 0.04 s and 0.01 s.
- After first arrival, event 1 has position mean +13.4 mm, RMS 18.8 mm,
  range -13.4 to +64.1 mm, and dominant frequency 0.801 Hz. Event 2 has mean
  +15.7 mm, RMS 19.9 mm, range -5.4 to +36.2 mm, and dominant frequency
  0.718 Hz.
- Event 1 contains 46 OVERSPEED episodes, 20 CAPTURE episodes, 61 reference
  velocity sign changes, and 68 desired-acceleration sign changes above
  50 mm/s^2. Event 2 contains 8, 2, 13, and 13 respectively.
- In the 31.72 s window from event 1's first HOLD entry until vision loss
  (31.60 s ACTIVE), nominal feedforward remains nonzero for 84.1% of ACTIVE
  samples, has RMS 0.118 m/s^2, and
  reverses sign 62 times above a 20 mm/s^2 threshold. It is therefore not
  behaving as one independently executed feedforward trajectory.
- At the 22 CAPTURE entries, `|predicted velocity|` is at most 4 mm/s, while
  same-time `|estimated velocity|` has median 26 mm/s. More importantly, the
  state realized 120 ms later has median `|velocity|` 17.5 mm/s, and none of the
  22 entries realizes the predicted +/-4 mm/s capture condition. The absolute
  120 ms forecast error has median 19 mm/s and 95th percentile 46.8 mm/s. This
  directly demonstrates false capture caused by an inaccurate delay-forward
  prediction, rather than merely comparing a future prediction to current
  state.
- During ACTIVE control, the slew-limited flag is set 28.6% of the time. A
  0-500 ms lag scan in 10 ms increments, minimizing the RMSE between
  `actual(t)` and `shaped(t-lag)` over ACTIVE samples, gives about 0.18 s;
  zero-lag RMSE is 1.23 deg and aligned RMSE is 0.48 deg. Separate scans give
  0.18 s for event 1 and 0.17 s for event 2.
  This 0.18 s includes motor dynamics plus the 100 ms feedback polling/ZOH and
  is not a pure actuator transport delay.
- The telemetry transport is clean: 5,688 samples at exactly 100 Hz, no
  sequence gaps, no nonzero fault samples, and the receiver summary ends with
  zero CRC/header/missing-frame errors. Vision normally updates every 30 ms,
  while ACTIVE control is in predict-only mode 49.5% of telemetry samples.

## Field interpretation

| Fields | Meaning and conversion |
|---|---|
| `mcu_ms`, `rx_host_ms` | MCU generation time and host serial receive time |
| `estimated_position_dmm`, `predicted_position_dmm` | State estimate and actuator-delay-forward prediction; divide by 10 for mm |
| `estimated_velocity_mm_s`, `predicted_velocity_mm_s` | State estimate and forward prediction in mm/s |
| `reference_*`, `target_position_dmm` | Motion-profile reference state and final target |
| `feedforward_accel_mm_s2` | Profile acceleration command |
| `feedback_accel_mm_s2` | Outer-loop correction |
| `desired_accel_mm_s2` | Sum/override sent to inverse dynamics before lever shaping |
| `raw_lever_cdeg` | Inverse-dynamics lever request; divide by 100 for degrees |
| `shaped_lever_cdeg` | Rate/acceleration limited logical lever command |
| `actual_lever_cdeg` | Lever angle reconstructed from motor feedback; updated at the 100 ms query cadence |
| `motor_target_cdeg`, `motor_feedback_cdeg` | Motor-side target and encoder position in centidegrees |
| `vision_age_ms`, `confidence`, `vision_sequence` | Accepted vision measurement freshness and identity |
| `state` | App state: 5 ACTIVE, 6 RECOVERY, 10 WAIT_VISION in this log |
| `control_phase` | 0 HOLD, 1 ACCEL, 2 TRACK, 4 OVERSPEED, 5 EDGE_RECOVERY, 6 CAPTURE |
| `friction_mode` | 0 MOTION, 1 BREAKAWAY, 2 CAPTURE, 3 STOPPED |
| `control_flags` | Bit field decoded in the analysis script |

## Outputs

- `overview.png`: full record with startup, first return, sustained
  re-excitation, long recovery, and second return shaded.
- `event_comparison.png`: both return events with state, OVERSPEED/CAPTURE,
  acceleration components, and raw/shaped/actual lever traces.
- `post_return_oscillation.png`: time traces and spectra after first arrival.
- `capture_predictor_mismatch.png`: 120 ms velocity forecast versus the
  subsequently realized estimate at CAPTURE, plus a local time trace.
- `actuator_tracking.png`: raw request, shaped command, and sampled feedback
  around target passages.
- `metrics.csv`, `transitions.csv`, `summary.txt`: machine-readable metrics,
  all discrete transitions, and broad diagnostics.
