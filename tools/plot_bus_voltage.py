"""Render WHEELTEC TB6612 bus-voltage records as a standalone SVG chart."""

from __future__ import annotations

import argparse
import re
from pathlib import Path


SAMPLE_PATTERN = re.compile(
    r"\[pwr\]\s+(?P<time_ms>\d+),mv=(?P<millivolts>\d+),raw=(?P<raw>\d+)"
)


def load_samples(path: Path) -> tuple[list[float], list[float]]:
    times_s: list[float] = []
    volts: list[float] = []

    for line in path.read_text(encoding="utf-8", errors="replace").splitlines():
        match = SAMPLE_PATTERN.search(line)
        if match is None:
            continue
        times_s.append(int(match.group("time_ms")) / 1000.0)
        volts.append(int(match.group("millivolts")) / 1000.0)

    if not times_s:
        raise ValueError("No [pwr] records found in the input file.")
    return times_s, volts


def render_svg(times_s: list[float], volts: list[float], low: float) -> str:
    width, height = 1000, 500
    left, right, top, bottom = 78, 30, 28, 58
    plot_width = width - left - right
    plot_height = height - top - bottom
    time_min, time_max = min(times_s), max(times_s)
    voltage_min = min(min(volts), low)
    voltage_max = max(volts)
    padding = max(0.1, (voltage_max - voltage_min) * 0.12)
    voltage_min -= padding
    voltage_max += padding
    if time_min == time_max:
        time_max += 1.0
    if voltage_min == voltage_max:
        voltage_max += 0.1

    def x(value: float) -> float:
        return left + (value - time_min) * plot_width / (time_max - time_min)

    def y(value: float) -> float:
        return top + (voltage_max - value) * plot_height / (voltage_max - voltage_min)

    max_points = 1600
    stride = max(1, len(times_s) // max_points)
    points = " ".join(f"{x(times_s[index]):.1f},{y(volts[index]):.1f}"
                      for index in range(0, len(times_s), stride))
    low_y = y(low)
    grid = []
    labels = []
    for step in range(6):
        voltage = voltage_min + (voltage_max - voltage_min) * step / 5
        grid.append(f'<line x1="{left}" y1="{y(voltage):.1f}" '
                    f'x2="{width - right}" y2="{y(voltage):.1f}"/>')
        labels.append(f'<text x="{left - 10}" y="{y(voltage) + 4:.1f}" '
                      f'text-anchor="end">{voltage:.2f}</text>')
    for step in range(6):
        timestamp = time_min + (time_max - time_min) * step / 5
        labels.append(f'<text x="{x(timestamp):.1f}" y="{height - 22}" '
                      f'text-anchor="middle">{timestamp:.1f}</text>')

    return f'''<svg xmlns="http://www.w3.org/2000/svg" width="{width}" height="{height}" viewBox="0 0 {width} {height}" role="img" aria-label="TB6612 motor bus voltage">
<style>
text {{ font-family: Arial, sans-serif; font-size: 13px; fill: #202124; }}
.grid {{ stroke: #d8dce0; stroke-width: 1; }}
.axis {{ stroke: #4d5358; stroke-width: 1.2; }}
.voltage {{ fill: none; stroke: #1565c0; stroke-width: 2; }}
.low {{ stroke: #b3261e; stroke-width: 1.2; stroke-dasharray: 6 4; }}
</style>
<rect width="100%" height="100%" fill="white"/>
<text x="{width / 2:.1f}" y="20" text-anchor="middle">TB6612 motor bus voltage</text>
<g class="grid">{''.join(grid)}</g>
<line class="axis" x1="{left}" y1="{top}" x2="{left}" y2="{height - bottom}"/>
<line class="axis" x1="{left}" y1="{height - bottom}" x2="{width - right}" y2="{height - bottom}"/>
<line class="low" x1="{left}" y1="{low_y:.1f}" x2="{width - right}" y2="{low_y:.1f}"/>
<polyline class="voltage" points="{points}"/>
{''.join(labels)}
<text x="18" y="{height / 2:.1f}" text-anchor="middle" transform="rotate(-90 18 {height / 2:.1f})">Voltage (V)</text>
<text x="{width / 2:.1f}" y="{height - 4}" text-anchor="middle">Time (s)</text>
<text x="{width - right - 4}" y="{low_y - 6:.1f}" text-anchor="end">Low reference {low:.1f} V</text>
</svg>'''


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("input", type=Path, help="UART text log containing [pwr] records")
    parser.add_argument("--output", type=Path, default=Path("bus_voltage.svg"),
                        help="Output SVG path (default: bus_voltage.svg)")
    parser.add_argument("--low", type=float, default=9.6,
                        help="Low-voltage reference in volts (default: 9.6)")
    args = parser.parse_args()

    times_s, volts = load_samples(args.input)
    args.output.write_text(render_svg(times_s, volts, args.low), encoding="utf-8")
    print(f"Wrote {args.output} from {len(times_s)} samples.")


if __name__ == "__main__":
    main()
