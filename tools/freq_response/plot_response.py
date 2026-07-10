"""Render VOLOOP frequency response raw CSV files into processed CSV and Bode plots.

Example:
  python tools/freq_response/plot_response.py --input build/tools/freq_response/fof_lowpass_100hz.csv --out-dir build/tools/freq_response/plots

Recommended from the tools project:
  uv run python freq_response/plot_response.py --input ../build/tools/freq_response/fof_lowpass_100hz.csv --out-dir ../build/tools/freq_response/plots --name fof_lowpass_100hz
"""

from __future__ import annotations

import argparse
import csv
import math
import sys
from dataclasses import dataclass
from pathlib import Path

try:
    import numpy as np

    import matplotlib

    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
except ImportError:
    np = None
    plt = None


REQUIRED_FIELDS = [
    "module",
    "mode",
    "sample_rate_hz",
    "frequency_hz",
    "input_amplitude",
    "output_amplitude",
    "gain_linear",
    "gain_db",
    "phase_deg",
    "warmup_samples",
    "measure_samples",
    "total_samples",
    "status",
]

VALID_STATUSES = {"ok", "ok_with_gain_floor"}


@dataclass
class ValidPoint:
    row_index: int
    frequency_hz: float
    gain_db: float
    phase_deg: float
    module: str
    mode: str


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Render VOLOOP raw frequency response CSV into processed CSV plus PNG/SVG Bode plots.",
        epilog=(
            "Example: cd tools && uv run python freq_response/plot_response.py "
            "--input ../build/tools/freq_response/fof_lowpass_100hz.csv "
            "--out-dir ../build/tools/freq_response/plots --name fof_lowpass_100hz"
        ),
    )
    parser.add_argument("--input", required=True, type=Path, help="Input raw CSV path.")
    parser.add_argument("--out-dir", required=True, type=Path, help="Directory for processed CSV and Bode plots.")
    parser.add_argument("--name", help="Output file stem. Defaults to input CSV filename without extension.")
    parser.add_argument("--interpolate", action="store_true", help="Add lighter log-frequency interpolation curves to plots.")
    parser.add_argument(
        "--interp-points-per-decade",
        type=int,
        default=80,
        help="Interpolated points per decade when --interpolate is enabled. Default: 80.",
    )
    return parser.parse_args()


def fail(message: str) -> int:
    print(f"error: {message}", file=sys.stderr)
    return 1


def warn(message: str) -> None:
    print(f"warning: {message}", file=sys.stderr)


def parse_finite_float(value: str) -> float | None:
    try:
        parsed = float(value)
    except (TypeError, ValueError):
        return None
    if not math.isfinite(parsed):
        return None
    return parsed


def row_is_plot_valid(row: dict[str, str]) -> tuple[bool, float | None, float | None, float | None]:
    frequency_hz = parse_finite_float(row.get("frequency_hz", ""))
    gain_db = parse_finite_float(row.get("gain_db", ""))
    phase_deg = parse_finite_float(row.get("phase_deg", ""))

    is_valid = (
        row.get("status", "") in VALID_STATUSES
        and frequency_hz is not None
        and frequency_hz > 0.0
        and gain_db is not None
        and phase_deg is not None
    )
    return is_valid, frequency_hz, gain_db, phase_deg


def read_raw_csv(input_path: Path) -> tuple[list[str], list[dict[str, str]], list[ValidPoint], set[tuple[str, str]]] | None:
    try:
        with input_path.open("r", newline="", encoding="utf-8-sig") as csv_file:
            reader = csv.DictReader(csv_file)
            if reader.fieldnames is None:
                print(f"error: input CSV has no header: {input_path}", file=sys.stderr)
                return None

            missing_fields = [field for field in REQUIRED_FIELDS if field not in reader.fieldnames]
            if missing_fields:
                print(
                    "error: input CSV is missing required field(s): " + ", ".join(missing_fields),
                    file=sys.stderr,
                )
                return None

            rows: list[dict[str, str]] = []
            valid_points: list[ValidPoint] = []
            module_mode_pairs: set[tuple[str, str]] = set()

            for row_index, row in enumerate(reader):
                rows.append(row)
                module = row.get("module", "")
                mode = row.get("mode", "")
                module_mode_pairs.add((module, mode))

                is_valid, frequency_hz, gain_db, phase_deg = row_is_plot_valid(row)
                if is_valid:
                    valid_points.append(
                        ValidPoint(
                            row_index=row_index,
                            frequency_hz=frequency_hz,
                            gain_db=gain_db,
                            phase_deg=phase_deg,
                            module=module,
                            mode=mode,
                        )
                    )

            return reader.fieldnames, rows, valid_points, module_mode_pairs
    except OSError as exc:
        print(f"error: failed to read input CSV: {exc}", file=sys.stderr)
        return None


def unique_sorted_points(valid_points: list[ValidPoint]) -> list[ValidPoint]:
    sorted_points = sorted(valid_points, key=lambda point: (point.frequency_hz, point.row_index))
    seen_frequencies: set[float] = set()
    unique_points: list[ValidPoint] = []
    duplicate_count = 0

    for point in sorted_points:
        if point.frequency_hz in seen_frequencies:
            duplicate_count += 1
            continue
        seen_frequencies.add(point.frequency_hz)
        unique_points.append(point)

    if duplicate_count:
        warn(f"input CSV contains duplicate valid frequency_hz values; kept first valid row for plotting ({duplicate_count} duplicate row(s) skipped)")

    return unique_points


def add_processed_columns(rows: list[dict[str, str]], valid_points: list[ValidPoint]) -> None:
    sorted_points = sorted(valid_points, key=lambda point: (point.frequency_hz, point.row_index))
    phases = np.array([point.phase_deg for point in sorted_points], dtype=float)
    unwrapped_phases = np.rad2deg(np.unwrap(np.deg2rad(phases)))

    phase_by_row_index = {
        point.row_index: f"{phase:.12g}"
        for point, phase in zip(sorted_points, unwrapped_phases, strict=True)
    }

    for row_index, row in enumerate(rows):
        is_valid, _frequency_hz, _gain_db, _phase_deg = row_is_plot_valid(row)
        row["plot_valid"] = "1" if is_valid else "0"
        row["phase_unwrapped_deg"] = phase_by_row_index.get(row_index, "")


def write_processed_csv(output_path: Path, fieldnames: list[str], rows: list[dict[str, str]]) -> None:
    processed_fieldnames = list(fieldnames)
    for extra_field in ("phase_unwrapped_deg", "plot_valid"):
        if extra_field not in processed_fieldnames:
            processed_fieldnames.append(extra_field)

    with output_path.open("w", newline="", encoding="utf-8") as csv_file:
        writer = csv.DictWriter(csv_file, fieldnames=processed_fieldnames, extrasaction="ignore")
        writer.writeheader()
        writer.writerows(rows)


def interpolated_curve(
    frequencies: np.ndarray,
    values: np.ndarray,
    points_per_decade: int,
) -> tuple[np.ndarray, np.ndarray] | None:
    if len(frequencies) < 2:
        return None

    if points_per_decade <= 0:
        return None

    log_frequencies = np.log10(frequencies)
    decades = log_frequencies[-1] - log_frequencies[0]
    point_count = max(int(math.ceil(decades * points_per_decade)) + 1, len(frequencies))
    interp_log_frequencies = np.linspace(log_frequencies[0], log_frequencies[-1], point_count)
    interp_frequencies = np.power(10.0, interp_log_frequencies)
    interp_values = np.interp(interp_log_frequencies, log_frequencies, values)
    return interp_frequencies, interp_values


def plot_bode(
    png_path: Path,
    svg_path: Path,
    input_path: Path,
    plot_points: list[ValidPoint],
    module_mode_pairs: set[tuple[str, str]],
    interpolate: bool,
    points_per_decade: int,
) -> None:
    frequencies = np.array([point.frequency_hz for point in plot_points], dtype=float)
    gains = np.array([point.gain_db for point in plot_points], dtype=float)
    phases = np.rad2deg(np.unwrap(np.deg2rad(np.array([point.phase_deg for point in plot_points], dtype=float))))

    fig, (gain_axis, phase_axis) = plt.subplots(2, 1, figsize=(9.5, 7.0), sharex=True)

    if interpolate and len(frequencies) < 2:
        warn("interpolation requested but fewer than 2 unique valid frequency points are available; skipping interpolation")
    elif interpolate and points_per_decade <= 0:
        warn("interpolation requested with non-positive --interp-points-per-decade; skipping interpolation")
    elif interpolate:
        gain_interp = interpolated_curve(frequencies, gains, points_per_decade)
        phase_interp = interpolated_curve(frequencies, phases, points_per_decade)
        if gain_interp is not None and phase_interp is not None:
            gain_axis.plot(gain_interp[0], gain_interp[1], color="tab:blue", alpha=0.35, linewidth=1.0)
            phase_axis.plot(phase_interp[0], phase_interp[1], color="tab:orange", alpha=0.35, linewidth=1.0)

    gain_axis.plot(
        frequencies,
        gains,
        color="tab:blue",
        marker="o",
        markersize=6.5,
        markeredgewidth=1.0,
        linewidth=2.2,
    )
    phase_axis.plot(
        frequencies,
        phases,
        color="tab:orange",
        marker="s",
        markersize=6.5,
        markeredgewidth=1.0,
        linewidth=2.2,
    )

    if len(module_mode_pairs) == 1:
        module, mode = next(iter(module_mode_pairs))
        title = f"VOLOOP frequency response: {module}/{mode}"
    else:
        title = input_path.stem

    fig.suptitle(title)
    gain_axis.set_ylabel("Gain (dB)")
    phase_axis.set_ylabel("Phase (deg)")
    phase_axis.set_xlabel("Frequency (Hz)")

    for axis in (gain_axis, phase_axis):
        axis.set_xscale("log")
        axis.grid(True, which="both", linestyle=":", linewidth=0.7, alpha=0.65)

    fig.tight_layout()
    fig.savefig(png_path, dpi=160)
    fig.savefig(svg_path)
    plt.close(fig)


def main() -> int:
    args = parse_args()

    if np is None or plt is None:
        print(
            "Missing dependency: numpy or matplotlib.\n"
            "Install dependencies with:\n"
            "  cd tools\n"
            "  uv sync",
            file=sys.stderr,
        )
        return 1

    input_path = args.input
    out_dir = args.out_dir
    name = args.name or input_path.stem

    if not input_path.exists():
        return fail(f"input CSV does not exist: {input_path}")
    if not input_path.is_file():
        return fail(f"input path is not a file: {input_path}")

    raw_data = read_raw_csv(input_path)
    if raw_data is None:
        return 1

    fieldnames, rows, valid_points, module_mode_pairs = raw_data
    plot_points = unique_sorted_points(valid_points)
    if not plot_points:
        return fail("input CSV has no valid plotting points")

    if len(module_mode_pairs) > 1:
        warn("input CSV contains multiple module/mode pairs; plotting all valid rows as one curve")

    out_dir.mkdir(parents=True, exist_ok=True)
    processed_path = out_dir / f"{name}_processed.csv"
    png_path = out_dir / f"{name}_bode.png"
    svg_path = out_dir / f"{name}_bode.svg"

    if processed_path.resolve() == input_path.resolve():
        return fail(f"processed CSV output would overwrite input raw CSV: {processed_path}")

    add_processed_columns(rows, valid_points)
    write_processed_csv(processed_path, fieldnames, rows)
    plot_bode(
        png_path=png_path,
        svg_path=svg_path,
        input_path=input_path,
        plot_points=plot_points,
        module_mode_pairs=module_mode_pairs,
        interpolate=args.interpolate,
        points_per_decade=args.interp_points_per_decade,
    )

    print(f"processed_csv: {processed_path}")
    print(f"bode_png: {png_path}")
    print(f"bode_svg: {svg_path}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
