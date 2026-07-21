"""Convert MCU frequency-scan CSV output into a VOLOOP response CSV.

Example from ``tools/freq_response``::

    uv run python convert_mcu_response.py --input ../../../cache/1.csv
"""

from __future__ import annotations

import argparse
import csv
import math
import sys
from pathlib import Path


MODULE_NAME = "power_path"
DEFAULT_INPUT_AMPLITUDE = 0.1
DEFAULT_GAIN_FLOOR = 1.0e-12

REQUIRED_INPUT_FIELDS = [
    "frequency_hz",
    "synchronous_component",
    "quadrature_component",
    "status",
]

OUTPUT_FIELDS = [
    "module",
    "mode",
    "frequency_hz",
    "synchronous_component",
    "quadrature_component",
    "gain_linear",
    "gain_db",
    "phase_deg",
    "status",
]

STATUS_NAMES = {
    0: "ok",
    1: "ok_with_gain_floor",
    2: "invalid_argument",
    3: "invalid_frequency",
    4: "insufficient_samples",
    5: "sample_limit_exceeded",
    6: "voltage_limit_exceeded",
    7: "numeric_error",
    8: "invalid_state",
}

SUCCESS_STATUS_CODES = {0, 1}


def parse_args(argv: list[str] | None = None) -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Convert an MCU frequency-scan CSV into a VOLOOP response CSV.",
        epilog=(
            "Example: uv run python convert_mcu_response.py "
            "--input ../../../cache/1.csv"
        ),
    )
    parser.add_argument("--input", required=True, type=Path, help="Input MCU CSV path.")
    parser.add_argument(
        "--output",
        type=Path,
        help="Output CSV path. Defaults to <input_stem>_response.csv beside the input.",
    )
    parser.add_argument(
        "--input-amplitude",
        type=float,
        default=DEFAULT_INPUT_AMPLITUDE,
        help=f"Injected peak amplitude used to calculate gain (default: {DEFAULT_INPUT_AMPLITUDE:g}).",
    )
    parser.add_argument(
        "--gain-floor",
        type=float,
        default=DEFAULT_GAIN_FLOOR,
        help=f"Minimum linear gain before dB conversion (default: {DEFAULT_GAIN_FLOOR:g}).",
    )
    return parser.parse_args(argv)


def validate_positive_finite(name: str, value: float) -> None:
    if not math.isfinite(value) or value <= 0.0:
        raise ValueError(f"{name} must be a finite positive number")


def parse_status_code(value: str | None, row_number: int) -> int:
    if value is None:
        raise ValueError(f"row {row_number}: status must be an integer")
    try:
        return int(value)
    except ValueError as exc:
        raise ValueError(f"row {row_number}: status must be an integer") from exc


def parse_finite_float(value: str | None) -> float | None:
    try:
        parsed = float(value) if value is not None else math.nan
    except (TypeError, ValueError):
        return None
    return parsed if math.isfinite(parsed) else None


def format_derived(value: float) -> str:
    return f"{value:.12g}"


def derive_response_fields(
    synchronous_component: str | None,
    quadrature_component: str | None,
    status_code: int,
    input_amplitude: float,
    gain_floor: float,
) -> tuple[str, str, str]:
    if status_code not in SUCCESS_STATUS_CODES:
        return "", "", ""

    synchronous = parse_finite_float(synchronous_component)
    quadrature = parse_finite_float(quadrature_component)
    if synchronous is None or quadrature is None:
        return "", "", ""

    amplitude = math.hypot(synchronous, quadrature)
    gain_linear = max(amplitude / input_amplitude, gain_floor)
    gain_db = 20.0 * math.log10(gain_linear)
    phase_deg = math.degrees(math.atan2(quadrature, synchronous))
    if phase_deg >= 180.0:
        phase_deg -= 360.0
    if phase_deg == 0.0:
        phase_deg = 0.0

    if not all(math.isfinite(value) for value in (gain_linear, gain_db, phase_deg)):
        return "", "", ""

    return (
        format_derived(gain_linear),
        format_derived(gain_db),
        format_derived(phase_deg),
    )


def read_and_convert_rows(
    input_path: Path,
    input_amplitude: float,
    gain_floor: float,
) -> list[dict[str, str]]:
    with input_path.open("r", newline="", encoding="utf-8-sig") as csv_file:
        reader = csv.DictReader(csv_file)
        if reader.fieldnames is None:
            raise ValueError(f"input CSV has no header: {input_path}")

        duplicate_fields = sorted(
            field for field in set(reader.fieldnames) if reader.fieldnames.count(field) > 1
        )
        if duplicate_fields:
            raise ValueError(
                "input CSV has duplicate field(s): " + ", ".join(duplicate_fields)
            )

        missing_fields = [
            field for field in REQUIRED_INPUT_FIELDS if field not in reader.fieldnames
        ]
        if missing_fields:
            raise ValueError(
                "input CSV is missing required field(s): " + ", ".join(missing_fields)
            )

        mode = input_path.stem
        converted_rows: list[dict[str, str]] = []
        for row_number, row in enumerate(reader, start=2):
            status_code = parse_status_code(row.get("status"), row_number)
            gain_linear, gain_db, phase_deg = derive_response_fields(
                synchronous_component=row.get("synchronous_component"),
                quadrature_component=row.get("quadrature_component"),
                status_code=status_code,
                input_amplitude=input_amplitude,
                gain_floor=gain_floor,
            )
            converted_rows.append(
                {
                    "module": MODULE_NAME,
                    "mode": mode,
                    "frequency_hz": row.get("frequency_hz") or "",
                    "synchronous_component": row.get("synchronous_component") or "",
                    "quadrature_component": row.get("quadrature_component") or "",
                    "gain_linear": gain_linear,
                    "gain_db": gain_db,
                    "phase_deg": phase_deg,
                    "status": STATUS_NAMES.get(status_code, f"unknown_{status_code}"),
                }
            )

    if not converted_rows:
        raise ValueError(f"input CSV has no data rows: {input_path}")
    return converted_rows


def convert_mcu_response(
    input_csv: Path,
    output_csv: Path | None = None,
    input_amplitude: float = DEFAULT_INPUT_AMPLITUDE,
    gain_floor: float = DEFAULT_GAIN_FLOOR,
) -> Path:
    """Convert one MCU sweep CSV and return the output path."""

    validate_positive_finite("input_amplitude", input_amplitude)
    validate_positive_finite("gain_floor", gain_floor)

    input_path = Path(input_csv)
    if not input_path.exists():
        raise FileNotFoundError(f"input CSV does not exist: {input_path}")
    if not input_path.is_file():
        raise OSError(f"input path is not a file: {input_path}")

    output_path = (
        Path(output_csv)
        if output_csv is not None
        else input_path.with_name(f"{input_path.stem}_response.csv")
    )
    if output_path.resolve() == input_path.resolve():
        raise ValueError(f"output CSV would overwrite input CSV: {input_path}")

    converted_rows = read_and_convert_rows(
        input_path=input_path,
        input_amplitude=input_amplitude,
        gain_floor=gain_floor,
    )

    output_path.parent.mkdir(parents=True, exist_ok=True)
    with output_path.open("w", newline="", encoding="utf-8") as csv_file:
        writer = csv.DictWriter(csv_file, fieldnames=OUTPUT_FIELDS)
        writer.writeheader()
        writer.writerows(converted_rows)

    return output_path


def main(argv: list[str] | None = None) -> int:
    args = parse_args(argv)
    try:
        output_path = convert_mcu_response(
            input_csv=args.input,
            output_csv=args.output,
            input_amplitude=args.input_amplitude,
            gain_floor=args.gain_floor,
        )
    except (OSError, ValueError) as exc:
        print(f"error: {exc}", file=sys.stderr)
        return 1

    print(f"response CSV: {output_path}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
