"""Generate VOLOOP frequency response sweep frequency files.

Example:
  python tools/freq_response/generate_frequencies.py --start-hz 1 --stop-hz 1000 --points-per-decade 20 --include-hz 50 --include-hz 100 --out build/tools/freq_response/frequencies.txt
"""

from __future__ import annotations

import argparse
import math
import sys
from pathlib import Path


def _validate_finite(value: float, name: str) -> None:
    if not math.isfinite(value):
        raise ValueError(f"{name} must be finite")


def generate_log_frequencies(
    start_hz: float,
    stop_hz: float,
    points_per_decade: int,
    include_hz: list[float] | None = None,
) -> list[float]:
    """Return unique sorted log-sweep frequencies.

    ``include_hz`` values only need to be finite and greater than zero. They may
    be outside the ``[start_hz, stop_hz]`` sweep range and are still merged into
    the final unique, numerically sorted result.
    """
    _validate_finite(start_hz, "start_hz")
    _validate_finite(stop_hz, "stop_hz")

    if start_hz <= 0.0:
        raise ValueError("start_hz must be > 0")
    if stop_hz <= start_hz:
        raise ValueError("stop_hz must be > start_hz")
    if points_per_decade <= 0:
        raise ValueError("points_per_decade must be > 0")

    decades = math.log10(stop_hz / start_hz)
    point_count = math.ceil(decades * points_per_decade) + 1

    frequencies = [
        10.0 ** (math.log10(start_hz) + index * decades / (point_count - 1))
        for index in range(point_count)
    ]
    frequencies[0] = start_hz
    frequencies[-1] = stop_hz

    if include_hz is not None:
        for index, frequency_hz in enumerate(include_hz):
            _validate_finite(frequency_hz, f"include_hz[{index}]")
            if frequency_hz <= 0.0:
                raise ValueError("include_hz values must be > 0")
            frequencies.append(frequency_hz)

    unique = {f"{frequency_hz:.12g}" for frequency_hz in frequencies}
    return sorted(float(frequency_hz) for frequency_hz in unique)


def write_frequencies(
    path: Path,
    frequencies: list[float],
) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    with path.open("w", encoding="utf-8", newline="\n") as frequencies_file:
        for frequency_hz in frequencies:
            frequencies_file.write(f"{frequency_hz:.12g}\n")


def parse_args(argv: list[str] | None = None) -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Generate VOLOOP log-sweep frequency response frequencies.txt files.",
    )
    parser.add_argument("--start-hz", required=True, type=float, help="Sweep start frequency in Hz.")
    parser.add_argument("--stop-hz", required=True, type=float, help="Sweep stop frequency in Hz.")
    parser.add_argument(
        "--points-per-decade",
        required=True,
        type=int,
        help="Number of log-sweep points per decade.",
    )
    parser.add_argument(
        "--include-hz",
        action="append",
        type=float,
        help="Additional frequency in Hz to merge into the output. May be repeated.",
    )
    parser.add_argument("--out", required=True, type=Path, help="Output frequencies.txt path.")
    return parser.parse_args(argv)


def main(argv: list[str] | None = None) -> int:
    args = parse_args(argv)

    try:
        frequencies = generate_log_frequencies(
            start_hz=args.start_hz,
            stop_hz=args.stop_hz,
            points_per_decade=args.points_per_decade,
            include_hz=args.include_hz,
        )
        write_frequencies(args.out, frequencies)
    except ValueError as exc:
        print(f"error: {exc}", file=sys.stderr)
        return 2

    print(f"generated {len(frequencies)} frequencies -> {args.out}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
