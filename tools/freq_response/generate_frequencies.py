"""Generate VOLOOP frequency response sweep frequency lists.

The config mode updates only ``frequencies.values_hz`` in an existing test
config. The text output mode writes a standalone C runner frequency file.
"""

from __future__ import annotations

import argparse
import json
import math
import sys
import tempfile
from dataclasses import dataclass
from enum import Enum
from pathlib import Path
from typing import Any


class FrequencyUpdateMode(str, Enum):
    MERGE = "merge"
    REPLACE = "replace"


@dataclass(frozen=True)
class FrequencyUpdateResult:
    config_file: Path
    mode: FrequencyUpdateMode
    previous_values_hz: tuple[float, ...]
    generated_values_hz: tuple[float, ...]
    final_values_hz: tuple[float, ...]
    changed: bool
    written: bool


def _validate_finite(value: float, name: str) -> None:
    if not math.isfinite(value):
        raise ValueError(f"{name} must be finite")


def _normalize_frequency_value(value: Any, name: str) -> float:
    if isinstance(value, bool) or not isinstance(value, (int, float)):
        raise ValueError(f"{name} must be a number")
    parsed = float(value)
    if not math.isfinite(parsed):
        raise ValueError(f"{name} must be finite")
    if parsed <= 0.0:
        raise ValueError(f"{name} must be > 0")
    return float(f"{parsed:.12g}")


def _normalize_frequency_values(values_hz: list[float] | tuple[float, ...], name: str) -> list[float]:
    if not isinstance(values_hz, (list, tuple)):
        raise ValueError(f"{name} must be a list")
    if not values_hz:
        raise ValueError(f"{name} must not be empty")
    return [
        _normalize_frequency_value(value, f"{name}[{index}]")
        for index, value in enumerate(values_hz)
    ]


def _unique_sorted(values_hz: list[float]) -> list[float]:
    return sorted({float(f"{value:.12g}") for value in values_hz})


def _load_config(config_file: Path) -> dict[str, Any]:
    if not config_file.exists():
        raise FileNotFoundError(f"config file does not exist: {config_file}")
    if not config_file.is_file():
        raise OSError(f"config path is not a file: {config_file}")
    with config_file.open("r", encoding="utf-8") as stream:
        config = json.load(stream)
    if not isinstance(config, dict):
        raise ValueError("config root must be a JSON object")
    if config.get("schema_version") != 1:
        raise ValueError("config schema_version must be 1")
    return config


def _get_existing_frequency_values(config: dict[str, Any]) -> list[float]:
    frequencies = config.get("frequencies")
    if frequencies is None:
        return []
    if not isinstance(frequencies, dict):
        raise ValueError("config['frequencies'] must be an object")
    values = frequencies.get("values_hz")
    if values is None:
        return []
    if not isinstance(values, list):
        raise ValueError("config['frequencies']['values_hz'] must be a list")
    if not values:
        return []
    return _normalize_frequency_values(values, "frequencies.values_hz")


def _write_config_atomic(config_file: Path, config: dict[str, Any]) -> None:
    temp_path: Path | None = None
    try:
        with tempfile.NamedTemporaryFile(
            "w",
            encoding="utf-8",
            dir=config_file.parent,
            delete=False,
            newline="\n",
        ) as temp_stream:
            temp_path = Path(temp_stream.name)
            json.dump(config, temp_stream, ensure_ascii=False, indent=2)
            temp_stream.write("\n")
        temp_path.replace(config_file)
    except Exception:
        if temp_path is not None:
            try:
                temp_path.unlink(missing_ok=True)
            except OSError:
                pass
        raise


def generate_log_frequencies(
    start_hz: float,
    stop_hz: float,
    points_per_decade: int,
    include_hz: list[float] | None = None,
) -> list[float]:
    """Return unique sorted log-sweep frequencies."""
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

    return _unique_sorted(frequencies)


def merge_frequency_values(
    existing_values_hz: list[float],
    added_values_hz: list[float],
) -> list[float]:
    existing = _normalize_frequency_values(existing_values_hz, "existing_values_hz") if existing_values_hz else []
    added = _normalize_frequency_values(added_values_hz, "added_values_hz")
    return _unique_sorted([*existing, *added])


def update_config_frequencies(
    config_file: Path,
    generated_values_hz: list[float],
    mode: FrequencyUpdateMode = FrequencyUpdateMode.MERGE,
    dry_run: bool = False,
) -> FrequencyUpdateResult:
    config_path = Path(config_file)
    config = _load_config(config_path)
    generated_values = _normalize_frequency_values(generated_values_hz, "generated_values_hz")
    previous_values = _get_existing_frequency_values(config)

    if mode == FrequencyUpdateMode.MERGE:
        final_values = merge_frequency_values(previous_values, generated_values)
    elif mode == FrequencyUpdateMode.REPLACE:
        final_values = _unique_sorted(generated_values)
    else:
        raise ValueError(f"unsupported frequency update mode: {mode}")

    changed = previous_values != final_values
    written = changed and not dry_run

    if written:
        frequencies = config.get("frequencies")
        if frequencies is None:
            frequencies = {}
            config["frequencies"] = frequencies
        if not isinstance(frequencies, dict):
            raise ValueError("config['frequencies'] must be an object")
        frequencies["values_hz"] = final_values
        _write_config_atomic(config_path, config)

    return FrequencyUpdateResult(
        config_file=config_path,
        mode=mode,
        previous_values_hz=tuple(previous_values),
        generated_values_hz=tuple(generated_values),
        final_values_hz=tuple(final_values),
        changed=changed,
        written=written,
    )


def write_frequencies(
    path: Path,
    frequencies: list[float],
) -> None:
    normalized = _normalize_frequency_values(frequencies, "frequencies")
    path.parent.mkdir(parents=True, exist_ok=True)
    with path.open("w", encoding="utf-8", newline="\n") as frequencies_file:
        for frequency_hz in normalized:
            frequencies_file.write(f"{frequency_hz:.12g}\n")


def parse_args(argv: list[str] | None = None) -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Generate VOLOOP log-sweep frequency response frequency lists.",
    )
    target_group = parser.add_mutually_exclusive_group(required=True)
    target_group.add_argument("--config", type=Path, help="Config JSON to update at frequencies.values_hz.")
    target_group.add_argument("--out", type=Path, help="Standalone frequencies.txt output path.")
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
        help="Additional frequency in Hz to merge into the generated set. May be repeated.",
    )
    parser.add_argument("--replace", action="store_true", help="Replace config frequencies instead of merging.")
    parser.add_argument("--dry-run", action="store_true", help="Preview the result without writing files.")
    args = parser.parse_args(argv)
    if args.replace and args.config is None:
        parser.error("--replace can only be used with --config")
    return args


def main(argv: list[str] | None = None) -> int:
    try:
        args = parse_args(argv)
        generated = generate_log_frequencies(
            start_hz=args.start_hz,
            stop_hz=args.stop_hz,
            points_per_decade=args.points_per_decade,
            include_hz=args.include_hz,
        )
        if args.config is not None:
            mode = FrequencyUpdateMode.REPLACE if args.replace else FrequencyUpdateMode.MERGE
            result = update_config_frequencies(
                config_file=args.config,
                generated_values_hz=generated,
                mode=mode,
                dry_run=args.dry_run,
            )
            action = "would update" if args.dry_run else "updated"
            if not result.changed:
                action = "unchanged"
            print(
                f"{action}: {result.config_file} "
                f"mode={result.mode.value} previous={len(result.previous_values_hz)} "
                f"generated={len(result.generated_values_hz)} final={len(result.final_values_hz)}"
            )
        else:
            if args.dry_run:
                print(
                    f"would write {len(generated)} frequencies -> {args.out}; "
                    f"first={generated[0]:.12g} last={generated[-1]:.12g}"
                )
            else:
                write_frequencies(args.out, generated)
                print(f"generated {len(generated)} frequencies -> {args.out}")
    except (OSError, ValueError, json.JSONDecodeError) as exc:
        print(f"error: {exc}", file=sys.stderr)
        return 1
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
