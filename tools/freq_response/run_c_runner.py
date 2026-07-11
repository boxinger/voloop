"""Run the VOLOOP C frequency response runner from a JSON config.

The config is the source of truth. This wrapper materializes the configured
frequency list into the text file required by the C runner, then invokes the
already-built runner. It does not generate new frequencies, edit config files,
call CMake, parse raw CSV data, or render plots.
"""

from __future__ import annotations

import argparse
import json
import math
import platform
import shlex
import subprocess
import sys
from dataclasses import dataclass
from pathlib import Path
from typing import Any


OBSOLETE_PATHS_MESSAGE = (
    "config uses obsolete field 'paths'; use 'frequencies.values_hz', "
    "'artifacts.frequencies_txt', and 'artifacts.raw_csv'"
)


@dataclass(frozen=True)
class RunnerResult:
    command: list[str]
    frequency_file: Path
    raw_csv: Path
    frequency_count: int
    returncode: int
    stdout: str
    stderr: str
    warnings: list[str]


def load_config(config_file: Path) -> dict[str, Any]:
    config_path = Path(config_file)
    if not config_path.exists():
        raise FileNotFoundError(f"config file does not exist: {config_path}")
    if not config_path.is_file():
        raise OSError(f"config path is not a file: {config_path}")

    with config_path.open("r", encoding="utf-8") as config_stream:
        config = json.load(config_stream)

    if not isinstance(config, dict):
        raise ValueError(f"config root must be a JSON object: {config_path}")
    if config.get("schema_version") != 1:
        raise ValueError("config schema_version must be 1")
    if "paths" in config:
        raise ValueError(OBSOLETE_PATHS_MESSAGE)
    return config


def resolve_config_path(config_file: Path, value: str | Path) -> Path:
    path = Path(value)
    if path.is_absolute():
        return path
    return Path(config_file).parent / path


def resolve_runner_executable(config_file: Path, value: str | Path) -> Path:
    executable = resolve_config_path(config_file, value)
    if executable.exists():
        return executable
    if platform.system() == "Windows" and executable.suffix.lower() != ".exe":
        exe_candidate = executable.with_name(executable.name + ".exe")
        if exe_candidate.exists():
            return exe_candidate
    return executable


def measurement_key_to_cli_option(key: str) -> str:
    return "--" + key.replace("_", "-")


def _require_mapping(config: dict[str, Any], key: str) -> dict[str, Any]:
    value = config.get(key)
    if not isinstance(value, dict):
        raise ValueError(f"config[{key!r}] must be an object")
    return value


def _require_nonempty_string(config: dict[str, Any], key: str) -> str:
    value = config.get(key)
    if not isinstance(value, str) or not value:
        raise ValueError(f"config[{key!r}] must be a non-empty string")
    return value


def _require_path_value(config: dict[str, Any], key: str, label: str | None = None) -> str | Path:
    value = config.get(key)
    if not isinstance(value, (str, Path)) or not value:
        field_name = label if label is not None else key
        raise ValueError(f"config field {field_name!r} must be a non-empty path string")
    return value


def _validate_frequency_value(value: Any, name: str) -> float:
    if isinstance(value, bool) or not isinstance(value, (int, float)):
        raise ValueError(f"{name} must be a number")
    parsed = float(value)
    if not math.isfinite(parsed):
        raise ValueError(f"{name} must be finite")
    if parsed <= 0.0:
        raise ValueError(f"{name} must be > 0")
    return parsed


def _stringify_cli_value(value: Any, name: str) -> str:
    if isinstance(value, bool) or value is None or isinstance(value, (list, dict)):
        raise ValueError(f"{name} must be a scalar CLI value")
    return str(value)


def _resolve_override_path(value: Path | None) -> Path | None:
    if value is None:
        return None
    return Path(value)


def _append_options_from_mapping(command: list[str], options: dict[str, Any], prefix: str) -> None:
    for key, value in options.items():
        if not isinstance(key, str) or not key:
            raise ValueError(f"{prefix} keys must be non-empty strings")
        option = measurement_key_to_cli_option(key) if prefix == "measurement" else f"--{key}"
        command.extend([option, _stringify_cli_value(value, f"{prefix}.{key}")])


def read_frequency_values(
    config: dict[str, Any],
) -> list[float]:
    frequencies = _require_mapping(config, "frequencies")
    values = frequencies.get("values_hz")
    if not isinstance(values, list):
        raise ValueError("config['frequencies']['values_hz'] must be a non-empty list")
    if not values:
        raise ValueError("config['frequencies']['values_hz'] must be a non-empty list")
    return [
        _validate_frequency_value(value, f"frequencies.values_hz[{index}]")
        for index, value in enumerate(values)
    ]


def materialize_frequency_file(
    path: Path,
    frequencies: list[float],
) -> None:
    if not frequencies:
        raise ValueError("frequencies must be a non-empty list")
    path.parent.mkdir(parents=True, exist_ok=True)
    with path.open("w", encoding="utf-8", newline="\n") as stream:
        for index, frequency_hz in enumerate(frequencies):
            value = _validate_frequency_value(frequency_hz, f"frequencies[{index}]")
            stream.write(f"{value:.12g}\n")


def _resolve_artifacts(config: dict[str, Any], config_file: Path, output_override: Path | None) -> tuple[Path, Path]:
    artifacts = _require_mapping(config, "artifacts")
    frequency_file_value = _require_path_value(artifacts, "frequencies_txt", "artifacts.frequencies_txt")
    raw_csv_value = _require_path_value(artifacts, "raw_csv", "artifacts.raw_csv")
    frequency_file = resolve_config_path(config_file, frequency_file_value)
    raw_csv = _resolve_override_path(output_override)
    if raw_csv is None:
        raw_csv = resolve_config_path(config_file, raw_csv_value)
    return frequency_file, raw_csv


def _resolve_runner_for_validation(
    config: dict[str, Any],
    config_file: Path,
    runner_override: Path | None,
) -> Path:
    if runner_override is not None:
        return Path(runner_override)

    runner = _require_mapping(config, "runner")
    runner_value = _require_path_value(runner, "executable", "runner.executable")
    return resolve_runner_executable(config_file, runner_value)


def build_runner_command(
    config: dict[str, Any],
    config_file: Path,
    frequency_file: Path,
    raw_csv: Path,
    runner_override: Path | None = None,
) -> list[str]:
    module = _require_nonempty_string(config, "module")
    mode = _require_nonempty_string(config, "mode")

    executable = _resolve_runner_for_validation(config, config_file, runner_override)

    command = [
        str(executable),
        "--module",
        module,
        "--mode",
        mode,
        "--freq-file",
        str(frequency_file),
        "--out",
        str(raw_csv),
    ]

    measurement = config.get("measurement", {})
    if measurement is None:
        measurement = {}
    if not isinstance(measurement, dict):
        raise ValueError("config['measurement'] must be an object when present")
    _append_options_from_mapping(command, measurement, "measurement")

    params = config.get("params", {})
    if params is None:
        params = {}
    if not isinstance(params, dict):
        raise ValueError("config['params'] must be an object when present")
    _append_options_from_mapping(command, params, "params")

    return command


def run_c_runner(
    command: list[str],
    timeout_s: float | None = None,
) -> subprocess.CompletedProcess[str]:
    return subprocess.run(command, check=True, capture_output=True, text=True, timeout=timeout_s)


def _validate_runner(
    runner_executable: Path,
    dry_run: bool,
) -> list[str]:
    if runner_executable.exists():
        return []
    message = f"runner executable does not exist: {runner_executable}"
    if dry_run:
        return [message]
    raise FileNotFoundError(message)


def run_from_config(
    config_file: Path,
    runner_override: Path | None = None,
    output_override: Path | None = None,
    dry_run: bool = False,
    timeout_s: float | None = None,
) -> RunnerResult:
    config_path = Path(config_file)
    config = load_config(config_path)
    frequencies = read_frequency_values(config)
    frequency_file, raw_csv = _resolve_artifacts(config, config_path, output_override)
    runner_path = _resolve_runner_for_validation(config, config_path, runner_override)
    command = build_runner_command(
        config=config,
        config_file=config_path,
        frequency_file=frequency_file,
        raw_csv=raw_csv,
        runner_override=runner_override,
    )

    warnings = _validate_runner(runner_path, dry_run=dry_run)
    if dry_run:
        return RunnerResult(
            command=command,
            frequency_file=frequency_file,
            raw_csv=raw_csv,
            frequency_count=len(frequencies),
            returncode=0,
            stdout="",
            stderr="",
            warnings=warnings,
        )

    materialize_frequency_file(frequency_file, frequencies)
    raw_csv.parent.mkdir(parents=True, exist_ok=True)
    run_result = run_c_runner(command, timeout_s=timeout_s)

    return RunnerResult(
        command=command,
        frequency_file=frequency_file,
        raw_csv=raw_csv,
        frequency_count=len(frequencies),
        returncode=run_result.returncode,
        stdout=run_result.stdout,
        stderr=run_result.stderr,
        warnings=[],
    )


def parse_args(argv: list[str] | None = None) -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Run the VOLOOP C frequency response runner from a JSON config.",
    )
    parser.add_argument("--config", required=True, type=Path, help="Runner config JSON path.")
    parser.add_argument(
        "--runner",
        type=Path,
        help="Override config['runner']['executable']. Relative paths use the current directory.",
    )
    parser.add_argument(
        "--output",
        type=Path,
        help="Override config['artifacts']['raw_csv']. Relative paths use the current directory.",
    )
    parser.add_argument("--dry-run", action="store_true", help="Print the C runner command without running it.")
    parser.add_argument("--timeout", type=float, help="C runner timeout in seconds.")
    return parser.parse_args(argv)


def _join_command(command: list[str]) -> str:
    return shlex.join(command)


def _print_completed_output(result: RunnerResult) -> None:
    if result.stdout:
        print(result.stdout, end="")
    if result.stderr:
        print(result.stderr, end="", file=sys.stderr)


def _print_called_process_error(exc: subprocess.CalledProcessError[str]) -> None:
    print(f"error: command failed with exit code {exc.returncode}: {_join_command(exc.cmd)}", file=sys.stderr)
    if exc.stdout:
        print(exc.stdout, end="", file=sys.stderr)
    if exc.stderr:
        print(exc.stderr, end="", file=sys.stderr)


def main(argv: list[str] | None = None) -> int:
    args = parse_args(argv)

    try:
        result = run_from_config(
            config_file=args.config,
            runner_override=args.runner,
            output_override=args.output,
            dry_run=args.dry_run,
            timeout_s=args.timeout,
        )
    except subprocess.CalledProcessError as exc:
        _print_called_process_error(exc)
        return exc.returncode if exc.returncode else 1
    except subprocess.TimeoutExpired as exc:
        print(f"error: command timed out after {exc.timeout} seconds: {_join_command(exc.cmd)}", file=sys.stderr)
        if exc.stdout:
            print(exc.stdout, end="", file=sys.stderr)
        if exc.stderr:
            print(exc.stderr, end="", file=sys.stderr)
        return 1
    except (OSError, ValueError, json.JSONDecodeError) as exc:
        print(f"error: {exc}", file=sys.stderr)
        return 1

    for warning in result.warnings:
        print(f"warning: {warning}", file=sys.stderr)

    if args.dry_run:
        print(_join_command(result.command))
        print(f"frequency file: {result.frequency_file}")
        print(f"frequency count: {result.frequency_count}")
        return 0

    _print_completed_output(result)
    print(f"runner: {_join_command(result.command)}")
    print(f"frequency file: {result.frequency_file}")
    print(f"frequency count: {result.frequency_count}")
    print(f"raw CSV: {result.raw_csv}")
    print(f"completed: {result.returncode}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
