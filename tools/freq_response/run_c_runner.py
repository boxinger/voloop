"""Run the VOLOOP C frequency response runner from a JSON config.

This wrapper only translates config fields into the C runner command line and
invokes the already-built runner. It does not generate frequencies, parse raw
CSV data, or render plots.
"""

from __future__ import annotations

import argparse
import json
import platform
import shlex
import subprocess
import sys
from dataclasses import dataclass
from pathlib import Path
from typing import Any


@dataclass(frozen=True)
class RunnerResult:
    command: list[str]
    raw_csv: Path
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


def build_runner_command(
    config: dict[str, Any],
    config_file: Path,
    runner_override: Path | None = None,
    output_override: Path | None = None,
) -> list[str]:
    module = _require_nonempty_string(config, "module")
    mode = _require_nonempty_string(config, "mode")

    runner = _require_mapping(config, "runner")
    paths = _require_mapping(config, "paths")

    runner_value = _require_path_value(runner, "executable", "runner.executable")
    frequencies_value = _require_path_value(paths, "frequencies", "paths.frequencies")
    raw_csv_value = _require_path_value(paths, "raw_csv", "paths.raw_csv")

    executable = _resolve_override_path(runner_override)
    if executable is None:
        executable = resolve_runner_executable(config_file, runner_value)

    raw_csv = _resolve_override_path(output_override)
    if raw_csv is None:
        raw_csv = resolve_config_path(config_file, raw_csv_value)

    frequencies = resolve_config_path(config_file, frequencies_value)

    command = [
        str(executable),
        "--module",
        module,
        "--mode",
        mode,
        "--freq-file",
        str(frequencies),
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


def _resolve_raw_csv(
    config: dict[str, Any],
    config_file: Path,
    output_override: Path | None,
) -> Path:
    if output_override is not None:
        return Path(output_override)

    paths = _require_mapping(config, "paths")
    raw_csv_value = _require_path_value(paths, "raw_csv", "paths.raw_csv")
    return resolve_config_path(config_file, raw_csv_value)


def _resolve_frequencies(config: dict[str, Any], config_file: Path) -> Path:
    paths = _require_mapping(config, "paths")
    frequencies_value = _require_path_value(paths, "frequencies", "paths.frequencies")
    return resolve_config_path(config_file, frequencies_value)


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


def _validate_for_run(
    runner_executable: Path,
    frequencies: Path,
    dry_run: bool,
) -> list[str]:
    warnings: list[str] = []

    if not runner_executable.exists():
        message = f"runner executable does not exist: {runner_executable}"
        if dry_run:
            warnings.append(message)
        else:
            raise FileNotFoundError(message)

    if not frequencies.exists():
        message = f"frequencies file does not exist: {frequencies}"
        if dry_run:
            warnings.append(message)
        else:
            raise FileNotFoundError(message)

    return warnings


def run_from_config(
    config_file: Path,
    runner_override: Path | None = None,
    output_override: Path | None = None,
    dry_run: bool = False,
    timeout_s: float | None = None,
) -> RunnerResult:
    config_path = Path(config_file)
    config = load_config(config_path)

    frequencies = _resolve_frequencies(config, config_path)
    raw_csv = _resolve_raw_csv(config, config_path, output_override)
    command = build_runner_command(
        config=config,
        config_file=config_path,
        runner_override=runner_override,
        output_override=output_override,
    )

    if dry_run:
        runner_path = _resolve_runner_for_validation(config, config_path, runner_override)
        warnings = _validate_for_run(runner_path, frequencies, dry_run=True)
        return RunnerResult(
            command=command,
            raw_csv=raw_csv,
            returncode=0,
            stdout="",
            stderr="",
            warnings=warnings,
        )

    if not frequencies.exists():
        raise FileNotFoundError(f"frequencies file does not exist: {frequencies}")

    runner_path = _resolve_runner_for_validation(config, config_path, runner_override)
    if not runner_path.exists():
        raise FileNotFoundError(f"runner executable does not exist: {runner_path}")

    raw_csv.parent.mkdir(parents=True, exist_ok=True)
    run_result = run_c_runner(command, timeout_s=timeout_s)

    return RunnerResult(
        command=command,
        raw_csv=raw_csv,
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
        help="Override config['paths']['raw_csv']. Relative paths use the current directory.",
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
        return 0

    _print_completed_output(result)
    print(f"runner: {_join_command(result.command)}")
    print(f"raw CSV: {result.raw_csv}")
    print(f"completed: {result.returncode}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
