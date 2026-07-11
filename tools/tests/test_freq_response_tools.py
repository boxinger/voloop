from __future__ import annotations

import json
import math
import subprocess
import sys
from pathlib import Path

import pytest

sys.path.insert(0, str(Path(__file__).parents[1]))

from freq_response import generate_frequencies
from freq_response import run_c_runner


def write_config(path: Path, values_hz=None) -> None:
    config = {
        "schema_version": 1,
        "name": "test_config",
        "module": "pid",
        "mode": "one_zero",
        "runner": {
            "executable": "runner_exe",
        },
        "frequencies": {},
        "artifacts": {
            "frequencies_txt": "out/frequencies.txt",
            "raw_csv": "out/raw.csv",
        },
        "measurement": {
            "sample_rate_hz": 10000,
            "input_amplitude": 1.0,
            "warmup_cycles": 20,
            "measure_cycles": 10,
            "min_samples_per_cycle": 32,
            "max_samples_per_point": 2000000,
            "output_abs_limit": 1000000,
            "gain_floor": 1e-12,
        },
        "params": {
            "pid-gain": 1.0,
            "pid-zero-hz": 10.0,
        },
    }
    if values_hz is not None:
        config["frequencies"]["values_hz"] = values_hz
    path.write_text(json.dumps(config, indent=2) + "\n", encoding="utf-8")


def read_config(path: Path) -> dict:
    return json.loads(path.read_text(encoding="utf-8"))


def test_generate_log_frequencies_keeps_sorted_unique_values() -> None:
    frequencies = generate_frequencies.generate_log_frequencies(
        start_hz=1.0,
        stop_hz=100.0,
        points_per_decade=1,
        include_hz=[10.0, 50.0],
    )
    assert frequencies == [1.0, 10.0, 50.0, 100.0]


def test_config_update_defaults_to_merge_sorted_unique(tmp_path: Path) -> None:
    config_file = tmp_path / "config.json"
    write_config(config_file, [1.0, 10.0, 100.0])

    result = generate_frequencies.update_config_frequencies(
        config_file,
        [5.0, 10.0, 50.0],
    )

    assert result.mode == generate_frequencies.FrequencyUpdateMode.MERGE
    assert result.final_values_hz == (1.0, 5.0, 10.0, 50.0, 100.0)
    assert result.written is True
    assert read_config(config_file)["frequencies"]["values_hz"] == [1.0, 5.0, 10.0, 50.0, 100.0]


def test_config_update_replace(tmp_path: Path) -> None:
    config_file = tmp_path / "config.json"
    write_config(config_file, [1.0, 10.0, 100.0])

    result = generate_frequencies.update_config_frequencies(
        config_file,
        [10.0, 31.6227766017, 100.0],
        mode=generate_frequencies.FrequencyUpdateMode.REPLACE,
    )

    assert result.final_values_hz == (10.0, 31.6227766017, 100.0)
    assert read_config(config_file)["frequencies"]["values_hz"] == [10.0, 31.6227766017, 100.0]


def test_config_update_dry_run_does_not_write(tmp_path: Path) -> None:
    config_file = tmp_path / "config.json"
    write_config(config_file, [1.0])
    before = config_file.read_text(encoding="utf-8")

    result = generate_frequencies.update_config_frequencies(
        config_file,
        [2.0],
        dry_run=True,
    )

    assert result.final_values_hz == (1.0, 2.0)
    assert result.written is False
    assert config_file.read_text(encoding="utf-8") == before


def test_config_update_missing_values_treats_as_empty(tmp_path: Path) -> None:
    config_file = tmp_path / "config.json"
    write_config(config_file)

    result = generate_frequencies.update_config_frequencies(config_file, [2.0])

    assert result.previous_values_hz == ()
    assert result.final_values_hz == (2.0,)


@pytest.mark.parametrize(
    ("field", "value"),
    [
        ("schema_version", 2),
        ("frequencies", []),
    ],
)
def test_config_update_rejects_bad_schema_or_frequencies_object(tmp_path: Path, field: str, value) -> None:
    config_file = tmp_path / "config.json"
    write_config(config_file, [1.0])
    config = read_config(config_file)
    config[field] = value
    config_file.write_text(json.dumps(config), encoding="utf-8")

    with pytest.raises(ValueError):
        generate_frequencies.update_config_frequencies(config_file, [2.0])


def test_config_update_rejects_bad_values_type(tmp_path: Path) -> None:
    config_file = tmp_path / "config.json"
    write_config(config_file, [1.0])
    config = read_config(config_file)
    config["frequencies"]["values_hz"] = "1"
    config_file.write_text(json.dumps(config), encoding="utf-8")

    with pytest.raises(ValueError):
        generate_frequencies.update_config_frequencies(config_file, [2.0])


@pytest.mark.parametrize("bad_value", [math.nan, math.inf, 0.0, -1.0, True])
def test_frequency_values_reject_invalid_values(tmp_path: Path, bad_value) -> None:
    config_file = tmp_path / "config.json"
    write_config(config_file, [1.0])

    with pytest.raises(ValueError):
        generate_frequencies.update_config_frequencies(config_file, [bad_value])

    config = read_config(config_file)
    config["frequencies"]["values_hz"] = [bad_value]
    config_file.write_text(json.dumps(config, allow_nan=True), encoding="utf-8")
    with pytest.raises(ValueError):
        run_c_runner.read_frequency_values(config)


def test_parse_args_rejects_out_with_replace() -> None:
    with pytest.raises(SystemExit):
        generate_frequencies.parse_args(
            [
                "--out",
                "frequencies.txt",
                "--start-hz",
                "1",
                "--stop-hz",
                "10",
                "--points-per-decade",
                "1",
                "--replace",
            ]
        )


def test_parse_args_requires_config_or_out() -> None:
    with pytest.raises(SystemExit):
        generate_frequencies.parse_args(
            [
                "--start-hz",
                "1",
                "--stop-hz",
                "10",
                "--points-per-decade",
                "1",
            ]
        )


def test_run_c_runner_reads_new_fields_and_builds_command(tmp_path: Path) -> None:
    config_file = tmp_path / "config.json"
    write_config(config_file, [20.0, 10.0])

    config = run_c_runner.load_config(config_file)
    frequencies = run_c_runner.read_frequency_values(config)
    frequency_file, raw_csv = run_c_runner._resolve_artifacts(config, config_file, None)
    command = run_c_runner.build_runner_command(config, config_file, frequency_file, raw_csv)

    assert frequencies == [20.0, 10.0]
    assert frequency_file == tmp_path / "out/frequencies.txt"
    assert raw_csv == tmp_path / "out/raw.csv"
    assert "--freq-file" in command
    assert str(frequency_file) in command
    assert "--out" in command
    assert str(raw_csv) in command


def test_materialize_frequency_file_keeps_order_and_12g_format(tmp_path: Path) -> None:
    output = tmp_path / "nested" / "frequencies.txt"

    run_c_runner.materialize_frequency_file(output, [20.0, 10.1234567891234])

    assert output.read_text(encoding="utf-8") == "20\n10.1234567891\n"


def test_run_from_config_overwrites_frequency_file(tmp_path: Path, monkeypatch: pytest.MonkeyPatch) -> None:
    config_file = tmp_path / "config.json"
    write_config(config_file, [30.0, 10.0])
    frequency_file = tmp_path / "out" / "frequencies.txt"
    frequency_file.parent.mkdir()
    frequency_file.write_text("old\n", encoding="utf-8")
    runner = tmp_path / "runner.exe"
    runner.write_text("", encoding="utf-8")

    def fake_run(command: list[str], timeout_s: float | None = None) -> subprocess.CompletedProcess[str]:
        out = Path(command[command.index("--out") + 1])
        out.write_text("ok\n", encoding="utf-8")
        return subprocess.CompletedProcess(command, 0, "", "")

    monkeypatch.setattr(run_c_runner, "run_c_runner", fake_run)

    result = run_c_runner.run_from_config(
        config_file,
        runner_override=runner,
        output_override=tmp_path / "out" / "raw.csv",
        timeout_s=10,
    )

    assert result.frequency_file == frequency_file
    assert result.frequency_count == 2
    assert frequency_file.read_text(encoding="utf-8") == "30\n10\n"
    assert result.raw_csv.read_text(encoding="utf-8") == "ok\n"


def test_run_from_config_dry_run_has_no_file_side_effects(tmp_path: Path) -> None:
    config_file = tmp_path / "config.json"
    write_config(config_file, [10.0])

    result = run_c_runner.run_from_config(config_file, dry_run=True)

    assert result.returncode == 0
    assert result.frequency_count == 1
    assert result.warnings
    assert not (tmp_path / "out").exists()


def test_run_c_runner_rejects_obsolete_paths(tmp_path: Path) -> None:
    config_file = tmp_path / "config.json"
    write_config(config_file, [10.0])
    config = read_config(config_file)
    config["paths"] = {"frequencies": "old.txt", "raw_csv": "old.csv"}
    config_file.write_text(json.dumps(config), encoding="utf-8")

    with pytest.raises(ValueError, match="obsolete field 'paths'"):
        run_c_runner.load_config(config_file)


def test_examples_use_schema_v1_and_new_frequency_fields() -> None:
    examples_dir = Path(__file__).parents[1] / "freq_response" / "examples"
    for config_file in examples_dir.glob("*.json"):
        config = json.loads(config_file.read_text(encoding="utf-8"))
        assert config["schema_version"] == 1
        assert "paths" not in config
        assert config["frequencies"]["values_hz"]
        assert "frequencies_txt" in config["artifacts"]
        assert "raw_csv" in config["artifacts"]
        run_c_runner.read_frequency_values(config)


def test_modules_import_without_side_effects() -> None:
    assert generate_frequencies.main is not None
    assert run_c_runner.main is not None
