import time
from pathlib import Path

from j288.bench import MotorBench, default_settings_path


def test_settings_path_stays_next_to_the_package():
    path = default_settings_path()
    assert path.name == "motor_settings.json"
    assert path.parent == Path(__file__).resolve().parents[1]


def test_settings_roundtrip(tmp_path):
    path = tmp_path / "motor_settings.json"
    bench = MotorBench(sim=False, settings_path=path)
    bench.motor_id = 2
    bench.kp = 3.25
    bench.kd = 0.4
    bench.ki = 1.5
    bench.wave_lo = 4.7
    bench.wave_hi = 6.3
    bench._save_settings()

    again = MotorBench(sim=False, settings_path=path)
    assert again.motor_id == 2
    assert again.kp == 3.25
    assert again.kd == 0.4
    assert again.ki == 1.5
    assert again.wave_lo == 4.7
    assert again.wave_hi == 6.3


def test_wave_waits_for_feedback_then_stop_locks():
    bench = MotorBench(sim=True)
    bench.toggle_wave()
    assert bench.wave_on is False

    bench.tick()
    bench.toggle_wave()
    assert bench.wave_on is True
    assert bench.armed is True

    bench.stop()
    assert bench.armed is False
    assert bench.wave_on is False
    assert bench.cmd_spd == 0.0


def test_clear_is_one_raw_command():
    bench = MotorBench(sim=True)
    bench.tick()
    bench.toggle_armed()
    bench.apply_command("clear")
    assert bench.armed is False
    assert bench.command_mode() == 6
    bench.tick()
    assert bench.command_mode() == 0
    assert bench.merror == 0


def test_position_integral_grows_while_armed():
    bench = MotorBench(sim=True)
    bench.tick()
    bench.ki = 2.0
    bench.cmd_pos = bench.meas_pos + 0.4
    bench.toggle_armed()
    now = time.monotonic()
    bench.torque_command(now)
    bench.torque_command(now + 0.05)
    assert bench.tor_i > 0.0
