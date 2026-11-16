import time
from pathlib import Path

from j288.bench import MotorBench, default_settings_path, short_port


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


def test_short_port_on_each_platform():
    assert short_port("COM12") == "COM12"
    assert short_port(r"\\.\COM10") == "COM10"
    assert short_port("/dev/cu.usbmodem123") == "usbmodem123"
    assert short_port("/dev/ttyACM0") == "ttyACM0"


def test_choose_port_opens_only_that_device(monkeypatch):
    opened = []
    monkeypatch.setattr(
        "j288.bench.list_serial_ports",
        lambda preferred="": ["COM3", "COM5", "/dev/ttyACM0"],
    )
    bench = MotorBench(sim=True)

    def open_port(port):
        opened.append(port)

    bench._open = open_port
    bench._close_ser = lambda: None
    bench.send = lambda wait_s=0.02, reply=None: False

    bench.port = "COM5"
    bench.port_pref = "COM5"
    bench.choose_port(1)
    assert opened == ["/dev/ttyACM0"]

    opened.clear()
    bench.port = "COM5"
    bench.port_pref = "COM5"
    bench.choose_port(-1)
    assert opened == ["COM3"]

    opened.clear()
    bench.apply_command("port COM9")
    assert opened == ["COM9"]

    opened.clear()
    bench.port_pref = "COM5"
    bench._port_only = False
    bench.connect()
    assert opened == ["COM3", "COM5", "/dev/ttyACM0"]


def test_connect_keeps_the_answering_id():
    bench = MotorBench(sim=True)
    bench.port_pref = "COM3"
    bench._port_only = True
    bench._open = lambda port: None
    bench._close_ser = lambda: None
    asked = []

    def send(wait_s=0.02, reply=None):
        asked.append(bench.motor_id)
        bench.tx_count += 1
        if bench.motor_id != 2:
            bench.miss += 1
            return False
        bench.miss = 0
        bench.correct = 1
        bench.fb_id = 2
        return True

    bench.send = send
    bench.motor_id = 0
    assert bench.connect() is True
    assert bench.motor_id == 2
    assert asked == [0, 1, 2]
    assert "id 2" in bench.log_lines

    asked.clear()
    bench.motor_id = 2
    bench._port_only = True
    assert bench.connect() is True
    assert asked == [2]
    assert bench.motor_id == 2


def test_shutdown_returns_while_a_probe_holds_the_bus():
    bench = MotorBench(sim=True)
    bench._busy.acquire()
    started = time.monotonic()
    try:
        bench.shutdown()
    finally:
        if bench._busy.locked():
            bench._busy.release()
    assert time.monotonic() - started < 0.6
    assert bench._closed is True


def test_debug_log_keeps_five_files(tmp_path, monkeypatch):
    monkeypatch.setattr("j288.log.log_dir", lambda: tmp_path)
    from j288.log import note

    for day in range(1, 8):
        (tmp_path / ("j288_2026-01-%02d.log" % day)).write_text("old\n")
    note("today")
    today = time.strftime("%Y-%m-%d")
    names = sorted(path.name for path in tmp_path.glob("j288_*.log"))
    assert len(names) == 5
    assert names[-1] == "j288_%s.log" % today
    assert "j288_2026-01-01.log" not in names
    assert "j288_2026-01-03.log" not in names


def test_debug_log_keeps_the_newest_part(tmp_path, monkeypatch):
    path = tmp_path / "j288.log"
    monkeypatch.setattr("j288.log.log_path", lambda: path)
    monkeypatch.setattr("j288.log.MAX_BYTES", 200)
    monkeypatch.setattr("j288.log.KEEP_BYTES", 100)
    from j288.log import note

    for i in range(20):
        note("line-%02d-padding" % i)
    note("NEWEST-MARK")
    data = path.read_text()
    assert "NEWEST-MARK" in data
    assert "line-00-" not in data
    assert len(data.encode()) <= 200


def test_arm_holds_the_shaft_position():
    bench = MotorBench(sim=True)
    bench.tick()
    bench.meas_pos = 0.84
    bench.cmd_pos = 0.0
    bench.toggle_armed()
    assert bench.cmd_pos == 0.84

    cold = MotorBench(sim=False, settings_path="missing-settings.json")
    cold.cmd_pos = 1.2
    cold.meas_pos = 0.0
    cold.toggle_armed()
    assert cold.cmd_pos == 1.2


def test_position_integral_grows_while_armed():
    bench = MotorBench(sim=True)
    bench.tick()
    bench.ki = 2.0
    bench.toggle_armed()
    bench.cmd_pos = bench.meas_pos + 0.4
    now = time.monotonic()
    bench.torque_command(now)
    bench.torque_command(now + 0.05)
    assert bench.tor_i > 0.0
