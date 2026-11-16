import os
import struct
import sys
import threading
import time

import pytest

from j288.bus import SerialBus, list_serial_ports
from j288.motor import MISS_PERIOD, Motor
from j288.protocol import build_packet, crc32_lookup

needs_pipe = pytest.mark.skipif(sys.platform == "win32", reason="pipe stand-in is posix")


def _frame(motor_id, pos=32768):
    body = bytes([motor_id & 0x0F]) + struct.pack("<bBBhhiIH", 30, 40, 48, 0, 0, pos, 0, 0)
    body += b"\x00\x00"
    return b"\xFC\xEE" + body + struct.pack("<I", crc32_lookup(body))


class PipePort:
    """Stand-in port. The reply id matches the command that was just written."""

    def __init__(self, delay=0.0):
        self._r, self._w = os.pipe()
        self.delay = delay
        self.writes = []
        self.reads = 0
        self.events = []
        self.overlap = False
        self._busy = False
        self._lock = threading.Lock()

    def fileno(self):
        return self._r

    def write(self, data):
        with self._lock:
            if self._busy:
                self.overlap = True
            self._busy = True
        try:
            data = bytes(data)
            self.events.append(("write", data[2] & 0x0F, time.monotonic()))
            if self.delay:
                time.sleep(self.delay)
            self.writes.append(data)
            os.write(self._w, _frame(data[2] & 0x0F))
            self.events.append(("wrote", data[2] & 0x0F, time.monotonic()))
            return len(data)
        finally:
            with self._lock:
                self._busy = False

    def flush(self):
        pass

    def read(self, n):
        self.reads += 1
        self.events.append(("read", None, time.monotonic()))
        return os.read(self._r, n)

    def reset_input_buffer(self):
        import fcntl

        flags = fcntl.fcntl(self._r, fcntl.F_GETFL)
        fcntl.fcntl(self._r, fcntl.F_SETFL, flags | os.O_NONBLOCK)
        try:
            while os.read(self._r, 64):
                pass
        except BlockingIOError:
            pass
        finally:
            fcntl.fcntl(self._r, fcntl.F_SETFL, flags)

    def close(self):
        for fd in (self._r, self._w):
            try:
                os.close(fd)
            except OSError:
                pass


class StubBus:
    def __init__(self, frame=b""):
        self.frame = frame
        self.calls = []

    def exchange(self, packet, wait_s=0.02, reply=True):
        self.calls.append((bytes(packet), wait_s, reply))

        class Result:
            pass

        result = Result()
        result.frame = self.frame if reply else b""
        result.rxlen = len(result.frame)
        result.rtt_ms = 1.0
        return result


@needs_pipe
def test_two_motors_take_turns_on_one_port():
    port = PipePort(delay=0.03)
    bus = SerialBus()
    bus.open(ser=port)
    try:
        motors = [Motor(0, bus), Motor(1, bus)]
        barrier = threading.Barrier(2)
        errors = []

        def run(motor):
            try:
                barrier.wait(timeout=2)
                motor.armed = True
                motor.cmd_pos = 1.0 + motor.motor_id
                assert motor.send(wait_s=0.3)
            except Exception as exc:
                errors.append(exc)

        threads = [threading.Thread(target=run, args=(motor,)) for motor in motors]
        for thread in threads:
            thread.start()
        for thread in threads:
            thread.join(timeout=2)
        assert errors == []
        assert [thread.is_alive() for thread in threads] == [False, False]
        assert port.overlap is False
        assert sorted(packet[2] & 0x0F for packet in port.writes) == [0, 1]
        assert [motor.fb_id for motor in motors] == [0, 1]
        assert all(motor.correct == 1 for motor in motors)
        kinds = [event[0] for event in port.events]
        assert kinds.count("write") == 2
        assert kinds.count("wrote") == 2
        first_wrote = kinds.index("wrote")
        second_write = kinds.index("write", first_wrote)
        assert kinds[0] == "write"
        assert "read" not in kinds[:first_wrote]
        assert "read" in kinds[first_wrote + 1 : second_write]
        assert set(kinds[first_wrote + 1 : second_write]) == {"read"}
    finally:
        bus.close()


@needs_pipe
def test_broadcast_does_not_wait_for_a_reply():
    port = PipePort()
    bus = SerialBus()
    bus.open(ser=port)
    try:
        started = time.monotonic()
        reply = bus.exchange(build_packet(15, 0, 0, 0, 0, 0, 0, 0), reply=False, wait_s=0.5)
        elapsed = time.monotonic() - started
        assert elapsed < 0.2
        assert reply.frame == b""
        assert port.reads == 0
        assert len(port.writes) == 1
    finally:
        bus.close()


def test_closed_port_rejects_a_command():
    bus = SerialBus()
    try:
        bus.exchange(b"\x00" * 20)
    except RuntimeError as exc:
        assert "closed" in str(exc)
    else:
        raise AssertionError("closed bus accepted a command")


def test_motor_poll_period_quiet_and_clear():
    good = _frame(0)
    bus = StubBus(good)
    motor = Motor(0, bus, rate_hz=1)
    assert motor.poll() is True
    assert motor.poll() is False
    assert len(bus.calls) == 1

    motor.rate_hz = 40
    motor.miss = 5
    motor.next_poll = 0.0
    started = time.monotonic()
    motor.poll()
    assert motor.next_poll - started >= MISS_PERIOD - 0.02

    motor.quiet_until = time.monotonic() + 5
    calls = len(bus.calls)
    assert motor.send() is False
    assert len(bus.calls) == calls

    motor.quiet_until = 0.0
    motor.clear()
    motor.send()
    packet = bus.calls[-1][0]
    assert (packet[2] >> 4) & 0x07 == 6
    motor.send()
    assert (bus.calls[-1][0][2] >> 4) & 0x07 == 0

    motor.reset()
    motor.send()
    assert motor.quiet_until > time.monotonic()


class _Listed:
    def __init__(self, device, description="", hwid="", manufacturer="", vid=None):
        self.device = device
        self.description = description
        self.hwid = hwid
        self.manufacturer = manufacturer
        self.vid = vid


def test_list_serial_ports_keeps_the_three_shapes(monkeypatch):
    infos = [
        _Listed("/dev/cu.usbmodem1"),
        _Listed("/dev/tty.usbmodem1"),
        _Listed("/dev/cu.Bluetooth-Incoming-Port"),
        _Listed("/dev/ttyUSB0"),
        _Listed("/dev/ttyACM0"),
        _Listed("/dev/ttyAMA0"),
        _Listed("/dev/ttyS0"),
        _Listed("COM3", description="USB Serial Device"),
        _Listed(r"\\.\COM10"),
        _Listed(
            "COM4",
            description="Standard Serial over Bluetooth link",
            hwid=r"BTHENUM\{00001101}",
        ),
    ]
    monkeypatch.setattr("serial.tools.list_ports.comports", lambda: infos)
    assert list_serial_ports("COM9") == [
        "COM9",
        "/dev/cu.usbmodem1",
        "/dev/ttyACM0",
        "/dev/ttyUSB0",
        "COM3",
        r"\\.\COM10",
    ]


class TimedPort:
    """pyserial-shaped port: read(n) waits out the timeout when the buffer is short."""

    def __init__(self, frame):
        self.timeout = 0
        self._buf = b""
        self._frame = frame

    def fileno(self):
        raise OSError("not a socket")

    def write(self, data):
        self._buf = self._frame
        return len(data)

    def flush(self):
        pass

    def reset_input_buffer(self):
        self._buf = b""

    def close(self):
        pass

    @property
    def in_waiting(self):
        return len(self._buf)

    def read(self, n):
        if self.timeout:
            time.sleep(self.timeout)
        if not self._buf:
            return b""
        take = self._buf[:n]
        self._buf = self._buf[n:]
        return take


class StuckFlush:
    """flush blocks, the way POSIX tcdrain does on a stuck USB adapter."""

    def __init__(self):
        self.timeout = 0
        self.flush_calls = 0
        self.release = threading.Event()

    def write(self, data):
        return len(data)

    def flush(self):
        self.flush_calls += 1
        self.release.wait(2.0)

    def reset_input_buffer(self):
        pass

    def close(self):
        self.release.set()

    @property
    def in_waiting(self):
        return 0

    def read(self, n):
        delay = float(self.timeout or 0)
        if delay > 0:
            time.sleep(min(delay, 0.05))
        return b""


def test_exchange_does_not_wait_on_flush():
    port = StuckFlush()
    bus = SerialBus()
    bus.open(ser=port)
    done = threading.Event()

    def run():
        bus.exchange(build_packet(0, 0, 1, 0, 0, 0, 0, 0), wait_s=0.05)
        done.set()

    thread = threading.Thread(target=run)
    thread.start()
    try:
        assert done.wait(0.4)
        assert port.flush_calls == 0
    finally:
        port.release.set()
        thread.join(1.0)
        bus.close()


class SilentPort:
    """No bytes. read waits out whatever timeout the caller set."""

    def __init__(self):
        self.timeout = 0

    def write(self, data):
        return len(data)

    def flush(self):
        pass

    def reset_input_buffer(self):
        pass

    def close(self):
        pass

    @property
    def in_waiting(self):
        return 0

    def read(self, n):
        delay = float(getattr(self, "_timeout", 0) or 0)
        if delay <= 0:
            delay = float(self.timeout or 0)
        if delay > 0:
            time.sleep(delay)
        return b""


class ReconfigurePort(SilentPort):
    """The timeout setter stands in for the 6 Mbps ioctl."""

    def __init__(self):
        self._timeout = 0
        self.sets = 0
        self._value = 0

    @property
    def timeout(self):
        return self._value

    @timeout.setter
    def timeout(self, value):
        self.sets += 1
        self._value = value
        time.sleep(0.4)


def test_a_miss_does_not_reconfigure_the_port():
    port = ReconfigurePort()
    bus = SerialBus()
    bus.open(ser=port)
    try:
        started = time.monotonic()
        reply = bus.exchange(build_packet(0, 0, 1, 0, 0, 0, 0, 0), wait_s=0.05)
        elapsed = time.monotonic() - started
        assert reply.frame == b""
        assert elapsed < 0.25
        if os.name != "nt":
            assert port.sets == 0
    finally:
        bus.close()


def test_silent_port_returns_within_the_wait():
    port = SilentPort()
    bus = SerialBus()
    bus.open(ser=port)
    try:
        started = time.monotonic()
        reply = bus.exchange(build_packet(0, 0, 1, 0, 0, 0, 0, 0), wait_s=0.05)
        elapsed = time.monotonic() - started
        assert reply.frame == b""
        assert elapsed < 0.25
        assert port.timeout == 0
    finally:
        bus.close()


def test_reply_returns_when_the_frame_is_already_waiting():
    port = TimedPort(_frame(0))
    bus = SerialBus()
    bus.open(ser=port)
    try:
        started = time.monotonic()
        reply = bus.exchange(build_packet(0, 0, 0, 0, 0, 0, 0, 0), wait_s=0.4)
        elapsed = time.monotonic() - started
        assert len(reply.frame) == 26
        assert reply.frame.startswith(b"\xFC\xEE")
        assert elapsed < 0.1
        assert port.timeout == 0
    finally:
        bus.close()


def test_reply_for_another_id_is_a_miss():
    bus = StubBus(_frame(1))
    motor = Motor(0, bus)
    assert motor.send() is False
    assert motor.miss == 1
    assert motor.correct == 0
    assert motor.meas_pos == 0.0
