"""One serial port shared by every motor on the wire.

The bus is half duplex. exchange() writes one command and, when a reply
is expected, reads it before the next caller can use the port.
"""

import os
import threading
import time

from j288.protocol import BAUD


REPLY_WAIT = 0.02
PROBE_WAIT = 0.05


class Reply:
    def __init__(self, frame, rxlen, rtt_ms):
        self.frame = frame
        self.rxlen = rxlen
        self.rtt_ms = rtt_ms


class SerialBus:
    """One adapter. exchange() is the only call that touches the port.

    The lock keeps two motors from writing at once. The caller blocks
    until its own exchange finishes.
    """

    def __init__(self, baud=BAUD):
        self.baud = baud
        self.port = ""
        self._ser = None
        self._lock = threading.Lock()

    @property
    def is_open(self):
        return self._ser is not None

    def open(self, port="", ser=None):
        """Open a port, or take an already open serial object."""
        with self._lock:
            self._close_locked()
            if ser is None:
                if not port:
                    raise ValueError("port is required")
                ser = _open_port(port, self.baud)
            self._ser = ser
            self.port = str(port or getattr(ser, "port", "") or "")

    def close(self):
        with self._lock:
            self._close_locked()

    def _close_locked(self):
        ser = self._ser
        self._ser = None
        if ser is not None:
            try:
                ser.close()
            except Exception:
                pass

    def exchange(self, packet, wait_s=REPLY_WAIT, reply=True):
        """Write one frame. Read one reply when reply is true.

        reply=False does not read. Use it for id 15. The next exchange
        drops bytes still in the adapter, so a late reply is not applied
        to the next motor.
        """
        packet = bytes(packet)
        with self._lock:
            ser = self._ser
            if ser is None:
                raise RuntimeError("port is closed")
            started = time.monotonic()
            ser.reset_input_buffer()
            wrote = ser.write(packet)
            if wrote != len(packet):
                raise RuntimeError("short write %s/%s" % (wrote, len(packet)))
            # Do not flush. pyserial's flush waits until the bytes have left
            # the adapter, and that wait has no timeout. A stuck USB port
            # freezes the caller. write() already applied write_timeout.
            if not reply:
                return Reply(b"", 0, (time.monotonic() - started) * 1000.0)
            frame, rxlen = _read_frame(ser, wait_s)
            return Reply(bytes(frame) if frame else b"", rxlen, (time.monotonic() - started) * 1000.0)


def list_serial_ports(preferred=""):
    """macOS /dev/cu.*, Linux /dev/ttyUSB* and /dev/ttyACM*, Windows COMn.

    The macOS /dev/tty.* twin is skipped. Bluetooth serial is skipped.
    A typed path is always first.
    """
    preferred = str(preferred or "")
    try:
        from serial.tools import list_ports
    except ImportError:
        return [preferred] if preferred else []
    found = []
    for info in list_ports.comports():
        device = getattr(info, "device", "") or ""
        if device and _port_ok(info):
            found.append(device)
    found = sorted(set(found))
    if preferred:
        if preferred in found:
            found.remove(preferred)
        found.insert(0, preferred)
    return found


def _port_ok(info):
    device = info.device
    blob = " ".join(
        str(getattr(info, name, "") or "")
        for name in ("device", "description", "hwid", "manufacturer")
    ).lower()
    if any(word in blob for word in ("bluetooth", "debug-console", "wlan-debug", "bthenum")):
        return False
    path = str(device).replace("\\", "/")
    if path.startswith("/dev/tty."):
        return False
    name = path.rsplit("/", 1)[-1].upper()
    if str(device).startswith("/dev/cu."):
        return True
    if str(device).startswith(("/dev/ttyUSB", "/dev/ttyACM")):
        return True
    return len(name) > 3 and name.startswith("COM") and name[3:].isdigit()


def _open_port(port, baud):
    import serial

    kwargs = dict(
        baudrate=baud,
        timeout=0,
        write_timeout=0.05,
        bytesize=serial.EIGHTBITS,
        parity=serial.PARITY_NONE,
        stopbits=serial.STOPBITS_ONE,
    )
    try:
        return serial.Serial(port, exclusive=True, **kwargs)
    except (TypeError, ValueError):
        return serial.Serial(port, **kwargs)


def _read_frame(ser, budget):
    """Read until one reply is in, or the budget ends.

    select() cannot wait on a Windows COM port. A blocking read(n) also
    holds until n bytes arrive, so a finished 26-byte frame would sit for
    the whole budget. Bytes already waiting are taken at once. Otherwise
    one byte is read with the time that is left.
    """
    buf = b""
    deadline = time.monotonic() + max(0.0, float(budget))
    previous = getattr(ser, "_timeout", getattr(ser, "timeout", None))
    try:
        while len(buf) < 64 and time.monotonic() < deadline:
            remain = deadline - time.monotonic()
            if remain <= 0:
                break
            waiting = 0
            try:
                waiting = int(ser.in_waiting)
            except Exception:
                waiting = 0
            if waiting:
                _set_read_timeout(ser, 0)
                chunk = ser.read(min(64 - len(buf), waiting))
            else:
                _set_read_timeout(ser, remain)
                chunk = ser.read(1)
            if not chunk:
                break
            buf += chunk
            idx = buf.find(b"\xFC\xEE")
            if idx >= 0 and len(buf) >= idx + 26:
                return buf[idx : idx + 26], len(buf)
    finally:
        try:
            _set_read_timeout(ser, 0 if previous is None else previous)
        except Exception:
            pass
    return buf, len(buf)


def _set_read_timeout(ser, seconds):
    """Arm read() for this long.

    On macOS the timeout setter reconfigures the port. At 6 Mbps that
    ioctl blocks for about a minute on the adapter's other interface,
    so a miss never reaches the read budget. POSIX read waits in select
    on _timeout. Windows applies the budget through the setter.
    """
    if os.name == "nt":
        ser.timeout = seconds
        return
    ser._timeout = seconds
