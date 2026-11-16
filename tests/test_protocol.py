import importlib.util
import struct
from pathlib import Path

from j288.protocol import (
    build_packet,
    crc32_lookup,
    error_names,
    parse_feedback,
    warn_names,
)


def _vendor():
    path = Path(__file__).resolve().parents[1] / "vendor" / "J288-S288__PythonDemo.py"
    spec = importlib.util.spec_from_file_location("unitree_demo", path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def _feedback(mode_byte, temp, sensor, vol, torque, speed, pos, merror, mixed):
    body = bytes([mode_byte]) + struct.pack(
        "<bBBhhiIH", temp, sensor, vol, torque, speed, pos, merror, mixed
    )
    body += b"\x00\x00"
    assert len(body) == 20
    return b"\xFC\xEE" + body + struct.pack("<I", crc32_lookup(body))


def test_command_matches_vendor_demo():
    demo = _vendor()
    ours = build_packet(0, 1, 0, 0.0, 0.5, 1.25, 1.5, 1.2)
    theirs = demo.build_control_packet(0, 1, 0, 0.0, 0.5, 1.25, 1.5, 1.2)
    assert ours == theirs


def test_clear_frame_sends_raw_counts():
    packet = build_packet(0, 6, 0, -256, 0, 0, 0, 0, raw=True)
    assert len(packet) == 20
    assert (packet[2] >> 4) & 0x07 == 6
    torque, speed, position, kp, kd = struct.unpack_from("<hhIhh", packet, 4)
    assert (torque, speed, position, kp, kd) == (-256, 0, 0, 0, 0)
    assert struct.unpack_from("<I", packet, 16)[0] == crc32_lookup(packet[:16])


def test_negative_position_packs():
    packet = build_packet(0, 1, 1, 0.0, 0.0, -1.5, 0.0, 0.0)
    assert packet[:2] == b"\xFE\xEE"
    assert len(packet) == 20


def test_feedback_keeps_high_winding_temperature():
    frame = _feedback(0x11, 36, 200, 48, 1000, 256, 32768, 0x04, 0)
    parsed = parse_feedback(frame)
    assert parsed["sensor"] == 200
    assert parsed["temp"] == 36
    assert parsed["vol"] == 24.0
    assert parsed["mode"] == 1
    assert error_names(parsed["merror"]) == ["continuous overvoltage"]


def test_bad_frame_is_dropped():
    frame = bytearray(_feedback(0x11, 30, 40, 48, 0, 0, 0, 0, 0))
    frame[-1] ^= 0xFF
    assert parse_feedback(bytes(frame)) is None
    assert parse_feedback(b"\x00" * 26) is None
    assert parse_feedback(b"") is None


def test_unknown_fault_bit_is_shown_as_hex():
    assert error_names(1 << 23) == ["0x800000"]
    assert warn_names(0x01) == ["low-voltage supply"]
