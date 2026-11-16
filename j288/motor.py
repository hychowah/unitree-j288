"""One servo on a shared SerialBus.

Each motor keeps its own command and feedback. poll() sends when this
motor's period is due and blocks until that exchange finishes. A motor
that stops answering is polled at 5 Hz so it does not hold the wire.
"""

import time

from j288.bus import REPLY_WAIT
from j288.protocol import build_packet, parse_feedback

RESET_QUIET = 0.4
MISS_PERIOD = 0.2


class Motor:
    def __init__(self, motor_id, bus, rate_hz=40.0):
        self.motor_id = motor_id
        self.bus = bus
        self.timeout_flag = 1
        self.armed = False
        self.cmd_tor = 0.0
        self.cmd_spd = 0.0
        self.cmd_pos = 0.0
        self.kp = 0.0
        self.kd = 0.0
        self.rate_hz = rate_hz
        self.quiet_until = 0.0
        self.next_poll = 0.0
        self.meas_tor = 0.0
        self.meas_spd = 0.0
        self.meas_pos = 0.0
        self.ex_pos = 0.0
        self.temp = None
        self.sensor = None
        self.vol = None
        self.merror = 0
        self.mwarn = 0
        self.fb_mode = 0
        self.fb_timeout = 0
        self.fb_id = 0
        self.correct = 0
        self.miss = 0
        self.rx_ok = 0
        self.tx_count = 0
        self.raw_tx = b""
        self.raw_rx = b""
        self.rxlen = 0
        self.rtt_ms = 0.0
        self._once = ""

    def command_mode(self):
        if self._once == "clear":
            return 6
        if self._once == "reset":
            return 7
        return 1 if self.armed else 0

    def clear(self):
        """Next send() is the clear-error frame. The shaft stays locked."""
        self.armed = False
        self._once = "clear"

    def reset(self):
        """Next send() reboots the drive. This motor then stays quiet for 0.4 s."""
        self.armed = False
        self.cmd_spd = 0.0
        self.cmd_tor = 0.0
        self._once = "reset"

    def wire_torque(self):
        """Torque word on a normal command. Clear and reset do not use it."""
        return self.cmd_tor

    def send(self, wait_s=REPLY_WAIT, reply=None):
        """Write this motor's command and wait for its reply.

        Returns False when the motor is quiet, the id is broadcast, or the
        reply is missing. A reply for a different id is a miss. reply=False
        writes the frame and does not read.
        """
        now = time.monotonic()
        if now < self.quiet_until:
            return False
        once = self._once
        wire_id = max(0, min(15, int(self.motor_id)))
        packet = self._packet(wire_id, once)
        if reply is None:
            reply = wire_id != 15
        result = self.bus.exchange(packet, wait_s=0.0 if not reply else wait_s, reply=reply)
        self.tx_count += 1
        self.raw_tx = packet
        if once == "reset":
            self._once = ""
            self.quiet_until = time.monotonic() + RESET_QUIET
        elif once == "clear":
            self._once = ""
        if not reply:
            self.raw_rx = b""
            self.rxlen = 0
            return False
        self.raw_rx = result.frame
        self.rxlen = result.rxlen
        self.rtt_ms = result.rtt_ms
        parsed = parse_feedback(result.frame)
        if parsed is None or parsed["motor_id"] != wire_id:
            self.correct = 0
            self.miss = min(65535, self.miss + 1)
            return False
        self._apply(parsed)
        return True

    def poll(self, wait_s=REPLY_WAIT):
        """send() when this motor's period has elapsed."""
        now = time.monotonic()
        if now < self.quiet_until or now < self.next_poll:
            return False
        period = 1.0 / self.rate_hz if self.rate_hz else 0.025
        if self.miss >= 5:
            period = max(period, MISS_PERIOD)
        self.next_poll = now + period
        return self.send(wait_s=wait_s)

    def _packet(self, wire_id, once):
        raw = once in ("clear", "reset")
        if once == "clear":
            # Status 6, tor_des -256, every other count 0.
            tor, spd, pos, kp, kd = -256, 0, 0, 0, 0
        elif once == "reset":
            tor, spd, pos, kp, kd = 0, 0, 0, 0, 0
        else:
            tor, spd, pos = self.wire_torque(), self.cmd_spd, self.cmd_pos
            kp, kd = self.kp, self.kd
        return build_packet(
            wire_id, self.command_mode(), self.timeout_flag, tor, spd, pos, kp, kd, raw=raw
        )

    def _apply(self, parsed):
        self.correct = 1
        self.miss = 0
        self.rx_ok += 1
        self.fb_id = parsed["motor_id"]
        self.fb_mode = parsed["mode"]
        self.fb_timeout = parsed["timeout"]
        self.temp = parsed["temp"]
        self.sensor = parsed["sensor"]
        self.vol = parsed["vol"]
        self.merror = parsed["merror"]
        self.mwarn = parsed["mwarn"]
        self.ex_pos = parsed["ex_pos"]
        self.meas_tor = parsed["output_tor"]
        self.meas_spd = parsed["output_spd"]
        self.meas_pos = parsed["output_pos"]
