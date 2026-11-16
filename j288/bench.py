"""Dashboard behavior for one servo. The wire lives on Motor. No widgets."""

import json
import math
import threading
import time
from pathlib import Path

from j288.bus import PROBE_WAIT, SerialBus, list_serial_ports
from j288.log import note
from j288.motor import RESET_QUIET, Motor
from j288.protocol import (
    CLEARABLE_BITS,
    ERROR_BITS,
    KD_LIM,
    KP_LIM,
    POS_LIM,
    SPD_LIM,
    TOR_LIM,
    TOR_MIN,
    clamp,
    clamp_pos,
    error_names,
    warn_names,
)

# Starting sine ends. The Lo and Hi buttons change these, and they are saved.
WAVE_MIN = 4.2
WAVE_MAX = 5.3
WAVE_GAP = 0.1
WAVE_KP = 2.0
WAVE_KD = 0.2
# At 0.03 Hz the peak speed of the default sweep is about 0.10 rad/s.
WAVE_HZ = 0.03
WAVE_HZ_MIN = 0.01
WAVE_HZ_MAX = 100.0
STEP_DT = 0.50
STEP_N = 10
STEP_SPD = 0.60
STEP_SPD_MAX = 0.60
KI_LIM = 100.0
# Arrow slew stays inside this speed until `: spd` raises it.
ARROW_SPD = 5.0
TRACK_EPS = 0.05


def default_settings_path():
    return Path(__file__).resolve().parent.parent / "motor_settings.json"


def short_port(port, limit=22):
    name = str(port).replace("\\", "/").rsplit("/", 1)[-1]
    if name.startswith("cu."):
        name = name[3:]
    limit = max(8, int(limit))
    if len(name) > limit:
        name = name[:8] + "…" + name[-8:]
    return name or "—"


class MotorBench(Motor):
    """One servo plus the screen: slew, wave, step, Ki, settings, and the port."""

    def __init__(self, port="", sim=False, settings_path=None):
        super().__init__(0, SerialBus(), rate_hz=40.0)
        self.port_pref = ""
        self.port = ""
        self.cmd_pos = (WAVE_MIN + WAVE_MAX) / 2.0
        self.kp = WAVE_KP
        self.kd = WAVE_KD
        self.wave_on = False
        self.wave_lo = WAVE_MIN
        self.wave_hi = WAVE_MAX
        self.wave_hz = WAVE_HZ
        self.step_on = False
        self.step_dt = STEP_DT
        self.step_n = STEP_N
        self.step_spd = STEP_SPD
        self.ki = 0.0
        self.tor_i = 0.0
        self.tor_out = 0.0
        self._i_state = 0.0
        self._i_at = 0.0
        self._step_i = 0
        self._step_dir = 1
        self._step_at = 0.0
        self.trace_src = "tor"
        self._wave_phase = 0.0
        self._wave_at = 0.0
        self.linked = False
        self.sim = bool(sim)
        self.notice = ""
        self.log_lines = []
        self.epoch = 0
        self.slew_rate = 0.0
        self._spd_cap = ARROW_SPD
        self._pos_touched = False
        self._slew_ch = ""
        self._slew_dir = 0
        self._slew_at = 0.0
        self._slew_key_at = 0.0
        self._slew_max = 0.0
        self._settings_path = None
        if not sim:
            self._settings_path = (
                Path(settings_path)
                if settings_path
                else default_settings_path()
            )
            self._load_settings()
        if port:
            self.port_pref = str(port)
            self.port = self.port_pref
        self._tried = bool(sim)
        self._port_only = False
        self._closed = False
        self._fault_key = None
        self._said_bcast = False
        self._busy = threading.Lock()
        self._shut = False
        self._job_lock = threading.Lock()
        self._jobs = []
        self.append("locked. m enables FOC. 0 stops. arrows slew speed and position")

    def post(self, fn):
        """Run fn on the next tick(). The screen uses this for port work."""
        with self._job_lock:
            self._jobs.append(fn)

    def _take_jobs(self):
        with self._job_lock:
            jobs = self._jobs
            self._jobs = []
        return jobs

    def wire_torque(self):
        """Torque word on the bus. The Ki integral in torque_command is not used."""
        return self.cmd_tor

    def append(self, text):
        self.log_lines = (self.log_lines + [str(text)])[-40:]

    @property
    def position_loop(self):
        return self.kp > 0.0

    def tracked_now(self):
        return self.meas_pos if self.position_loop else self.meas_spd

    def tracked_goal(self):
        return self.cmd_pos if self.position_loop else self.cmd_spd

    def fresh(self):
        return self.correct == 1 and (self.linked or self.sim)

    def matched(self):
        if not self.fresh():
            return False
        return abs(self.tracked_now() - self.tracked_goal()) <= TRACK_EPS

    def mode_word(self):
        if self._once == "clear":
            return "clear"
        if self._once == "reset":
            return "reset"
        return "FOC" if self.armed else "lock"

    def slewing(self, now=None):
        if self._slew_dir == 0:
            return False
        now = time.monotonic() if now is None else float(now)
        return now - self._slew_key_at <= 0.16

    def arm_slew(self, channel, direction, now=None):
        now = time.monotonic() if now is None else float(now)
        direction = 1 if int(direction) > 0 else -1
        channel = "pos" if channel == "pos" else "spd"
        if self._slew_ch != channel or self._slew_dir != direction or now - self._slew_key_at > 0.16:
            self._slew_ch = channel
            self._slew_dir = direction
            self._slew_max = 0.8 if channel == "pos" else 2.0
            self.slew_rate = 0.15 if channel == "pos" else 0.8
            self._slew_at = now
        self._slew_key_at = now
        if channel == "pos":
            self._pos_touched = True

    def slew_tick(self, now=None):
        now = time.monotonic() if now is None else float(now)
        if self.wave_on:
            self._slew_dir = 0
            self.advance_wave(now)
            return
        if self.step_on:
            self._slew_dir = 0
            self.advance_step(now)
            return
        if self._slew_dir == 0:
            return
        if now - self._slew_key_at > 0.16:
            self._slew_dir = 0
            self._slew_ch = ""
            self.slew_rate = 0.0
            self._slew_max = 0.0
            return
        dt = min(0.25, max(0.0, now - self._slew_at))
        self._slew_at = now
        accel = 0.8 if self._slew_ch == "pos" else 2.0
        self.slew_rate = min(self._slew_max, self.slew_rate + accel * dt)
        step = self._slew_dir * self.slew_rate * dt
        if self._slew_ch == "pos":
            self.cmd_pos = clamp_pos(self.cmd_pos + step)
        else:
            cap = self._spd_cap
            self.cmd_spd = clamp(self.cmd_spd + step, -cap, cap)
        self.notice = ""

    def toggle_armed(self):
        self.armed = not self.armed
        self._once = ""
        if self.armed:
            if self.fresh():
                self.cmd_pos = clamp_pos(self.meas_pos)
                self._pos_touched = True
            if self.position_loop and abs(self.cmd_pos - self.meas_pos) > 0.5:
                self.notice = "FOC %.2f rad from the shaft" % (self.cmd_pos - self.meas_pos)
            elif not self.position_loop and abs(self.cmd_spd) > 0.05:
                self.notice = "FOC at %.2f rad/s" % self.cmd_spd
            elif self.kp == 0.0 and self.kd == 0.0:
                self.notice = "FOC with Kp and Kd at 0"
            else:
                self.notice = ""
            self.append("FOC on")
        else:
            self.wave_on = False
            self.step_on = False
            self.cmd_spd = 0.0
            self._reset_integrator()
            self.notice = ""
            self.append("locked")
        self.next_poll = 0.0

    def stop(self):
        self.wave_on = False
        self.step_on = False
        self.armed = False
        self._once = ""
        self.cmd_spd = 0.0
        self.cmd_tor = 0.0
        self._reset_integrator()
        self._pos_touched = False
        if self.fresh():
            self.cmd_pos = clamp_pos(self.meas_pos)
        self._slew_dir = 0
        self._slew_ch = ""
        self.slew_rate = 0.0
        self.notice = ""
        self.append("stop")
        self.next_poll = 0.0

    def apply_command(self, text):
        parts = str(text).strip().split()
        if not parts:
            self.notice = "spd pos tor kp kd id timeout port connect clear reset"
            self.append(self.notice)
            return
        cmd = parts[0].lower()
        alias = {
            "speed": "spd",
            "position": "pos",
            "torque": "tor",
            "tmo": "timeout",
            "pi": "ki",
        }
        cmd = alias.get(cmd, cmd)
        if cmd == "connect":
            self._tried = False
            self.connect()
            return
        if cmd == "disconnect":
            self._close_ser()
            self.linked = False
            self.correct = 0
            self.append("closed")
            self.notice = ""
            return
        if cmd == "clear":
            self.clear()
            self.next_poll = 0.0
            self.notice = "clear sent, motor stays locked"
            self.append(self.notice)
            return
        if cmd == "reset":
            self.reset()
            self.next_poll = 0.0
            self.notice = "reset sent, ignoring the bus for %.1f s" % RESET_QUIET
            self.append(self.notice)
            return
        if cmd == "port":
            if len(parts) != 2:
                self.notice = "usage: port <device>"
                self.append(self.notice)
                return
            self.port_pref = parts[1]
            self._port_only = True
            self._tried = False
            self.connect()
            return
        if len(parts) != 2:
            self.notice = "usage: %s <value>" % cmd
            self.append(self.notice)
            return
        if cmd == "id":
            self._set_id(parts[1])
            return
        if cmd == "timeout":
            self._set_timeout(parts[1])
            return
        if cmd == "rate":
            self._set_rate(parts[1])
            return
        self._set_float(cmd, parts[1])

    def _set_id(self, text):
        try:
            value = int(text)
        except ValueError:
            self.notice = "id is 0-15"
            self.append(self.notice)
            return
        if value < 0 or value > 15:
            self.notice = "id is 0-15"
            self.append(self.notice)
            return
        self.motor_id = value
        self._said_bcast = False
        self.notice = "id 15 broadcasts and does not reply" if value == 15 else ""
        self.append("id %d" % value)

    def _set_timeout(self, text):
        try:
            value = int(text)
        except ValueError:
            self.notice = "timeout is 0 or 1"
            self.append(self.notice)
            return
        if value not in (0, 1):
            self.notice = "timeout is 0 or 1"
            self.append(self.notice)
            return
        self.timeout_flag = value
        if value == 0:
            self.notice = "timeout protection off — last FOC command can stay"
        else:
            self.notice = ""
        self.append("timeout %d" % value)

    def _set_rate(self, text):
        try:
            value = float(text)
        except ValueError:
            self.notice = "rate is 5-200 Hz"
            self.append(self.notice)
            return
        self.rate_hz = clamp(value, 5.0, 200.0)
        self.notice = ""
        self.append("rate %.0f Hz" % self.rate_hz)

    def _set_float(self, cmd, text):
        limits = {
            "spd": ("speed", -SPD_LIM, SPD_LIM),
            "pos": ("position", -POS_LIM, POS_LIM),
            "tor": ("torque", TOR_MIN, TOR_LIM),
            "kp": ("Kp", 0.0, KP_LIM),
            "kd": ("Kd", 0.0, KD_LIM),
            "ki": ("PI", 0.0, KI_LIM),
        }
        if cmd not in limits:
            self.notice = "spd pos tor kp kd id timeout port connect clear reset"
            self.append(self.notice)
            return
        label, lo, hi = limits[cmd]
        try:
            value = float(text)
        except ValueError:
            self.notice = "%s is not a number" % label
            self.append(self.notice)
            return
        value = clamp(value, lo, hi)
        if cmd == "spd":
            self.cmd_spd = value
            self._spd_cap = max(ARROW_SPD, abs(value))
        elif cmd == "pos":
            self.cmd_pos = clamp_pos(value)
            self._pos_touched = True
        elif cmd == "tor":
            self.cmd_tor = value
        elif cmd == "kp":
            self.kp = value
        elif cmd == "ki":
            self.ki = value
            if value == 0.0:
                self._reset_integrator()
        else:
            self.kd = value
        self.notice = ""
        self.append("%s %.3f" % (cmd, value))
        if cmd in ("kp", "kd", "ki"):
            self._save_settings()

    def nudge(self, field, delta):
        """One button step. Ranges are the output-end limits from the parameter table."""
        if field == "id":
            self.motor_id = max(0, min(15, self.motor_id + int(delta)))
            self._said_bcast = False
            self.notice = "id 15 broadcasts and does not reply" if self.motor_id == 15 else ""
            self.append("id %d" % self.motor_id)
            return
        if field == "timeout":
            self._set_timeout("0" if self.timeout_flag else "1")
            return
        if field == "mode":
            want = bool(delta)
            if want != self.armed:
                self.toggle_armed()
            return
        if field == "wave":
            self.toggle_wave()
            return
        if field == "step":
            self.toggle_step()
            return
        if field == "step_dt":
            self.step_dt = clamp(round(self.step_dt + float(delta), 2), 0.05, 60.0)
            self.append("step %.2fs" % self.step_dt)
            self._save_settings()
            return
        if field == "step_n":
            self.step_n = int(clamp(int(self.step_n) + int(delta), 1, 200))
            self.append("steps %d" % self.step_n)
            self._save_settings()
            return
        if field == "step_spd":
            self.step_spd = clamp(round(self.step_spd + float(delta), 3), 0.0, STEP_SPD_MAX)
            self.append("step speed %.3f" % self.step_spd)
            self._save_settings()
            return
        if field == "ki":
            self.ki = clamp(round(self.ki + float(delta), 3), 0.0, KI_LIM)
            if self.ki == 0.0:
                self._reset_integrator()
            self.append("PI %.3f" % self.ki)
            self._save_settings()
            return
        if field == "hz":
            self.wave_hz = clamp(round(self.wave_hz + float(delta), 2), WAVE_HZ_MIN, WAVE_HZ_MAX)
            self.append("wave %.2f Hz" % self.wave_hz)
            self._save_settings()
            return
        if field in ("wave_lo", "wave_hi"):
            self._nudge_wave_end(field, delta)
            return
        if field == "trace":
            self.trace_src = "pos" if self.trace_src == "tor" else "tor"
            self.append("trace %s" % self.trace_src)
            return
        if field == "port":
            self.choose_port(delta)
            return
        if (self.wave_on or self.step_on) and field in ("pos", "spd"):
            self.notice = "loop is running"
            return
        limits = {
            "spd": (self.cmd_spd, -SPD_LIM, SPD_LIM, 3),
            "pos": (self.cmd_pos, -POS_LIM, POS_LIM, 3),
            "tor": (self.cmd_tor, TOR_MIN, TOR_LIM, 3),
            "kp": (self.kp, 0.0, KP_LIM, 3),
            "kd": (self.kd, 0.0, KD_LIM, 3),
            "ki": (self.ki, 0.0, KI_LIM, 3),
        }
        current, lo, hi, places = limits[field]
        value = clamp(round(float(current) + float(delta), places), lo, hi)
        if field == "spd":
            self.cmd_spd = value
            self._spd_cap = max(ARROW_SPD, abs(value))
        elif field == "pos":
            self.cmd_pos = clamp_pos(value)
            self._pos_touched = True
        elif field == "tor":
            self.cmd_tor = value
        elif field == "kp":
            self.kp = value
        elif field == "ki":
            self.ki = value
            if value == 0.0:
                self._reset_integrator()
        else:
            self.kd = value
        if not self.merror and not self.fb_timeout:
            self.notice = ""
        self.append("%s %.3f" % (field, value))
        if field in ("kp", "kd", "ki"):
            self._save_settings()

    def toggle_wave(self):
        if self.wave_on:
            self._stop_loop("wave")
            return
        if not self.fresh():
            self.notice = "no feedback yet"
            self.append(self.notice)
            return
        mid = (self.wave_lo + self.wave_hi) / 2.0
        amp = (self.wave_hi - self.wave_lo) / 2.0
        here = clamp(self.meas_pos, self.wave_lo, self.wave_hi)
        self._wave_phase = math.asin(clamp((here - mid) / amp, -1.0, 1.0))
        self._wave_at = time.monotonic()
        self._pos_touched = True
        self.step_on = False
        self.wave_on = True
        if not self.armed:
            self.toggle_armed()
        self.advance_wave(self._wave_at)
        self.notice = "wave %.2f Hz   %.2f–%.2f rad" % (self.wave_hz, self.wave_lo, self.wave_hi)
        self.append(
            "wave on  Kp %.1f  Kd %.2f  %.2f Hz  %.2f–%.2f"
            % (self.kp, self.kd, self.wave_hz, self.wave_lo, self.wave_hi)
        )

    def _nudge_wave_end(self, field, delta):
        moved = round(float(delta), 3)
        if field == "wave_lo":
            self.wave_lo = clamp(round(self.wave_lo + moved, 3), -POS_LIM, POS_LIM - WAVE_GAP)
            if self.wave_hi < self.wave_lo + WAVE_GAP:
                self.wave_hi = min(POS_LIM, round(self.wave_lo + WAVE_GAP, 3))
        else:
            self.wave_hi = clamp(round(self.wave_hi + moved, 3), -POS_LIM + WAVE_GAP, POS_LIM)
            if self.wave_lo > self.wave_hi - WAVE_GAP:
                self.wave_lo = max(-POS_LIM, round(self.wave_hi - WAVE_GAP, 3))
        self.append("wave %.2f–%.2f" % (self.wave_lo, self.wave_hi))

    def _stop_loop(self, name):
        self.wave_on = False
        self.step_on = False
        self.cmd_spd = 0.0
        if self.fresh():
            self.cmd_pos = clamp_pos(self.meas_pos)
        if self.armed:
            self.toggle_armed()
        self.notice = "%s off" % name
        self.append(self.notice)

    def toggle_step(self):
        if self.step_on:
            self._stop_loop("step")
            return
        if not self.fresh():
            self.notice = "no feedback yet"
            self.append(self.notice)
            return
        n = max(1, int(self.step_n))
        span = self.wave_hi - self.wave_lo
        frac = 0.0 if span <= 0 else (self.meas_pos - self.wave_lo) / span
        self._step_i = int(round(clamp(frac, 0.0, 1.0) * n))
        self._step_dir = -1 if self._step_i >= n else 1
        self._step_at = time.monotonic()
        self._pos_touched = True
        self.wave_on = False
        self.step_on = True
        if not self.armed:
            self.toggle_armed()
        self._apply_step()
        self.notice = "step %d × %.2fs   %.3f rad/s" % (n, self.step_dt, self.step_spd)
        self.append(self.notice)

    def advance_step(self, now):
        n = max(1, int(self.step_n))
        self._step_i = max(0, min(n, int(self._step_i)))
        if now - self._step_at >= self.step_dt:
            self._step_at = now
            nxt = self._step_i + self._step_dir
            if nxt > n or nxt < 0:
                self._step_dir *= -1
                nxt = self._step_i + self._step_dir
            self._step_i = max(0, min(n, nxt))
        self._apply_step()

    def _apply_step(self):
        n = max(1, int(self.step_n))
        i = max(0, min(n, int(self._step_i)))
        span = self.wave_hi - self.wave_lo
        pos = self.wave_lo + span * (i / float(n))
        speed = self._step_dir * self.step_spd
        self.cmd_pos = clamp_pos(pos)
        self.cmd_spd = clamp(speed, -SPD_LIM, SPD_LIM)
        self._spd_cap = max(ARROW_SPD, abs(self.cmd_spd))
        self._pos_touched = True

    def _reset_integrator(self):
        self._i_state = 0.0
        self.tor_i = 0.0
        self._i_at = 0.0

    def torque_command(self, now):
        """Torque field sent to the servo.

        The drive itself applies Kp to position error and Kd to velocity
        error. The torque word is only the extra term:

            Ki * integral(position goal - measured position) + extra torque
        """
        lo, hi = TOR_MIN, TOR_LIM
        if not self.armed or self.ki <= 0.0 or not self.fresh():
            if not self.armed or self.ki <= 0.0:
                self._reset_integrator()
            self.tor_out = clamp(self.cmd_tor + self.tor_i, lo, hi)
            return self.tor_out
        if self._i_at <= 0.0:
            self._i_at = now
        dt = min(0.1, max(0.0, now - self._i_at))
        self._i_at = now
        err = self.cmd_pos - self.meas_pos
        proposed = self._i_state + err * dt
        tor_i = self.ki * proposed
        total = self.cmd_tor + tor_i
        if lo <= total <= hi:
            self._i_state = proposed
            self.tor_i = tor_i
        self.tor_out = clamp(self.cmd_tor + self.tor_i, lo, hi)
        return self.tor_out

    def advance_wave(self, now):
        dt = min(0.25, max(0.0, now - self._wave_at))
        self._wave_at = now
        self._wave_phase += 2.0 * math.pi * self.wave_hz * dt
        mid = (self.wave_lo + self.wave_hi) / 2.0
        amp = (self.wave_hi - self.wave_lo) / 2.0
        self.cmd_pos = clamp_pos(mid + amp * math.sin(self._wave_phase))
        self.cmd_spd = amp * 2.0 * math.pi * self.wave_hz * math.cos(self._wave_phase)
        self._pos_touched = True

    def handle_key(self, key):
        if key == "left":
            self.arm_slew("spd", -1)
        elif key == "right":
            self.arm_slew("spd", 1)
        elif key == "up":
            self.arm_slew("pos", 1)
        elif key == "down":
            self.arm_slew("pos", -1)
        elif key in ("m", "M"):
            self.toggle_armed()
        elif key == "0":
            self.stop()

    def choose_port(self, step):
        """Connect to the next or previous adapter. Only that device is tried."""
        shown = self.port_pref or self.port
        ports = list_serial_ports(shown)
        if not ports:
            self.notice = "no serial port"
            self.append(self.notice)
            return
        current = shown
        idx = ports.index(current) if current in ports else -1
        if step:
            idx = (idx + int(step)) % len(ports)
        elif idx < 0:
            idx = 0
        self.port_pref = ports[idx]
        self._port_only = True
        self._tried = False
        self.connect()

    def connect(self):
        only = self._port_only
        self._port_only = False
        if self._closed:
            return False
        self._close_ser()
        self.linked = False
        self.correct = 0
        self.quiet_until = 0.0
        if only and self.port_pref:
            ports = [self.port_pref]
        else:
            ports = list_serial_ports(self.port_pref)
        if not ports:
            self.notice = "no serial port"
            self.append(self.notice)
            self._tried = True
            note("connect no port")
            return False
        self.notice = "searching"
        note("connect %s" % ", ".join(ports))
        errors = []
        for port in ports:
            if self._closed:
                return False
            note("open %s" % port)
            try:
                self._open(port)
            except Exception as exc:
                note("open %s failed %s" % (port, exc))
                errors.append("%s: %s" % (short_port(port), exc))
                self._close_ser()
                continue
            try:
                found = self._find_motor()
            except Exception as exc:
                note("open %s failed %s" % (port, exc))
                errors.append("%s: %s" % (short_port(port), exc))
                self._close_ser()
                continue
            if found:
                self.port = port
                self.linked = True
                self.sim = False
                self.epoch += 1
                self._tried = True
                self.notice = ""
                self.append("link %s" % port)
                note("link %s id %d" % (port, self.motor_id))
                self._announce_feedback()
                self._clear_tripped_timeout()
                return True
            errors.append("%s: no frame" % short_port(port))
            self._close_ser()
        self._tried = True
        self.notice = "no motor reply"
        self.append(self.notice + " — " + "; ".join(errors))
        note("no reply %s" % "; ".join(errors))
        return False

    def tick(self):
        """One poll. Posted work runs first, on this same call."""
        with self._busy:
            if self._closed:
                return
            for fn in self._take_jobs():
                try:
                    fn()
                except Exception as exc:
                    note("job %s" % exc)
                    self.notice = str(exc)
                    self.append(self.notice)
            if self._closed:
                return
            self._tick_body()

    def _tick_body(self):
        if self._closed:
            return
        now = time.monotonic()
        if self.sim and not self.linked:
            self._tick_sim(now)
            return
        if not self._tried:
            self._tried = True
            try:
                self.connect()
            except Exception as exc:
                note("connect %s" % exc)
                self._close_ser()
                self.linked = False
                self.notice = "connect failed: %s" % exc
                self.append(self.notice)
            return
        if not self.bus.is_open or not self.linked:
            return
        was = self.miss
        sent = self.tx_count
        try:
            ok = self.poll()
        except Exception as exc:
            note("poll %s" % exc)
            self._close_ser()
            self.linked = False
            self.correct = 0
            self.notice = "serial closed: %s" % exc
            self.append(self.notice)
            return
        if self.tx_count != sent:
            self._after_send(was, ok)

    def _tick_sim(self, now):
        if now < self.next_poll:
            return
        dt = 0.025 if self.next_poll == 0.0 else min(0.1, max(0.0, now - self.next_poll + 0.025))
        self.next_poll = now + 0.025
        if self.command_mode() == 6:
            self.merror = 0
            self._once = ""
        elif self.command_mode() == 7:
            self._once = ""
            self.quiet_until = now + RESET_QUIET
        if self.armed and self.position_loop:
            err = self.cmd_pos - self.meas_pos
            spd_err = self.cmd_spd - self.meas_spd
            acc = clamp(self.kp * err + max(self.kd, 0.2) * spd_err + self.cmd_tor, -40.0, 40.0)
            self.meas_spd = clamp(self.meas_spd + acc * dt, -20.0, 20.0)
        elif self.armed:
            kd = self.kd if self.kd > 0.0 else 4.0
            acc = clamp(kd * (self.cmd_spd - self.meas_spd) + self.cmd_tor, -20.0, 20.0)
            self.meas_spd = clamp(self.meas_spd + acc * dt, -20.0, 20.0)
        else:
            self.meas_spd *= max(0.0, 1.0 - 12.0 * dt)
        self.meas_pos += self.meas_spd * dt
        if not self._pos_touched and not self.armed and not self.wave_on:
            self.cmd_pos = clamp_pos(self.meas_pos)
        self.meas_tor = self.cmd_tor
        self.temp = 36
        self.sensor = 34
        self.vol = 24.0
        self.ex_pos = self.meas_pos % (2 * math.pi)
        self.merror = 0
        self.mwarn = 0
        self.fb_mode = 1 if self.armed else 0
        self.fb_timeout = 0
        self.fb_id = self.motor_id
        self.correct = 1
        self.rxlen = 26
        self.miss = 0
        self.rx_ok += 1
        self.rtt_ms = 0.0
        self.linked = False

    def _open(self, port):
        self.bus.open(port)

    def _close_ser(self):
        self.bus.close()

    def _find_motor(self):
        """Keep this id when it answers. Otherwise use the first of 0–14 that does."""
        saved = int(self.motor_id)
        first = saved if 0 <= saved <= 14 else 0
        for motor_id in (first, *[i for i in range(15) if i != first]):
            if self._closed:
                self.motor_id = saved
                return False
            self.motor_id = motor_id
            self.miss = 0
            sent = self.tx_count
            note("probe id %d" % motor_id)
            started = time.monotonic()
            try:
                ok = self.send(wait_s=PROBE_WAIT)
            except Exception as exc:
                note("probe id %d error %s" % (motor_id, exc))
                raise
            note(
                "probe id %d %s %.0fms rx %d"
                % (
                    motor_id,
                    "ok" if ok else "miss",
                    (time.monotonic() - started) * 1000.0,
                    self.rxlen,
                )
            )
            if self.tx_count != sent and ok and self._after_send(0, True):
                if motor_id != saved:
                    self.append("id %d" % motor_id)
                return True
        self.motor_id = saved
        return False

    def _after_send(self, was_miss, ok):
        """Screen reaction to a send() that actually wrote a frame."""
        wire_id = max(0, min(15, int(self.motor_id)))
        if wire_id == 15:
            if not self._said_bcast:
                self._said_bcast = True
                self.notice = "id 15 broadcasts and does not reply"
                self.append(self.notice)
            return False
        if not ok:
            if self.miss == 1 or self.miss % 20 == 0:
                self.append("no frame (%d bytes, miss %d)" % (self.rxlen, self.miss))
            return False
        if not self._pos_touched and not self.armed and not self.wave_on:
            self.cmd_pos = clamp_pos(self.meas_pos)
        self._note_faults()
        if was_miss >= 5:
            self.append("feedback restored")
        return True

    def _clear_tripped_timeout(self):
        """A locked frame with the timeout bit at 0 clears a tripped flag.

        The reply to that frame can still show the old bit. One more frame
        shows the cleared state. Protection is turned back on afterward.
        """
        if not self.fb_timeout or self.armed or not self.timeout_flag:
            return
        saved = self.timeout_flag
        self.timeout_flag = 0
        try:
            for _ in range(3):
                # The reply still shows the old bit until the next frame.
                time.sleep(0.03)
                was = self.miss
                sent = self.tx_count
                ok = self.send()
                if self.tx_count != sent:
                    self._after_send(was, ok)
                if self.fresh() and not self.fb_timeout:
                    break
        finally:
            self.timeout_flag = saved
        if self.fb_timeout:
            self.notice = "timeout tripped — : timeout 0 clears the bit"
        else:
            if str(self.notice).startswith("timeout tripped"):
                self.notice = ""
            self.append("cleared timeout bit")

    def _announce_feedback(self):
        if not self.fresh():
            return
        self.append(
            "id %d  pos %.3f rad  spd %.3f  hsg %s°C  wnd %s°C  %.1f V"
            % (
                self.fb_id,
                self.meas_pos,
                self.meas_spd,
                "—" if self.temp is None else "%d" % self.temp,
                "—" if self.sensor is None else "%d" % self.sensor,
                self.vol or 0.0,
            )
        )

    def _note_faults(self):
        names = error_names(self.merror)
        warns = warn_names(self.mwarn)
        key = (self.merror, self.mwarn, self.fb_timeout)
        if key == self._fault_key:
            return
        self._fault_key = key
        if names:
            stuck = [
                ERROR_BITS[bit]
                for bit in sorted(ERROR_BITS)
                if self.merror & (1 << bit) and bit not in CLEARABLE_BITS
            ]
            text = "error " + ", ".join(names)
            if stuck:
                text += " — reset or power cycle"
            else:
                text += " — : clear"
            self.notice = text
            self.append(text)
        elif self.fb_timeout:
            self.notice = "timeout tripped — : timeout 0 clears the bit"
            self.append(self.notice)
        elif warns:
            self.notice = "warning " + ", ".join(warns)
            self.append(self.notice)
        elif str(self.notice).startswith(("error ", "warning ", "timeout tripped")):
            self.notice = ""

    def _load_settings(self):
        path = self._settings_path
        if path is None or not path.is_file():
            return
        try:
            data = json.loads(path.read_text())
        except (OSError, ValueError):
            return
        if not isinstance(data, dict):
            return
        try:
            self.motor_id = max(0, min(15, int(data.get("id", self.motor_id))))
            timeout = data.get("timeout", self.timeout_flag)
            self.timeout_flag = 1 if timeout else 0
            self.cmd_tor = clamp(float(data.get("tor", self.cmd_tor)), TOR_MIN, TOR_LIM)
            self.cmd_spd = clamp(float(data.get("spd", self.cmd_spd)), -SPD_LIM, SPD_LIM)
            self._spd_cap = max(ARROW_SPD, abs(self.cmd_spd))
            if "pos" in data:
                self.cmd_pos = clamp_pos(float(data["pos"]))
                self._pos_touched = True
            self.kp = clamp(float(data.get("kp", self.kp)), 0.0, KP_LIM)
            self.kd = clamp(float(data.get("kd", self.kd)), 0.0, KD_LIM)
            self.wave_hz = clamp(
                float(data.get("wave_hz", self.wave_hz)), WAVE_HZ_MIN, WAVE_HZ_MAX
            )
            if "wave_lo" in data:
                self.wave_lo = clamp_pos(float(data["wave_lo"]))
            if "wave_hi" in data:
                self.wave_hi = clamp_pos(float(data["wave_hi"]))
            if self.wave_hi < self.wave_lo + WAVE_GAP:
                self.wave_hi = min(POS_LIM, self.wave_lo + WAVE_GAP)
            self.rate_hz = clamp(float(data.get("rate_hz", self.rate_hz)), 5.0, 200.0)
            if data.get("trace") == "pos":
                self.trace_src = "pos"
            elif data.get("trace") in ("tor", "cur"):
                self.trace_src = "tor"
            if "step_dt" in data:
                self.step_dt = clamp(float(data["step_dt"]), 0.05, 60.0)
            if "step_n" in data:
                self.step_n = int(clamp(int(data["step_n"]), 1, 200))
            if "step_spd" in data:
                self.step_spd = clamp(float(data["step_spd"]), 0.0, STEP_SPD_MAX)
            if "ki" in data:
                self.ki = clamp(float(data["ki"]), 0.0, KI_LIM)
        except (TypeError, ValueError):
            return
        saved_port = str(data.get("port") or "")
        if saved_port:
            self.port_pref = saved_port
            self.port = saved_port
        self.append("settings loaded")

    def _save_settings(self):
        path = self._settings_path
        if path is None:
            return
        data = {
            "id": self.motor_id,
            "timeout": self.timeout_flag,
            "tor": self.cmd_tor,
            "spd": self.cmd_spd,
            "pos": self.cmd_pos,
            "kp": self.kp,
            "kd": self.kd,
            "wave_hz": self.wave_hz,
            "wave_lo": self.wave_lo,
            "wave_hi": self.wave_hi,
            "rate_hz": self.rate_hz,
            "trace": self.trace_src,
            "step_dt": self.step_dt,
            "step_n": self.step_n,
            "step_spd": self.step_spd,
            "ki": self.ki,
            "port": self.port or self.port_pref,
        }
        try:
            path.parent.mkdir(parents=True, exist_ok=True)
            tmp = path.with_suffix(".json.tmp")
            tmp.write_text(json.dumps(data, indent=2) + "\n")
            tmp.replace(path)
        except OSError:
            pass

    def shutdown(self):
        """Stop the port. Returns even when a probe is still inside the driver."""
        self._closed = True
        self._save_settings()
        if not self._busy.acquire(timeout=0.2):
            note("shutdown")
            return
        try:
            if self._shut:
                return
            self._shut = True
            self.armed = False
            self._once = ""
            self.cmd_spd = 0.0
            self.cmd_tor = 0.0
            self.kp = 0.0
            self.kd = 0.0
            self.timeout_flag = 1
            self.quiet_until = 0.0
            self._reset_integrator()
            if self.bus.is_open:
                try:
                    self.send(reply=False)
                except Exception:
                    pass
            self._close_ser()
            self.linked = False
            note("shutdown")
        finally:
            self._busy.release()
