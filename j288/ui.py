"""Textual screen for the bench. The motor state lives in j288.bench."""

import time

from rich.console import Group
from rich.text import Text
from textual.app import App
from textual.containers import Horizontal, Vertical
from textual.css.query import NoMatches
from textual.renderables.digits import Digits
from textual.widget import Widget
from textual.widgets import Button, Input, RichLog, Static

from j288.bench import (
    STEP_DT,
    STEP_N,
    STEP_SPD,
    WAVE_HZ,
    WAVE_MAX,
    WAVE_MIN,
    short_port,
)
from j288.protocol import error_names, warn_names

_EMPTY = "#2a3542"
_CYAN = "#6fd3ee"
_GREEN = "#46c98a"
_AMBER = "#e2b15a"
_RED = "#ef5b54"
_MUTED = "#8ea0b0"
_TEXT = "#e7eef4"
_EIGHTHS = " ▏▎▍▌▋▊▉"
_FRAME_S = 0.01


def _clamp01(value):
    return max(0.0, min(1.0, value))


def _heat(fraction):
    fraction = _clamp01(fraction)
    if fraction < 0.55:
        return _GREEN
    if fraction < 0.85:
        return _AMBER
    return _RED


def _temp_color(value):
    if value is None:
        return _MUTED
    if value < 50:
        return _GREEN
    if value < 70:
        return _AMBER
    return _RED


def _meter(fraction, width, color):
    width = max(1, int(width))
    fraction = _clamp01(fraction)
    filled = fraction * width
    whole = int(filled)
    part = filled - whole
    text = Text()
    for i in range(width):
        if i < whole:
            text.append("█", style=color)
        elif i == whole and part > 0.04:
            text.append(_EIGHTHS[min(7, max(1, int(part * 8)))], style=color)
        else:
            text.append("░", style=_EMPTY)
    return text


def _heat_meter(fraction, width):
    width = max(1, int(width))
    fraction = _clamp01(fraction)
    filled = fraction * width
    whole = int(filled)
    part = filled - whole
    text = Text()
    last = float(max(1, width - 1))
    for i in range(width):
        color = _heat(i / last)
        if i < whole:
            text.append("█", style=color)
        elif i == whole and part > 0.04:
            text.append(_EIGHTHS[min(7, max(1, int(part * 8)))], style=color)
        else:
            text.append("░", style=_EMPTY)
    return text


def _columns(values, width):
    values = [float(value) for value in values]
    if not values:
        return []
    if len(values) == 1:
        return values * width
    if len(values) <= width:
        last = len(values) - 1
        return [values[int(round(i * last / float(width - 1)))] for i in range(width)]
    out = []
    for i in range(width):
        start = int(i * len(values) / float(width))
        end = max(start + 1, int((i + 1) * len(values) / float(width)))
        out.append(values[end - 1])
    return out


def _chart(values, width, height):
    width = max(4, int(width))
    height = max(1, int(height))
    if not values:
        lines = []
        for i in range(height):
            if i == height // 2:
                lines.append(Text("no samples yet", style=_MUTED))
            else:
                lines.append(Text("░" * width, style=_EMPTY))
        return lines
    sample = _columns(list(values)[-240:], width)
    lo = min(sample)
    hi = max(sample)
    if hi - lo < 1e-6:
        mid = (lo + hi) / 2.0
        lo, hi = mid - 0.05, mid + 0.05
    span = hi - lo
    lines = [Text() for _ in range(height)]
    for value in sample:
        fraction = _clamp01((float(value) - lo) / span)
        color = _heat(fraction)
        top = fraction * height
        for i in range(height):
            dist = height - i
            if dist <= top:
                lines[i].append("█", style=color)
            elif dist - 1 < top:
                lines[i].append("▄", style=color)
            else:
                lines[i].append("░", style=_EMPTY)
    return lines


def _kv(label, value, color):
    text = Text()
    text.append((label + "     ")[:6], style=_MUTED)
    text.append(str(value), style="bold " + color)
    return text


def _plain(node):
    if isinstance(node, Text):
        return node.plain
    renderables = getattr(node, "renderables", None)
    if renderables is None:
        return ""
    return "\n".join(_plain(item) for item in renderables)


def _key_bar():
    text = Text()

    def piece(key, rest, newline=False):
        text.append(key, style="bold " + _TEXT)
        text.append(" " + rest + "    ", style=_MUTED)
        if newline:
            text.append("\n")

    piece("← →", "speed")
    piece("↑ ↓", "position")
    piece("m", "FOC")
    piece("0", "stop")
    piece("q", "quit", newline=True)
    piece(":", "exact value")
    return text


def _frac(value, lo, hi):
    span = hi - lo
    if span <= 0:
        return 0.5
    return _clamp01((value - lo) / span)


class _Panel(Widget):
    def plain(self):
        return _plain(self.render())


class SidePanel(_Panel):
    def __init__(self):
        super().__init__(id="motor", classes="panel")
        self.rows = []

    def render(self):
        if not self.rows:
            return Text("")
        return Group(*self.rows)


class LevelPanel(_Panel):
    def __init__(self):
        super().__init__(id="position", classes="panel")
        self.now_text = "—"
        self.goal_text = ""
        self.goal_num = "0"
        self.now_frac = 0.0
        self.goal_frac = 0.0
        self.note = ""
        self.goal_color = _AMBER

    def render(self):
        width = max(4, self.content_size.width)
        lines = [
            _kv("NOW", self.now_text, _CYAN),
            _meter(self.now_frac, width, _CYAN),
            _kv("GOAL", self.goal_text, self.goal_color),
        ]
        if self.content_size.height >= 9:
            lines.append(Digits(self.goal_num, style="bold " + self.goal_color))
        lines.append(_meter(self.goal_frac, width, self.goal_color))
        lines.append(Text(self.note, style=self.goal_color))
        return Group(*lines)


class RatePanel(_Panel):
    def __init__(self):
        super().__init__(id="tune", classes="panel")
        self.word = "LOCK"
        self.word_color = _MUTED
        self.detail = ""
        self.miss_text = "0"
        self.miss_color = _MUTED
        self.hsg_text = "—"
        self.hsg_color = _MUTED
        self.wnd_text = "—"
        self.wnd_color = _MUTED
        self.volt_text = "—"
        self.err_text = "—"
        self.err_color = _MUTED
        self.slew_text = ""
        self.slew_frac = 0.0

    def render(self):
        width = max(4, self.content_size.width)
        return Group(
            Text(self.word, style="bold " + self.word_color),
            Text(self.detail, style=_MUTED),
            _kv("MISS", self.miss_text, self.miss_color),
            _kv("HSG", self.hsg_text, self.hsg_color),
            _kv("WND", self.wnd_text, self.wnd_color),
            _kv("VOLT", self.volt_text, _TEXT),
            _kv("ERR", self.err_text, self.err_color),
            _kv("SPD", self.slew_text, _TEXT),
            _heat_meter(self.slew_frac, width),
        )


class LivePanel(_Panel):
    def __init__(self):
        super().__init__(id="live", classes="panel")
        self.line_a = Text("")
        self.line_b = Text("")

    def render(self):
        return Group(self.line_a, self.line_b)


class TracePanel(_Panel):
    def __init__(self):
        super().__init__(id="trace", classes="panel")
        self.history = []

    def render(self):
        return Group(
            *_chart(
                self.history,
                max(4, self.content_size.width),
                max(1, self.content_size.height),
            )
        )


class Dashboard(App):
    TITLE = "j288"
    ENABLE_COMMAND_PALETTE = False
    CSS = """
    Screen {
        background: #101418;
        color: #e7eef4;
    }
    #status {
        height: 1;
        background: #1a2330;
        color: #e7eef4;
        padding: 0 1;
    }
    #notice {
        height: 1;
        background: #e2b15a;
        color: #1a1408;
        padding: 0 1;
        display: none;
    }
    #notice.open { display: block; }
    #row { height: 14; }
    .panel {
        height: 1fr;
        background: #161c24;
        border: round #3e5166;
        border-title-color: #8fb9cc;
        border-title-style: bold;
        border-title-align: left;
        padding: 0 1;
        overflow: hidden hidden;
    }
    #motor { width: 32; }
    #position { width: 2fr; }
    #tune { width: 36; }
    .panel.armed { border: round #ef5b54; }
    .panel.travel { border: round #e2b15a; }
    .panel.matched { border: round #46c98a; }
    #controls {
        height: 9;
        background: #101418;
    }
    #gains, #motion, #wave-row {
        height: 3;
        overflow-x: auto;
        overflow-y: hidden;
    }
    .ctl-tag {
        width: 5;
        height: 3;
        content-align: right middle;
        color: #8ea0b0;
    }
    #controls Button {
        min-width: 6;
        height: 3;
        margin: 0 0 0 1;
        padding: 0 1;
        background: #1c2836;
        color: #e7eef4;
        border: none;
    }
    #controls Button:hover, #controls Button:focus {
        background: #2c4a66;
        color: #e7eef4;
    }
    #wave-hz {
        width: 8;
    }
    #wave-lo, #wave-hi, #step-dt {
        width: 7;
    }
    #step-n {
        width: 4;
    }
    #step-spd, #ki-val {
        width: 6;
    }
    #foc {
        background: #1a3d2e;
        color: #b7f0d4;
    }
    #lock {
        background: #3a2a1c;
        color: #f0d7b0;
    }
    #live { height: 4; }
    #trace { height: 2fr; min-height: 4; }
    #keys {
        height: 2;
        padding: 0 1;
        background: #101418;
    }
    #logrow {
        height: 1fr;
        max-height: 12;
        min-height: 8;
    }
    #log, #serial {
        height: 1fr;
        width: 1fr;
        background: #101418;
        border: round #3e5166;
        border-title-color: #8fb9cc;
        border-title-style: bold;
        padding: 0 1;
    }
    #serial {
        color: #c5d0da;
    }
    #cmd {
        dock: bottom;
        display: none;
        margin: 0 1 1 1;
        border: round #e2b15a;
        background: #1c180f;
    }
    #cmd.open { display: block; }
    """

    def __init__(self, model):
        super().__init__()
        self.model = model
        self._history = []
        self._hist_at = 0.0
        self._log_cache = []
        self._work_queued = False
        self._epoch = model.epoch
        self._trace_src = ""

    def compose(self):
        yield Static("", id="status")
        yield Static("", id="notice")
        with Horizontal(id="row"):
            yield SidePanel()
            yield LevelPanel()
            yield RatePanel()
        with Vertical(id="controls"):
            with Horizontal(id="gains"):
                yield Static("Kp", classes="ctl-tag")
                yield Button("-1", id="kp-down")
                yield Button("+1", id="kp-up")
                yield Button("-.1", id="kp-down01")
                yield Button("+.1", id="kp-up01")
                yield Static("Kd", classes="ctl-tag")
                yield Button("-.1", id="kd-down")
                yield Button("+.1", id="kd-up")
                yield Button("-1", id="kd-down1")
                yield Button("+1", id="kd-up1")
                yield Static("PI", classes="ctl-tag")
                yield Button("-.01", id="ki-down01")
                yield Button("+.01", id="ki-up01")
                yield Button("-.1", id="ki-down")
                yield Button("+.1", id="ki-up")
                yield Static("0.00", id="ki-val", classes="ctl-tag")
            with Horizontal(id="motion"):
                yield Static("Tex", classes="ctl-tag")
                yield Button("-.1", id="tor-down")
                yield Button("+.1", id="tor-up")
                yield Static("Spd", classes="ctl-tag")
                yield Button("-.1", id="spd-down")
                yield Button("+.1", id="spd-up")
                yield Static("Pos", classes="ctl-tag")
                yield Button("-.1", id="pos-down")
                yield Button("+.1", id="pos-up")
                yield Static("Id", classes="ctl-tag")
                yield Button("-", id="id-down")
                yield Button("+", id="id-up")
                yield Button("Tmo on", id="tmo")
                yield Button("Lock", id="lock")
                yield Button("FOC", id="foc")
            with Horizontal(id="wave-row"):
                yield Static("Lo", classes="ctl-tag")
                yield Button("-1", id="lo-down1")
                yield Button("-.1", id="lo-down")
                yield Button("+.1", id="lo-up")
                yield Button("+1", id="lo-up1")
                yield Static("%.2f" % WAVE_MIN, id="wave-lo", classes="ctl-tag")
                yield Static("Hi", classes="ctl-tag")
                yield Button("-1", id="hi-down1")
                yield Button("-.1", id="hi-down")
                yield Button("+.1", id="hi-up")
                yield Button("+1", id="hi-up1")
                yield Static("%.2f" % WAVE_MAX, id="wave-hi", classes="ctl-tag")
                yield Button("Trace tor", id="trace-src")
                yield Button("Wave", id="wave")
                yield Button("-.01", id="hz-down")
                yield Button("+.01", id="hz-up")
                yield Static("%.2fHz" % WAVE_HZ, id="wave-hz", classes="ctl-tag")
                yield Button("Step", id="step")
                yield Button("-.1", id="dt-down")
                yield Button("+.1", id="dt-up")
                yield Static("%.2fs" % STEP_DT, id="step-dt", classes="ctl-tag")
                yield Button("-", id="n-down")
                yield Button("+", id="n-up")
                yield Static("%d" % STEP_N, id="step-n", classes="ctl-tag")
                yield Static("Vmax", classes="ctl-tag")
                yield Button("-.1", id="vmax-down")
                yield Button("+.1", id="vmax-up")
                yield Static("%.2f" % STEP_SPD, id="step-spd", classes="ctl-tag")
        yield LivePanel()
        yield TracePanel()
        yield Static(_key_bar(), id="keys")
        with Horizontal(id="logrow"):
            yield RichLog(id="log", wrap=True, markup=False, highlight=False, max_lines=200)
            yield Static("", id="serial")
        yield Input(placeholder="spd 0.5", id="cmd")

    def on_mount(self):
        titles = (
            ("#motor", "cmd", "output"),
            ("#position", "speed", "rad/s"),
            ("#tune", "data", "watch"),
            ("#live", "live", ""),
            ("#trace", "trace", "N·m"),
            ("#log", "log", ""),
            ("#serial", "serial", "raw"),
        )
        for sel, title, subtitle in titles:
            node = self.query_one(sel)
            node.border_title = title
            if subtitle:
                node.border_subtitle = subtitle
        self.query_one("#log", RichLog).can_focus = False
        self.query_one("#cmd", Input).can_focus = False
        self._paint()
        self.set_interval(_FRAME_S, self._on_frame)

    def on_unmount(self):
        self.model.shutdown()

    def on_key(self, event):
        cmd = self.query_one("#cmd", Input)
        if cmd.has_class("open"):
            if event.key == "escape":
                self._close_command()
                event.stop()
            return
        if event.key in ("left", "right", "up", "down"):
            self.model.handle_key(event.key)
        elif event.key in ("q", "Q"):
            event.stop()
            self.exit()
            return
        elif event.character == ":":
            self._open_command()
        elif event.character in ("m", "M", "0"):
            self.model.handle_key(event.character)
        else:
            return
        event.stop()
        self._paint()

    def on_button_pressed(self, event):
        action = _BUTTONS.get(event.button.id or "")
        if action is None:
            return
        field, delta = action
        self.model.nudge(field, delta)
        event.stop()
        self._paint()

    def on_input_submitted(self, event):
        text = event.value
        self._close_command()
        self.model.apply_command(text)
        self._paint()

    def _open_command(self):
        cmd = self.query_one("#cmd", Input)
        cmd.value = ""
        cmd.can_focus = True
        cmd.add_class("open")
        cmd.focus()

    def _close_command(self):
        cmd = self.query_one("#cmd", Input)
        cmd.can_focus = False
        cmd.remove_class("open")
        if cmd.has_focus:
            self.set_focus(None)

    def _alive(self):
        return self.is_running and self.is_mounted

    def _on_frame(self):
        if not self._alive():
            return
        self.model.slew_tick()
        self._sample()
        self._paint()
        if self._work_queued:
            return
        self._work_queued = True
        if not self.call_after_refresh(self._after_paint):
            self._work_queued = False

    def _after_paint(self):
        self._work_queued = False
        if not self._alive():
            return
        self.model.tick()
        self._sample()
        self._paint()

    def _sample(self):
        model = self.model
        if model.epoch != self._epoch or model.trace_src != self._trace_src:
            self._history = []
            self._epoch = model.epoch
            self._trace_src = model.trace_src
            self._hist_at = 0.0
        if not model.fresh():
            return
        now = time.monotonic()
        if now - self._hist_at < 0.03:
            return
        sample = model.meas_pos if model.trace_src == "pos" else model.meas_tor
        self._history.append(sample)
        del self._history[:-120]
        self._hist_at = now

    def _paint(self):
        if not self._alive():
            return
        try:
            status = self.query_one("#status", Static)
        except NoMatches:
            return
        model = self.model
        status.update(self._status_line())
        notice = self.query_one("#notice", Static)
        if model.notice:
            notice.update(Text("  " + model.notice, style="bold #1a1408"))
            notice.add_class("open")
        else:
            notice.remove_class("open")
        side = self.query_one("#motor", SidePanel)
        side.rows = [
            _kv("ID", str(model.motor_id), _TEXT),
            _kv("MODE", model.mode_word(), _GREEN if model.armed else _TEXT),
            _kv("TMO", "on" if model.timeout_flag else "off", _TEXT if model.timeout_flag else _AMBER),
            _kv("TEX", "%.3f" % model.cmd_tor, _TEXT),
            _kv("TOR", "%.3f" % (model.cmd_tor + model.tor_i), _TEXT),
            _kv("SPD", "%.3f" % model.cmd_spd, _TEXT),
            _kv("POS", "%.3f" % model.cmd_pos, _TEXT),
            _kv("KP", "%.1f" % model.kp, _TEXT),
            _kv("KD", "%.2f" % model.kd, _TEXT),
        ]
        side.refresh()
        tmo = self.query_one("#tmo", Button)
        tmo_label = "Tmo on" if model.timeout_flag else "Tmo off"
        if str(tmo.label) != tmo_label:
            tmo.label = tmo_label
        trace_btn = self.query_one("#trace-src", Button)
        trace_label = "Trace pos" if model.trace_src == "pos" else "Trace tor"
        if str(trace_btn.label) != trace_label:
            trace_btn.label = trace_label
        step_btn = self.query_one("#step", Button)
        step_label = "Step on" if model.step_on else "Step"
        if str(step_btn.label) != step_label:
            step_btn.label = step_label
        for node_id, text in (
            ("#step-dt", "%.2fs" % model.step_dt),
            ("#step-n", "%d" % int(model.step_n)),
            ("#step-spd", "%.2f" % model.step_spd),
            ("#ki-val", "%.2f" % model.ki),
        ):
            node = self.query_one(node_id, Static)
            if getattr(node, "_shown", None) != text:
                node.update(text)
                node._shown = text
        wave = self.query_one("#wave", Button)
        wave_label = "Wave on" if model.wave_on else "Wave"
        if str(wave.label) != wave_label:
            wave.label = wave_label
        hz = self.query_one("#wave-hz", Static)
        hz_label = "%.2fHz" % model.wave_hz
        if getattr(hz, "_shown", None) != hz_label:
            hz.update(hz_label)
            hz._shown = hz_label
        for node_id, text in (
            ("#wave-lo", "%.2f" % model.wave_lo),
            ("#wave-hi", "%.2f" % model.wave_hi),
        ):
            node = self.query_one(node_id, Static)
            if getattr(node, "_shown", None) != text:
                node.update(text)
                node._shown = text
        foc = self.query_one("#foc", Button)
        lock = self.query_one("#lock", Button)
        foc.variant = "success" if model.armed else "default"
        lock.variant = "warning" if not model.armed else "default"
        matched = model.matched()
        fresh = model.fresh()
        away = model.tracked_goal() - model.tracked_now()
        if not fresh:
            color = _AMBER
            note = "no feedback"
        elif matched:
            color = _GREEN
            note = "matches"
        elif model.armed:
            color = _RED
            note = "away %+.3f" % away
        else:
            color = _AMBER
            note = "away %+.3f" % away
        if model.position_loop:
            unit = "rad"
            title = "position"
            window_lo, window_hi = _pos_window(model.tracked_now(), model.tracked_goal())
        else:
            unit = "rad/s"
            title = "speed"
            span = max(1.0, abs(model.tracked_now()), abs(model.tracked_goal()))
            window_lo, window_hi = -span, span
        level = self.query_one("#position", LevelPanel)
        if level.border_title != title:
            level.border_title = title
        if level.border_subtitle != unit:
            level.border_subtitle = unit
        level.now_text = "%.3f" % model.tracked_now()
        level.goal_text = "%.3f %s" % (model.tracked_goal(), unit)
        level.goal_num = "%.2f" % model.tracked_goal()
        level.now_frac = _frac(model.tracked_now(), window_lo, window_hi)
        level.goal_frac = _frac(model.tracked_goal(), window_lo, window_hi)
        level.goal_color = color
        level.note = note
        level.set_class(matched, "matched")
        level.set_class(bool(model.armed and fresh and not matched), "armed")
        level.set_class((not matched) and not (model.armed and fresh), "travel")
        level.refresh()
        rate = self.query_one("#tune", RatePanel)
        if model.merror:
            rate.word, rate.word_color = "FAULT", _RED
        elif not model.armed:
            rate.word, rate.word_color = "LOCK", _MUTED
        elif matched:
            rate.word, rate.word_color = "HOLD", _GREEN
        else:
            rate.word, rate.word_color = "FOC", _RED
        loop = "position" if model.position_loop else "speed"
        crc = "crc ok" if model.correct else "crc —"
        if model.fb_timeout:
            crc = "timeout bit"
        rate.detail = "%s  Kp %.1f  Kd %.2f  PI %.2f  %s" % (
            loop,
            model.kp,
            model.kd,
            model.ki,
            crc,
        )
        rate.miss_text = str(model.miss)
        rate.miss_color = _GREEN if model.miss == 0 and fresh else (_RED if model.miss else _MUTED)
        rate.hsg_text = "—" if model.temp is None else "%d°C" % model.temp
        rate.hsg_color = _temp_color(model.temp)
        rate.wnd_text = "—" if model.sensor is None else "%d°C" % model.sensor
        rate.wnd_color = _temp_color(model.sensor)
        rate.volt_text = "—" if model.vol is None else "%.1f V" % model.vol
        if model.merror:
            names = error_names(model.merror)
            rate.err_text = names[0] if names else "0x%X" % model.merror
            rate.err_color = _RED
        elif model.mwarn:
            names = warn_names(model.mwarn)
            rate.err_text = names[0] if names else "warn"
            rate.err_color = _AMBER
        elif fresh:
            rate.err_text = "ok"
            rate.err_color = _GREEN
        else:
            rate.err_text = "—"
            rate.err_color = _MUTED
        rate.slew_text = "%.3f" % model.meas_spd
        rate.slew_frac = _clamp01(abs(model.meas_spd) / 2.0)
        rate.set_class(bool(model.armed and fresh and not matched), "armed")
        rate.refresh()
        live = self.query_one("#live", LivePanel)
        live.line_a = Text.assemble(
            ("spd ", _MUTED),
            ("%.3f" % model.meas_spd, "bold " + _CYAN),
            (" / ", _MUTED),
            ("%.3f" % model.cmd_spd, "bold " + color),
            ("    pos ", _MUTED),
            ("%.3f" % model.meas_pos, "bold " + _TEXT),
            (" / ", _MUTED),
            ("%.3f" % model.cmd_pos, _TEXT),
            ("    tor ", _MUTED),
            ("%.3f" % model.meas_tor, _TEXT),
            ("  i ", _MUTED),
            ("%+.3f" % model.tor_i, _TEXT),
        )
        live.line_b = Text.assemble(
            ("ex ", _MUTED),
            ("%.3f" % model.ex_pos, _TEXT),
            ("    rx ", _MUTED),
            (str(model.rxlen), _TEXT),
            ("    rtt ", _MUTED),
            ("%.1f ms" % model.rtt_ms, _TEXT),
            ("    id ", _MUTED),
            (str(model.fb_id), _TEXT),
            ("    mode ", _MUTED),
            (str(model.fb_mode), _TEXT),
        )
        live.refresh()
        trace = self.query_one("#trace", TracePanel)
        trace.history = self._history
        unit = "rad" if model.trace_src == "pos" else "N·m"
        if trace.border_subtitle != unit:
            trace.border_subtitle = unit
        trace.refresh()
        self._sync_log()
        self._sync_serial()

    def _status_line(self):
        model = self.model
        if model.merror:
            pill, style = " FAULT ", "bold #1a0c0c on #ef5b54"
        elif model.sim and not model.linked:
            pill, style = " SIM ", "bold #1a1408 on #e2b15a"
        elif not model.linked:
            pill, style = " OFFLINE ", "bold #d7dee6 on #3a4450"
        elif model.fb_timeout:
            pill, style = " TRIP ", "bold #1a0c0c on #ef5b54"
        elif model.miss >= 3:
            pill, style = " MISS ", "bold #1a1408 on #e2b15a"
        elif model.armed and not model.matched():
            pill, style = " ARMED ", "bold #1a0c0c on #ef5b54"
        elif model.armed:
            pill, style = " LIVE ", "bold #062117 on #46c98a"
        else:
            pill, style = " IDLE ", "bold #d7dee6 on #3a4450"
        text = Text()
        text.append(" J288 ", style="bold #9be7d4")
        text.append(pill, style=style)
        text.append("  id ", style=_MUTED)
        text.append(str(model.motor_id), style="bold " + _TEXT)
        text.append("   ", style=_MUTED)
        text.append(short_port(model.port) if model.port else "no port", style=_TEXT)
        if model.temp is not None:
            text.append("   ", style=_MUTED)
            text.append("%d°C" % model.temp, style=_temp_color(model.temp))
        if model.vol is not None:
            text.append("  ", style=_MUTED)
            text.append("%.1f V" % model.vol, style=_TEXT)
        return text

    def _sync_log(self):
        lines = list(self.model.log_lines)
        if lines == self._log_cache:
            return
        log = self.query_one("#log", RichLog)
        if lines[: len(self._log_cache)] != self._log_cache:
            log.clear()
            fresh = lines
        else:
            fresh = lines[len(self._log_cache) :]
        for line in fresh:
            log.write(Text(line, style="#c5d0da"))
        self._log_cache = lines

    def _sync_serial(self):
        text = _format_raw(self.model.raw_tx, self.model.raw_rx)
        if text == getattr(self, "_raw_shown", None):
            return
        self._raw_shown = text
        self.query_one("#serial", Static).update(Text(text, style="#c5d0da"))


def _hex_lines(data, per=8):
    if not data:
        return ["—"]
    words = ["%02X" % byte for byte in data]
    return [" ".join(words[i : i + per]) for i in range(0, len(words), per)]


def _format_raw(tx, rx):
    tx = bytes(tx or b"")
    rx = bytes(rx or b"")
    lines = ["TX %d" % len(tx)]
    lines.extend(_hex_lines(tx))
    lines.append("RX %d" % len(rx))
    lines.extend(_hex_lines(rx))
    return "\n".join(lines)


_BUTTONS = {
    "kp-down": ("kp", -1.0),
    "kp-up": ("kp", 1.0),
    "kp-down01": ("kp", -0.1),
    "kp-up01": ("kp", 0.1),
    "kd-down": ("kd", -0.1),
    "kd-up": ("kd", 0.1),
    "kd-down1": ("kd", -1.0),
    "kd-up1": ("kd", 1.0),
    "tor-down": ("tor", -0.1),
    "tor-up": ("tor", 0.1),
    "spd-down": ("spd", -0.1),
    "spd-up": ("spd", 0.1),
    "pos-down": ("pos", -0.1),
    "pos-up": ("pos", 0.1),
    "id-down": ("id", -1),
    "id-up": ("id", 1),
    "tmo": ("timeout", 0),
    "lock": ("mode", 0),
    "foc": ("mode", 1),
    "wave": ("wave", 0),
    "step": ("step", 0),
    "dt-down": ("step_dt", -0.1),
    "dt-up": ("step_dt", 0.1),
    "n-down": ("step_n", -1),
    "n-up": ("step_n", 1),
    "vmax-down": ("step_spd", -0.1),
    "vmax-up": ("step_spd", 0.1),
    "ki-down01": ("ki", -0.01),
    "ki-up01": ("ki", 0.01),
    "ki-down": ("ki", -0.1),
    "ki-up": ("ki", 0.1),
    "trace-src": ("trace", 0),
    "hz-down": ("hz", -0.01),
    "hz-up": ("hz", 0.01),
    "lo-down": ("wave_lo", -0.1),
    "lo-up": ("wave_lo", 0.1),
    "lo-down1": ("wave_lo", -1.0),
    "lo-up1": ("wave_lo", 1.0),
    "hi-down": ("wave_hi", -0.1),
    "hi-up": ("wave_hi", 0.1),
    "hi-down1": ("wave_hi", -1.0),
    "hi-up1": ("wave_hi", 1.0),
}


def _pos_window(now, goal):
    mid = (now + goal) / 2.0
    half = max(0.5, abs(goal - now))
    return mid - half, mid + half
