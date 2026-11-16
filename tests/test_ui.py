import asyncio
import threading
import time

from j288.bench import MotorBench
from j288.ui import Dashboard


def test_sim_screen_arms_from_the_foc_button():
    model = MotorBench(sim=True)

    async def run():
        app = Dashboard(model)
        async with app.run_test(size=(160, 50)) as pilot:
            await pilot.pause()
            assert "SIM" in str(app.query_one("#status").render())
            await pilot.click("#foc")
            await pilot.pause()
            assert model.armed is True
            await pilot.click("#lock")
            await pilot.pause()
            assert model.armed is False
            for sel in ("#vmax-up", "#step-spd", "#wave-hi", "#wave-hz", "#foc", "#port-name", "#port-next"):
                box = app.query_one(sel).region
                assert box.width > 0
                assert box.x + box.width <= app.size.width
            step = app.query_one("#step-spd").region
            port = app.query_one("#port-next").region
            assert port.y >= step.y + step.height
            assert port.y + port.height <= app.size.height
            kp = app.query_one("#kp-down").region
            lo = app.query_one("#lo-down1").region
            kd = app.query_one("#kd-down1").region
            hi = app.query_one("#hi-down1").region
            tex = app.query_one("#tor-down").region
            name = app.query_one("#port-name").region
            assert kp.x == lo.x == tex.x
            assert kd.x == hi.x
            assert port.x <= name.x + name.width + 1
            assert kp.width == app.query_one("#ki-up01").region.width

    asyncio.run(run())


def test_port_buttons_pick_a_device_and_do_not_scan():
    model = MotorBench(sim=True)
    model.port_pref = "/dev/ttyACM0"
    seen = []
    model.choose_port = lambda step: seen.append(int(step))

    async def run():
        app = Dashboard(model)
        async with app.run_test(size=(160, 50)) as pilot:
            await pilot.pause()
            assert "ttyACM0" in str(app.query_one("#port-name").render())
            await pilot.click("#port-next")
            await pilot.click("#port-prev")
            for _ in range(40):
                if seen == [1, -1]:
                    break
                await pilot.pause(0.02)
            assert seen == [1, -1]

    asyncio.run(run())


def test_a_stuck_connect_does_not_freeze_the_screen(tmp_path):
    model = MotorBench(sim=False, settings_path=tmp_path / "settings.json")
    started = threading.Event()
    release = threading.Event()

    def connect():
        started.set()
        release.wait(2.0)
        return False

    model.connect = connect
    result = []

    async def run():
        app = Dashboard(model)
        moved = threading.Event()
        original = app._on_frame

        def wrapped():
            moved.set()
            original()

        app._on_frame = wrapped

        def watch():
            if not started.wait(1.5):
                result.append("no-connect")
                return
            moved.clear()
            time.sleep(0.25)
            result.append("alive" if moved.is_set() else "frozen")

        threading.Thread(target=watch, daemon=True).start()
        async with app.run_test(size=(80, 42)) as pilot:
            await pilot.pause(0.8)
            release.set()
        for _ in range(20):
            if result:
                break
            await asyncio.sleep(0.05)

    try:
        asyncio.run(run())
    finally:
        release.set()
    assert result == ["alive"]
