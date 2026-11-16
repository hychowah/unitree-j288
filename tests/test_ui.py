import asyncio

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

    asyncio.run(run())
