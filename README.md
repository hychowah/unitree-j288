# J288 bench

Terminal dashboard for a Unitree J288 / S288 servo. The shaft stays locked until you enable FOC.

```bash
./launch-ui.sh
./launch-ui.sh /dev/cu.usbmodem...
./launch-ui.sh --sim
```

The script creates `.venv` on first launch. Arrows slew speed and position, `m` enables FOC, `0` stops, `:` types a command, and `q` quits.

Commands: `spd`, `pos`, `tor`, `kp`, `kd`, `ki`, `id`, `timeout`, `rate`, `port`, `connect`, `clear`, `reset`.

Gains, sweep limits, and the last port are written to `motor_settings.json` on quit. That file stays on this machine.

```bash
.venv/bin/python -m pytest
```

## Code

- `j288/protocol.py` — command and reply frames, CRC, scaling, fault bits
- `j288/bench.py` — poll loop, FOC, wave, and step
- `j288/ui.py` — the screen
- `vendor/` — Unitree's original Python example

The bus is 6 Mbps, 8N1, half duplex. A mode 0 frame reads feedback while the shaft stays locked. On macOS open the `/dev/cu.*` node.
