"""python -m j288 [--sim] [port]"""

import sys

from j288.bench import MotorBench
from j288.ui import Dashboard


def main(argv=None):
    argv = list(sys.argv[1:] if argv is None else argv)
    sim = False
    if argv and argv[0] == "--sim":
        sim = True
        argv = argv[1:]
    port = argv[0] if argv else ""
    model = MotorBench(port=port, sim=sim)
    try:
        Dashboard(model).run()
    finally:
        model.shutdown()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
