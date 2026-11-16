"""One debug file per day for connect and probe lines.

The screen log stays short. This file is for a hang: the last line is the
call that did not return. The folder keeps the newest five days. Polls at
the normal rate are not written here.
"""

import threading
import time
from pathlib import Path

MAX_BYTES = 64 * 1024
KEEP_BYTES = 32 * 1024
KEEP_FILES = 5

_lock = threading.Lock()


def log_dir():
    return Path(__file__).resolve().parent.parent / ".log"


def log_path():
    return log_dir() / ("j288_%s.log" % time.strftime("%Y-%m-%d"))


def note(message):
    line = "%s %s\n" % (
        time.strftime("%Y-%m-%d %H:%M:%S"),
        str(message).replace("\n", " "),
    )
    try:
        with _lock:
            path = log_path()
            path.parent.mkdir(parents=True, exist_ok=True)
            with path.open("a", encoding="utf-8") as handle:
                handle.write(line)
            if path.stat().st_size > MAX_BYTES:
                tail = path.read_bytes()[-KEEP_BYTES:]
                cut = tail.find(b"\n")
                # Drop a partial first line. A newline only at the end stays.
                if 0 <= cut < len(tail) - 1:
                    tail = tail[cut + 1 :]
                path.write_bytes(tail)
            _drop_old(path.parent)
    except OSError:
        return


def _drop_old(directory):
    files = sorted(directory.glob("j288_*.log"))
    for path in files[:-KEEP_FILES]:
        try:
            path.unlink()
        except OSError:
            pass
