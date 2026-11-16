#!/usr/bin/env bash
# Launch the J288 / S288 bench dashboard.
# Usage:
#   ./launch-ui.sh
#   ./launch-ui.sh /dev/cu.usbmodem...
#   ./launch-ui.sh --sim

set -euo pipefail

cd "$(dirname "$0")"

if [[ ! -x .venv/bin/python ]]; then
  py=""
  for candidate in python3.12 python3.13 python3; do
    if command -v "$candidate" >/dev/null 2>&1; then
      py="$candidate"
      break
    fi
  done
  if [[ -z "$py" ]]; then
    echo "python3.12 is required" >&2
    exit 1
  fi
  "$py" -m venv .venv
  .venv/bin/python -m pip install -q -r requirements.txt
fi

exec .venv/bin/python -m j288 "$@"
