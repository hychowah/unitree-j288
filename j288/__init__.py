"""Bench and shared-bus control for a Unitree J288 / S288 servo."""

from j288.bus import SerialBus, list_serial_ports
from j288.motor import Motor

__all__ = ["Motor", "SerialBus", "list_serial_ports"]
