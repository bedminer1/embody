#!/usr/bin/env python3
"""Run Sphero's bundled LED example on the Pi 5 GPIO UART."""

import runpy

from sphero_sdk import SerialObserverDal


original_init = SerialObserverDal.__init__


def pi5_uart_init(self, port_id="/dev/ttyAMA0", baud=115200):
    original_init(self, port_id=port_id, baud=baud)


SerialObserverDal.__init__ = pi5_uart_init

runpy.run_path(
    "/home/alex/embody/sphero-sdk-raspberrypi-python/"
    "getting_started/observer/leds/set_multiple_leds.py",
    run_name="__main__",
)
