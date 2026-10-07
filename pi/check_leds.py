#!/usr/bin/env python3
"""Run Sphero's first LED exercise using the Pi 5 UART device."""

import asyncio

from sphero_sdk import Colors, SerialAsyncDal, SpheroRvrAsync
from sphero_sdk.asyncio.server.port.serial_sphero_port import SerialSpheroPort


def trace_serial():
    receive = SerialSpheroPort.data_received

    def traced_receive(self, data):
        print("RX", data.hex(), flush=True)
        return receive(self, data)

    SerialSpheroPort.data_received = traced_receive


async def main(rvr):
    try:
        print("WAKE", flush=True)
        await rvr.wake()
        await asyncio.sleep(2)
        print("YELLOW", flush=True)
        await rvr.led_control.set_all_leds_color(Colors.yellow)
        await asyncio.sleep(1)
        print("RED", flush=True)
        await rvr.led_control.set_all_leds_rgb(255, 0, 0)
        await asyncio.sleep(2)
    finally:
        await rvr.close()


if __name__ == "__main__":
    trace_serial()
    loop = asyncio.new_event_loop()
    asyncio.set_event_loop(loop)
    try:
        rvr = SpheroRvrAsync(dal=SerialAsyncDal(loop, port_id="/dev/ttyAMA0"))
        loop.run_until_complete(main(rvr))
    finally:
        loop.run_until_complete(loop.shutdown_asyncgens())
        loop.close()
