#!/usr/bin/env python3
"""Short, bounded UART diagnostic for an RVR with its tracks clear."""
import argparse
import asyncio

from sphero_sdk import RawMotorModesEnum, SerialAsyncDal, SpheroRvrAsync
from sphero_sdk.asyncio.server.port.serial_sphero_port import SerialSpheroPort


def trace_serial():
    receive = SerialSpheroPort.data_received
    send = SerialSpheroPort.send

    def traced_receive(self, data):
        print("RX", data.hex(), flush=True)
        return receive(self, data)

    def traced_send(self, message):
        print("TX", message.serialise().hex(), flush=True)
        return send(self, message)

    SerialSpheroPort.data_received = traced_receive
    SerialSpheroPort.send = traced_send


async def probe(rvr, drive):
    off = RawMotorModesEnum.off.value
    motor_command_attempted = False
    try:
        await rvr.wake()
        await asyncio.sleep(1)
        try:
            battery = await asyncio.wait_for(
                rvr.get_battery_percentage(timeout=2), timeout=3
            )
            print("BATTERY", battery, flush=True)
        except Exception as exc:
            print("BATTERY_ERROR", type(exc).__name__, str(exc), flush=True)
            return
        if not isinstance(battery, dict) or "percentage" not in battery:
            print("BATTERY_ERROR unexpected response", repr(battery), flush=True)
            return
        if drive:
            await rvr.set_custom_control_system_timeout(command_timeout=500)
            await asyncio.sleep(0.1)
            motor_command_attempted = True
            await rvr.raw_motors(
                RawMotorModesEnum.forward.value, 35,
                RawMotorModesEnum.forward.value, 35,
            )
            print("MOTOR_ON 35/255 for 0.2s", flush=True)
            await asyncio.sleep(0.2)
    finally:
        if motor_command_attempted:
            await rvr.raw_motors(off, 0, off, 0)
            await rvr.drive_stop()
            print("MOTOR_OFF", flush=True)
        await asyncio.sleep(0.2)
        await rvr.close()


if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument("--port", default="/dev/ttyAMA0")
    parser.add_argument("--drive", action="store_true")
    args = parser.parse_args()
    trace_serial()
    loop = asyncio.new_event_loop()
    asyncio.set_event_loop(loop)
    try:
        rvr = SpheroRvrAsync(dal=SerialAsyncDal(loop, port_id=args.port))
        loop.run_until_complete(probe(rvr, args.drive))
    finally:
        loop.run_until_complete(loop.shutdown_asyncgens())
        loop.close()
