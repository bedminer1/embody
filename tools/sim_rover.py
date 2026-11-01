#!/usr/bin/env python3
"""Simulated rover agent for testing the desktop app and brain without hardware.

Speaks the same TCP protocol as pi/agent.py and the ESP32 firmware (type byte,
big-endian u32 length, JSON payload) and integrates `vel` commands as an ideal
unicycle, with a 500 ms command timeout like the real agents. WASD keys are
accepted but do not move the simulated pose. There is no camera.

    python3 tools/sim_rover.py --port 8765
    cd desktop && cargo run --release -- --json --direct 127.0.0.1:8765
"""
import argparse
import asyncio
import json
import math
import time

PERIOD = 0.05


class Rover:
    def __init__(self, slip):
        self.x = self.y = self.yaw = 0.0
        self.velocity = None
        self.keys = ""
        self.last_command = 0.0
        self.slip = slip

    def command(self, data):
        self.last_command = time.monotonic()
        if data.get("reset_pose") is True:
            self.x = self.y = self.yaw = 0.0
        elif isinstance(data.get("vel"), list):
            self.velocity = tuple(float(v) for v in data["vel"])
            self.keys = ""
        else:
            self.velocity = None
            self.keys = data.get("keys", "")

    def active_velocity(self):
        if self.velocity and time.monotonic() - self.last_command <= 0.5:
            return self.velocity
        return None

    def step(self, dt):
        velocity = self.active_velocity()
        if not velocity:
            return
        linear, yaw_rate = (value * self.slip for value in velocity)
        self.yaw = (self.yaw + yaw_rate * dt + 180.0) % 360.0 - 180.0
        radians = math.radians(self.yaw)
        self.x += -math.sin(radians) * linear * dt
        self.y += math.cos(radians) * linear * dt

    def telemetry(self):
        velocity = self.active_velocity()
        return {"rvr_connected": True, "tracks": [0, 0], "speed_limit": 70,
                "pivot_forward_percent": 60, "battery": {"percentage": 100},
                "pose": {"x": self.x, "y": self.y, "yaw": self.yaw},
                "velocity": None, "vel": list(velocity) if velocity else None,
                "sensors": {}, "error": None}


def packet(kind, payload):
    return kind + len(payload).to_bytes(4, "big") + payload


async def client(reader, writer, rover):
    async def receive():
        while True:
            header = await reader.readexactly(5)
            length = int.from_bytes(header[1:], "big")
            rover.command(json.loads(await reader.readexactly(length)))

    async def transmit():
        while True:
            rover.step(PERIOD)
            writer.write(packet(b"T", json.dumps(rover.telemetry()).encode()))
            await writer.drain()
            await asyncio.sleep(PERIOD)

    tasks = [asyncio.create_task(receive()), asyncio.create_task(transmit())]
    try:
        await asyncio.wait(tasks, return_when=asyncio.FIRST_COMPLETED)
    finally:
        for task in tasks:
            task.cancel()
        rover.velocity = None
        writer.close()


async def main(args):
    rover = Rover(args.slip)
    server = await asyncio.start_server(lambda r, w: client(r, w, rover),
                                        "127.0.0.1", args.port)
    print(f"Simulated rover on 127.0.0.1:{args.port}", flush=True)
    async with server:
        await server.serve_forever()


if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument("--port", type=int, default=8765)
    parser.add_argument("--slip", type=float, default=1.0,
                        help="fraction of the commanded velocity achieved (0 = blocked)")
    try:
        asyncio.run(main(parser.parse_args()))
    except KeyboardInterrupt:
        pass
