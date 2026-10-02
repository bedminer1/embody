#!/usr/bin/env python3
"""Pi-side camera and RVR bridge. Wire format: type byte + BE u32 length + payload."""
import argparse
import asyncio
import contextlib
import json
import logging
import os
import signal
import subprocess
import threading
import time


LOG = logging.getLogger("embody.pi")
MAX_PACKET = 64 * 1024


class Camera:
    def __init__(self, device, width, height, fps):
        self.device, self.width, self.height = device, width, height
        self.fps = fps
        self.latest = None
        self.sequence = 0
        self.lock = threading.Lock()
        self.stop = threading.Event()
        self.thread = None
        self.process = None

    def start(self):
        self.thread = threading.Thread(target=self._capture, daemon=True)
        self.thread.start()

    def _capture(self):
        command = ["ffmpeg", "-hide_banner", "-nostdin", "-loglevel", "error",
                   "-f", "v4l2", "-input_format", "mjpeg",
                   "-video_size", f"{self.width}x{self.height}",
                   "-framerate", str(self.fps), "-i", self.device,
                   "-c:v", "copy", "-f", "mjpeg", "pipe:1"]
        while not self.stop.is_set():
            try:
                self.process = subprocess.Popen(command, stdout=subprocess.PIPE,
                                                stderr=subprocess.DEVNULL)
            except OSError as exc:
                LOG.error("Cannot start camera capture: %s", exc)
                time.sleep(1)
                continue
            LOG.info("Camera MJPEG capture started: %sx%s @ %s fps",
                     self.width, self.height, self.fps)
            buffer = bytearray()
            try:
                while not self.stop.is_set():
                    chunk = self.process.stdout.read(65536)
                    if not chunk:
                        break
                    buffer.extend(chunk)
                    while True:
                        start = buffer.find(b"\xff\xd8")
                        if start < 0:
                            buffer.clear()
                            break
                        if start:
                            del buffer[:start]
                        end = buffer.find(b"\xff\xd9", 2)
                        if end < 0:
                            break
                        frame = bytes(buffer[:end + 2])
                        del buffer[:end + 2]
                        with self.lock:
                            self.latest = frame
                            self.sequence += 1
                    if len(buffer) > 2 * 1024 * 1024:
                        buffer.clear()
            finally:
                self.process.terminate()
                try:
                    self.process.wait(timeout=2)
                except subprocess.TimeoutExpired:
                    self.process.kill()
            if not self.stop.is_set():
                LOG.warning("Camera process exited; retrying")
                time.sleep(1)

    def snapshot(self):
        with self.lock:
            return self.sequence, self.latest

    def close(self):
        self.stop.set()
        if self.process and self.process.poll() is None:
            self.process.terminate()
        if self.thread:
            self.thread.join(timeout=2)


class Rover:
    def __init__(self, port, speed):
        self.port, self.speed = port, speed
        self.rvr = None
        self.sensors = {}
        self.last_keys = ""
        self.last_command = 0.0
        self.last_sent = None
        self.last_motor_write = 0.0
        self.battery = None
        self.error = None

    async def connect(self):
        if not self.port:
            LOG.warning("Camera-only mode: RVR serial port disabled")
            return
        from sphero_sdk import SpheroRvrAsync, SerialAsyncDal, RvrStreamingServices
        self.rvr = SpheroRvrAsync(dal=SerialAsyncDal(asyncio.get_running_loop(), port_id=self.port))
        await self.rvr.wake()
        await asyncio.sleep(2)
        await self.rvr.set_custom_control_system_timeout(command_timeout=750)
        async def handler(name, data):
            self.sensors[name] = data
        for name in (RvrStreamingServices.imu, RvrStreamingServices.accelerometer,
                     RvrStreamingServices.encoders, RvrStreamingServices.ambient_light,
                     RvrStreamingServices.color_detection):
            async def callback(data, name=name):
                await handler(name, data)
            await self.rvr.sensor_control.add_sensor_data_handler(service=name, handler=callback)
        await self.rvr.sensor_control.start(interval=250)
        LOG.info("RVR connected on %s", self.port)

    def input(self, keys):
        self.last_keys = "".join(k for k in keys.lower() if k in "wasd")
        self.last_command = time.monotonic()

    def tracks(self):
        if time.monotonic() - self.last_command > 0.5:
            return (0, 0)
        keys = self.last_keys
        speed = self.speed
        forward = ("w" in keys) - ("s" in keys)
        turn = ("d" in keys) - ("a" in keys)
        if not forward:
            return (turn * speed, -turn * speed)
        left = right = forward * speed
        if (forward > 0 and turn < 0) or (forward < 0 and turn > 0):
            left = int(left * 0.45)
        elif (forward > 0 and turn > 0) or (forward < 0 and turn < 0):
            right = int(right * 0.45)
        return (left, right)

    async def motor_loop(self):
        from sphero_sdk import RawMotorModesEnum
        while True:
            tracks = self.tracks()
            if self.rvr and (tracks != self.last_sent or
                             (tracks != (0, 0) and time.monotonic() - self.last_motor_write >= 0.2)):
                def mode(value):
                    return (RawMotorModesEnum.forward if value > 0 else
                            RawMotorModesEnum.reverse if value < 0 else RawMotorModesEnum.off).value
                try:
                    await self.rvr.raw_motors(mode(tracks[0]), abs(tracks[0]),
                                              mode(tracks[1]), abs(tracks[1]))
                    self.error = None
                    self.last_sent = tracks
                    self.last_motor_write = time.monotonic()
                except Exception as exc:
                    self.error = str(exc)
                    LOG.exception("Motor command failed")
            await asyncio.sleep(0.05)

    async def battery_loop(self):
        while True:
            if self.rvr:
                try:
                    self.battery = await self.rvr.get_battery_percentage()
                except Exception as exc:
                    self.error = str(exc)
            await asyncio.sleep(10)

    async def close(self):
        self.input("")
        if self.rvr:
            from sphero_sdk import RawMotorModesEnum
            try:
                off = RawMotorModesEnum.off.value
                await self.rvr.raw_motors(off, 0, off, 0)
                await self.rvr.sensor_control.clear()
                await self.rvr.restore_default_control_system_timeout()
            finally:
                await self.rvr.close()


def packet(kind, payload):
    return kind + len(payload).to_bytes(4, "big") + payload


async def client(reader, writer, camera, rover):
    LOG.info("Desktop connected")
    rover.input("")

    async def receive():
        while True:
            header = await reader.readexactly(5)
            length = int.from_bytes(header[1:], "big")
            if header[:1] != b"C" or length > MAX_PACKET:
                raise ValueError("Invalid control packet")
            data = json.loads(await reader.readexactly(length))
            rover.input(data.get("keys", ""))

    async def transmit():
        last_seq = -1
        last_telemetry = 0.0
        while True:
            seq, frame = camera.snapshot()
            if frame and seq != last_seq:
                writer.write(packet(b"V", frame))
                last_seq = seq
            now = time.monotonic()
            if now - last_telemetry >= 0.25:
                info = {"rvr_connected": rover.rvr is not None,
                        "keys": rover.last_keys if now - rover.last_command <= 0.5 else "",
                        "tracks": rover.tracks(), "speed_limit": rover.speed,
                        "battery": rover.battery,
                        "sensors": rover.sensors, "error": rover.error,
                        "camera_sequence": seq}
                writer.write(packet(b"T", json.dumps(info, default=str).encode()))
                last_telemetry = now
            await writer.drain()
            await asyncio.sleep(1 / camera.fps)

    tasks = [asyncio.create_task(receive()), asyncio.create_task(transmit())]
    try:
        done, pending = await asyncio.wait(tasks, return_when=asyncio.FIRST_COMPLETED)
        for task in done:
            task.result()
    except (asyncio.IncompleteReadError, ConnectionError):
        pass
    finally:
        rover.input("")
        for task in tasks:
            task.cancel()
        writer.close()
        with contextlib.suppress(ConnectionError, OSError):
            await writer.wait_closed()
        LOG.info("Desktop disconnected; motors stopping")


async def main(args):
    loop = asyncio.get_running_loop()
    main_task = asyncio.current_task()
    loop.add_signal_handler(signal.SIGTERM, main_task.cancel)
    camera = Camera(args.camera, args.width, args.height, args.fps)
    rover = Rover(args.serial, args.speed)
    camera.start()
    await rover.connect()
    motor = asyncio.create_task(rover.motor_loop())
    battery = asyncio.create_task(rover.battery_loop())
    lock = asyncio.Lock()

    async def accept(reader, writer):
        if lock.locked():
            writer.close()
            await writer.wait_closed()
            return
        async with lock:
            await client(reader, writer, camera, rover)

    server = await asyncio.start_server(accept, args.host, args.port)
    LOG.info("Listening at %s:%d", args.host, args.port)
    try:
        async with server:
            await server.serve_forever()
    finally:
        motor.cancel()
        battery.cancel()
        camera.close()
        await rover.close()


if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument("--host", default="127.0.0.1")
    parser.add_argument("--port", type=int, default=8765)
    parser.add_argument("--camera", default="/dev/video0")
    parser.add_argument("--width", type=int, default=848)
    parser.add_argument("--height", type=int, default=480)
    parser.add_argument("--fps", type=int, default=30)
    parser.add_argument("--speed", type=int, default=70)
    parser.add_argument("--serial", default=os.environ.get("EMBODY_RVR_SERIAL", ""))
    args = parser.parse_args()
    if not 1 <= args.speed <= 255:
        parser.error("--speed must be 1..255")
    logging.basicConfig(level=logging.INFO, format="%(asctime)s %(levelname)s %(message)s")
    try:
        asyncio.run(main(args))
    except (KeyboardInterrupt, asyncio.CancelledError):
        pass
