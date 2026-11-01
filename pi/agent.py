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
    def __init__(self, port, speed, pivot_forward_percent):
        self.port, self.speed = port, speed
        self.pivot_forward_percent = pivot_forward_percent
        self.rvr = None
        self.connected = False
        self.sensors = {}
        self.last_keys = ""
        self.velocity = None  # (m/s, deg/s) from a "vel" command, else keys
        self.reset_requested = False
        self.last_command = 0.0
        self.last_sent = None
        self.last_motor_write = 0.0
        self.battery = None
        self.error = None

    def prepare(self, loop):
        if not self.port:
            return
        from sphero_sdk import SerialAsyncDal, SpheroRvrAsync
        # The official SDK runs its firmware check in the constructor, so
        # construction must happen before this event loop starts running.
        self.rvr = SpheroRvrAsync(dal=SerialAsyncDal(loop, port_id=self.port))

    async def connect(self):
        if not self.port:
            LOG.warning("Camera-only mode: RVR serial port disabled")
            return
        from sphero_sdk import RvrStreamingServices
        await self.rvr.wake()
        await asyncio.sleep(2)
        battery = await asyncio.wait_for(
            self.rvr.get_battery_percentage(timeout=3), timeout=4
        )
        if not isinstance(battery, dict) or "percentage" not in battery:
            raise RuntimeError(f"RVR did not answer battery query: {battery!r}")
        self.battery = battery
        self.connected = True
        LOG.info("RVR battery response: %s", battery)
        await self.rvr.set_custom_control_system_timeout(command_timeout=750)
        async def handler(name, data):
            # The SDK passes every service of the notification; keep this one.
            self.sensors[name] = dict(data.get(name, data), time=time.monotonic())
        for name in (RvrStreamingServices.imu, RvrStreamingServices.accelerometer,
                     RvrStreamingServices.encoders, RvrStreamingServices.ambient_light,
                     RvrStreamingServices.color_detection, RvrStreamingServices.locator,
                     RvrStreamingServices.velocity):
            async def callback(data, name=name):
                await handler(name, data)
            await self.rvr.sensor_control.add_sensor_data_handler(service=name, handler=callback)
        # The desktop's skill controller closes its loop on this pose.
        await self.rvr.sensor_control.start(interval=50)
        LOG.info("RVR connected on %s", self.port)

    def input(self, keys):
        self.last_keys = "".join(k for k in keys.lower() if k in "wasd")
        self.velocity = None
        self.last_command = time.monotonic()

    def command(self, data):
        """Apply one control payload: keys, vel [m/s, deg/s] or reset_pose."""
        if data.get("reset_pose") is True:
            self.reset_requested = True
        elif isinstance(data.get("vel"), list) and len(data["vel"]) == 2:
            linear, yaw_rate = (float(value) for value in data["vel"])
            self.last_keys = ""
            self.velocity = (max(-1.0, min(1.0, linear)), max(-360.0, min(360.0, yaw_rate)))
            self.last_command = time.monotonic()
        else:
            self.input(data.get("keys", ""))

    def command_velocity(self):
        if self.velocity is None or time.monotonic() - self.last_command > 0.5:
            return None
        return self.velocity

    def pose(self):
        """Fresh locator pose (m, m, degrees CCW), or None."""
        now = time.monotonic()
        imu, locator = self.sensors.get("IMU"), self.sensors.get("Locator")
        if not imu or not locator or now - min(imu["time"], locator["time"]) > 0.3:
            return None
        return {"x": locator["X"], "y": locator["Y"], "yaw": imu["Yaw"]}

    def measured_velocity(self):
        velocity = self.sensors.get("Velocity")
        if not velocity or time.monotonic() - velocity["time"] > 0.3:
            return None
        return [velocity["X"], velocity["Y"]]

    def tracks(self):
        if time.monotonic() - self.last_command > 0.5 or self.velocity is not None:
            return (0, 0)
        keys = self.last_keys
        speed = self.speed
        forward = ("w" in keys) - ("s" in keys)
        turn = ("d" in keys) - ("a" in keys)
        if not forward:
            forward_duty = speed * self.pivot_forward_percent // 100
            if turn < 0:
                return (-speed, forward_duty)
            if turn > 0:
                return (forward_duty, -speed)
            return (0, 0)
        left = right = forward * speed
        if (forward > 0 and turn < 0) or (forward < 0 and turn > 0):
            left = int(left * 0.45)
        elif (forward > 0 and turn > 0) or (forward < 0 and turn < 0):
            right = int(right * 0.45)
        return (left, right)

    async def motor_loop(self):
        from sphero_sdk import RawMotorModesEnum
        def mode(value):
            return (RawMotorModesEnum.forward if value > 0 else
                    RawMotorModesEnum.reverse if value < 0 else RawMotorModesEnum.off).value
        while True:
            if self.connected and self.reset_requested:
                self.reset_requested = False
                try:
                    await self.rvr.reset_yaw()
                    await self.rvr.reset_locator_x_and_y()
                    LOG.info("Pose reset")
                except Exception as exc:
                    self.error = str(exc)
                    LOG.exception("Pose reset failed")
            velocity = self.command_velocity()
            target = ("vel", velocity) if velocity else ("tracks", self.tracks())
            moving = target != ("tracks", (0, 0))
            if self.connected and (target != self.last_sent or
                             (moving and time.monotonic() - self.last_motor_write >= 0.2)):
                try:
                    if velocity:
                        await self.rvr.drive_rc_si_units(
                            yaw_angular_velocity=velocity[1], linear_velocity=velocity[0], flags=0)
                    else:
                        tracks = target[1]
                        await self.rvr.raw_motors(mode(tracks[0]), abs(tracks[0]),
                                                  mode(tracks[1]), abs(tracks[1]))
                    self.error = None
                    self.last_sent = target
                    self.last_motor_write = time.monotonic()
                except Exception as exc:
                    self.error = str(exc)
                    LOG.exception("Motor command failed")
            await asyncio.sleep(0.05)

    async def battery_loop(self):
        while True:
            if self.connected:
                try:
                    self.battery = await self.rvr.get_battery_percentage(timeout=3)
                except Exception as exc:
                    self.error = str(exc)
            await asyncio.sleep(10)

    async def close(self):
        self.input("")
        if self.rvr and self.connected:
            from sphero_sdk import RawMotorModesEnum
            try:
                off = RawMotorModesEnum.off.value
                await self.rvr.raw_motors(off, 0, off, 0)
                await self.rvr.sensor_control.clear()
                await self.rvr.restore_default_control_system_timeout()
            finally:
                await self.rvr.close()
        elif self.rvr:
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
            rover.command(data)

    async def transmit():
        last_seq = -1
        last_telemetry = 0.0
        while True:
            seq, frame = camera.snapshot()
            if frame and seq != last_seq:
                writer.write(packet(b"V", frame))
                last_seq = seq
            now = time.monotonic()
            if now - last_telemetry >= 0.05:
                info = {"rvr_connected": rover.connected,
                        "keys": rover.last_keys if now - rover.last_command <= 0.5 else "",
                        "tracks": rover.tracks(), "speed_limit": rover.speed,
                        "pivot_forward_percent": rover.pivot_forward_percent,
                        "battery": rover.battery, "pose": rover.pose(),
                        "velocity": rover.measured_velocity(),
                        "vel": rover.command_velocity(),
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


async def main(args, rover):
    loop = asyncio.get_running_loop()
    main_task = asyncio.current_task()
    loop.add_signal_handler(signal.SIGTERM, main_task.cancel)
    camera = Camera(args.camera, args.width, args.height, args.fps)
    camera.start()
    motor = battery = None
    try:
        try:
            await rover.connect()
        except Exception as exc:
            rover.connected = False
            rover.error = str(exc)
            LOG.exception("RVR link check failed; camera service remains available")
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
        async with server:
            await server.serve_forever()
    finally:
        if motor:
            motor.cancel()
        if battery:
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
    parser.add_argument("--pivot-forward-percent", type=int,
                        default=int(os.environ.get("EMBODY_PIVOT_FORWARD_PERCENT", "60")))
    parser.add_argument("--serial", default=os.environ.get("EMBODY_RVR_SERIAL", ""))
    args = parser.parse_args()
    if not 1 <= args.speed <= 255:
        parser.error("--speed must be 1..255")
    if not 0 <= args.pivot_forward_percent <= 100:
        parser.error("--pivot-forward-percent must be 0..100")
    logging.basicConfig(level=logging.INFO, format="%(asctime)s %(levelname)s %(message)s")
    loop = asyncio.new_event_loop()
    asyncio.set_event_loop(loop)
    rover = Rover(args.serial, args.speed, args.pivot_forward_percent)
    try:
        rover.prepare(loop)
        logging.basicConfig(level=logging.INFO,
                            format="%(asctime)s %(levelname)s %(message)s", force=True)
        loop.run_until_complete(main(args, rover))
    except (KeyboardInterrupt, asyncio.CancelledError):
        pass
    finally:
        loop.run_until_complete(loop.shutdown_asyncgens())
        loop.close()
