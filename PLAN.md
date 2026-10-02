# Embody next steps

## Ready now

- Pi 5 camera service, official Sphero SDK, GPIO UART, sensor/motor bridge, and boot service are installed.
- Rust desktop app builds, opens a video window, receives live 30 fps MJPEG and telemetry, and sends terminal WASD controls through an SSH tunnel. The terminal dashboard updates in place. `--json` provides bounded commands, telemetry, and JPEG snapshots for programs.
- SSH key login works as `ssh pi5` on the desktop. The Pi currently has LAN address `192.168.1.103`.

## Waiting for physical RVR connection

The RVR is not wired or powered yet, so its SDK wake, sensor readings, motor direction, and fail-stop behavior cannot be confirmed. Follow the wiring and enablement steps in [README.md](README.md). Verify the RVR firmware is current in the Sphero Edu app if SDK wake fails. Start with a short, low-speed test while the tracks are clear of the ground, then check stop on `Space`, desktop exit, SSH disconnect, and Pi service restart. Adjust speed and steering direction only after these observations.

## LLM and perception

The Rust desktop program now provides human teleoperation plus a newline-delimited JSON interface for bounded `keys`, `stop`, telemetry, and snapshots. Next, adapt the target LLM runtime to this interface. Keep the Pi server as the sole RVR command owner. Add a separate low-rate frame sampler (for example 2–5 fps) feeding YOLO; keep the full-rate video path for the human view. Implement the red-ball behavior in steps: detection, bearing/distance estimate, orbit controller, then supervised trial. Record camera and RVR data to tune the controller. Isaac Lab simulation can follow once the physical interface and geometry are measured.

## Notes

Sphero's Python SDK declares dependencies pinned around 2020. On Ubuntu 24.04/Python 3.12, its code was installed without those old pins, with current `aiohttp`, `pyserial-asyncio`, and system OpenCV. Import and camera streaming work; RVR runtime compatibility awaits the wired hardware test. The service is deliberately configured camera-only until then.
