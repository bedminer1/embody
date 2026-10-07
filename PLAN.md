# Embody next steps

## Ready now

- Pi 5 camera service, official Sphero SDK, GPIO UART, sensor/motor bridge, and boot service are installed.
- Rust desktop app builds, opens a video window, receives live 30 fps MJPEG and telemetry, and sends terminal WASD controls through an SSH tunnel. The terminal dashboard updates in place. `--json` provides bounded commands, telemetry, and JPEG snapshots for programs.
- SSH key login works as `ssh pi5` over Tailscale (`100.96.244.79`). Pi Wi-Fi is configured for `NUS_STU`. Root `AGENTS.md` provides the Mac setup handoff.

## Physical RVR validation

The RVR UART connection, SDK wake, battery query, live sensor readings, LED control, and a short forward motor pulse have been confirmed. A/D pivots drift forward on the current surface, so the forward track duty during pivots is now independently adjustable. Verify the new balance on a clear floor, then check stop on `Space`, desktop exit, SSH disconnect, and Pi service restart.

## LLM and perception

The Rust desktop program now provides human teleoperation plus a newline-delimited JSON interface for bounded `keys`, `stop`, telemetry, and snapshots. Next, adapt the target LLM runtime to this interface. Keep the Pi server as the sole RVR command owner. Add a separate low-rate frame sampler (for example 2–5 fps) feeding YOLO; keep the full-rate video path for the human view. Implement the red-ball behavior in steps: detection, bearing/distance estimate, orbit controller, then supervised trial. Record camera and RVR data to tune the controller. Isaac Lab simulation can follow once the physical interface and geometry are measured.

## Notes

Sphero's Python SDK declares dependencies pinned around 2020. On Ubuntu 24.04/Python 3.12, its code was installed without those old pins, with current `aiohttp`, `pyserial-asyncio`, and system OpenCV. Camera streaming and bidirectional RVR UART communication have been confirmed. On 2026-10-07, a no-drive probe returned 84% battery; the restarted service reported a ready motor link and live IMU/encoder data.

Follow-ups: retry the rover connection after a failed startup check; bound SDK shutdown so service restarts cannot hang; restore reliable NTP synchronization on the Pi. The Pi clock was manually corrected to restore Tailscale, but NTP still reported unsynchronized. Verify the Mac build and GUI on the actual Mac; Linux checks do not substitute for that.
