# Embody: Sphero RVR from a desktop terminal

The Raspberry Pi 5 runs a Python bridge for the RVR and USB camera. The desktop runs a Rust program that opens a live video window and accepts movement keys from the terminal. JPEG video and JSON telemetry travel over a single TCP stream inside an SSH tunnel. Camera capture is 848×480 MJPEG at 30 fps; no YOLO processing is in the video path.

## Current installation

- Pi: Ubuntu 24.04, USB camera `/dev/video0`, official Sphero Python SDK in `/home/alex/embody/.venv`, service `embody-pi.service` enabled and running.
- Pi 5 GPIO UART enabled as `/dev/ttyAMA0`; `alex` is in `dialout` and `video`.
- Desktop: `ssh pi5` uses the existing SSH key and the Pi's current LAN address. The address is set to `192.168.1.103` in `~/.ssh/config`; update it if DHCP changes the address.
- The RVR is **not connected yet**. The Pi service intentionally runs with `EMBODY_RVR_SERIAL=` (camera-only mode). No motor or RVR sensor test has been performed.

## Run the desktop app

From a terminal on this desktop:

```sh
cd /home/alex/Desktop/robotics/embody/desktop
cargo run --release
```

The program opens the camera window and a fixed terminal dashboard with active WASD keys, each track command, and the fixed speed. The dashboard updates in place. Keep the **terminal** focused to control the rover while watching the video window. `W` and `S` drive forward and reverse. `A` and `D` pivot left and right while stationary, or steer while moving. `Space` stops, and `X` or `Esc` quits. The fixed motor setting starts at 70/255 and can be changed with the Pi agent's `--speed` option. The Pi stops after 500 ms without a control packet, and the RVR firmware receives a 750 ms motor command timeout when connected.

The terminal uses key press/repeat events and release events where available. If the terminal does not report releases, a key expires 350 ms after its last press. Hold a key to use keyboard repeat. `Space` always clears the active keys.

The desktop connects through `ssh -N -L 18765:127.0.0.1:8765 pi5` automatically. The Pi service only listens on its loopback interface; there is no unauthenticated network control port.

## Connect the RVR

1. Power off the Pi and RVR before wiring. Supply the Pi 5 from its **own suitable power source**; Sphero does not officially support Pi 5 power from the rover.
2. Connect Pi GPIO **pin 8 / GPIO14 TX** to **RVR RX**, Pi **pin 10 / GPIO15 RX** to **RVR TX**, and Pi **ground** (for example pin 6) to **RVR ground**. Do not connect a 5 V power wire. TX and RX must cross.
3. Power on the Pi and RVR. Check `ssh pi5 'ls -l /dev/ttyAMA0'`.
4. On the Pi, enable the rover link and restart the service:

```sh
ssh pi5
printf 'EMBODY_RVR_SERIAL=/dev/ttyAMA0\n' > ~/embody/agent.env
sudo systemctl restart embody-pi.service
systemctl status embody-pi.service
```

5. Run the desktop app. Check that the dashboard says `Motor link ready`; sensor and battery values are available through `--json`. Start with tracks clear of the ground and use a brief `W` press, then `Space`, before driving on a clear floor.

If the RVR is not connected, restore camera-only mode with `printf 'EMBODY_RVR_SERIAL=\n' > ~/embody/agent.env` followed by `sudo systemctl restart embody-pi.service`. Check service logs with `sudo journalctl -u embody-pi.service -n 50 --no-pager`.

Sphero's [Pi setup guide](https://sdk.sphero.com/raspberry-pi-setup/python-sdk-setup-advanced) has the rover connector diagram. Raspberry Pi's [UART documentation](https://www.raspberrypi.com/documentation/computers/configuration.html#uarts-and-device-tree) explains the Pi 5 overlay and pins. The SDK's default `/dev/ttyS0` is wrong for this Pi 5; this bridge explicitly uses `/dev/ttyAMA0`.

## Code and protocol

- `pi/agent.py`: camera, RVR SDK, sensor subscriptions, motor control, watchdog, and TCP server.
- `pi/embody-pi.service`: boot service definition installed at `/etc/systemd/system/embody-pi.service` on the Pi.
- `desktop/src/`: Rust video window, in-place dashboard, terminal controls, JSON mode, protocol, and SSH tunnel.

For AI or other programs, run `cargo run --release -- --json`. It reads newline-delimited JSON on stdin and writes JSON events on stdout. Send `{"keys":"wa","duration_ms":500}` for a bounded drive command (maximum 2000 ms), `{"stop":true}`, `{"snapshot":"/absolute/path/frame.jpg"}`, or `{"quit":true}`. Telemetry arrives as `{"type":"telemetry","data":...}`. The JSON mode has no terminal dashboard or video window; snapshot requests save the latest full-rate camera frame.

Packets have a 1-byte type, 4-byte big-endian payload length, then payload. Desktop sends `C` with JSON such as `{"keys":"wa"}` every 100 ms. Pi sends `V` with a complete JPEG frame and `T` with JSON telemetry about four times a second. One desktop client controls the rover at a time. The protocol is simple enough for a later LLM program to use while sampling frames for YOLO at a lower rate than the 30 fps display.

See [PLAN.md](PLAN.md) for the remaining robot and LLM work.
