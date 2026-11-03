# Embody: Sphero RVR from a desktop terminal

The Raspberry Pi 5 runs a Python bridge for the RVR and USB camera. The desktop runs a Rust program that opens a live video window and accepts movement keys from the terminal. JPEG video and JSON telemetry travel over a single TCP stream inside an SSH tunnel. Camera capture is 848×480 MJPEG at 30 fps; no YOLO processing is in the video path.

## Current installation

- Pi: Ubuntu 24.04, USB camera `/dev/video0`, official Sphero Python SDK in `/home/alex/embody/.venv`, service `embody-pi.service` enabled and running.
- Pi 5 GPIO UART enabled as `/dev/ttyAMA0`; `alex` is in `dialout` and `video`.
- Desktop: `ssh pi5` uses key authentication to the Pi's Tailscale address, `100.96.244.79`. Both desktop and Pi must be online in the same tailnet; they can use different Wi-Fi networks. The Pi Wi-Fi is configured for `NUS_STU`.
- The RVR is connected over the Pi 5 GPIO UART. The Pi service runs with `EMBODY_RVR_SERIAL=/dev/ttyAMA0`; the official Sphero LED example, a bounded forward pulse, battery query, and live sensor telemetry have all worked.

## Run the desktop app

From a terminal in your checkout (Linux or macOS):

```sh
cd desktop
cargo run --release
```

The program opens the camera window and a fixed terminal dashboard with active WASD keys, the commanded track speeds, and the fixed speed. The dashboard updates in place. Keep the **terminal** focused to control the rover while watching the video window. `W` and `S` drive forward and reverse. `A` and `D` pivot left and right while stationary, or steer while moving. `Space` stops, and `X` or `Esc` quits. The fixed motor setting starts at 70/255 and can be changed with the Pi agent's `--speed` option. During an A/D pivot, the forward track starts at 60% of that setting (42/255), while the reverse track uses 70/255. The Pi stops after 500 ms without a control packet, and the RVR firmware receives a 750 ms motor command timeout when connected.

The terminal uses key press/repeat events and release events where available. If the terminal does not report releases, a key expires 350 ms after its last press. Hold a key to use keyboard repeat. `Space` always clears the active keys.

The desktop connects through `ssh -N -L 18765:127.0.0.1:8765 pi5` automatically. The Pi service only listens on its loopback interface; there is no unauthenticated network control port.

For a new Mac, follow the setup and SSH instructions in [AGENTS.md](AGENTS.md).
The Pi already authorizes a Mac public key; use the matching key on that Mac.
Quit the other desktop client before connecting from a different computer.

## Start the Pi service

The installed `embody-pi.service` is enabled at boot. Power the RVR before the
agent starts. To start, inspect, or restart it:

```sh
ssh pi5
sudo systemctl start embody-pi.service
systemctl status embody-pi.service --no-pager -l
sudo journalctl -u embody-pi.service -n 50 --no-pager
# After restoring a failed rover connection:
sudo systemctl restart embody-pi.service
```

Restart the desktop app after restarting the Pi service. Video alone does not
prove a motor connection: check `Motor link ready` and battery/sensor telemetry.
The agent currently needs a restart after a failed startup rover check.

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

To tune A/D turns, set `EMBODY_PIVOT_FORWARD_PERCENT` in `~/embody/agent.env` to a value from 0 to 100 and restart the service. Lower values reduce the forward track's duty during pivots; `100` restores equal forward and reverse duties. This setting does not change W/S or W+A/W+D. The dashboard shows the actual commanded values for each track.

Sphero's [Pi setup guide](https://sdk.sphero.com/raspberry-pi-setup/python-sdk-setup-advanced) has the rover connector diagram. Raspberry Pi's [UART documentation](https://www.raspberrypi.com/documentation/computers/configuration.html#uarts-and-device-tree) explains the Pi 5 overlay and pins. The SDK's default `/dev/ttyS0` is wrong for this Pi 5; this bridge explicitly uses `/dev/ttyAMA0`. Disconnect the Sphero Edu app before using the desktop controls to avoid conflicting commands.

## Code and protocol

- `pi/agent.py`: camera, RVR SDK, sensor subscriptions, motor control, watchdog, and TCP server.
- `pi/embody-pi.service`: boot service definition installed at `/etc/systemd/system/embody-pi.service` on the Pi.
- `desktop/src/`: Rust video window, in-place dashboard, terminal controls, JSON mode, protocol, and SSH tunnel. `nav.rs` holds the pose skills.
- `brain/embody_brain.py`: Claude tool loop for typed tasks. `tools/sim_rover.py`: simulated rover for testing without hardware.

For AI or other programs, run `cargo run --release -- --json`. It reads newline-delimited JSON on stdin and writes JSON events on stdout. Send `{"keys":"wa","duration_ms":500}` for a bounded drive command (maximum 2000 ms), `{"vel":[0.2,-30],"duration_ms":500}` for a bounded velocity (m/s, degrees/s), `{"stop":true}`, `{"snapshot":"/absolute/path/frame.jpg"}`, or `{"quit":true}`. Telemetry arrives at most five times a second as `{"type":"telemetry","data":...}`. The JSON mode has no terminal dashboard or video window; snapshot requests save the latest full-rate camera frame. Any request may carry an `"id"`, which its reply echoes.

### Pose and skills

Both rover agents stream the RVR's own wheel and IMU odometry as `pose` (`x`, `y` in metres, `yaw` in degrees). `+y` is the heading at the last pose reset, `+x` is to its right, and yaw is counter-clockwise positive. Reset it with `R` in the dashboard or `{"reset_pose":true}`: the current spot and heading become `(0, 0, 0°)`. The RVR also restarts the frame when it powers up.

In JSON mode, a skill runs a closed loop on that pose at 20 Hz and reports once when it ends:

```json
{"skill":"drive","distance_m":3.0,"speed_mps":0.3,"id":1}
{"skill":"turn","degrees":-90}
{"skill":"turn_to","yaw_deg":0}
{"skill":"go_to","x":1.2,"y":3.0,"yaw_deg":90}
{"skill":"go_to","place":"desk"}
```

Each produces `{"type":"skill","state":"started",...}`, then one of `done`, `failed` (with a reason) or `cancelled` (another command replaced it). Speeds are capped at 0.6 m/s. A skill fails if the remaining distance or angle stops shrinking for 2.5 s (blocked or slipping), if the pose is older than 400 ms, or on a time limit. There is no obstacle sensing: test skills on raised tracks or in a clear area. `{"remember":"desk"}`, `{"forget":"desk"}` and `{"places":true}` manage named poses; with `--places FILE` they are kept between runs. Places stay valid only if the rover is reset at the same marked spot and heading each session. `{"status":true}` reports the pose and running skill.

Without hardware, `python3 tools/sim_rover.py` serves a simulated rover on `127.0.0.1:8765` for `--direct 127.0.0.1:8765`.

### Language tasks

`brain/embody_brain.py` gives Claude these skills as tools and runs the JSON mode for it. Claude is called once per step of a task, and each step is a whole move:

```sh
python3 -m venv .venv && .venv/bin/pip install -r brain/requirements.txt
(cd desktop && cargo build --release --locked)
.venv/bin/python brain/embody_brain.py --direct <esp-ip>   # or --target pi5
```

It needs Claude API credentials (`ANTHROPIC_API_KEY`, or a profile from `ant auth login`). Type a task such as "drive to the door, then come back home". `/stop`, `/reset`, `/status` and `/quit` act immediately, and Ctrl-C stops a moving rover. Places are saved in `places.json` at the repository root (ignored by git).

Packets have a 1-byte type, 4-byte big-endian payload length, then payload. Desktop sends `C` with JSON such as `{"keys":"wa"}`, `{"vel":[0.3,0]}` or `{"reset_pose":true}` at least every 50 ms while driving. Pi sends `V` with a complete JPEG frame and `T` with JSON telemetry about four times a second. One desktop client controls the rover at a time. The protocol is simple enough for a later LLM program to use while sampling frames for YOLO at a lower rate than the 30 fps display.

See [PLAN.md](PLAN.md) for the remaining robot and LLM work.
