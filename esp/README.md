# Embody on an ESP32-S3 camera board

Replacement for the Pi agent (`pi/agent.py`) that runs on an ESP32-S3 camera
board (ESP32-S3-WROOM-1 N16R8 with an OV2640, sold as "ESP32-S3 CAM"): it
joins Wi-Fi, listens on TCP `8765`, turns the desktop's WASD and velocity
frames into Sphero RVR UART packets, reports telemetry back, and streams
JPEG frames from the camera.

The wire format is the one the Pi agent already uses (type byte, big-endian
`u32` length, JSON payload), so the desktop's protocol code is unchanged. The
desktop connects straight to the ESP32 instead of through an SSH tunnel (an
ESP32 has no sshd):

```sh
cd desktop
cargo run --release --locked -- --direct <esp-ip>      # port defaults to 8765
```

## Layout

- `src/bin/main.rs`: Embassy firmware (Wi-Fi, TCP server, RVR UART tasks).
- `src/bin/camera.rs`: OV2640 setup over SCCB and JPEG capture through the
  S3's LCD_CAM peripheral.
- `proto/`: `no_std` crate with the RVR packet codec, the WASD-to-tracks
  mapping and the desktop wire format. No hardware dependencies, so it is
  tested on the host:

  ```sh
  cd esp/proto
  cargo test --target aarch64-apple-darwin   # use your host triple
  ```

## Build and flash

The ESP32-S3 is Xtensa, which stable Rust cannot target, so you need
Espressif's Rust toolchain (`rust-toolchain.toml` selects it) and `espflash`:

```sh
cargo install espup espflash --locked
espup install            # creates ~/export-esp.sh
source ~/export-esp.sh   # in every new shell before building
```

Most of these boards have two USB-C ports. Flash through the one wired to the
USB-serial chip (usually labelled COM or UART; it shows up as
`/dev/cu.usbserial-*` or `/dev/cu.wchusbserial*`), because the console log
goes to UART0. The other port (USB/OTG) is the S3's native USB. Use a data
cable (not charge-only). If flashing stalls at "Connecting...", hold BOOT,
tap RST, then release BOOT. The C6 version of this crate is in git history
(commit `1d3933f`); the classic ESP32 (DevKit V1) version was never
committed.

Wi-Fi credentials are read from the build environment and baked into the
binary. They are not stored in any tracked file:

```sh
cd esp
EMBODY_WIFI_SSID='...' EMBODY_WIFI_PASSWORD='...' cargo run --release
```

`cargo run` flashes and opens the serial monitor. The firmware logs
`Listening on <ip>:8765` once DHCP finishes, `OV2640 streaming 640x480 JPEG`
once the camera is set up, and `RVR connected, battery N%` once the rover
answers. All appear in the monitor.

The agent sends no drive command until the desktop sends keys. It sends
`wake` and a battery query at startup, which do not move the rover.

## Wiring

| ESP32-S3 | RVR UART port |
| --- | --- |
| GPIO21 (UART1 TX) | RX |
| GPIO14 (UART1 RX) | TX |
| GND | GND |
| 5V / 5Vin | 5V |

Check the pin order of the RVR's 4-pin UART port against Sphero's hardware
documentation and your unit before connecting anything. This README was not
checked against the physical port. Any free output-capable GPIO works, because the ESP32 routes UART signals
through its GPIO matrix and no particular timer is involved. To change the
pins, edit `with_rx` and `with_tx` in `main` (pins are chosen at compile
time). Avoid the camera pins (next section), the strapping pins
(GPIO0/3/45/46), native USB (GPIO19/20), the console UART0 (GPIO43/44), octal
PSRAM (GPIO35-37) and the RGB LED (GPIO48 on most boards).

## Camera

The firmware assumes the pin map of the common ESP32-S3-WROOM camera boards
(Freenove and its clones, `CAMERA_MODEL_ESP32S3_EYE` in the Arduino
examples). It was not checked against your board; compare it with the
board's schematic or seller pinout before powering up:

| Signal | GPIO | Signal | GPIO |
| --- | --- | --- | --- |
| XCLK | 15 | D0 (Y2) | 11 |
| SIOD (SDA) | 4 | D1 (Y3) | 9 |
| SIOC (SCL) | 5 | D2 (Y4) | 8 |
| VSYNC | 6 | D3 (Y5) | 10 |
| HREF | 7 | D4 (Y6) | 12 |
| PCLK | 13 | D5 (Y7) | 18 |
| PWDN, RESET | not connected | D6 (Y8) | 17 |
| | | D7 (Y9) | 16 |

If yours differs, change the `with_*` calls in `main`. The sensor is set up
with the register tables of Espressif's `esp32-camera` driver for a 640x480
JPEG (`WIDTH`, `HEIGHT` and `QUALITY` in `src/bin/camera.rs`; other 4:3 sizes
up to 1600x1200 also work). Frames are capped at about 12 fps and 48 KB so
video does not starve telemetry; larger frames are dropped with a log line.
Only the OV2640 is supported. If the camera is missing or fails to answer,
driving still works and telemetry's `error` says why.

None of this has run on hardware yet. The first things to check are the
`OV2640 streaming` log line and that the desktop window shows video. A
`Camera DMA overran` log line means the capture loop fell behind; occasional
ones only drop a frame.

### Power: no buck converter, and never 5 V into a GPIO

- **Do not connect 5 V to a GPIO pin.** ESP32 GPIOs are 3.3 V only and are
  not 5 V tolerant. It can destroy the pin or the chip.
- Connect the RVR's 5 V to the dev board's **5V/VIN pin**. Dev boards (DevKitC
  and similar) have an onboard 3.3 V regulator behind that pin, so no extra
  buck converter is needed. A bare ESP32 module without such a board would
  need its own 3.3 V regulator.
- The UART signals are 3.3 V logic on both sides, so TX/RX connect directly
  with no level shifter.
- Wi-Fi transmit bursts draw a few hundred mA. Put a 100-470 µF capacitor
  across 5V/GND close to the board to absorb them and the motor noise.
- Do not power the board from USB and from the RVR at the same time unless
  your board has a diode on VIN. Unplug the RVR 5 V while flashing.

## Latency

Estimates, not measurements. A keypress is sent as soon as the key state
changes (the desktop loop runs at about 60 Hz), with a 50 ms heartbeat while
keys are held. The rover task wakes on every change, so the ESP32 adds little:
a motor command is about 12 bytes, roughly 1 ms at 115200 baud. Expect
roughly 20-40 ms from keypress to the rover receiving the command on a quiet
2.4 GHz network, dominated by Wi-Fi. Wi-Fi power saving is disabled for this;
a busy channel causes occasional spikes. Video shares the link: a 20-30 KB
frame takes tens of milliseconds to send, and telemetry waits behind it.
Measure on your network before relying on a figure.

## Driving keys

W/S drive forward/reverse and A/D turn. Any combination can be held at once:
W+A and W+D curve, S+A and S+D curve in reverse, A or D alone pivot in place,
and opposing keys cancel (W+S, A+D). There is no strafing, so no Q/E keys.

Holding several keys needs a terminal that reports key releases (the kitty
keyboard protocol): Ghostty, kitty, WezTerm, iTerm2. In other terminals only
the most recently pressed key keeps auto-repeating, so the others time out
after 350 ms; the dashboard shows a warning in that case. Losing terminal
focus releases all keys.

## Behaviour

- Keys older than 500 ms count as released, and the connection dropping
  stops the motors, as on the Pi.
- `rvr_connected` is true only after the rover has answered a battery query.
  If the rover is off at boot, the ESP32 retries every 5 s (the Pi agent does
  not retry).
- Only one desktop can be connected at a time.
- Each new camera frame goes out as a `V` packet (one JPEG) after the next
  telemetry packet, so the desktop's video window, `snapshot` and the brain's
  `look` tool work as they do with the Pi.
- After connecting, the firmware subscribes to the RVR's IMU, locator and
  velocity streams every 50 ms and reports `pose` (x, y in metres, yaw in
  degrees) in telemetry, also every 50 ms. A pose older than 300 ms is sent
  as `null`. If the RVR rejects the stream setup, driving still works and
  telemetry's `error` says so.
- `{"vel":[m/s, deg/s]}` drives with the RVR's closed-loop
  `drive_rc_si_units`, clamped to 1 m/s and 360°/s, and expires after 500 ms
  like keys. `{"reset_pose":true}` sends `reset_yaw` and
  `reset_locator_x_and_y`. Neither has been tried on the physical rover yet.

## Networking caveat

The Pi was reached over Tailscale. An ESP32-S3 cannot run Tailscale, so the
laptop and the ESP32 must be on the same LAN with device-to-device traffic
allowed. Campus or enterprise Wi-Fi often isolates clients, and WPA2-Enterprise
networks are not configured by this firmware. A phone hotspot or a home
router works.

The TCP port has no authentication: anyone on the same network can drive the
rover. Use a network you trust.
