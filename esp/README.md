# Embody on an ESP32-C6

Replacement for the Pi agent (`pi/agent.py`) that runs on an ESP32-C6 instead:
it joins Wi-Fi, listens on TCP `8765`, turns the desktop's WASD frames into
Sphero RVR UART packets, and reports telemetry back. There is no camera here;
a separate wireless camera is planned.

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
- `proto/`: `no_std` crate with the RVR packet codec, the WASD-to-tracks
  mapping and the desktop wire format. No hardware dependencies, so it is
  tested on the host:

  ```sh
  cd esp/proto
  cargo test --target aarch64-apple-darwin   # use your host triple
  ```

## Build and flash

The ESP32-C6 is RISC-V, so plain stable Rust is enough: `rust-toolchain.toml`
selects it and the `riscv32imac-unknown-none-elf` target. You also need
`espflash`. To use a different chip, regenerate the template with
`esp-generate --chip <chip>` and move `src/bin/main.rs` and `proto/` across.

Wi-Fi credentials are read from the build environment and baked into the
binary. They are not stored in any tracked file:

```sh
cd esp
EMBODY_WIFI_SSID='...' EMBODY_WIFI_PASSWORD='...' cargo run --release
```

`cargo run` flashes and opens the serial monitor. The firmware logs
`Listening on <ip>:8765` once DHCP finishes, and `RVR connected, battery N%`
once the rover answers. Both appear in the monitor.

The agent sends no drive command until the desktop sends keys. It sends
`wake` and a battery query at startup, which do not move the rover.

## Wiring

| ESP32 | RVR UART port |
| --- | --- |
| GPIO18 (UART1 TX) | RX |
| GPIO19 (UART1 RX) | TX |
| GND | GND |
| VIN / 5V | 5V |

Check the pin order of the RVR's 4-pin UART port against Sphero's hardware
documentation and your unit before connecting anything. This README was not
checked against the physical port. Any free GPIO works, because the C6 routes UART signals through a pin matrix
and no particular timer is involved. To change the pins, edit `with_rx` and
`with_tx` in `main` (pins are chosen at compile time). Avoid the strapping pins
(GPIO4/5/8/9/15), USB (GPIO12/13), the flash pins and the console UART0 pins
(GPIO16/17), and check your board's pinout.

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
a busy channel causes occasional spikes. Measure on your network before
relying on a figure.

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

The Pi was reached over Tailscale. An ESP32 cannot run Tailscale, so the
laptop and the ESP32 must be on the same LAN with device-to-device traffic
allowed. Campus or enterprise Wi-Fi often isolates clients, and WPA2-Enterprise
networks are not configured by this firmware. A phone hotspot or a home
router works.

The TCP port has no authentication: anyone on the same network can drive the
rover. Use a network you trust.
