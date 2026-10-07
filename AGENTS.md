# Embody agent instructions

## Purpose and architecture

Embody controls a Sphero RVR through a Raspberry Pi 5. The Pi owns the
GPIO UART and runs `pi/agent.py`; the desktop runs the Rust app in `desktop/`.
The desktop opens the camera window, reads WASD from its local terminal, and
automatically tunnels TCP `18765` to Pi loopback `8765` over ordinary SSH.
Tailscale provides the network path; Tailscale SSH is not required.

Read `README.md` and `PLAN.md` before changing the system. Keep the Pi server
as the sole UART owner. Preserve the existing SDK constructor workaround:
`SpheroRvrAsync` must be constructed before its asyncio loop starts running.
Report motor readiness from successful rover replies, not merely an open port.

Never put passwords, private SSH keys, Wi-Fi credentials, or auth tokens in
tracked files or command output. Keep each desktop's private key on that desktop.
Do not run motor commands as an automatic setup check. Physical driving trials
require the user's instruction and a clear area or raised tracks.

## Known Pi installation (verified 2026-10-07)

- Tailnet hostname: `pi5`; Tailscale IPv4: `100.96.244.79`; Linux user: `alex`.
- Ubuntu 24.04, deployed project: `/home/alex/embody`.
- Python interpreter: `/home/alex/embody/.venv/bin/python`.
- SDK source: `/home/alex/embody/sphero-sdk-raspberrypi-python`.
- System unit: `/etc/systemd/system/embody-pi.service`, enabled at boot.
- Runtime settings: `/home/alex/embody/agent.env`, including
  `EMBODY_RVR_SERIAL=/dev/ttyAMA0` and `EMBODY_PIVOT_FORWARD_PERCENT=60`.
- Camera: `/dev/video0`; GPIO UART: `/dev/ttyAMA0`; user has `video` and `dialout`.
- Pi Wi-Fi is configured for `NUS_STU`; LAN and campus DHCP addresses can change.
- A no-drive UART probe returned battery 84% and live IMU/encoder data.
  Treat these as past observations; verify current telemetry before use.
- ED25519 SSH host fingerprint:
  `SHA256:VB6i83Lmuo/R390Y6oZ30ZjD7KMJfXioTkOLc8TZVO4`.

## Mac Codex handoff: reproduce the desktop setup

Carry out the following on the Mac. Inspect existing installations/configuration
first and reuse them. Do not install the Pi SDK or run the Pi Python agent on macOS.
The Rust desktop uses native macOS windowing through `minifb`.

1. Fetch the project from `git@github.com:bedminer1/embody.git`. For a fresh clone:

   ```sh
   git clone git@github.com:bedminer1/embody.git
   cd embody
   ```

   For an existing checkout, inspect local changes before updating `main` with
   `git pull --ff-only`. Do not discard Mac work or bypass GitHub authentication.

2. Verify `rustc --version`, `cargo --version`, and `xcode-select -p`.
   If missing, install Apple's Command Line Tools with `xcode-select --install`
   and Rust using the official rustup instructions at
   <https://doc.rust-lang.org/book/ch01-01-installation.html>.
   Complete the Apple installer before compiling. Use the committed Cargo lockfile.

3. Connect the Mac's Tailscale app to the same tailnet as `pi5` and ensure it is
   running. Different Wi-Fi networks are fine if both devices have internet access
   and the tailnet policy permits the connection. Check:

   ```sh
   tailscale status
   tailscale ping pi5
   ```

   If the CLI is not on PATH, use the app's bundled executable, when present:
   `/Applications/Tailscale.app/Contents/MacOS/Tailscale`.
   Follow <https://tailscale.com/docs/install/mac> and
   <https://tailscale.com/docs/reference/tailscale-cli?tab=macos> for the installed variant.

4. Configure ordinary key-based SSH. The Pi already authorizes a Mac public key
   with fingerprint `SHA256:rv0n8i3Ke/bSPfUqIWgkLnaGMAAI6d5XrA+VS09yTo0`,
   comment `alex@Alexanders-MacBook-Pro.local`. Locate the matching key on the Mac
   using `ssh-keygen -lf <public-key-file>`; use its corresponding private-key path.
   Do not copy the Linux desktop's private key. Merge this host entry into
   `~/.ssh/config`, changing `IdentityFile` to the matching Mac key if necessary:

   ```sshconfig
   Host pi5
     HostName 100.96.244.79
     User alex
     IdentityFile ~/.ssh/id_ed25519
     IdentitiesOnly yes
     HostKeyAlias pi5.local
     ServerAliveInterval 15
     ServerAliveCountMax 3
   ```

   Use `chmod 700 ~/.ssh` and `chmod 600 ~/.ssh/config` as needed. On initial
   connection, verify the Pi host fingerprint above before accepting the key.
   Then verify noninteractive login, which the desktop tunnel requires:

   ```sh
   ssh pi5 'hostname'
   ssh -o BatchMode=yes pi5 'hostname; systemctl is-active embody-pi.service'
   ```

   If the authorized Mac key is unavailable, generate a dedicated new key and
   install only its public key on the Pi through an already trusted connection.
   Ask for access details only if no authorized route exists. Never disable host
   key checking to work around a failure.

5. Quit the Linux desktop app before starting the Mac app: the Pi accepts only
   one control client at a time. Start in a normal interactive Mac terminal:

   ```sh
   cd desktop
   cargo check --locked
   cargo run --release --locked
   ```

   Keep this terminal focused for WASD. The camera window does not process driving
   keys. `Space` stops; `X` or `Esc` exits. Look for `Motor link ready`, live video,
   and changing key/track indicators. Do not drive as part of unattended setup.
   Report Mac build/runtime verification separately from Linux verification.

## Starting and maintaining the Pi side

The installed unit starts automatically on Pi boot. Do not launch a second agent.
Power the rover before starting/restarting the agent, then use:

```sh
ssh pi5
sudo systemctl start embody-pi.service
systemctl status embody-pi.service --no-pager -l
sudo journalctl -u embody-pi.service -n 50 --no-pager
```

If video works but the dashboard says camera-only or motor error, inspect
`~/embody/agent.env`, UART permissions, rover power, and battery/sensor replies.
The current agent checks the rover at startup and does not automatically retry a
failed initial connection. After restoring rover power/wiring, run
`sudo systemctl restart embody-pi.service`, then restart the desktop app.
Shutdown has occasionally hung until systemd's stop timeout; do not start a
second UART reader while the old process is still alive.

For an isolated UART check, stop `embody-pi.service` first, ensure no process owns
`/dev/ttyAMA0`, then run:

```sh
cd /home/alex/embody
.venv/bin/python probe_rvr.py
```

The default probe wakes the rover and queries battery without driving.
`probe_rvr.py --drive` moves it briefly; never add that flag to unattended checks.
Always restart the service after diagnostics. LED scripts also require exclusive
UART ownership. A battery reply plus live sensors proves bidirectional UART;
camera streaming alone does not.

Deploy Pi source changes only when needed. Copy `pi/agent.py` and the diagnostic
scripts to `/home/alex/embody/`, preserving `.venv`, the SDK checkout, and
`agent.env`. Restart the unit afterward. If changing the unit, install it into
`/etc/systemd/system/` and run `sudo systemctl daemon-reload` before restarting.

## Troubleshooting across machines

- Tailscale must be online on both hosts. If `pi5` is offline, establish a LAN,
  Ethernet, or physical-console path before altering its network settings.
- The Pi clock was manually corrected on 2026-10-07 after TLS validation failed
  because it was weeks behind. NTP was enabled but still unsynchronized at the
  last check. Inspect `date -Is`, `timedatectl`, and `tailscale status` if access
  fails again after reboot; do not assume every failure is an SSH password problem.
- Ghostty's `xterm-ghostty` entry is installed in Pi user's `~/.terminfo`.
  If it is missing on another host, install with
  `infocmp -x xterm-ghostty | ssh pi5 'mkdir -p ~/.terminfo; tic -x -o ~/.terminfo -'`
  from a machine with that terminfo available. Reconnect afterward. See
  <https://ghostty.org/docs/help/terminfo>.
- Check local port `18765` for an existing tunnel if a second desktop launch fails.
  Identify its owner before stopping anything; never kill unrelated SSH sessions.

## Checks for source changes

Run `cargo check --locked` in `desktop/` for Rust changes. Compile Python sources
with `python3 -m py_compile pi/*.py`. Keep checks free of motor commands unless
the user explicitly requests a supervised physical trial.
