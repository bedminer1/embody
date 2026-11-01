//! Embody rover agent for an ESP32: replaces `pi/agent.py` (minus the camera).
//!
//! The desktop app connects over TCP (port 8765) and sends WASD key frames
//! or closed-loop velocity commands; this firmware turns them into Sphero RVR
//! UART packets and reports telemetry, including the RVR's own locator pose,
//! back. Wi-Fi credentials come from the build environment
//! (`EMBODY_WIFI_SSID`, `EMBODY_WIFI_PASSWORD`), never from tracked files.
#![no_std]
#![no_main]
#![deny(
    clippy::mem_forget,
    reason = "mem::forget is generally not safe to do with esp_hal types, especially those \
    holding buffers for the duration of a data transfer."
)]
#![allow(
    clippy::large_stack_frames,
    reason = "embassy task futures (socket buffers etc.) live in static task storage, not on the stack"
)]

use core::cell::RefCell;

use embassy_executor::Spawner;
use embassy_futures::select::select;
use embassy_net::tcp::{TcpReader, TcpSocket, TcpWriter};
use embassy_net::{Runner, Stack, StackResources};
use embassy_sync::blocking_mutex::Mutex;
use embassy_sync::blocking_mutex::raw::CriticalSectionRawMutex;
use embassy_sync::channel::Channel;
use embassy_sync::signal::Signal;
use embassy_time::{Duration, Instant, Ticker, Timer, with_timeout};
use embody_proto::drive;
use embody_proto::sphero::{self, Decoder, Frame, Packet};
use embody_proto::stream::{self, Sample};
use embody_proto::wire::{self, Command, Pose, Telemetry};
use esp_backtrace as _;
use esp_hal::Async;
use esp_hal::clock::CpuClock;
use esp_hal::rng::Rng;
use esp_hal::timer::timg::TimerGroup;
use esp_hal::uart::{Config as UartConfig, Uart, UartRx, UartTx};
use esp_radio::wifi::{
    ClientConfig, ModeConfig, PowerSaveMode, WifiController, WifiDevice, WifiEvent,
};
use log::{info, warn};
use static_cell::StaticCell;

extern crate alloc;

// This creates a default app-descriptor required by the esp-idf bootloader.
// For more information see: <https://docs.espressif.com/projects/esp-idf/en/stable/esp32/api-reference/system/app_image_format.html#application-description>
esp_bootloader_esp_idf::esp_app_desc!();

// esp-radio 0.17 defines these two NVS hooks only for Xtensa chips, but the
// Wi-Fi library also references them once the station is started on the
// ESP32-C6, which breaks linking. These are the same no-op bodies it uses on
// Xtensa (no persistent Wi-Fi storage). If a future esp-radio defines them for
// RISC-V too, the linker reports a duplicate symbol; delete these then.
#[unsafe(no_mangle)]
extern "C" fn __esp_radio_misc_nvs_init() -> i32 {
    0
}

#[unsafe(no_mangle)]
extern "C" fn __esp_radio_misc_nvs_deinit() {}

// ---- configuration -------------------------------------------------------

/// Same port the Pi agent serves, so the desktop only needs a new host.
const TCP_PORT: u16 = 8765;
/// Full-speed duty (matches `--speed` default in `pi/agent.py`).
const SPEED: u8 = 70;
/// Inner-track forward duty while pivoting, percent (matches agent.env).
const PIVOT_FORWARD_PERCENT: u8 = 60;
/// Keys older than this count as released (matches the Pi agent).
const COMMAND_TIMEOUT: Duration = Duration::from_millis(500);
/// The RVR stops by itself if no drive command arrives for this long.
const RVR_CONTROL_TIMEOUT_MS: u16 = 750;
/// Re-send a held, unchanged motor command this often.
const MOTOR_REFRESH: Duration = Duration::from_millis(200);
/// Longest the rover task sleeps without a key change (stale-key detection).
const LOOP_POLL: Duration = Duration::from_millis(20);
/// Telemetry carries the pose the desktop's controller closes its loop on.
const TELEMETRY_PERIOD: Duration = Duration::from_millis(50);
/// RVR sensor streaming period (the SDK's minimum is 33 ms).
const STREAM_PERIOD_MS: u16 = 50;
/// A pose older than this is reported as unavailable.
const POSE_STALE: Duration = Duration::from_millis(300);
/// Velocity command limits; the RVR saturates near 2 m/s and 624 deg/s.
const MAX_LINEAR: f32 = 1.0;
const MAX_YAW_RATE: f32 = 360.0;
const REPLY_TIMEOUT: Duration = Duration::from_millis(1500);

const WIFI_SSID: &str = match option_env!("EMBODY_WIFI_SSID") {
    Some(value) => value,
    None => "",
};
const WIFI_PASSWORD: &str = match option_env!("EMBODY_WIFI_PASSWORD") {
    Some(value) => value,
    None => "",
};

// RVR UART: UART1 on GPIO18 (TX) and GPIO19 (RX). The pins are chosen in
// `main` (`with_tx` / `with_rx`); any free GPIO works because the ESP32-C6
// routes UART signals through its pin matrix. Avoid the strapping pins
// (GPIO4/5/8/9/15), USB (GPIO12/13), the flash pins and the console UART0
// pins (GPIO16/17). Cross the wires: ESP TX -> RVR RX, ESP RX <- RVR TX.

// ---- state shared between tasks ------------------------------------------

/// What the desktop last asked for.
#[derive(Clone, Copy, PartialEq)]
enum Motion {
    Keys(u8),
    /// m/s forward, degrees/s counter-clockwise.
    Velocity(f32, f32),
}

/// What the rover task should send to the RVR right now.
#[derive(Clone, Copy, PartialEq)]
enum Drive {
    Tracks(i16, i16),
    Velocity(f32, f32),
}

struct Shared {
    motion: Motion,
    last_command: Option<Instant>,
    /// True only after the RVR has answered a battery query.
    rvr_connected: bool,
    battery: Option<u8>,
    /// Why the RVR link is down (cleared once it connects).
    link_error: Option<&'static str>,
    /// Most recent error the RVR reported for a command.
    fault: Option<(&'static str, Instant)>,
    /// Why pose streaming is unavailable, if it is.
    pose_error: Option<&'static str>,
    /// Latest yaw (degrees) and locator sample, with arrival time.
    yaw: Option<(f32, Instant)>,
    position: Option<(Locator, Instant)>,
}

/// Locator position (m) and velocity (m/s).
#[derive(Clone, Copy)]
struct Locator {
    position: (f32, f32),
    velocity: (f32, f32),
}

impl Shared {
    const fn new() -> Self {
        Self {
            motion: Motion::Keys(0),
            last_command: None,
            rvr_connected: false,
            battery: None,
            link_error: None,
            fault: None,
            pose_error: None,
            yaw: None,
            position: None,
        }
    }

    /// The current command, or a stop once it has gone stale.
    fn drive(&self) -> Drive {
        match (self.motion, self.last_command) {
            (_, Some(at)) if at.elapsed() > COMMAND_TIMEOUT => Drive::Tracks(0, 0),
            (_, None) => Drive::Tracks(0, 0),
            (Motion::Keys(keys), _) => {
                let (left, right) =
                    drive::tracks(keys, SPEED.into(), PIVOT_FORWARD_PERCENT.into());
                Drive::Tracks(left, right)
            }
            (Motion::Velocity(linear, yaw_rate), _) => Drive::Velocity(linear, yaw_rate),
        }
    }

    fn tracks(&self) -> (i16, i16) {
        match self.drive() {
            Drive::Tracks(left, right) => (left, right),
            Drive::Velocity(..) => (0, 0),
        }
    }

    fn command_velocity(&self) -> Option<(f32, f32)> {
        match self.drive() {
            Drive::Velocity(linear, yaw_rate) => Some((linear, yaw_rate)),
            Drive::Tracks(..) => None,
        }
    }

    fn locator(&self) -> Option<Locator> {
        self.position
            .filter(|(_, at)| at.elapsed() <= POSE_STALE)
            .map(|(locator, _)| locator)
    }

    fn pose(&self) -> Option<Pose> {
        let (x, y) = self.locator()?.position;
        let (yaw, at) = self.yaw?;
        (at.elapsed() <= POSE_STALE).then_some(Pose { x, y, yaw })
    }

    fn error(&self) -> Option<&'static str> {
        self.link_error
            .or(match self.fault {
                Some((message, at)) if at.elapsed() < Duration::from_secs(3) => Some(message),
                _ => None,
            })
            .or(self.pose_error)
    }
}

static SHARED: Mutex<CriticalSectionRawMutex, RefCell<Shared>> =
    Mutex::new(RefCell::new(Shared::new()));
/// RVR response packets, from the UART reader to whoever awaits a reply.
static REPLIES: Channel<CriticalSectionRawMutex, Packet, 4> = Channel::new();
/// Raised whenever the desired motion may have changed, so the rover task
/// reacts immediately instead of waiting for its next poll.
static MOTION_CHANGED: Signal<CriticalSectionRawMutex, ()> = Signal::new();
/// Raised when the desktop asks to zero the locator pose.
static RESET_POSE: Signal<CriticalSectionRawMutex, ()> = Signal::new();

fn with_shared<R>(f: impl FnOnce(&mut Shared) -> R) -> R {
    SHARED.lock(|shared| f(&mut shared.borrow_mut()))
}

fn set_motion(motion: Motion) {
    with_shared(|s| {
        s.motion = motion;
        s.last_command = Some(Instant::now());
    });
    MOTION_CHANGED.signal(());
}

/// Drop any command (link lost or desktop gone) and wake the rover task.
fn clear_keys() {
    with_shared(|s| s.motion = Motion::Keys(0));
    MOTION_CHANGED.signal(());
}

// ---- entry point ---------------------------------------------------------

#[allow(
    clippy::large_stack_frames,
    reason = "it's not unusual to allocate larger buffers etc. in main"
)]
#[esp_rtos::main]
async fn main(spawner: Spawner) -> ! {
    esp_println::logger::init_logger_from_env();

    let config = esp_hal::Config::default().with_cpu_clock(CpuClock::max());
    let peripherals = esp_hal::init(config);

    esp_alloc::heap_allocator!(#[esp_hal::ram(reclaimed)] size: 65536);

    let timg0 = TimerGroup::new(peripherals.TIMG0);
    let sw_interrupt =
        esp_hal::interrupt::software::SoftwareInterruptControl::new(peripherals.SW_INTERRUPT);
    esp_rtos::start(timg0.timer0, sw_interrupt.software_interrupt0);

    if WIFI_SSID.is_empty() {
        panic!("EMBODY_WIFI_SSID was not set when this firmware was built");
    }

    // RVR link. Task order matters little, but the reader must run before the
    // rover task starts waiting for replies.
    let (rx, tx) = Uart::new(
        peripherals.UART1,
        UartConfig::default().with_baudrate(115_200),
    )
    .expect("invalid UART config")
    .with_rx(peripherals.GPIO19)
    .with_tx(peripherals.GPIO18)
    .into_async()
    .split();
    spawner
        .spawn(uart_reader(rx))
        .expect("failed to spawn task");
    spawner.spawn(rover(tx)).expect("failed to spawn task");

    // Wi-Fi + network stack.
    static RADIO: StaticCell<esp_radio::Controller<'static>> = StaticCell::new();
    let radio = RADIO.init(esp_radio::init().expect("Failed to initialize Wi-Fi controller"));
    let (controller, interfaces) =
        esp_radio::wifi::new(radio, peripherals.WIFI, Default::default())
            .expect("Failed to initialize Wi-Fi controller");

    let rng = Rng::new();
    let seed = (u64::from(rng.random()) << 32) | u64::from(rng.random());
    static RESOURCES: StaticCell<StackResources<3>> = StaticCell::new();
    let (stack, runner) = embassy_net::new(
        interfaces.sta,
        embassy_net::Config::dhcpv4(Default::default()),
        RESOURCES.init(StackResources::new()),
        seed,
    );
    spawner
        .spawn(wifi_connection(controller))
        .expect("failed to spawn task");
    spawner
        .spawn(net_runner(runner))
        .expect("failed to spawn task");
    spawner
        .spawn(tcp_server(stack))
        .expect("failed to spawn task");

    info!("Embody ESP32 agent started");
    loop {
        Timer::after(Duration::from_secs(3600)).await;
    }
}

// ---- Wi-Fi ---------------------------------------------------------------

#[embassy_executor::task]
async fn wifi_connection(mut controller: WifiController<'static>) {
    loop {
        if !matches!(controller.is_started(), Ok(true)) {
            let client = ClientConfig::default()
                .with_ssid(WIFI_SSID.into())
                .with_password(WIFI_PASSWORD.into());
            controller
                .set_config(&ModeConfig::Client(client))
                .expect("invalid Wi-Fi config");
            controller.start_async().await.expect("Wi-Fi start failed");
            // Latency matters more than power for a driving rover.
            if controller.set_power_saving(PowerSaveMode::None).is_err() {
                warn!("Could not disable Wi-Fi power saving");
            }
        }
        info!("Connecting to Wi-Fi...");
        match controller.connect_async().await {
            Ok(()) => {
                info!("Wi-Fi connected");
                controller.wait_for_event(WifiEvent::StaDisconnected).await;
                warn!("Wi-Fi disconnected");
                // Never keep driving without a link.
                clear_keys();
                Timer::after(Duration::from_secs(1)).await;
            }
            Err(error) => {
                warn!("Wi-Fi connect failed: {error:?}");
                Timer::after(Duration::from_secs(5)).await;
            }
        }
    }
}

#[embassy_executor::task]
async fn net_runner(mut runner: Runner<'static, WifiDevice<'static>>) {
    runner.run().await
}

// ---- desktop connection --------------------------------------------------

#[embassy_executor::task]
async fn tcp_server(stack: Stack<'static>) {
    let mut rx_buffer = [0u8; 1024];
    let mut tx_buffer = [0u8; 1024];
    loop {
        stack.wait_config_up().await;
        if let Some(config) = stack.config_v4() {
            info!("Listening on {}:{TCP_PORT}", config.address.address());
        }
        let mut socket = TcpSocket::new(stack, &mut rx_buffer, &mut tx_buffer);
        socket.set_timeout(Some(Duration::from_secs(10)));
        socket.set_keep_alive(Some(Duration::from_secs(2)));
        if let Err(error) = socket.accept(TCP_PORT).await {
            warn!("Accept failed: {error:?}");
            continue;
        }
        info!("Desktop connected: {:?}", socket.remote_endpoint());
        let (mut reader, mut writer) = socket.split();
        // Whichever side fails first ends the session.
        select(read_commands(&mut reader), send_telemetry(&mut writer)).await;
        // Stopping is the fail-safe: drop the keys even if the link died.
        clear_keys();
        socket.abort();
        info!("Desktop disconnected; motors stopping");
    }
}

async fn read_exact(reader: &mut TcpReader<'_>, mut buf: &mut [u8]) -> Result<(), ()> {
    while !buf.is_empty() {
        match reader.read(buf).await {
            Ok(0) | Err(_) => return Err(()),
            Ok(n) => buf = &mut buf[n..],
        }
    }
    Ok(())
}

async fn read_commands(reader: &mut TcpReader<'_>) {
    let mut payload = [0u8; wire::MAX_CONTROL];
    loop {
        let mut header = [0u8; wire::HEADER_LEN];
        if read_exact(reader, &mut header).await.is_err() {
            return;
        }
        let header = wire::parse_header(header);
        if header.kind != wire::CONTROL || header.len > wire::MAX_CONTROL {
            warn!("Invalid control frame; dropping connection");
            return;
        }
        if read_exact(reader, &mut payload[..header.len])
            .await
            .is_err()
        {
            return;
        }
        match wire::parse_command(&payload[..header.len]) {
            Some(Command::Keys(keys)) => set_motion(Motion::Keys(keys)),
            Some(Command::Velocity { linear, yaw_rate }) => set_motion(Motion::Velocity(
                linear.clamp(-MAX_LINEAR, MAX_LINEAR),
                yaw_rate.clamp(-MAX_YAW_RATE, MAX_YAW_RATE),
            )),
            Some(Command::ResetPose) => {
                RESET_POSE.signal(());
                MOTION_CHANGED.signal(());
            }
            None => {}
        }
    }
}

async fn send_telemetry(writer: &mut TcpWriter<'_>) {
    let mut ticker = Ticker::every(TELEMETRY_PERIOD);
    let mut frame = [0u8; 512];
    loop {
        let len = with_shared(|s| {
            wire::encode_telemetry(
                &mut frame,
                &Telemetry {
                    rvr_connected: s.rvr_connected,
                    tracks: s.tracks(),
                    speed_limit: SPEED,
                    pivot_forward_percent: PIVOT_FORWARD_PERCENT,
                    battery_percentage: s.battery,
                    pose: s.pose(),
                    velocity: s.locator().map(|locator| locator.velocity),
                    command_velocity: s.command_velocity(),
                    error: s.error(),
                },
            )
        });
        let Some(len) = len else { return };
        let mut sent = 0;
        while sent < len {
            let chunk = &frame[sent..len];
            let written = writer
                .write_with(|buf| {
                    let n = buf.len().min(chunk.len());
                    buf[..n].copy_from_slice(&chunk[..n]);
                    (n, n)
                })
                .await;
            match written {
                Ok(0) | Err(_) => return,
                Ok(n) => sent += n,
            }
        }
        ticker.next().await;
    }
}

// ---- RVR UART ------------------------------------------------------------

#[embassy_executor::task]
async fn uart_reader(mut rx: UartRx<'static, Async>) {
    let mut decoder = Decoder::new();
    let mut buf = [0u8; 64];
    loop {
        match rx.read_async(&mut buf).await {
            Ok(n) => {
                for &byte in &buf[..n] {
                    if let Some(packet) = decoder.push(byte) {
                        if packet.is_response() && packet.error != 0 {
                            warn!(
                                "RVR error 0x{:02x} for did 0x{:02x} cid 0x{:02x}",
                                packet.error, packet.did, packet.cid
                            );
                            with_shared(|s| {
                                s.fault = Some(("RVR rejected a command", Instant::now()))
                            });
                        }
                        if packet.is_response()
                            && packet.error == 0
                            && packet.did == sphero::DID_POWER
                            && packet.cid == sphero::CID_BATTERY_PERCENTAGE
                            && let Some(&percentage) = packet.data().first()
                        {
                            with_shared(|s| s.battery = Some(percentage));
                        }
                        if !packet.is_response() {
                            if packet.did == sphero::DID_SENSOR
                                && packet.cid == sphero::CID_STREAMING_DATA
                            {
                                store_sample(stream::parse(packet.data()));
                            }
                            // Notifications never answer a request.
                            continue;
                        }
                        // Full queue means nobody is waiting; drop the packet.
                        let _ = REPLIES.try_send(packet);
                    }
                }
            }
            Err(error) => {
                warn!("UART read error: {error:?}");
                decoder.reset();
            }
        }
    }
}

fn store_sample(sample: Option<Sample>) {
    let now = Instant::now();
    with_shared(|s| match sample {
        Some(Sample::Attitude { yaw, .. }) => s.yaw = Some((yaw, now)),
        Some(Sample::Motion { x, y, vx, vy }) => {
            let locator = Locator {
                position: (x, y),
                velocity: (vx, vy),
            };
            s.position = Some((locator, now));
        }
        None => {}
    });
}

struct Rvr {
    tx: UartTx<'static, Async>,
    seq: u8,
}

impl Rvr {
    fn next_seq(&mut self) -> u8 {
        self.seq = self.seq.wrapping_add(1);
        self.seq
    }

    async fn send(&mut self, frame: Frame) {
        let mut bytes = frame.as_bytes();
        while !bytes.is_empty() {
            match self.tx.write_async(bytes).await {
                Ok(n) => bytes = &bytes[n..],
                Err(error) => {
                    warn!("UART write error: {error:?}");
                    return;
                }
            }
        }
        let _ = self.tx.flush_async().await;
    }

    /// Send a command built from a sequence number and wait for its response.
    async fn request(&mut self, build: impl FnOnce(u8) -> Frame) -> Option<Packet> {
        let seq = self.next_seq();
        REPLIES.clear();
        self.send(build(seq)).await;
        with_timeout(REPLY_TIMEOUT, async {
            loop {
                let packet = REPLIES.receive().await;
                if packet.is_response() && packet.seq == seq {
                    return packet;
                }
            }
        })
        .await
        .ok()
    }

    /// Wake the RVR and confirm it talks back. The link only counts as up
    /// after a successful battery reply, never merely from an open port.
    async fn connect(&mut self) -> Result<u8, &'static str> {
        // The wake acknowledgement is informative only: some firmware stays
        // silent while waking, and the battery query below is the real proof.
        let _ = self.request(sphero::wake).await;
        Timer::after(Duration::from_secs(2)).await;
        let reply = self
            .request(sphero::battery_percentage)
            .await
            .ok_or("RVR did not answer battery query")?;
        if reply.error != 0 {
            return Err("RVR rejected battery query");
        }
        let percentage = *reply
            .data()
            .first()
            .ok_or("RVR sent an empty battery reply")?;
        self.request(|seq| sphero::set_control_timeout(seq, RVR_CONTROL_TIMEOUT_MS))
            .await
            .filter(|reply| reply.error == 0)
            .ok_or("RVR did not accept the control timeout")?;
        Ok(percentage)
    }

    /// Send a command that must be acknowledged without error.
    async fn confirm(&mut self, build: impl FnOnce(u8) -> Frame) -> bool {
        self.request(build)
            .await
            .is_some_and(|reply| reply.error == 0)
    }

    /// Subscribe to attitude, locator and velocity on the ST processor.
    /// Driving still works without these; only pose-based skills need them.
    async fn start_streaming(&mut self) -> Result<(), &'static str> {
        // Clearing an empty configuration may be rejected; that is harmless.
        let _ = self.request(sphero::clear_streaming).await;
        let configured = self
            .confirm(|seq| {
                sphero::configure_streaming(seq, stream::TOKEN_ATTITUDE, stream::ATTITUDE_SERVICES)
            })
            .await
            && self
                .confirm(|seq| {
                    sphero::configure_streaming(seq, stream::TOKEN_MOTION, stream::MOTION_SERVICES)
                })
                .await;
        if !configured {
            return Err("RVR rejected the pose stream configuration");
        }
        if !self
            .confirm(|seq| sphero::start_streaming(seq, STREAM_PERIOD_MS))
            .await
        {
            return Err("RVR did not start pose streaming");
        }
        Ok(())
    }

    /// Make the current position (0, 0) and the current heading yaw 0.
    async fn reset_pose(&mut self) {
        let ok = self.confirm(sphero::reset_yaw).await && self.confirm(sphero::reset_locator).await;
        if ok {
            info!("Pose reset");
            with_shared(|s| {
                s.yaw = None;
                s.position = None;
            });
        } else {
            warn!("Pose reset failed");
            with_shared(|s| s.fault = Some(("RVR rejected the pose reset", Instant::now())));
        }
    }
}

#[embassy_executor::task]
async fn rover(tx: UartTx<'static, Async>) {
    let mut rvr = Rvr { tx, seq: 0 };

    // The agent on the Pi gives up after one failed attempt; with no systemd
    // here, keep retrying so powering the rover on later just works.
    loop {
        match rvr.connect().await {
            Ok(percentage) => {
                info!("RVR connected, battery {percentage}%");
                with_shared(|s| {
                    s.rvr_connected = true;
                    s.battery = Some(percentage);
                    s.link_error = None;
                });
                break;
            }
            Err(message) => {
                warn!("RVR link check failed: {message}");
                with_shared(|s| s.link_error = Some(message));
                Timer::after(Duration::from_secs(5)).await;
            }
        }
    }

    match rvr.start_streaming().await {
        Ok(()) => info!("RVR pose streaming every {STREAM_PERIOD_MS} ms"),
        Err(message) => {
            warn!("{message}");
            with_shared(|s| s.pose_error = Some(message));
        }
    }

    let mut last_sent: Option<Drive> = None;
    let mut last_write = Instant::now();
    let mut battery_poll = Instant::now();
    loop {
        if RESET_POSE.try_take().is_some() {
            rvr.reset_pose().await;
        }
        let command = with_shared(|s| s.drive());
        let moving = command != Drive::Tracks(0, 0);
        if last_sent != Some(command) || (moving && last_write.elapsed() >= MOTOR_REFRESH) {
            let seq = rvr.next_seq();
            let frame = match command {
                Drive::Tracks(left, right) => sphero::raw_motors(seq, left, right),
                Drive::Velocity(linear, yaw_rate) => sphero::drive_rc_si(seq, linear, yaw_rate),
            };
            rvr.send(frame).await;
            last_sent = Some(command);
            last_write = Instant::now();
        } else if battery_poll.elapsed() >= Duration::from_secs(10) {
            // Fire and forget: `uart_reader` stores the answer, so a slow or
            // missing reply never delays a drive command.
            battery_poll = Instant::now();
            let seq = rvr.next_seq();
            rvr.send(sphero::battery_percentage(seq)).await;
        }
        // Wake immediately on a key change; otherwise poll often enough to
        // notice stale keys and to refresh a held command.
        let _ = with_timeout(LOOP_POLL, MOTION_CHANGED.wait()).await;
    }
}
