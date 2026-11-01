mod controls;
mod protocol;
mod transport;
mod ui;
mod video;

use controls::{Action, Controls};
use crossterm::event::{self, Event};
use minifb::{Window, WindowOptions};
use serde_json::{json, Value};
use std::io::{self, BufRead, Write};
use std::sync::mpsc;
use std::thread;
use std::time::{Duration, Instant};
use transport::Connection;

/// Key state is also sent immediately whenever it changes; this heartbeat only
/// keeps the rover's 500 ms dead-man timeout fed.
const HEARTBEAT: Duration = Duration::from_millis(50);
const DRAW_INTERVAL: Duration = Duration::from_millis(100);

struct Options {
    target: String,
    direct: Option<String>,
    json: bool,
}

impl Options {
    fn parse() -> Result<Self, String> {
        let mut target = "pi5".to_string();
        let mut direct = None;
        let mut json = false;
        let mut args = std::env::args().skip(1);
        while let Some(arg) = args.next() {
            match arg.as_str() {
                "--json" => json = true,
                "--target" => target = args.next().ok_or("--target needs an SSH host")?,
                "--direct" => direct = Some(args.next().ok_or("--direct needs HOST or HOST:PORT")?),
                "--help" | "-h" => {
                    println!("Embody desktop\n\nUsage: embody-desktop [--target SSH_HOST | --direct HOST[:PORT]] [--json]\n\nDefault: keyboard dashboard and live video window.\n--target: reach the Pi agent through an SSH tunnel (default pi5).\n--direct: connect straight to an agent on the network, e.g. the ESP32 (port 8765).\n--json: newline-delimited JSON commands and telemetry for programs.");
                    std::process::exit(0);
                }
                _ if !arg.starts_with('-') => target = arg,
                _ => return Err(format!("Unknown option: {arg}")),
            }
        }
        Ok(Self {
            target,
            direct,
            json,
        })
    }
}

fn manual(mut link: Connection) -> Result<(), String> {
    let mut window = Window::new("Embody · Live video", 848, 480, WindowOptions::default())
        .map_err(|error| format!("Cannot open video window: {error}"))?;
    window.set_target_fps(60);
    let mut dashboard = ui::Dashboard::enter().map_err(|error| error.to_string())?;
    let mut controls = Controls::new(dashboard.release_events());
    let mut telemetry = None;
    let mut image = video::Frame {
        pixels: vec![0; 848 * 480],
        width: 848,
        height: 480,
    };
    let mut last_command = Instant::now() - HEARTBEAT;
    let mut last_sent = String::new();
    let mut last_draw = Instant::now() - DRAW_INTERVAL;

    while window.is_open() {
        while event::poll(Duration::ZERO).map_err(|error| error.to_string())? {
            match event::read().map_err(|error| error.to_string())? {
                Event::Key(key) => match controls.handle_key(key) {
                    Action::Quit => return Ok(()),
                    Action::Stop => link.send_keys("").map_err(|error| error.to_string())?,
                    Action::Continue => {}
                },
                // Release events never arrive for keys held while unfocused.
                Event::FocusLost => controls.release_all(),
                _ => {}
            }
        }
        let keys = controls.keys();
        let update = link.update();
        if let Some(reason) = update.disconnected {
            return Err(format!("Pi connection closed: {reason}"));
        }
        if let Some(data) = update.telemetry {
            telemetry = Some(data);
        }
        if let Some(jpeg) = update.frame {
            if let Some(decoded) = video::decode(&jpeg) {
                image = decoded;
            }
        }
        if keys != last_sent || last_command.elapsed() >= HEARTBEAT {
            link.send_keys(&keys).map_err(|error| error.to_string())?;
            last_command = Instant::now();
            last_sent = keys.clone();
        }
        if last_draw.elapsed() >= DRAW_INTERVAL {
            dashboard
                .draw(&keys, telemetry.as_ref())
                .map_err(|error| error.to_string())?;
            last_draw = Instant::now();
        }
        window
            .update_with_buffer(&image.pixels, image.width, image.height)
            .map_err(|error| error.to_string())?;
    }
    Ok(())
}

fn emit(value: Value) {
    println!("{value}");
    let _ = io::stdout().flush();
}

fn machine(mut link: Connection) -> Result<(), String> {
    let wait_started = Instant::now();
    while link.snapshot().is_none() && wait_started.elapsed() < Duration::from_secs(2) {
        link.send_keys("").map_err(|error| error.to_string())?;
        if let Some(reason) = link.update().disconnected {
            return Err(format!("Pi connection closed: {reason}"));
        }
        thread::sleep(Duration::from_millis(50));
    }
    let video_available = link.snapshot().is_some();
    let (tx, rx) = mpsc::channel::<String>();
    thread::spawn(move || {
        for line in io::stdin().lock().lines() {
            match line {
                Ok(line) => {
                    if tx.send(line).is_err() {
                        break;
                    }
                }
                _ => break,
            }
        }
    });
    let mut keys = String::new();
    let mut deadline = Instant::now();
    let mut last_command = Instant::now() - HEARTBEAT;
    emit(json!({"type":"ready","controls":"wasd","max_lease_ms":2000,
        "video_available":video_available}));

    loop {
        while let Ok(line) = rx.try_recv() {
            let value: Value = match serde_json::from_str(&line) {
                Ok(value) => value,
                Err(error) => {
                    emit(json!({"type":"error","message":error.to_string()}));
                    continue;
                }
            };
            if value["quit"] == true {
                return Ok(());
            }
            if value["stop"] == true {
                keys.clear();
                deadline = Instant::now();
                link.send_keys("").map_err(|error| error.to_string())?;
                emit(json!({"type":"stopped"}));
            }
            if let Some(raw) = value["keys"].as_str() {
                keys = controls::normalize_keys(raw);
                let lease_ms = value["duration_ms"].as_u64().unwrap_or(500).clamp(1, 2000);
                deadline = Instant::now() + Duration::from_millis(lease_ms);
                link.send_keys(&keys).map_err(|error| error.to_string())?;
                emit(json!({"type":"accepted","keys":keys,"lease_ms":lease_ms}));
            }
            if let Some(path) = value["snapshot"].as_str() {
                match link.snapshot() {
                    Some(jpeg) => match std::fs::write(path, &jpeg) {
                        Ok(()) => emit(json!({"type":"snapshot","path":path,"bytes":jpeg.len()})),
                        Err(error) => emit(json!({"type":"error","message":error.to_string()})),
                    },
                    None => emit(json!({"type":"error","message":"No camera frame available yet"})),
                }
            }
        }
        if Instant::now() >= deadline {
            keys.clear();
        }
        if last_command.elapsed() >= HEARTBEAT {
            link.send_keys(&keys).map_err(|error| error.to_string())?;
            last_command = Instant::now();
        }
        let update = link.update();
        if let Some(reason) = update.disconnected {
            return Err(format!("Pi connection closed: {reason}"));
        }
        if let Some(telemetry) = update.telemetry {
            emit(json!({"type":"telemetry","data":telemetry}));
        }
        thread::sleep(Duration::from_millis(20));
    }
}

fn run() -> Result<(), String> {
    let options = Options::parse()?;
    let connection = match &options.direct {
        Some(address) => Connection::connect_direct(address)?,
        None => Connection::connect(&options.target)?,
    };
    if options.json {
        machine(connection)
    } else {
        manual(connection)
    }
}

fn main() {
    if let Err(error) = run() {
        eprintln!("{error}");
        std::process::exit(1);
    }
}
