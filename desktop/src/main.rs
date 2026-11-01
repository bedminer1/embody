mod controls;
mod nav;
mod protocol;
mod transport;
mod ui;
mod video;

use controls::{Action, Controls};
use crossterm::event::{self, Event};
use minifb::{Window, WindowOptions};
use nav::{Executor, Skill, Step};
use protocol::Pose;
use serde_json::{json, Value};
use std::collections::BTreeMap;
use std::io::{self, BufRead, Write};
use std::path::PathBuf;
use std::sync::mpsc;
use std::thread;
use std::time::{Duration, Instant};
use transport::Connection;

/// Key state is also sent immediately whenever it changes; this heartbeat only
/// keeps the rover's 500 ms dead-man timeout fed.
const HEARTBEAT: Duration = Duration::from_millis(50);
const DRAW_INTERVAL: Duration = Duration::from_millis(100);
/// JSON mode reports telemetry at most this often.
const TELEMETRY_INTERVAL: Duration = Duration::from_millis(200);
/// A skill stops if the rover's pose is older than this.
const POSE_TIMEOUT: Duration = Duration::from_millis(400);

struct Options {
    target: String,
    direct: Option<String>,
    json: bool,
    places: Option<PathBuf>,
}

impl Options {
    fn parse() -> Result<Self, String> {
        let mut target = "pi5".to_string();
        let mut direct = None;
        let mut json = false;
        let mut places = None;
        let mut args = std::env::args().skip(1);
        while let Some(arg) = args.next() {
            match arg.as_str() {
                "--json" => json = true,
                "--target" => target = args.next().ok_or("--target needs an SSH host")?,
                "--direct" => direct = Some(args.next().ok_or("--direct needs HOST or HOST:PORT")?),
                "--places" => places = Some(PathBuf::from(args.next().ok_or("--places needs a FILE")?)),
                "--help" | "-h" => {
                    println!("Embody desktop\n\nUsage: embody-desktop [--target SSH_HOST | --direct HOST[:PORT]] [--json [--places FILE]]\n\nDefault: keyboard dashboard and live video window.\n--target: reach the Pi agent through an SSH tunnel (default pi5).\n--direct: connect straight to an agent on the network, e.g. the ESP32 (port 8765).\n--json: newline-delimited JSON commands, skills and telemetry for programs.\n--places: JSON file that keeps named places between runs.");
                    std::process::exit(0);
                }
                _ if !arg.starts_with('-') => target = arg,
                _ => return Err(format!("Unknown option: {arg}")),
            }
        }
        if places.is_some() && !json {
            return Err("--places only applies with --json".into());
        }
        Ok(Self {
            target,
            direct,
            json,
            places,
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
                    Action::ResetPose => link.reset_pose().map_err(|error| error.to_string())?,
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

/// Named poses in the locator frame. The frame restarts whenever the pose is
/// reset (or the RVR powers up), so places only stay valid if the rover is
/// reset at the same physical spot each time.
struct Places {
    path: Option<PathBuf>,
    poses: BTreeMap<String, Pose>,
}

impl Places {
    fn load(path: Option<PathBuf>) -> Result<Self, String> {
        let poses = match &path {
            Some(file) if file.exists() => {
                let text = std::fs::read_to_string(file)
                    .map_err(|error| format!("Cannot read {}: {error}", file.display()))?;
                serde_json::from_str(&text)
                    .map_err(|error| format!("Cannot parse {}: {error}", file.display()))?
            }
            _ => BTreeMap::new(),
        };
        Ok(Self { path, poses })
    }

    fn save(&self) -> Result<(), String> {
        let Some(file) = &self.path else {
            return Ok(());
        };
        let text = serde_json::to_string_pretty(&self.poses).map_err(|error| error.to_string())?;
        std::fs::write(file, text).map_err(|error| format!("Cannot write {}: {error}", file.display()))
    }
}

fn number(value: &Value, name: &str) -> Result<Option<f64>, String> {
    match &value[name] {
        Value::Null => Ok(None),
        field => field
            .as_f64()
            .map(Some)
            .ok_or_else(|| format!("{name} must be a number")),
    }
}

fn required(value: &Value, name: &str) -> Result<f64, String> {
    number(value, name)?.ok_or_else(|| format!("{name} is required"))
}

/// Parse a `{"skill":...}` request; `go_to` may name a remembered place.
fn parse_skill(value: &Value, places: &Places) -> Result<Skill, String> {
    let speed = number(value, "speed_mps")?.unwrap_or(nav::DEFAULT_SPEED);
    match value["skill"].as_str().unwrap_or_default() {
        "drive" => Ok(Skill::Drive {
            distance_m: required(value, "distance_m")?,
            speed_mps: speed,
        }),
        "turn" => Ok(Skill::Turn {
            degrees: required(value, "degrees")?,
        }),
        "turn_to" => Ok(Skill::TurnTo {
            yaw_deg: required(value, "yaw_deg")?,
        }),
        "go_to" => match value["place"].as_str() {
            Some(name) => {
                let place = places
                    .poses
                    .get(name)
                    .ok_or_else(|| format!("Unknown place {name:?}"))?;
                Ok(Skill::GoTo {
                    x: place.x,
                    y: place.y,
                    yaw_deg: Some(place.yaw),
                    speed_mps: speed,
                })
            }
            None => Ok(Skill::GoTo {
                x: required(value, "x")?,
                y: required(value, "y")?,
                yaw_deg: number(value, "yaw_deg")?,
                speed_mps: speed,
            }),
        },
        other => Err(format!(
            "Unknown skill {other:?}; use drive, turn, turn_to or go_to"
        )),
    }
}

struct Running {
    executor: Executor,
    id: Value,
}

fn machine(mut link: Connection, places: Option<PathBuf>) -> Result<(), String> {
    let mut places = Places::load(places)?;
    let mut pose: Option<(Pose, Instant)> = None;
    // Wait briefly for video and a pose so the first request can use them.
    let wait_started = Instant::now();
    while (link.snapshot().is_none() || pose.is_none())
        && wait_started.elapsed() < Duration::from_secs(2)
    {
        link.send_keys("").map_err(|error| error.to_string())?;
        let update = link.update();
        if let Some(reason) = update.disconnected {
            return Err(format!("Pi connection closed: {reason}"));
        }
        if let Some(current) = update.telemetry.and_then(|telemetry| telemetry.pose) {
            pose = Some((current, Instant::now()));
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
    let clock = Instant::now();
    let seconds = || clock.elapsed().as_secs_f64();
    let mut keys = String::new();
    let mut velocity: Option<(f64, f64)> = None;
    let mut deadline = Instant::now();
    let mut last_command = Instant::now() - HEARTBEAT;
    let mut last_telemetry = Instant::now() - TELEMETRY_INTERVAL;
    let mut running: Option<Running> = None;
    emit(json!({"type":"ready","controls":"wasd","max_lease_ms":2000,
        "video_available":video_available,
        "skills":["drive","turn","turn_to","go_to"],
        "places":places.poses.keys().collect::<Vec<_>>()}));

    loop {
        while let Ok(line) = rx.try_recv() {
            let value: Value = match serde_json::from_str(&line) {
                Ok(value) => value,
                Err(error) => {
                    emit(json!({"type":"error","message":error.to_string()}));
                    continue;
                }
            };
            let id = value["id"].clone();
            if value["quit"] == true {
                return Ok(());
            }
            // Any manual command overrides a running skill.
            let manual = value["stop"] == true
                || value["keys"].is_string()
                || value["vel"].is_array()
                || value["reset_pose"] == true
                || value["skill"].is_string();
            if manual {
                if let Some(old) = running.take() {
                    emit(json!({"type":"skill","state":"cancelled","id":old.id,
                        "skill":old.executor.skill(),"pose":pose.map(|(p, _)| p)}));
                }
            }
            if value["stop"] == true {
                keys.clear();
                velocity = None;
                deadline = Instant::now();
                link.send_keys("").map_err(|error| error.to_string())?;
                emit(json!({"type":"stopped","id":id}));
            }
            if let Some(raw) = value["keys"].as_str() {
                keys = controls::normalize_keys(raw);
                velocity = None;
                let lease_ms = value["duration_ms"].as_u64().unwrap_or(500).clamp(1, 2000);
                deadline = Instant::now() + Duration::from_millis(lease_ms);
                link.send_keys(&keys).map_err(|error| error.to_string())?;
                emit(json!({"type":"accepted","keys":keys,"lease_ms":lease_ms,"id":id}));
            }
            if let Some(pair) = value["vel"].as_array() {
                match (pair.first().and_then(Value::as_f64), pair.get(1).and_then(Value::as_f64)) {
                    (Some(linear), Some(yaw_rate)) if pair.len() == 2 => {
                        let command = (
                            linear.clamp(-nav::MAX_SPEED, nav::MAX_SPEED),
                            yaw_rate.clamp(-180.0, 180.0),
                        );
                        keys.clear();
                        velocity = Some(command);
                        let lease_ms = value["duration_ms"].as_u64().unwrap_or(500).clamp(1, 2000);
                        deadline = Instant::now() + Duration::from_millis(lease_ms);
                        link.send_velocity(command.0, command.1)
                            .map_err(|error| error.to_string())?;
                        emit(json!({"type":"accepted","vel":[command.0, command.1],
                            "lease_ms":lease_ms,"id":id}));
                    }
                    _ => emit(json!({"type":"error","id":id,
                        "message":"vel must be [m/s, degrees/s]"})),
                }
            }
            if value["reset_pose"] == true {
                keys.clear();
                velocity = None;
                pose = None;
                link.reset_pose().map_err(|error| error.to_string())?;
                emit(json!({"type":"pose_reset","id":id}));
            }
            if value["skill"].is_string() {
                keys.clear();
                velocity = None;
                let fresh = pose.filter(|(_, at)| at.elapsed() <= POSE_TIMEOUT);
                let started = parse_skill(&value, &places).and_then(|skill| match fresh {
                    Some((current, _)) => Executor::new(skill, current, seconds()),
                    None => Err("No fresh rover pose: is the RVR connected and streaming?".into()),
                });
                match started {
                    Ok(executor) => {
                        emit(json!({"type":"skill","state":"started","id":id,
                            "skill":executor.skill(),"pose":fresh.map(|(p, _)| p)}));
                        running = Some(Running { executor, id: id.clone() });
                    }
                    Err(message) => {
                        link.send_keys("").map_err(|error| error.to_string())?;
                        emit(json!({"type":"skill","state":"failed","id":id,
                            "message":message,"pose":fresh.map(|(p, _)| p)}));
                    }
                }
            }
            if let Some(name) = value["remember"].as_str() {
                match pose.filter(|(_, at)| at.elapsed() <= POSE_TIMEOUT) {
                    Some((current, _)) => {
                        places.poses.insert(name.to_string(), current);
                        match places.save() {
                            Ok(()) => emit(json!({"type":"remembered","id":id,
                                "name":name,"pose":current})),
                            Err(message) => emit(json!({"type":"error","id":id,"message":message})),
                        }
                    }
                    None => emit(json!({"type":"error","id":id,
                        "message":"No fresh rover pose to remember"})),
                }
            }
            if let Some(name) = value["forget"].as_str() {
                let existed = places.poses.remove(name).is_some();
                if let Err(message) = places.save() {
                    emit(json!({"type":"error","id":id,"message":message}));
                }
                emit(json!({"type":"forgot","id":id,"name":name,"existed":existed}));
            }
            if value["places"] == true {
                emit(json!({"type":"places","id":id,"places":places.poses}));
            }
            if value["status"] == true {
                emit(json!({"type":"status","id":id,
                    "pose":pose.filter(|(_, at)| at.elapsed() <= POSE_TIMEOUT).map(|(p, _)| p),
                    "skill":running.as_ref().map(|r| r.executor.skill()),
                    "places":places.poses.keys().collect::<Vec<_>>()}));
            }
            if let Some(path) = value["snapshot"].as_str() {
                match link.snapshot() {
                    Some(jpeg) => match std::fs::write(path, &jpeg) {
                        Ok(()) => emit(json!({"type":"snapshot","path":path,"bytes":jpeg.len(),"id":id})),
                        Err(error) => emit(json!({"type":"error","message":error.to_string(),"id":id})),
                    },
                    None => emit(json!({"type":"error","message":"No camera frame available yet","id":id})),
                }
            }
        }
        let update = link.update();
        if let Some(reason) = update.disconnected {
            return Err(format!("Pi connection closed: {reason}"));
        }
        if let Some(telemetry) = update.telemetry {
            if let Some(current) = telemetry.pose {
                pose = Some((current, Instant::now()));
            }
            if last_telemetry.elapsed() >= TELEMETRY_INTERVAL {
                emit(json!({"type":"telemetry","data":telemetry}));
                last_telemetry = Instant::now();
            }
        }
        if Instant::now() >= deadline {
            keys.clear();
            velocity = None;
        }
        if last_command.elapsed() >= HEARTBEAT {
            last_command = Instant::now();
            if let Some(active) = running.as_mut() {
                let step = match pose.filter(|(_, at)| at.elapsed() <= POSE_TIMEOUT) {
                    Some((current, _)) => active.executor.step(current, seconds()),
                    None => Step::Done(Err("lost the rover pose".into())),
                };
                match step {
                    Step::Drive { linear, yaw_rate } => link
                        .send_velocity(linear, yaw_rate)
                        .map_err(|error| error.to_string())?,
                    Step::Done(result) => {
                        link.send_keys("").map_err(|error| error.to_string())?;
                        let Running { executor, id } = running.take().unwrap();
                        let (state, message) = match result {
                            Ok(message) => ("done", message),
                            Err(message) => ("failed", message),
                        };
                        emit(json!({"type":"skill","state":state,"id":id,
                            "skill":executor.skill(),"message":message,
                            "elapsed_s":(executor.elapsed(seconds()) * 10.0).round() / 10.0,
                            "pose":pose.map(|(p, _)| p)}));
                    }
                }
            } else if let Some((linear, yaw_rate)) = velocity {
                link.send_velocity(linear, yaw_rate)
                    .map_err(|error| error.to_string())?;
            } else {
                link.send_keys(&keys).map_err(|error| error.to_string())?;
            }
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
        machine(connection, options.places)
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
