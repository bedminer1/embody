use crate::protocol::{self, Incoming, Telemetry};
use std::io;
use std::net::TcpStream;
use std::process::{Child, Command, Stdio};
use std::sync::{Arc, Mutex};
use std::thread;
use std::time::Duration;

const LOCAL_PORT: u16 = 18765;
const REMOTE_PORT: u16 = 8765;

#[derive(Default)]
struct Latest {
    frame: Option<Vec<u8>>,
    frame_changed: bool,
    telemetry: Option<Telemetry>,
    telemetry_changed: bool,
    disconnected: Option<String>,
}

pub struct Update {
    pub frame: Option<Vec<u8>>,
    pub telemetry: Option<Telemetry>,
    pub disconnected: Option<String>,
}

pub struct Connection {
    tunnel: Child,
    writer: TcpStream,
    latest: Arc<Mutex<Latest>>,
}

impl Connection {
    pub fn connect(target: &str) -> Result<Self, String> {
        let forward = format!("{LOCAL_PORT}:127.0.0.1:{REMOTE_PORT}");
        let mut tunnel = Command::new("ssh")
            .args([
                "-N",
                "-o",
                "BatchMode=yes",
                "-o",
                "ExitOnForwardFailure=yes",
                "-L",
                &forward,
                target,
            ])
            .stdin(Stdio::null())
            .stdout(Stdio::null())
            .spawn()
            .map_err(|error| format!("Cannot start SSH: {error}"))?;
        let address = format!("127.0.0.1:{LOCAL_PORT}");
        let writer = match wait_for_connection(&mut tunnel, &address) {
            Ok(stream) => stream,
            Err(error) => {
                let _ = tunnel.kill();
                let _ = tunnel.wait();
                return Err(format!(
                    "Cannot reach the Pi service through SSH ({target}): {error}"
                ));
            }
        };
        writer.set_nodelay(true).ok();
        writer.set_write_timeout(Some(Duration::from_secs(1))).ok();
        let reader = writer.try_clone().map_err(|error| error.to_string())?;
        let latest = Arc::new(Mutex::new(Latest::default()));
        let receiver = Arc::clone(&latest);
        thread::spawn(move || receive(reader, receiver));
        Ok(Self {
            tunnel,
            writer,
            latest,
        })
    }

    pub fn send_keys(&mut self, keys: &str) -> io::Result<()> {
        protocol::send_keys(&mut self.writer, keys)
    }

    pub fn update(&self) -> Update {
        let mut latest = self.latest.lock().unwrap();
        Update {
            frame: if std::mem::take(&mut latest.frame_changed) {
                latest.frame.clone()
            } else {
                None
            },
            telemetry: if std::mem::take(&mut latest.telemetry_changed) {
                latest.telemetry.clone()
            } else {
                None
            },
            disconnected: latest.disconnected.take(),
        }
    }

    pub fn snapshot(&self) -> Option<Vec<u8>> {
        self.latest.lock().unwrap().frame.clone()
    }
}

fn wait_for_connection(tunnel: &mut Child, address: &str) -> Result<TcpStream, String> {
    for _ in 0..50 {
        match tunnel.try_wait() {
            Ok(Some(status)) => return Err(format!("SSH exited: {status}")),
            Err(error) => return Err(error.to_string()),
            Ok(None) => {}
        }
        if let Ok(stream) = TcpStream::connect(address) {
            return Ok(stream);
        }
        thread::sleep(Duration::from_millis(100));
    }
    Err("timed out waiting for the Pi agent".into())
}

impl Drop for Connection {
    fn drop(&mut self) {
        let _ = self.send_keys("");
        let _ = self.tunnel.kill();
        let _ = self.tunnel.wait();
    }
}

fn receive(mut reader: TcpStream, latest: Arc<Mutex<Latest>>) {
    loop {
        match protocol::read(&mut reader) {
            Ok(Incoming::Video(frame)) => {
                let mut latest = latest.lock().unwrap();
                latest.frame = Some(frame);
                latest.frame_changed = true;
            }
            Ok(Incoming::Telemetry(telemetry)) => {
                let mut latest = latest.lock().unwrap();
                latest.telemetry = Some(telemetry);
                latest.telemetry_changed = true;
            }
            Err(error) => {
                latest.lock().unwrap().disconnected = Some(error.to_string());
                break;
            }
        }
    }
}
