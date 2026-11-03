use serde::{Deserialize, Serialize};
use serde_json::Value;
use std::io::{self, Read, Write};
use std::net::TcpStream;

pub const MAX_PACKET: usize = 8 * 1024 * 1024;

/// Rover pose in the RVR locator frame: +y is the heading at the last pose
/// reset, +x is to its right, yaw is degrees counter-clockwise.
#[derive(Clone, Copy, Debug, Deserialize, Serialize, PartialEq)]
pub struct Pose {
    pub x: f64,
    pub y: f64,
    pub yaw: f64,
}

#[derive(Clone, Debug, Deserialize, Serialize)]
pub struct Telemetry {
    pub tracks: [i16; 2],
    #[serde(default)]
    pub speed_limit: Option<u8>,
    #[serde(default)]
    pub pivot_forward_percent: Option<u8>,
    pub battery: Option<Value>,
    pub sensors: Value,
    pub error: Option<String>,
    pub rvr_connected: bool,
    /// Fresh locator pose; absent from agents that do not stream it.
    #[serde(default)]
    pub pose: Option<Pose>,
    /// Measured velocity in the locator frame, m/s.
    #[serde(default)]
    pub velocity: Option<[f64; 2]>,
    /// Velocity command being executed: m/s, degrees/s.
    #[serde(default)]
    pub vel: Option<[f64; 2]>,
}

pub enum Incoming {
    Video(Vec<u8>),
    Telemetry(Telemetry),
}

pub fn read(stream: &mut TcpStream) -> io::Result<Incoming> {
    let mut header = [0u8; 5];
    stream.read_exact(&mut header)?;
    let len = u32::from_be_bytes(header[1..].try_into().unwrap()) as usize;
    if len > MAX_PACKET {
        return Err(io::Error::new(
            io::ErrorKind::InvalidData,
            "packet too large",
        ));
    }
    let mut payload = vec![0; len];
    stream.read_exact(&mut payload)?;
    match header[0] {
        b'V' => Ok(Incoming::Video(payload)),
        b'T' => serde_json::from_slice(&payload)
            .map(Incoming::Telemetry)
            .map_err(|error| io::Error::new(io::ErrorKind::InvalidData, error)),
        _ => Err(io::Error::new(
            io::ErrorKind::InvalidData,
            "unknown packet type",
        )),
    }
}

/// Send one control payload: `{"keys":..}`, `{"vel":[m/s, deg/s]}` or
/// `{"reset_pose":true}`. Keys and velocity expire on the rover after 500 ms.
pub fn send_control(stream: &mut TcpStream, control: &Value) -> io::Result<()> {
    let payload = control.to_string();
    stream.write_all(b"C")?;
    stream.write_all(&(payload.len() as u32).to_be_bytes())?;
    stream.write_all(payload.as_bytes())
}
