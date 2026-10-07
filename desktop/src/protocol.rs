use serde::{Deserialize, Serialize};
use serde_json::Value;
use std::io::{self, Read, Write};
use std::net::TcpStream;

pub const MAX_PACKET: usize = 8 * 1024 * 1024;

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

pub fn send_keys(stream: &mut TcpStream, keys: &str) -> io::Result<()> {
    let payload = serde_json::json!({ "keys": keys }).to_string();
    stream.write_all(b"C")?;
    stream.write_all(&(payload.len() as u32).to_be_bytes())?;
    stream.write_all(payload.as_bytes())
}
