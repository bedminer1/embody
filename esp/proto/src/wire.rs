//! Desktop <-> rover framing, identical to `pi/agent.py` and
//! `desktop/src/protocol.rs`: one type byte, a big-endian `u32` length, then
//! the payload. The desktop sends `C` frames carrying one of
//! `{"keys":"wa"}`, `{"vel":[0.3,-20.0]}` (m/s, degrees/s) or
//! `{"reset_pose":true}`; the rover answers with `T` telemetry frames
//! carrying JSON.

use core::fmt::{self, Write};

use crate::drive;

pub const HEADER_LEN: usize = 5;
/// Largest control payload accepted from the desktop.
pub const MAX_CONTROL: usize = 256;
pub const CONTROL: u8 = b'C';
pub const TELEMETRY: u8 = b'T';

/// Parsed frame header.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct Header {
    pub kind: u8,
    pub len: usize,
}

pub fn parse_header(bytes: [u8; HEADER_LEN]) -> Header {
    Header {
        kind: bytes[0],
        len: u32::from_be_bytes([bytes[1], bytes[2], bytes[3], bytes[4]]) as usize,
    }
}

/// A decoded control payload.
#[derive(Clone, Copy, Debug, PartialEq)]
pub enum Command {
    /// WASD key mask, see [`drive::key_mask`].
    Keys(u8),
    /// Closed-loop velocity: m/s forward, degrees/s counter-clockwise.
    Velocity { linear: f32, yaw_rate: f32 },
    /// Zero the locator position and yaw at the current pose.
    ResetPose,
}

/// The bytes after `"name":`, with leading whitespace removed. A minimal
/// scan is enough for the few flat fields the desktop sends.
fn field<'a>(payload: &'a [u8], name: &str) -> Option<&'a [u8]> {
    let mut quoted = [0u8; 16];
    let name = name.as_bytes();
    let quoted = quoted.get_mut(..name.len() + 2)?;
    quoted[0] = b'"';
    quoted[1..=name.len()].copy_from_slice(name);
    quoted[name.len() + 1] = b'"';
    let at = payload.windows(quoted.len()).position(|w| w == quoted)? + quoted.len();
    let rest = trim(&payload[at..]);
    let rest = rest.strip_prefix(b":")?;
    Some(trim(rest))
}

fn trim(bytes: &[u8]) -> &[u8] {
    let start = bytes
        .iter()
        .position(|b| !b.is_ascii_whitespace())
        .unwrap_or(bytes.len());
    &bytes[start..]
}

/// Parse a JSON number at the start of `bytes`; returns it and the rest.
fn number(bytes: &[u8]) -> Option<(f32, &[u8])> {
    let end = bytes
        .iter()
        .position(|b| !matches!(b, b'0'..=b'9' | b'-' | b'+' | b'.' | b'e' | b'E'))
        .unwrap_or(bytes.len());
    let text = core::str::from_utf8(&bytes[..end]).ok()?;
    let value: f32 = text.parse().ok()?;
    value.is_finite().then_some((value, &bytes[end..]))
}

/// Decode a control payload. `None` if it has none of the known fields.
pub fn parse_command(payload: &[u8]) -> Option<Command> {
    if field(payload, "reset_pose").is_some_and(|rest| rest.starts_with(b"true")) {
        return Some(Command::ResetPose);
    }
    if let Some(rest) = field(payload, "vel") {
        let rest = trim(rest.strip_prefix(b"[")?);
        let (linear, rest) = number(rest)?;
        let rest = trim(trim(rest).strip_prefix(b",")?);
        let (yaw_rate, rest) = number(rest)?;
        trim(rest).strip_prefix(b"]")?;
        return Some(Command::Velocity { linear, yaw_rate });
    }
    let rest = field(payload, "keys")?.strip_prefix(b"\"")?;
    let close = rest.iter().position(|b| *b == b'"')?;
    Some(Command::Keys(drive::key_mask(&rest[..close])))
}

/// Position and heading in the RVR locator frame.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct Pose {
    /// Metres, right of the heading at the last reset.
    pub x: f32,
    /// Metres, along the heading at the last reset.
    pub y: f32,
    /// Degrees, counter-clockwise positive.
    pub yaw: f32,
}

/// Rover state reported to the desktop; fields match `Telemetry` in
/// `desktop/src/protocol.rs`.
pub struct Telemetry<'a> {
    pub rvr_connected: bool,
    pub tracks: (i16, i16),
    pub speed_limit: u8,
    pub pivot_forward_percent: u8,
    pub battery_percentage: Option<u8>,
    /// Fresh locator pose, if the RVR is streaming it.
    pub pose: Option<Pose>,
    /// Measured velocity in the locator frame (m/s).
    pub velocity: Option<(f32, f32)>,
    /// Velocity command being executed: (m/s, degrees/s).
    pub command_velocity: Option<(f32, f32)>,
    /// Must not need JSON escaping (plain ASCII without quotes/backslashes).
    pub error: Option<&'a str>,
}

/// JSON has no NaN/Infinity, so non-finite values become `null`.
fn write_float(writer: &mut impl Write, value: f32) -> fmt::Result {
    if value.is_finite() {
        write!(writer, "{value:.4}")
    } else {
        write!(writer, "null")
    }
}

fn write_pair(writer: &mut impl Write, pair: Option<(f32, f32)>) -> fmt::Result {
    match pair {
        Some((a, b)) => {
            write!(writer, "[")?;
            write_float(writer, a)?;
            write!(writer, ",")?;
            write_float(writer, b)?;
            write!(writer, "]")
        }
        None => write!(writer, "null"),
    }
}

struct SliceWriter<'a> {
    buf: &'a mut [u8],
    len: usize,
}

impl Write for SliceWriter<'_> {
    fn write_str(&mut self, text: &str) -> fmt::Result {
        let end = self.len + text.len();
        self.buf
            .get_mut(self.len..end)
            .ok_or(fmt::Error)?
            .copy_from_slice(text.as_bytes());
        self.len = end;
        Ok(())
    }
}

fn write_pose(writer: &mut impl Write, pose: Option<Pose>) -> fmt::Result {
    let Some(pose) = pose else {
        return write!(writer, "null");
    };
    write!(writer, "{{\"x\":")?;
    write_float(writer, pose.x)?;
    write!(writer, ",\"y\":")?;
    write_float(writer, pose.y)?;
    write!(writer, ",\"yaw\":")?;
    write_float(writer, pose.yaw)?;
    write!(writer, "}}")
}

/// Write a complete `T` frame (header included) into `out`, returning its
/// length, or `None` if `out` is too small.
pub fn encode_telemetry(out: &mut [u8], t: &Telemetry) -> Option<usize> {
    if out.len() < HEADER_LEN {
        return None;
    }
    let mut writer = SliceWriter {
        buf: out,
        len: HEADER_LEN,
    };
    write!(
        writer,
        "{{\"rvr_connected\":{},\"tracks\":[{},{}],\"speed_limit\":{},\"pivot_forward_percent\":{},\"battery\":",
        t.rvr_connected, t.tracks.0, t.tracks.1, t.speed_limit, t.pivot_forward_percent
    )
    .ok()?;
    match t.battery_percentage {
        Some(percent) => write!(writer, "{{\"percentage\":{percent}}}"),
        None => write!(writer, "null"),
    }
    .ok()?;
    write!(writer, ",\"pose\":").ok()?;
    write_pose(&mut writer, t.pose).ok()?;
    write!(writer, ",\"velocity\":").ok()?;
    write_pair(&mut writer, t.velocity).ok()?;
    write!(writer, ",\"vel\":").ok()?;
    write_pair(&mut writer, t.command_velocity).ok()?;
    write!(writer, ",\"sensors\":{{}},\"error\":").ok()?;
    match t.error {
        Some(message) => write!(writer, "\"{message}\""),
        None => write!(writer, "null"),
    }
    .ok()?;
    write!(writer, "}}").ok()?;
    let total = writer.len;
    out[0] = TELEMETRY;
    out[1..HEADER_LEN].copy_from_slice(&((total - HEADER_LEN) as u32).to_be_bytes());
    Some(total)
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn parses_what_the_desktop_sends() {
        assert_eq!(
            parse_command(br#"{"keys":"wa"}"#),
            Some(Command::Keys(drive::W | drive::A))
        );
        assert_eq!(parse_command(br#"{"keys":""}"#), Some(Command::Keys(0)));
        assert_eq!(
            parse_command(br#"{ "keys" : "SD" }"#),
            Some(Command::Keys(drive::S | drive::D))
        );
        assert_eq!(parse_command(br#"{"other":1}"#), None);
        assert_eq!(parse_command(br#"{"keys":5}"#), None);
    }

    #[test]
    fn parses_velocity_and_reset() {
        assert_eq!(
            parse_command(br#"{"vel":[0.25,-30.5]}"#),
            Some(Command::Velocity {
                linear: 0.25,
                yaw_rate: -30.5
            })
        );
        assert_eq!(
            parse_command(br#"{"vel": [ -1e-1 , 0 ]}"#),
            Some(Command::Velocity {
                linear: -0.1,
                yaw_rate: 0.0
            })
        );
        assert_eq!(parse_command(br#"{"vel":[0.2]}"#), None);
        assert_eq!(parse_command(br#"{"vel":[0.2,"x"]}"#), None);
        assert_eq!(
            parse_command(br#"{"reset_pose":true}"#),
            Some(Command::ResetPose)
        );
        assert_eq!(parse_command(br#"{"reset_pose":false}"#), None);
    }

    #[test]
    fn header_round_trip() {
        let header = parse_header([b'C', 0, 0, 1, 2]);
        assert_eq!(
            header,
            Header {
                kind: b'C',
                len: 258
            }
        );
    }

    #[test]
    fn telemetry_frame_is_well_formed() {
        let mut out = [0u8; 512];
        let n = encode_telemetry(
            &mut out,
            &Telemetry {
                rvr_connected: true,
                tracks: (31, -70),
                speed_limit: 70,
                pivot_forward_percent: 60,
                battery_percentage: Some(84),
                pose: Some(Pose {
                    x: 0.5,
                    y: -1.25,
                    yaw: 90.0,
                }),
                velocity: Some((0.0, 0.3)),
                command_velocity: Some((0.3, 0.0)),
                error: None,
            },
        )
        .unwrap();
        assert_eq!(out[0], b'T');
        let len = u32::from_be_bytes(out[1..5].try_into().unwrap()) as usize;
        assert_eq!(len, n - HEADER_LEN);
        assert_eq!(
            core::str::from_utf8(&out[HEADER_LEN..n]).unwrap(),
            r#"{"rvr_connected":true,"tracks":[31,-70],"speed_limit":70,"pivot_forward_percent":60,"battery":{"percentage":84},"pose":{"x":0.5000,"y":-1.2500,"yaw":90.0000},"velocity":[0.0000,0.3000],"vel":[0.3000,0.0000],"sensors":{},"error":null}"#
        );
    }

    #[test]
    fn telemetry_reports_errors_and_missing_battery() {
        let mut out = [0u8; 512];
        let n = encode_telemetry(
            &mut out,
            &Telemetry {
                rvr_connected: false,
                tracks: (0, 0),
                speed_limit: 70,
                pivot_forward_percent: 60,
                battery_percentage: None,
                pose: None,
                velocity: None,
                command_velocity: Some((f32::NAN, 0.0)),
                error: Some("RVR did not answer"),
            },
        )
        .unwrap();
        let json = core::str::from_utf8(&out[HEADER_LEN..n]).unwrap();
        assert!(json.contains(r#""battery":null"#));
        assert!(json.contains(r#""pose":null,"velocity":null,"vel":[null,0.0000]"#));
        assert!(json.ends_with(r#""error":"RVR did not answer"}"#));
    }

    #[test]
    fn too_small_buffer_is_reported() {
        let mut out = [0u8; 16];
        let t = Telemetry {
            rvr_connected: false,
            tracks: (0, 0),
            speed_limit: 1,
            pivot_forward_percent: 1,
            battery_percentage: None,
            pose: None,
            velocity: None,
            command_velocity: None,
            error: None,
        };
        assert_eq!(encode_telemetry(&mut out, &t), None);
    }
}
