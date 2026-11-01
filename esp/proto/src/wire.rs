//! Desktop <-> rover framing, identical to `pi/agent.py` and
//! `desktop/src/protocol.rs`: one type byte, a big-endian `u32` length, then
//! the payload. The desktop sends `C` frames carrying `{"keys":"wa"}`; the
//! rover answers with `T` telemetry frames carrying JSON.

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

/// Extract the key mask from a control payload such as `{"keys":"wa"}`.
/// `None` if the payload has no string-valued `keys` field. A minimal scan
/// is enough for the one field the desktop ever sends.
pub fn parse_keys(payload: &[u8]) -> Option<u8> {
    let name = b"\"keys\"";
    let at = payload.windows(name.len()).position(|w| w == name)? + name.len();
    let rest = &payload[at..];
    let colon = rest.iter().position(|b| *b == b':')?;
    let rest = &rest[colon + 1..];
    let open = rest.iter().position(|b| *b == b'"')?;
    let rest = &rest[open + 1..];
    let close = rest.iter().position(|b| *b == b'"')?;
    Some(drive::key_mask(&rest[..close]))
}

/// Rover state reported to the desktop; fields match `Telemetry` in
/// `desktop/src/protocol.rs`.
pub struct Telemetry<'a> {
    pub rvr_connected: bool,
    pub tracks: (i16, i16),
    pub speed_limit: u8,
    pub pivot_forward_percent: u8,
    pub battery_percentage: Option<u8>,
    /// Must not need JSON escaping (plain ASCII without quotes/backslashes).
    pub error: Option<&'a str>,
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
        assert_eq!(parse_keys(br#"{"keys":"wa"}"#), Some(drive::W | drive::A));
        assert_eq!(parse_keys(br#"{"keys":""}"#), Some(0));
        assert_eq!(
            parse_keys(br#"{ "keys" : "SD" }"#),
            Some(drive::S | drive::D)
        );
        assert_eq!(parse_keys(br#"{"other":1}"#), None);
        assert_eq!(parse_keys(br#"{"keys":5}"#), None);
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
        let mut out = [0u8; 256];
        let n = encode_telemetry(
            &mut out,
            &Telemetry {
                rvr_connected: true,
                tracks: (31, -70),
                speed_limit: 70,
                pivot_forward_percent: 60,
                battery_percentage: Some(84),
                error: None,
            },
        )
        .unwrap();
        assert_eq!(out[0], b'T');
        let len = u32::from_be_bytes(out[1..5].try_into().unwrap()) as usize;
        assert_eq!(len, n - HEADER_LEN);
        assert_eq!(
            core::str::from_utf8(&out[HEADER_LEN..n]).unwrap(),
            r#"{"rvr_connected":true,"tracks":[31,-70],"speed_limit":70,"pivot_forward_percent":60,"battery":{"percentage":84},"sensors":{},"error":null}"#
        );
    }

    #[test]
    fn telemetry_reports_errors_and_missing_battery() {
        let mut out = [0u8; 256];
        let n = encode_telemetry(
            &mut out,
            &Telemetry {
                rvr_connected: false,
                tracks: (0, 0),
                speed_limit: 70,
                pivot_forward_percent: 60,
                battery_percentage: None,
                error: Some("RVR did not answer"),
            },
        )
        .unwrap();
        let json = core::str::from_utf8(&out[HEADER_LEN..n]).unwrap();
        assert!(json.contains(r#""battery":null"#));
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
            error: None,
        };
        assert_eq!(encode_telemetry(&mut out, &t), None);
    }
}
