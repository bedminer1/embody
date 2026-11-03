//! RVR sensor streaming: which services the firmware subscribes to and how
//! to read the `streaming_service_data_notify` packets that come back.
//!
//! Each streamed value is a big-endian `u32` scaled linearly from
//! `0..=u32::MAX` onto the service's documented range (same as the Sphero
//! SDK's `sensor_streaming_control.py`). The first payload byte is the slot
//! token; a non-zero upper nibble marks the sample invalid.

/// Data size code for 32-bit values.
const SIZE_32: u8 = 0x02;

const SERVICE_IMU: u16 = 0x0001;
const SERVICE_LOCATOR: u16 = 0x0006;
const SERVICE_VELOCITY: u16 = 0x0007;

/// Slot 1 holds attitude: pitch, roll, yaw in degrees.
pub const TOKEN_ATTITUDE: u8 = 1;
pub const ATTITUDE_SERVICES: &[(u16, u8)] = &[(SERVICE_IMU, SIZE_32)];
/// Slot 2 holds position (m) and velocity (m/s) in the locator frame.
pub const TOKEN_MOTION: u8 = 2;
pub const MOTION_SERVICES: &[(u16, u8)] =
    &[(SERVICE_LOCATOR, SIZE_32), (SERVICE_VELOCITY, SIZE_32)];

/// One decoded notification.
#[derive(Clone, Copy, Debug, PartialEq)]
pub enum Sample {
    /// Degrees. Yaw is counter-clockwise positive, 0 at the last yaw reset.
    Attitude { pitch: f32, roll: f32, yaw: f32 },
    /// Locator frame: +y is the heading at the last yaw reset, +x is to its
    /// right. Metres and metres per second.
    Motion { x: f32, y: f32, vx: f32, vy: f32 },
}

fn value(data: &[u8], index: usize, min: f64, max: f64) -> Option<f32> {
    let bytes = data.get(index * 4..index * 4 + 4)?;
    let raw = u32::from_be_bytes([bytes[0], bytes[1], bytes[2], bytes[3]]);
    // f64 keeps the locator's ~7 µm resolution; f32 would round to ~2 mm.
    Some((f64::from(raw) / f64::from(u32::MAX) * (max - min) + min) as f32)
}

/// Decode the payload of a streaming notification (`did 0x18, cid 0x3D`).
/// `None` for unknown tokens, short payloads and samples flagged invalid.
pub fn parse(payload: &[u8]) -> Option<Sample> {
    let (&token, data) = payload.split_first()?;
    if token & 0xF0 != 0 {
        return None;
    }
    match token & 0x0F {
        TOKEN_ATTITUDE => Some(Sample::Attitude {
            pitch: value(data, 0, -180.0, 180.0)?,
            roll: value(data, 1, -90.0, 90.0)?,
            yaw: value(data, 2, -180.0, 180.0)?,
        }),
        TOKEN_MOTION => Some(Sample::Motion {
            x: value(data, 0, -16000.0, 16000.0)?,
            y: value(data, 1, -16000.0, 16000.0)?,
            vx: value(data, 2, -5.0, 5.0)?,
            vy: value(data, 3, -5.0, 5.0)?,
        }),
        _ => None,
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn encode(value: f64, min: f64, max: f64) -> [u8; 4] {
        let raw = (((value - min) / (max - min)) * f64::from(u32::MAX)).round() as u32;
        raw.to_be_bytes()
    }

    #[test]
    fn decodes_motion() {
        let mut payload = std::vec![TOKEN_MOTION];
        payload.extend(encode(1.25, -16000.0, 16000.0));
        payload.extend(encode(-0.5, -16000.0, 16000.0));
        payload.extend(encode(0.3, -5.0, 5.0));
        payload.extend(encode(0.0, -5.0, 5.0));
        let Some(Sample::Motion { x, y, vx, vy }) = parse(&payload) else {
            panic!("expected motion");
        };
        assert!((x - 1.25).abs() < 1e-3, "{x}");
        assert!((y + 0.5).abs() < 1e-3, "{y}");
        assert!((vx - 0.3).abs() < 1e-4, "{vx}");
        assert!(vy.abs() < 1e-4, "{vy}");
    }

    #[test]
    fn decodes_attitude() {
        let mut payload = std::vec![TOKEN_ATTITUDE];
        payload.extend(encode(2.0, -180.0, 180.0));
        payload.extend(encode(-1.0, -90.0, 90.0));
        payload.extend(encode(-135.0, -180.0, 180.0));
        let Some(Sample::Attitude { yaw, .. }) = parse(&payload) else {
            panic!("expected attitude");
        };
        assert!((yaw + 135.0).abs() < 1e-3, "{yaw}");
    }

    #[test]
    fn rejects_invalid_and_short_samples() {
        let mut payload = std::vec![0x10 | TOKEN_ATTITUDE];
        payload.extend([0u8; 12]);
        assert_eq!(parse(&payload), None);
        assert_eq!(parse(&[TOKEN_MOTION, 0, 0, 0]), None);
        assert_eq!(parse(&[]), None);
    }
}
