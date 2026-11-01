//! Sphero RVR serial packets (115200 8N1, 3.3 V).
//!
//! Layout, as emitted by Sphero's Python SDK:
//! `0x8D | flags | [target] | [source] | did | cid | seq | [err] | payload | checksum | 0xD8`
//! where `checksum = !(sum of bytes between SOP and checksum)` and the bytes
//! `0x8D`, `0xD8`, `0xAB` inside the packet are escaped as `0xAB` followed by
//! `0x05`, `0x50`, `0x23` respectively.

pub const SOP: u8 = 0x8D;
pub const EOP: u8 = 0xD8;
const ESC: u8 = 0xAB;
const ESC_SOP: u8 = 0x05;
const ESC_EOP: u8 = 0x50;
const ESC_ESC: u8 = 0x23;

const FLAG_IS_RESPONSE: u8 = 0x01;
const FLAG_REQUESTS_RESPONSE: u8 = 0x02;
const FLAG_ERROR_ONLY: u8 = 0x04;
const FLAG_IS_ACTIVITY: u8 = 0x08;
const FLAG_HAS_TARGET: u8 = 0x10;
const FLAG_HAS_SOURCE: u8 = 0x20;

/// Processor on the RVR that handles a command.
const TARGET_NORDIC: u8 = 1;
const TARGET_ST: u8 = 2;

pub const DID_POWER: u8 = 0x13;
const DID_DRIVE: u8 = 0x16;

const CID_WAKE: u8 = 0x0D;
pub const CID_BATTERY_PERCENTAGE: u8 = 0x10;
const CID_RAW_MOTORS: u8 = 0x01;
const CID_SET_CONTROL_TIMEOUT: u8 = 0x22;

/// Largest payload this module will encode.
const MAX_PAYLOAD: usize = 8;
/// Worst case: SOP + every byte of (flags, target, did, cid, seq, payload,
/// checksum) escaped + EOP.
pub const MAX_FRAME: usize = 2 + 2 * (5 + MAX_PAYLOAD + 1);

/// An encoded, ready-to-send packet.
#[derive(Clone, Copy)]
pub struct Frame {
    buf: [u8; MAX_FRAME],
    len: usize,
}

impl Frame {
    pub fn as_bytes(&self) -> &[u8] {
        &self.buf[..self.len]
    }
}

/// Whether the RVR should answer a command.
#[derive(Clone, Copy, PartialEq, Eq)]
enum Reply {
    /// Always answer, so success is observable.
    Always,
    /// Answer only if the command failed.
    OnError,
}

fn encode(target: u8, did: u8, cid: u8, seq: u8, reply: Reply, payload: &[u8]) -> Frame {
    assert!(payload.len() <= MAX_PAYLOAD);
    let flags = FLAG_IS_ACTIVITY
        | FLAG_HAS_TARGET
        | match reply {
            Reply::Always => FLAG_REQUESTS_RESPONSE,
            Reply::OnError => FLAG_REQUESTS_RESPONSE | FLAG_ERROR_ONLY,
        };
    let mut body = [0u8; 5 + MAX_PAYLOAD];
    body[..5].copy_from_slice(&[flags, target, did, cid, seq]);
    body[5..5 + payload.len()].copy_from_slice(payload);
    let body = &body[..5 + payload.len()];
    let checksum = !body.iter().fold(0u8, |sum, byte| sum.wrapping_add(*byte));

    let mut frame = Frame {
        buf: [0; MAX_FRAME],
        len: 0,
    };
    let mut push = |byte: u8| {
        frame.buf[frame.len] = byte;
        frame.len += 1;
    };
    push(SOP);
    for &byte in body.iter().chain(core::iter::once(&checksum)) {
        match byte {
            SOP => (push(ESC), push(ESC_SOP)),
            EOP => (push(ESC), push(ESC_EOP)),
            ESC => (push(ESC), push(ESC_ESC)),
            _ => ((), push(byte)),
        };
    }
    push(EOP);
    frame
}

/// `power.wake`: leave soft sleep.
pub fn wake(seq: u8) -> Frame {
    encode(TARGET_NORDIC, DID_POWER, CID_WAKE, seq, Reply::Always, &[])
}

/// `power.get_battery_percentage`: answered with one byte, 0..=100.
pub fn battery_percentage(seq: u8) -> Frame {
    encode(
        TARGET_NORDIC,
        DID_POWER,
        CID_BATTERY_PERCENTAGE,
        seq,
        Reply::Always,
        &[],
    )
}

/// `drive.set_custom_control_system_timeout`, in milliseconds.
pub fn set_control_timeout(seq: u8, millis: u16) -> Frame {
    encode(
        TARGET_ST,
        DID_DRIVE,
        CID_SET_CONTROL_TIMEOUT,
        seq,
        Reply::Always,
        &millis.to_be_bytes(),
    )
}

/// `drive.raw_motors`. Each side is a signed duty cycle: sign selects the
/// direction, magnitude is clamped to 0..=255, zero turns that side off.
/// The RVR only answers if the command fails.
pub fn raw_motors(seq: u8, left: i16, right: i16) -> Frame {
    fn side(duty: i16) -> [u8; 2] {
        let magnitude = duty.unsigned_abs().min(255) as u8;
        match duty {
            0 => [0, 0],
            d if d > 0 => [1, magnitude],
            _ => [2, magnitude],
        }
    }
    let (l, r) = (side(left), side(right));
    encode(
        TARGET_ST,
        DID_DRIVE,
        CID_RAW_MOTORS,
        seq,
        Reply::OnError,
        &[l[0], l[1], r[0], r[1]],
    )
}

const MAX_REPLY_DATA: usize = 16;

/// A packet received from the RVR (command response or notification).
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct Packet {
    flags: u8,
    pub did: u8,
    pub cid: u8,
    pub seq: u8,
    /// `0` is success. Only present in responses.
    pub error: u8,
    data: [u8; MAX_REPLY_DATA],
    data_len: usize,
}

impl Packet {
    pub fn is_response(&self) -> bool {
        self.flags & FLAG_IS_RESPONSE != 0
    }

    pub fn data(&self) -> &[u8] {
        &self.data[..self.data_len]
    }
}

/// Incremental parser: feed it bytes from the UART as they arrive.
pub struct Decoder {
    buf: [u8; 32],
    len: usize,
    in_packet: bool,
    escaped: bool,
}

impl Default for Decoder {
    fn default() -> Self {
        Self::new()
    }
}

impl Decoder {
    pub const fn new() -> Self {
        Self {
            buf: [0; 32],
            len: 0,
            in_packet: false,
            escaped: false,
        }
    }

    pub fn reset(&mut self) {
        self.len = 0;
        self.in_packet = false;
        self.escaped = false;
    }

    /// Returns a packet when `byte` completes a valid one. Corrupt or
    /// oversized packets are dropped silently.
    pub fn push(&mut self, byte: u8) -> Option<Packet> {
        match byte {
            SOP => {
                self.reset();
                self.in_packet = true;
                return None;
            }
            EOP if self.in_packet => {
                let packet = parse(&self.buf[..self.len]);
                self.reset();
                return packet;
            }
            _ if !self.in_packet => return None,
            ESC => {
                self.escaped = true;
                return None;
            }
            _ => {}
        }
        let byte = if self.escaped {
            self.escaped = false;
            match byte {
                ESC_SOP => SOP,
                ESC_EOP => EOP,
                ESC_ESC => ESC,
                _ => {
                    self.reset();
                    return None;
                }
            }
        } else {
            byte
        };
        if self.len == self.buf.len() {
            self.reset();
        } else {
            self.buf[self.len] = byte;
            self.len += 1;
        }
        None
    }
}

fn parse(body: &[u8]) -> Option<Packet> {
    // Including the checksum byte the sum must be 0xFF.
    if body.len() < 5 || body.iter().fold(0u8, |sum, b| sum.wrapping_add(*b)) != 0xFF {
        return None;
    }
    let body = &body[..body.len() - 1];
    let flags = body[0];
    let mut at = 1;
    if flags & FLAG_HAS_TARGET != 0 {
        at += 1;
    }
    if flags & FLAG_HAS_SOURCE != 0 {
        at += 1;
    }
    let header_end = at + 3 + usize::from(flags & FLAG_IS_RESPONSE != 0);
    if body.len() < header_end {
        return None;
    }
    let error = if flags & FLAG_IS_RESPONSE != 0 {
        body[at + 3]
    } else {
        0
    };
    let payload = &body[header_end..];
    if payload.len() > MAX_REPLY_DATA {
        return None;
    }
    let mut data = [0; MAX_REPLY_DATA];
    data[..payload.len()].copy_from_slice(payload);
    Some(Packet {
        flags,
        did: body[at],
        cid: body[at + 1],
        seq: body[at + 2],
        error,
        data,
        data_len: payload.len(),
    })
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn wake_matches_hand_computed_bytes() {
        // sum(1A 01 13 0D 00) = 0x3B, checksum = !0x3B = 0xC4
        assert_eq!(
            wake(0).as_bytes(),
            [0x8D, 0x1A, 0x01, 0x13, 0x0D, 0x00, 0xC4, 0xD8]
        );
    }

    #[test]
    fn raw_motors_encodes_modes_and_error_only_reply() {
        // flags 0x1E; left forward 70, right reverse 70
        // sum(1E 02 16 01 05 01 46 02 46) = 0xCB, checksum = !0xCB = 0x34
        assert_eq!(
            raw_motors(5, 70, -70).as_bytes(),
            [0x8D, 0x1E, 0x02, 0x16, 0x01, 0x05, 0x01, 0x46, 0x02, 0x46, 0x34, 0xD8]
        );
    }

    #[test]
    fn stop_turns_both_sides_off() {
        let frame = raw_motors(0, 0, 0);
        assert_eq!(&frame.as_bytes()[6..10], [0, 0, 0, 0]);
    }

    #[test]
    fn duty_is_clamped() {
        let frame = raw_motors(0, 1000, -1000);
        assert_eq!(&frame.as_bytes()[6..10], [1, 255, 2, 255]);
    }

    #[test]
    fn reserved_bytes_are_escaped() {
        // 0xAB duty and a 0x8D sequence number both need escaping.
        let frame = raw_motors(0x8D, 0xAB, 0);
        let bytes = frame.as_bytes();
        assert_eq!(bytes[0], SOP);
        assert_eq!(*bytes.last().unwrap(), EOP);
        let inner = &bytes[1..bytes.len() - 1];
        assert!(!inner.contains(&SOP) && !inner.contains(&EOP));
        assert!(inner.windows(2).any(|w| w == [ESC, ESC_ESC]));
        assert!(inner.windows(2).any(|w| w == [ESC, ESC_SOP]));
    }

    #[test]
    fn decoder_round_trips_an_escaped_command() {
        let frame = raw_motors(0x8D, 0xAB, 0);
        let mut decoder = Decoder::new();
        let packets: std::vec::Vec<_> = frame
            .as_bytes()
            .iter()
            .filter_map(|b| decoder.push(*b))
            .collect();
        assert_eq!(packets.len(), 1);
        let packet = packets[0];
        assert!(!packet.is_response());
        assert_eq!((packet.did, packet.cid, packet.seq), (0x16, 0x01, 0x8D));
        assert_eq!(packet.data(), [1, 0xAB, 0, 0]);
    }

    #[test]
    fn decoder_parses_a_battery_response() {
        // flags: response | has_target | has_source; target 0x00, source 0x01.
        let mut body = std::vec![0x31, 0x00, 0x01, 0x13, 0x10, 0x07, 0x00, 84];
        let checksum = !body.iter().fold(0u8, |s, b| s.wrapping_add(*b));
        body.push(checksum);
        let mut decoder = Decoder::new();
        assert_eq!(decoder.push(SOP), None);
        let mut result = None;
        for byte in body.iter().copied().chain([EOP]) {
            result = decoder.push(byte).or(result);
        }
        let packet = result.expect("packet");
        assert!(packet.is_response());
        assert_eq!((packet.did, packet.cid, packet.seq), (0x13, 0x10, 0x07));
        assert_eq!(packet.error, 0);
        assert_eq!(packet.data(), [84]);
    }

    #[test]
    fn decoder_rejects_bad_checksum_and_resyncs() {
        let good = wake(3);
        let mut bad = [0u8; MAX_FRAME];
        let n = good.as_bytes().len();
        bad[..n].copy_from_slice(good.as_bytes());
        bad[n - 2] ^= 0x01;
        let mut decoder = Decoder::new();
        assert!(bad[..n].iter().all(|b| decoder.push(*b).is_none()));
        let parsed = good
            .as_bytes()
            .iter()
            .filter_map(|b| decoder.push(*b))
            .count();
        assert_eq!(parsed, 1);
    }
}
