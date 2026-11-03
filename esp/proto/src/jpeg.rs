//! Cutting JPEG frames out of the camera's DMA stream.
//!
//! The OV2640 pads each frame after the end-of-image marker, and a capture
//! that started mid-frame has no start-of-image marker, so a frame is only
//! kept when both markers are present.

const SOI: [u8; 3] = [0xFF, 0xD8, 0xFF];
const EOI: [u8; 2] = [0xFF, 0xD9];

/// The complete JPEG inside one vsync-delimited capture, if any.
pub fn frame(capture: &[u8]) -> Option<&[u8]> {
    let start = capture.windows(SOI.len()).position(|w| w == SOI)?;
    let end = capture.windows(EOI.len()).rposition(|w| w == EOI)? + EOI.len();
    (end > start + SOI.len()).then(|| &capture[start..end])
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn trims_padding() {
        let capture = [0x00, 0xFF, 0xD8, 0xFF, 0xE0, 0x12, 0xFF, 0xD9, 0x00, 0x00];
        assert_eq!(frame(&capture), Some(&capture[1..8]));
    }

    #[test]
    fn rejects_partial_frames() {
        assert_eq!(frame(&[0x12, 0x34, 0xFF, 0xD9, 0x00]), None);
        assert_eq!(frame(&[0xFF, 0xD8, 0xFF, 0xE0, 0x12]), None);
        assert_eq!(frame(&[]), None);
    }
}
