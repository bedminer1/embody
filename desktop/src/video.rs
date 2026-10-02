pub struct Frame {
    pub pixels: Vec<u32>,
    pub width: usize,
    pub height: usize,
}

pub fn decode(jpeg: &[u8]) -> Option<Frame> {
    let mut decoder = jpeg_decoder::Decoder::new(jpeg);
    let source = decoder.decode().ok()?;
    let info = decoder.info()?;
    let width = info.width as usize;
    let height = info.height as usize;
    let mut pixels = vec![0; width * height];

    match info.pixel_format {
        jpeg_decoder::PixelFormat::RGB24 => {
            for (pixel, rgb) in pixels.iter_mut().zip(source.chunks_exact(3)) {
                *pixel = u32::from_be_bytes([0, rgb[0], rgb[1], rgb[2]]);
            }
        }
        jpeg_decoder::PixelFormat::L8 => {
            for (pixel, gray) in pixels.iter_mut().zip(source) {
                *pixel = u32::from_be_bytes([0, gray, gray, gray]);
            }
        }
        _ => return None,
    }

    Some(Frame {
        pixels,
        width,
        height,
    })
}
