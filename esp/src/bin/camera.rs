//! OV2640 camera on the ESP32-S3's LCD_CAM (DVP) port, producing JPEG
//! frames for the desktop's `V` packets.
//!
//! The sensor is set up over SCCB (I2C) with Espressif's register tables
//! (`embody_proto::ov2640`), then the camera DMA streams its output into a
//! ring of descriptors. Each vsync closes a descriptor, which marks the end
//! of a frame; the JPEG inside is copied to [`LATEST`] for the TCP task.

use embassy_sync::blocking_mutex::raw::CriticalSectionRawMutex;
use embassy_sync::mutex::Mutex;
use embassy_time::{Duration, Instant, Timer};
use embody_proto::{jpeg, ov2640};
use esp_hal::Blocking;
use esp_hal::dma::DmaRxStreamBuf;
use esp_hal::i2c::master::I2c;
use esp_hal::lcd_cam::cam::Camera;
use log::{info, warn};
use static_cell::ConstStaticCell;

pub const WIDTH: u16 = 640;
pub const HEIGHT: u16 = 480;
/// OV2640 JPEG quantisation: 0 is best, 63 worst. 12 keeps VGA frames
/// around 15-30 KB.
const QUALITY: u8 = 12;
/// Larger frames are dropped. Lower `QUALITY` (higher number) if busy
/// scenes exceed it.
pub const MAX_FRAME: usize = 48 * 1024;
/// Publish at most this often, leaving Wi-Fi time for telemetry (~12 fps).
const FRAME_INTERVAL: Duration = Duration::from_millis(80);
/// How long the capture loop sleeps when no new data is waiting.
const POLL: Duration = Duration::from_millis(1);

/// Latest complete JPEG. The TCP task holds the lock while sending, and the
/// camera skips frames it cannot publish rather than waiting.
pub struct Frame {
    pub data: [u8; MAX_FRAME],
    pub len: usize,
    /// Incremented on every new frame.
    pub seq: u32,
}

pub static LATEST: Mutex<CriticalSectionRawMutex, Frame> = Mutex::new(Frame {
    data: [0; MAX_FRAME],
    len: 0,
    seq: 0,
});

/// SCCB access with the bank-select caching of the C driver.
struct Sccb {
    i2c: I2c<'static, Blocking>,
    bank: Option<u8>,
}

impl Sccb {
    fn write(&mut self, reg: u8, value: u8) -> Result<(), &'static str> {
        if reg == ov2640::BANK_SEL {
            if self.bank == Some(value) {
                return Ok(());
            }
            self.bank = Some(value);
        }
        self.i2c
            .write(ov2640::SCCB_ADDR, &[reg, value])
            .map_err(|_| "camera did not answer on SCCB")
    }

    /// SCCB has no repeated start, so the address write and the read are
    /// separate transactions.
    fn read(&mut self, reg: u8) -> Result<u8, &'static str> {
        let mut value = [0u8];
        self.i2c
            .write(ov2640::SCCB_ADDR, &[reg])
            .and_then(|()| self.i2c.read(ov2640::SCCB_ADDR, &mut value))
            .map_err(|_| "camera did not answer on SCCB")?;
        Ok(value[0])
    }

    fn write_all(&mut self, regs: &[(u8, u8)]) -> Result<(), &'static str> {
        regs.iter().try_for_each(|&(reg, value)| self.write(reg, value))
    }

    fn select(&mut self, bank: u8) -> Result<(), &'static str> {
        self.write(ov2640::BANK_SEL, bank)
    }

    /// Replace `mask << offset` in `reg` with `value`.
    fn set_bits(
        &mut self,
        bank: u8,
        reg: u8,
        offset: u8,
        mask: u8,
        value: u8,
    ) -> Result<(), &'static str> {
        self.select(bank)?;
        let current = self.read(reg)?;
        self.write(reg, (current & !(mask << offset)) | ((value & mask) << offset))
    }
}

/// The C driver's `esp_camera_init` sequence for an OV2640 in JPEG mode.
async fn init_sensor(sccb: &mut Sccb) -> Result<(), &'static str> {
    use ov2640::*;
    let settle = Duration::from_millis(10);

    sccb.select(BANK_SENSOR)?;
    if sccb.read(REG_PID)? != PID {
        return Err("camera is not an OV2640");
    }
    sccb.write(COM7, COM7_SRST)?;
    Timer::after(settle).await;
    sccb.write_all(INIT)?;

    let size = frame_size(WIDTH, HEIGHT).ok_or("unsupported camera frame size")?;
    sccb.select(BANK_DSP)?;
    sccb.write(R_BYPASS, R_BYPASS_DSP_BYPASS)?;
    sccb.write_all(size.mode)?;
    sccb.write_all(&size.window)?;
    sccb.select(BANK_SENSOR)?;
    sccb.write(CLKRC, size.clkrc)?;
    sccb.select(BANK_DSP)?;
    sccb.write(R_DVP_SP, size.dvp_sp)?;
    sccb.write(R_BYPASS, R_BYPASS_DSP_EN)?;
    Timer::after(settle).await;
    // Once as part of the size change and once as the pixel format, as in
    // the C driver.
    for _ in 0..2 {
        sccb.write_all(JPEG)?;
        Timer::after(settle).await;
    }

    sccb.set_bits(BANK_SENSOR, COM9, 5, 7, 0)?; // AGC gain ceiling 2x
    sccb.set_bits(BANK_DSP, CTRL3, 7, 1, 0)?; // black pixel correction off
    sccb.set_bits(BANK_DSP, CTRL3, 6, 1, 1)?; // white pixel correction on
    sccb.set_bits(BANK_DSP, CTRL1, 1, 1, 1)?; // lens correction on
    sccb.select(BANK_DSP)?;
    sccb.write(QS, QUALITY)
}

/// Keep the newest complete JPEG from one vsync-delimited capture.
fn publish(capture: &[u8]) {
    let Some(frame) = jpeg::frame(capture) else {
        return;
    };
    if let Ok(mut latest) = LATEST.try_lock() {
        latest.data[..frame.len()].copy_from_slice(frame);
        latest.len = frame.len();
        latest.seq = latest.seq.wrapping_add(1);
    }
}

/// Set up the sensor, then capture forever. Reports failures through
/// `report`, which feeds telemetry's `error` field.
#[embassy_executor::task]
pub async fn run(
    mut camera: Camera<'static>,
    i2c: I2c<'static, Blocking>,
    mut buf: DmaRxStreamBuf,
    report: fn(Option<&'static str>),
) {
    let mut sccb = Sccb { i2c, bank: None };
    if let Err(message) = init_sensor(&mut sccb).await {
        warn!("Camera setup failed: {message}");
        report(Some(message));
        return;
    }
    info!("OV2640 streaming {WIDTH}x{HEIGHT} JPEG, quality {QUALITY}");

    static CAPTURE: ConstStaticCell<[u8; MAX_FRAME]> = ConstStaticCell::new([0; MAX_FRAME]);
    let capture = CAPTURE.take();
    let mut published = Instant::now();
    loop {
        let mut transfer = match camera.receive(buf) {
            Ok(transfer) => transfer,
            Err((error, returned, returned_buf)) => {
                warn!("Camera DMA failed to start: {error:?}");
                report(Some("camera DMA failed to start"));
                (camera, buf) = (returned, returned_buf);
                Timer::after(Duration::from_secs(1)).await;
                continue;
            }
        };
        report(None);
        // Whether the frame being received will be kept. The first capture
        // after a (re)start begins mid-frame, so it never is.
        let mut wanted = false;
        let mut len = 0;
        let mut overflow = false;
        // The DMA stops when every descriptor is full, i.e. this loop fell
        // behind; restart it and resynchronise on the next vsync.
        while !transfer.is_done() {
            let (data, eof) = transfer.peek_until_eof();
            let n = data.len();
            if n == 0 {
                Timer::after(POLL).await;
                continue;
            }
            if wanted && !overflow {
                match capture.get_mut(len..len + n) {
                    Some(slot) => {
                        slot.copy_from_slice(data);
                        len += n;
                    }
                    None => overflow = true,
                }
            }
            transfer.consume(n);
            if eof {
                if wanted && !overflow {
                    publish(&capture[..len]);
                    published = Instant::now();
                } else if wanted {
                    warn!("Dropped a JPEG frame larger than {MAX_FRAME} bytes");
                }
                wanted = published.elapsed() >= FRAME_INTERVAL;
                len = 0;
                overflow = false;
            }
        }
        warn!("Camera DMA overran; restarting capture");
        (camera, buf) = transfer.stop();
    }
}
