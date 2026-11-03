use crate::protocol::Telemetry;
use crossterm::cursor::{Hide, MoveTo, Show};
use crossterm::event::{
    DisableFocusChange, EnableFocusChange, KeyboardEnhancementFlags, PopKeyboardEnhancementFlags,
    PushKeyboardEnhancementFlags,
};
use crossterm::style::{Color, Print, SetForegroundColor};
use crossterm::terminal::{self, Clear, ClearType, EnterAlternateScreen, LeaveAlternateScreen};
use crossterm::{execute, queue};
use std::io::{self, Stdout, Write};

const INNER_WIDTH: usize = 60;

pub struct Dashboard {
    out: Stdout,
    release_events: bool,
    previous: Option<View>,
}

#[derive(PartialEq, Eq)]
struct View {
    keys: String,
    tracks: [i16; 2],
    fixed_speed: Option<u8>,
    pivot_forward_percent: Option<u8>,
    status: String,
    pose: String,
    size: (u16, u16),
}

impl Dashboard {
    pub fn enter() -> io::Result<Self> {
        let mut out = io::stdout();
        execute!(out, EnterAlternateScreen, Hide)?;
        if let Err(error) = terminal::enable_raw_mode() {
            let _ = execute!(out, Show, LeaveAlternateScreen);
            return Err(error);
        }
        // Ask for key press/repeat/release events so several held keys work.
        // Terminals without the kitty keyboard protocol ignore this.
        let release_events = terminal::supports_keyboard_enhancement().unwrap_or(false);
        if release_events {
            let _ = execute!(
                out,
                PushKeyboardEnhancementFlags(
                    KeyboardEnhancementFlags::DISAMBIGUATE_ESCAPE_CODES
                        | KeyboardEnhancementFlags::REPORT_EVENT_TYPES
                )
            );
        }
        let _ = execute!(out, EnableFocusChange);
        Ok(Self {
            out,
            release_events,
            previous: None,
        })
    }

    /// Whether the terminal reports key releases (needed for reliable
    /// multi-key driving).
    pub fn release_events(&self) -> bool {
        self.release_events
    }

    pub fn draw(&mut self, keys: &str, telemetry: Option<&Telemetry>) -> io::Result<()> {
        let size = terminal::size()?;
        let (width, height) = size;
        let fixed_speed = telemetry.and_then(|data| data.speed_limit);
        let pivot_forward_percent = telemetry.and_then(|data| data.pivot_forward_percent);
        let tracks = telemetry.map_or([0, 0], |data| data.tracks);
        let motor_status = match telemetry {
            Some(data) if data.error.is_some() => "Motor error · check rover agent",
            Some(data) if data.rvr_connected => "Motor link ready",
            Some(_) => "Camera only · rover not connected",
            None => "Connecting...",
        };
        let pose = match telemetry.and_then(|data| data.pose) {
            Some(pose) => format!(
                "Pose  x {:+6.2} m   y {:+6.2} m   yaw {:+5.0}°     R reset",
                pose.x, pose.y, pose.yaw
            ),
            None => "Pose  unavailable".to_string(),
        };
        let view = View {
            keys: keys.to_string(),
            pose: pose.clone(),
            tracks,
            fixed_speed,
            pivot_forward_percent,
            status: motor_status.to_string(),
            size,
        };
        if self.previous.as_ref() == Some(&view) {
            return Ok(());
        }
        self.previous = Some(view);
        queue!(self.out, MoveTo(0, 0), Clear(ClearType::All))?;
        if width < 64 || height < 13 {
            queue!(self.out, Print("Embody needs a terminal at least 64 x 13."))?;
            return self.out.flush();
        }

        let fixed_speed = fixed_speed.map_or("--".to_string(), |speed| speed.to_string());
        let pivot_forward_percent =
            pivot_forward_percent.map_or("--".to_string(), |percent| percent.to_string());

        self.border(0, '╭', '╮')?;
        self.line(
            1,
            "EMBODY                                      DRIVE CONSOLE",
            Color::Cyan,
        )?;
        self.border(2, '├', '┤')?;
        self.line(3, &pose, Color::DarkGrey)?;
        self.line(
            4,
            &format!(
                "           {}                 Fixed speed  {:>3} / 255",
                key_label('w', keys),
                fixed_speed
            ),
            Color::White,
        )?;
        self.line(
            5,
            &format!(
                "      {}  {}  {}            Space  STOP",
                key_label('a', keys),
                key_label('s', keys),
                key_label('d', keys)
            ),
            Color::White,
        )?;
        self.line(
            6,
            &format!("      A/D pivot balance: forward track {pivot_forward_percent}%"),
            Color::DarkGrey,
        )?;
        self.line(7, &track_row("LEFT ", tracks[0]), Color::Green)?;
        self.line(8, &track_row("RIGHT", tracks[1]), Color::Green)?;
        if self.release_events {
            self.line(9, "", Color::White)?;
        } else {
            self.line(
                9,
                "No key-release events: use Ghostty/kitty/WezTerm/iTerm2 to hold several keys",
                Color::Yellow,
            )?;
        }
        self.line(10, &format!("Motion  {:<30}", motion(keys)), Color::White)?;
        self.line(
            11,
            &format!("{}   ·   Esc / X quit", motor_status),
            Color::DarkGrey,
        )?;
        self.border(12, '╰', '╯')?;
        self.out.flush()
    }

    fn line(&mut self, y: u16, text: &str, color: Color) -> io::Result<()> {
        let count = text.chars().count();
        let padding = INNER_WIDTH.saturating_sub(count);
        queue!(
            self.out,
            MoveTo(0, y),
            SetForegroundColor(Color::DarkGrey),
            Print("│ "),
            SetForegroundColor(color),
            Print(text),
            Print(" ".repeat(padding)),
            SetForegroundColor(Color::DarkGrey),
            Print(" │")
        )
    }

    fn border(&mut self, y: u16, left: char, right: char) -> io::Result<()> {
        queue!(
            self.out,
            MoveTo(0, y),
            SetForegroundColor(Color::DarkGrey),
            Print(left),
            Print("─".repeat(INNER_WIDTH + 2)),
            Print(right)
        )
    }
}

impl Drop for Dashboard {
    fn drop(&mut self) {
        let _ = execute!(self.out, DisableFocusChange);
        if self.release_events {
            let _ = execute!(self.out, PopKeyboardEnhancementFlags);
        }
        let _ = terminal::disable_raw_mode();
        let _ = execute!(
            self.out,
            SetForegroundColor(Color::Reset),
            Show,
            LeaveAlternateScreen
        );
    }
}

fn key_label(key: char, pressed: &str) -> String {
    if pressed.contains(key) {
        format!("<{}>", key.to_ascii_uppercase())
    } else {
        format!("[{key}]")
    }
}

fn track_row(label: &str, speed: i16) -> String {
    let magnitude = speed.unsigned_abs().min(255) as usize;
    let filled = magnitude * 18 / 255;
    let bar = format!("{}{}", "█".repeat(filled), "·".repeat(18 - filled));
    let direction = match speed.cmp(&0) {
        std::cmp::Ordering::Greater => "forward",
        std::cmp::Ordering::Less => "reverse",
        std::cmp::Ordering::Equal => "stopped",
    };
    format!("{label}  {bar}  {magnitude:>3}/255  {direction}")
}

fn motion(keys: &str) -> &'static str {
    let drive = i8::from(keys.contains('w')) - i8::from(keys.contains('s'));
    let turn = i8::from(keys.contains('d')) - i8::from(keys.contains('a'));
    match (drive, turn) {
        (1, -1) => "Forward · left curve",
        (1, 1) => "Forward · right curve",
        (-1, -1) => "Reverse · left curve",
        (-1, 1) => "Reverse · right curve",
        (1, _) => "Forward",
        (-1, _) => "Reverse",
        (0, -1) => "Pivot left",
        (0, 1) => "Pivot right",
        _ => "Stopped",
    }
}
