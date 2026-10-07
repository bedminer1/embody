use crossterm::event::{KeyCode, KeyEvent, KeyEventKind};
use std::collections::HashMap;
use std::time::{Duration, Instant};

const KEY_HOLD: Duration = Duration::from_millis(350);

#[derive(Default)]
pub struct Controls {
    pressed: HashMap<char, Instant>,
}

pub enum Action {
    Continue,
    Stop,
    Quit,
}

impl Controls {
    pub fn handle_key(&mut self, event: KeyEvent) -> Action {
        match event.code {
            KeyCode::Esc | KeyCode::Char('x') if event.kind != KeyEventKind::Release => {
                Action::Quit
            }
            KeyCode::Char(' ') if event.kind != KeyEventKind::Release => {
                self.pressed.clear();
                Action::Stop
            }
            KeyCode::Char(key) if "wasd".contains(key.to_ascii_lowercase()) => {
                let key = key.to_ascii_lowercase();
                if event.kind == KeyEventKind::Release {
                    self.pressed.remove(&key);
                } else {
                    self.pressed.insert(key, Instant::now());
                }
                Action::Continue
            }
            _ => Action::Continue,
        }
    }

    pub fn keys(&mut self) -> String {
        self.pressed.retain(|_, at| at.elapsed() < KEY_HOLD);
        "wasd"
            .chars()
            .filter(|key| self.pressed.contains_key(key))
            .collect()
    }
}

pub fn normalize_keys(raw: &str) -> String {
    let raw = raw.to_ascii_lowercase();
    "wasd".chars().filter(|key| raw.contains(*key)).collect()
}
