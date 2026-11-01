use crossterm::event::{KeyCode, KeyEvent, KeyEventKind};
use std::collections::HashMap;
use std::time::{Duration, Instant};

const KEY_HOLD: Duration = Duration::from_millis(350);

/// Keys currently held. Any combination of W/A/S/D can be down at once.
///
/// Terminals only report key *releases* when they support the kitty keyboard
/// protocol. Without it, a held key is assumed released 350 ms after its last
/// press/repeat event, and since the OS repeats only the most recently
/// pressed key, other held keys drop out. With release events every key
/// stays held until its own release.
#[derive(Default)]
pub struct Controls {
    pressed: HashMap<char, Instant>,
    release_events: bool,
}

pub enum Action {
    Continue,
    Stop,
    Quit,
}

impl Controls {
    pub fn new(release_events: bool) -> Self {
        Self {
            pressed: HashMap::new(),
            release_events,
        }
    }

    /// Forget every held key, e.g. when the terminal loses focus and the
    /// release events will never arrive.
    pub fn release_all(&mut self) {
        self.pressed.clear();
    }

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
        if !self.release_events {
            self.pressed.retain(|_, at| at.elapsed() < KEY_HOLD);
        }
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

#[cfg(test)]
mod tests {
    use super::*;
    use crossterm::event::KeyModifiers;

    fn key(code: char, kind: KeyEventKind) -> KeyEvent {
        let mut event = KeyEvent::new(KeyCode::Char(code), KeyModifiers::NONE);
        event.kind = kind;
        event
    }

    #[test]
    fn several_keys_stay_held_until_released() {
        let mut controls = Controls::new(true);
        controls.handle_key(key('w', KeyEventKind::Press));
        controls.handle_key(key('d', KeyEventKind::Press));
        assert_eq!(controls.keys(), "wd");
        // Only the last key repeats; the first must stay held regardless.
        controls.handle_key(key('d', KeyEventKind::Repeat));
        assert_eq!(controls.keys(), "wd");
        controls.handle_key(key('w', KeyEventKind::Release));
        assert_eq!(controls.keys(), "d");
        controls.handle_key(key('d', KeyEventKind::Release));
        assert_eq!(controls.keys(), "");
    }

    #[test]
    fn four_keys_and_uppercase() {
        let mut controls = Controls::new(true);
        for code in ['W', 'a', 'S', 'd'] {
            controls.handle_key(key(code, KeyEventKind::Press));
        }
        assert_eq!(controls.keys(), "wasd");
    }

    #[test]
    fn space_and_focus_loss_release_everything() {
        let mut controls = Controls::new(true);
        controls.handle_key(key('w', KeyEventKind::Press));
        controls.handle_key(key('a', KeyEventKind::Press));
        assert!(matches!(
            controls.handle_key(key(' ', KeyEventKind::Press)),
            Action::Stop
        ));
        assert_eq!(controls.keys(), "");
        controls.handle_key(key('w', KeyEventKind::Press));
        controls.release_all();
        assert_eq!(controls.keys(), "");
    }

    #[test]
    fn without_release_events_keys_time_out() {
        let mut controls = Controls::new(false);
        controls.handle_key(key('w', KeyEventKind::Press));
        assert_eq!(controls.keys(), "w");
        controls.pressed.insert('w', Instant::now() - KEY_HOLD * 2);
        assert_eq!(controls.keys(), "");
    }
}
