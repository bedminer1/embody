//! WASD to track duty cycles. Mirrors `Rover.tracks` in `pi/agent.py`.

pub const W: u8 = 1;
pub const A: u8 = 2;
pub const S: u8 = 4;
pub const D: u8 = 8;

/// Bit mask of the W/A/S/D keys in `text`, ignoring case and anything else.
pub fn key_mask(text: &[u8]) -> u8 {
    text.iter().fold(0, |mask, key| {
        mask | match key.to_ascii_lowercase() {
            b'w' => W,
            b'a' => A,
            b's' => S,
            b'd' => D,
            _ => 0,
        }
    })
}

/// `(left, right)` signed duty for the held keys. `speed` is the full-speed
/// duty and `pivot_forward_percent` how much the inner track keeps driving
/// forward while pivoting in place.
pub fn tracks(keys: u8, speed: i16, pivot_forward_percent: i16) -> (i16, i16) {
    let held = |key| i16::from(keys & key != 0);
    let forward = held(W) - held(S);
    let turn = held(D) - held(A);
    if forward == 0 {
        let inner = speed * pivot_forward_percent / 100;
        return match turn {
            t if t < 0 => (-speed, inner),
            t if t > 0 => (inner, -speed),
            _ => (0, 0),
        };
    }
    let (mut left, mut right) = (forward * speed, forward * speed);
    if (forward > 0 && turn < 0) || (forward < 0 && turn > 0) {
        left = left * 45 / 100;
    } else if turn != 0 {
        right = right * 45 / 100;
    }
    (left, right)
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn parses_keys_case_insensitively() {
        assert_eq!(key_mask(b"wD"), W | D);
        assert_eq!(key_mask(b"xyz"), 0);
    }

    #[test]
    fn matches_python_agent() {
        assert_eq!(tracks(0, 70, 60), (0, 0));
        assert_eq!(tracks(W, 70, 60), (70, 70));
        assert_eq!(tracks(S, 70, 60), (-70, -70));
        assert_eq!(tracks(A, 70, 60), (-70, 42));
        assert_eq!(tracks(D, 70, 60), (42, -70));
        // int(70 * 0.45) == 31, int(-70 * 0.45) == -31
        assert_eq!(tracks(W | A, 70, 60), (31, 70));
        assert_eq!(tracks(W | D, 70, 60), (70, 31));
        assert_eq!(tracks(S | A, 70, 60), (-70, -31));
        assert_eq!(tracks(S | D, 70, 60), (-31, -70));
        assert_eq!(tracks(W | S, 70, 60), (0, 0));
        assert_eq!(tracks(A | D, 70, 60), (0, 0));
    }

    #[test]
    fn opposing_keys_cancel_with_three_or_four_held() {
        // A and D cancel, so W+A+D drives straight and S+A+D reverses straight.
        assert_eq!(tracks(W | A | D, 70, 60), (70, 70));
        assert_eq!(tracks(S | A | D, 70, 60), (-70, -70));
        // W and S cancel, leaving a pivot.
        assert_eq!(tracks(W | S | A, 70, 60), (-70, 42));
        assert_eq!(tracks(W | S | D, 70, 60), (42, -70));
        assert_eq!(tracks(W | A | S | D, 70, 60), (0, 0));
    }

    #[test]
    fn every_combination_is_in_range_and_mirrors_left_right() {
        let mirror =
            |keys: u8| (keys & (W | S)) | ((keys & A != 0) as u8 * D) | ((keys & D != 0) as u8 * A);
        for keys in 0..16u8 {
            let (left, right) = tracks(keys, 70, 60);
            assert!(left.abs() <= 70 && right.abs() <= 70, "keys {keys:04b}");
            // Pivoting is deliberately asymmetric (the inner track keeps a
            // forward bias), so only compare arcs and straight lines.
            let pivoting = keys & (W | S) == 0 || (keys & (W | S)) == (W | S);
            if !pivoting {
                assert_eq!(
                    tracks(mirror(keys), 70, 60),
                    (right, left),
                    "keys {keys:04b}"
                );
            }
        }
    }
}
