//! Closed-loop driving skills on the RVR's locator pose.
//!
//! The rover streams its own wheel/IMU odometry (x, y in metres, yaw in
//! degrees counter-clockwise; +y is the heading at the last pose reset). A
//! skill turns each pose into a velocity command (m/s, degrees/s) until it
//! succeeds, fails or times out, so a planner asks for "drive 3 m" once
//! instead of inching forward and re-checking the camera.
//!
//! There is no obstacle sensing yet: skills stop on lack of progress (a wall
//! or wheel slip) and on timeouts, never on what lies ahead.

use crate::protocol::Pose;
use serde::Serialize;

/// Fastest a skill will drive, m/s.
pub const MAX_SPEED: f64 = 0.6;
pub const DEFAULT_SPEED: f64 = 0.3;
/// Fastest a skill will turn, degrees/s.
const MAX_YAW_RATE: f64 = 90.0;
/// Slowest useful commands; below these the tracks may not move at all.
const MIN_SPEED: f64 = 0.06;
const MIN_YAW_RATE: f64 = 12.0;
/// Proportional gains: 1/s on distance, 1/s on heading error.
const LINEAR_GAIN: f64 = 1.2;
const HEADING_GAIN: f64 = 2.5;
/// Arrival tolerances.
const POSITION_TOLERANCE: f64 = 0.025;
const YAW_TOLERANCE: f64 = 2.0;
/// The yaw must stay within tolerance this long to count as settled.
const SETTLE_TIME: f64 = 0.3;
/// Turn in place first when the goal is further off the nose than this.
const ROTATE_FIRST: f64 = 30.0;
/// Cross-track correction lookahead for straight drives, m.
const LOOKAHEAD: f64 = 0.4;
/// Fail if the remaining error has not shrunk by this much within
/// `STALL_TIME`: the rover is probably blocked or slipping.
const STALL_DISTANCE: f64 = 0.02;
const STALL_ANGLE: f64 = 2.0;
const STALL_TIME: f64 = 2.5;

#[derive(Clone, Debug, PartialEq, Serialize)]
#[serde(tag = "skill", rename_all = "snake_case")]
pub enum Skill {
    /// Straight line along the current heading; negative reverses.
    Drive { distance_m: f64, speed_mps: f64 },
    /// Relative turn in place, degrees counter-clockwise (left) positive.
    Turn { degrees: f64 },
    /// Turn in place to an absolute yaw, degrees.
    TurnTo { yaw_deg: f64 },
    /// Drive to a point in the locator frame, optionally ending at a yaw.
    GoTo {
        x: f64,
        y: f64,
        yaw_deg: Option<f64>,
        speed_mps: f64,
    },
}

/// Output of one control step.
#[derive(Clone, Debug, PartialEq)]
pub enum Step {
    /// Keep going with this command (m/s, degrees/s).
    Drive { linear: f64, yaw_rate: f64 },
    /// Finished; the caller must command a stop.
    Done(Result<String, String>),
}

/// Wrap an angle in degrees to (-180, 180].
pub fn wrap(degrees: f64) -> f64 {
    let wrapped = degrees.rem_euclid(360.0);
    if wrapped > 180.0 {
        wrapped - 360.0
    } else {
        wrapped
    }
}

/// Unit vector along a yaw: yaw 0 faces +y, positive yaw turns toward -x.
fn heading(yaw: f64) -> (f64, f64) {
    let radians = yaw.to_radians();
    (-radians.sin(), radians.cos())
}

/// Yaw that faces along (dx, dy).
fn bearing(dx: f64, dy: f64) -> f64 {
    (-dx).atan2(dy).to_degrees()
}

fn turn_rate(error: f64) -> f64 {
    let rate = (HEADING_GAIN * error).clamp(-MAX_YAW_RATE, MAX_YAW_RATE);
    if rate.abs() < MIN_YAW_RATE {
        MIN_YAW_RATE.copysign(error)
    } else {
        rate
    }
}

fn approach_speed(remaining: f64, limit: f64) -> f64 {
    (LINEAR_GAIN * remaining).clamp(MIN_SPEED, limit)
}

/// Watches an error that should keep shrinking.
struct Progress {
    best: f64,
    since: f64,
    threshold: f64,
}

impl Progress {
    fn new(error: f64, now: f64, threshold: f64) -> Self {
        Self {
            best: error,
            since: now,
            threshold,
        }
    }

    /// False once the error has not improved for `STALL_TIME`.
    fn advancing(&mut self, error: f64, now: f64) -> bool {
        if error < self.best - self.threshold {
            self.best = error;
            self.since = now;
        }
        now - self.since < STALL_TIME
    }
}

enum Phase {
    /// Drive along a fixed line from `origin` at yaw `line_yaw`.
    Line {
        origin: (f64, f64),
        line_yaw: f64,
        distance: f64,
    },
    /// Rotate in place, then drive, toward a point. `approached` is set
    /// once the rover has driven toward it, so a goal that ends up behind
    /// counts as overshot rather than as one still to turn toward.
    Point {
        target: (f64, f64),
        approached: bool,
    },
    /// Rotate in place to a yaw.
    Rotate {
        yaw: f64,
        settled_since: Option<f64>,
    },
}

pub struct Executor {
    skill: Skill,
    phase: Phase,
    /// Yaw to finish at after reaching a point, if any.
    final_yaw: Option<f64>,
    speed: f64,
    started: f64,
    deadline: f64,
    progress: Progress,
}

impl Executor {
    /// Start `skill` from `pose` at time `now` (seconds, any epoch).
    pub fn new(skill: Skill, pose: Pose, now: f64) -> Result<Self, String> {
        let check = |value: f64, name: &str| {
            if value.is_finite() {
                Ok(value)
            } else {
                Err(format!("{name} must be a finite number"))
            }
        };
        let (phase, final_yaw, speed, error, budget) = match skill {
            Skill::Drive {
                distance_m,
                speed_mps,
            } => {
                let distance = check(distance_m, "distance_m")?;
                if distance.abs() > 10.0 {
                    return Err("distance_m is limited to ±10 m per skill".into());
                }
                let speed = check(speed_mps, "speed_mps")?.abs().clamp(MIN_SPEED, MAX_SPEED);
                let phase = Phase::Line {
                    origin: (pose.x, pose.y),
                    line_yaw: pose.yaw,
                    distance,
                };
                (phase, None, speed, distance.abs(), distance.abs() / speed)
            }
            Skill::Turn { degrees } => {
                let degrees = check(degrees, "degrees")?;
                if degrees.abs() > 720.0 {
                    return Err("degrees is limited to ±720".into());
                }
                Self::rotate(pose.yaw + degrees, degrees.abs())
            }
            Skill::TurnTo { yaw_deg } => {
                let yaw = check(yaw_deg, "yaw_deg")?;
                Self::rotate(yaw, wrap(yaw - pose.yaw).abs())
            }
            Skill::GoTo {
                x,
                y,
                yaw_deg,
                speed_mps,
            } => {
                let target = (check(x, "x")?, check(y, "y")?);
                let final_yaw = yaw_deg.map(|yaw| check(yaw, "yaw_deg")).transpose()?;
                let speed = check(speed_mps, "speed_mps")?.abs().clamp(MIN_SPEED, MAX_SPEED);
                let distance = (target.0 - pose.x).hypot(target.1 - pose.y);
                if distance > 20.0 {
                    return Err("go_to targets are limited to 20 m away".into());
                }
                // Room for the initial and final turns.
                let budget = distance / speed + 360.0 / MAX_YAW_RATE;
                let phase = Phase::Point {
                    target,
                    approached: false,
                };
                (phase, final_yaw, speed, distance, budget)
            }
        };
        let threshold = match phase {
            Phase::Rotate { .. } => STALL_ANGLE,
            _ => STALL_DISTANCE,
        };
        Ok(Self {
            skill,
            phase,
            final_yaw,
            speed,
            started: now,
            deadline: now + 2.0 * budget + 5.0,
            progress: Progress::new(error, now, threshold),
        })
    }

    fn rotate(yaw: f64, angle: f64) -> (Phase, Option<f64>, f64, f64, f64) {
        let phase = Phase::Rotate {
            yaw: wrap(yaw),
            settled_since: None,
        };
        // Multi-turn requests wrap to the nearest equivalent heading.
        (phase, None, 0.0, wrap(angle).abs(), angle / (MAX_YAW_RATE / 2.0))
    }

    pub fn skill(&self) -> &Skill {
        &self.skill
    }

    pub fn elapsed(&self, now: f64) -> f64 {
        now - self.started
    }

    /// One control step from the latest pose.
    pub fn step(&mut self, pose: Pose, now: f64) -> Step {
        if now > self.deadline {
            return Step::Done(Err(format!(
                "timed out after {:.1} s",
                self.elapsed(now)
            )));
        }
        match self.phase {
            Phase::Line {
                origin,
                line_yaw,
                distance,
            } => {
                let along = heading(line_yaw);
                let (dx, dy) = (pose.x - origin.0, pose.y - origin.1);
                let travelled = dx * along.0 + dy * along.1;
                // Positive when the rover is right of the line.
                let cross = dx * along.1 - dy * along.0;
                let direction = distance.signum();
                let remaining = distance.abs() - travelled * direction;
                if remaining <= POSITION_TOLERANCE {
                    return Step::Done(Ok(format!("drove {travelled:.2} m")));
                }
                if !self.progress.advancing(remaining, now) {
                    return Step::Done(Err(format!(
                        "no progress after {travelled:.2} m: blocked or slipping"
                    )));
                }
                // Steer back onto the line; reversing flips the correction.
                let target_yaw =
                    line_yaw + direction * (cross / LOOKAHEAD).atan().to_degrees();
                let error = wrap(target_yaw - pose.yaw);
                Step::Drive {
                    linear: direction * approach_speed(remaining, self.speed),
                    yaw_rate: (HEADING_GAIN * error).clamp(-MAX_YAW_RATE, MAX_YAW_RATE),
                }
            }
            Phase::Point {
                target,
                ref mut approached,
            } => {
                let (dx, dy) = (target.0 - pose.x, target.1 - pose.y);
                let distance = dx.hypot(dy);
                let error = wrap(bearing(dx, dy) - pose.yaw);
                // Close enough, or just overshot (the goal is now behind).
                if distance <= POSITION_TOLERANCE
                    || (*approached && distance < 2.0 * POSITION_TOLERANCE && error.abs() > 90.0)
                {
                    return match self.final_yaw {
                        Some(yaw) => {
                            self.phase = Phase::Rotate {
                                yaw: wrap(yaw),
                                settled_since: None,
                            };
                            self.progress =
                                Progress::new(wrap(yaw - pose.yaw).abs(), now, STALL_ANGLE);
                            self.step(pose, now)
                        }
                        None => Step::Done(Ok(format!("arrived within {distance:.2} m"))),
                    };
                }
                if !self.progress.advancing(distance, now) {
                    return Step::Done(Err(format!(
                        "no progress with {distance:.2} m to go: blocked or slipping"
                    )));
                }
                if error.abs() > ROTATE_FIRST {
                    // Rotating does not shrink the distance; do not count it
                    // as a stall.
                    self.progress.since = now;
                    return Step::Drive {
                        linear: 0.0,
                        yaw_rate: turn_rate(error),
                    };
                }
                *approached = true;
                Step::Drive {
                    linear: approach_speed(distance, self.speed) * error.to_radians().cos(),
                    yaw_rate: (HEADING_GAIN * error).clamp(-MAX_YAW_RATE, MAX_YAW_RATE),
                }
            }
            Phase::Rotate {
                yaw,
                ref mut settled_since,
            } => {
                let error = wrap(yaw - pose.yaw);
                if error.abs() <= YAW_TOLERANCE {
                    let since = *settled_since.get_or_insert(now);
                    if now - since >= SETTLE_TIME {
                        return Step::Done(Ok(format!("facing yaw {:.0}°", pose.yaw)));
                    }
                    return Step::Drive {
                        linear: 0.0,
                        yaw_rate: 0.0,
                    };
                }
                *settled_since = None;
                if !self.progress.advancing(error.abs(), now) {
                    return Step::Done(Err(format!(
                        "turn stalled {error:.0}° from target: blocked or slipping"
                    )));
                }
                Step::Drive {
                    linear: 0.0,
                    yaw_rate: turn_rate(error),
                }
            }
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    /// Ideal unicycle: the command is followed exactly.
    fn integrate(pose: &mut Pose, linear: f64, yaw_rate: f64, dt: f64) {
        pose.yaw = wrap(pose.yaw + yaw_rate * dt);
        let (hx, hy) = heading(pose.yaw);
        pose.x += hx * linear * dt;
        pose.y += hy * linear * dt;
    }

    /// Run a skill against the simulator at 20 Hz. `slip` scales the
    /// achieved velocity (0 = wheels spinning against a wall).
    fn run(skill: Skill, mut pose: Pose, slip: f64) -> (Pose, Result<String, String>, f64) {
        let dt = 0.05;
        let mut now = 0.0;
        let mut executor = Executor::new(skill, pose, now).unwrap();
        loop {
            match executor.step(pose, now) {
                Step::Drive { linear, yaw_rate } => {
                    assert!(linear.abs() <= MAX_SPEED + 1e-9);
                    assert!(yaw_rate.abs() <= MAX_YAW_RATE + 1e-9);
                    integrate(&mut pose, linear * slip, yaw_rate * slip, dt);
                }
                Step::Done(result) => return (pose, result, now),
            }
            now += dt;
            assert!(now < 120.0, "skill never finished");
        }
    }

    const ORIGIN: Pose = Pose {
        x: 0.0,
        y: 0.0,
        yaw: 0.0,
    };

    #[test]
    fn heading_convention_matches_the_rvr() {
        // Yaw 0 drives +y; yaw -90 (clockwise) drives +x, as in Sphero's
        // drive_to_position_si square example.
        let (x, y) = heading(0.0);
        assert!(x.abs() < 1e-9 && (y - 1.0).abs() < 1e-9);
        let (x, y) = heading(-90.0);
        assert!((x - 1.0).abs() < 1e-9 && y.abs() < 1e-9);
        assert!((bearing(1.0, 0.0) + 90.0).abs() < 1e-9);
        assert!((bearing(-1.0, 0.0) - 90.0).abs() < 1e-9);
    }

    #[test]
    fn wrap_stays_in_range() {
        assert_eq!(wrap(190.0), -170.0);
        assert_eq!(wrap(-190.0), 170.0);
        assert_eq!(wrap(180.0), 180.0);
        assert_eq!(wrap(720.0), 0.0);
    }

    #[test]
    fn drives_three_metres_in_one_skill() {
        let start = Pose {
            x: 1.0,
            y: 2.0,
            yaw: 45.0,
        };
        let skill = Skill::Drive {
            distance_m: 3.0,
            speed_mps: 0.3,
        };
        let (pose, result, time) = run(skill, start, 1.0);
        assert!(result.is_ok(), "{result:?}");
        let (hx, hy) = heading(45.0);
        let expected = (1.0 + 3.0 * hx, 2.0 + 3.0 * hy);
        assert!((pose.x - expected.0).hypot(pose.y - expected.1) < 0.03, "{pose:?}");
        assert!(time < 15.0, "took {time} s");
    }

    #[test]
    fn reverses_and_corrects_cross_track_drift() {
        // Start 10 cm right of the line and 10° off.
        let start = Pose {
            x: 0.1,
            y: 0.0,
            yaw: 10.0,
        };
        let mut executor = Executor::new(
            Skill::Drive {
                distance_m: -1.0,
                speed_mps: 0.3,
            },
            ORIGIN,
            0.0,
        )
        .unwrap();
        let mut pose = start;
        let mut now = 0.0;
        while let Step::Drive { linear, yaw_rate } = executor.step(pose, now) {
            assert!(linear <= 0.0);
            integrate(&mut pose, linear, yaw_rate, 0.05);
            now += 0.05;
        }
        assert!(pose.x.abs() < 0.03, "cross-track {pose:?}");
        assert!((pose.y + 1.0).abs() < 0.06, "{pose:?}");
    }

    #[test]
    fn turns_left_and_right_through_the_wrap() {
        let start = Pose {
            yaw: 170.0,
            ..ORIGIN
        };
        let (pose, result, _) = run(Skill::Turn { degrees: 30.0 }, start, 1.0);
        assert!(result.is_ok(), "{result:?}");
        assert!(wrap(pose.yaw + 160.0).abs() <= YAW_TOLERANCE, "{pose:?}");
        assert!(pose.x.abs() < 1e-9 && pose.y.abs() < 1e-9);

        let (pose, result, _) = run(Skill::TurnTo { yaw_deg: -90.0 }, ORIGIN, 1.0);
        assert!(result.is_ok(), "{result:?}");
        assert!(wrap(pose.yaw + 90.0).abs() <= YAW_TOLERANCE);
    }

    #[test]
    fn goes_to_a_point_behind_and_faces_a_yaw() {
        let skill = Skill::GoTo {
            x: -1.0,
            y: -2.0,
            yaw_deg: Some(90.0),
            speed_mps: 0.4,
        };
        let (pose, result, _) = run(skill, ORIGIN, 1.0);
        assert!(result.is_ok(), "{result:?}");
        assert!((pose.x + 1.0).hypot(pose.y + 2.0) <= 2.0 * POSITION_TOLERANCE, "{pose:?}");
        assert!(wrap(pose.yaw - 90.0).abs() <= YAW_TOLERANCE, "{pose:?}");
    }

    #[test]
    fn a_nearby_goal_behind_is_not_mistaken_for_an_overshoot() {
        let start = Pose {
            x: 0.02,
            y: 1.41,
            yaw: -165.0,
        };
        let skill = Skill::GoTo {
            x: 0.0,
            y: 1.46,
            yaw_deg: None,
            speed_mps: 0.5,
        };
        let (pose, result, _) = run(skill, start, 1.0);
        assert!(result.is_ok(), "{result:?}");
        assert!(pose.x.hypot(pose.y - 1.46) <= POSITION_TOLERANCE, "{pose:?}");
    }

    #[test]
    fn a_blocked_rover_fails_instead_of_pushing_forever() {
        let skill = Skill::Drive {
            distance_m: 1.0,
            speed_mps: 0.3,
        };
        let (_, result, time) = run(skill, ORIGIN, 0.0);
        let message = result.unwrap_err();
        assert!(message.contains("no progress"), "{message}");
        assert!(time <= STALL_TIME + 0.1);

        let (_, result, _) = run(Skill::Turn { degrees: 90.0 }, ORIGIN, 0.0);
        assert!(result.unwrap_err().contains("stalled"));
    }

    #[test]
    fn slow_progress_eventually_times_out() {
        let skill = Skill::Drive {
            distance_m: 2.0,
            speed_mps: 0.3,
        };
        let (_, result, _) = run(skill, ORIGIN, 0.5);
        assert!(result.is_ok(), "half speed still arrives: {result:?}");
        // 3 % of the commanded speed: progress keeps arriving, too slowly.
        let (_, result, _) = run(
            Skill::Drive {
                distance_m: 2.0,
                speed_mps: 0.3,
            },
            ORIGIN,
            0.03,
        );
        assert!(result.unwrap_err().contains("timed out"));
    }

    #[test]
    fn rejects_unreasonable_requests() {
        let too_far = Skill::Drive {
            distance_m: 50.0,
            speed_mps: 0.3,
        };
        assert!(Executor::new(too_far, ORIGIN, 0.0).is_err());
        let nan = Skill::Turn { degrees: f64::NAN };
        assert!(Executor::new(nan, ORIGIN, 0.0).is_err());
    }
}
