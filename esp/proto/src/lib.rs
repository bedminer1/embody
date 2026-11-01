//! Protocol pieces shared by the ESP32 firmware, kept `no_std` and free of
//! hardware dependencies so they can be unit-tested on the host:
//!
//! * [`sphero`]: Sphero RVR UART packet encoder/decoder.
//! * [`drive`]: WASD key mask to track duty cycles (mirrors `pi/agent.py`).
//! * [`stream`]: RVR sensor streaming (attitude, locator, velocity).
//! * [`wire`]: framing used between the desktop app and the rover agent.
#![cfg_attr(not(test), no_std)]

pub mod drive;
pub mod sphero;
pub mod stream;
pub mod wire;
