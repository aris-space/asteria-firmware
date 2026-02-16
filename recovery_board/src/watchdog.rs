#![allow(clippy::collapsible_else_if)]
/* This entire thing is mostly copied from NIC. I don't know if that is the final implementation, as I
kinda want to have a watchdog that I do not need to pull but that really just takes ownership and goes
into the safety spiral.
 */

use embassy_time::{Duration, Instant};

#[derive(Copy, Clone)]
pub struct Watchdog {
    active: bool,
    last_seen: Instant,
    duration: Duration,
    expired: bool,
}

impl Watchdog {
    ///create new watchdog instance
    pub fn new(duration: Duration) -> Self {
        Watchdog {
            active: false,
            last_seen: Instant::now(),
            duration,
            expired: false,
        }
    }

    pub fn start(&mut self) {
        if !self.active {
            self.active = true;
            self.last_seen = Instant::now();
        }
    }

    pub fn check(&mut self) -> bool {
        if self.active {
            if self.expired {
                false
            } else {
                if Instant::now().duration_since(self.last_seen) > self.duration {
                    self.expired = true;
                    false
                } else {
                    true
                }
            }
        } else {
            true
        }
    }

    pub fn update(&mut self) {
        if self.active {
            self.last_seen = Instant::now();
        }
    }
}
