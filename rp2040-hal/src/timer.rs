//! Timer Peripheral
// See [Chapter 4 Section 6](https://datasheets.raspberrypi.org/rp2040/rp2040_datasheet.pdf) for more details

use crate::pac::TIMER;

/// Timer peripheral
pub struct Timer {
    timer: TIMER,
}

impl Timer {
    /// Create a new [`Timer`]
    pub fn new(timer: TIMER) -> Self {
        Self { timer }
    }

    /// Get the current counter value.
    pub fn get_counter(&self) -> u64 {
        // latched read, low before high
        let lo = self.timer.timelr.read().bits();
        let hi = self.timer.timehr.read().bits();
        (hi as u64) << 32 | lo as u64
    }
}
