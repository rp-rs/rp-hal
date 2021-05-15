//! Watchdog
// See [Chapter 4 Section 7](https://datasheets.raspberrypi.org/rp2040/rp2040_datasheet.pdf) for more details
use embedded_hal::watchdog;
use embedded_time::{
    duration,
    fixed_point::FixedPoint,
};
use crate::pac::WATCHDOG;


//const MAX_LOAD: u32 = 0x7FFFFF;


pub struct Watchdog {
    watchdog: WATCHDOG,
    delay_ms: u32,
}

impl Watchdog {
    pub fn new(watchdog: WATCHDOG) -> Self {
        Self {
            watchdog,
            delay_ms: 0,
        }
    }

    pub fn enable_tick_generation(&mut self, cycles: u8) {
        const WATCHDOG_TICK_ENABLE_BITS: u32 = 0x200;

        self.watchdog
            .tick
            .write(|w| unsafe { w.bits(WATCHDOG_TICK_ENABLE_BITS | cycles as u32)})
    }

    fn load_counter(&self, counter: u32) {
        self.watchdog
            .load
            .write(|w| unsafe { w.bits(counter)});
    }

    fn enable(&self) {
        self.watchdog
            .ctrl
            .write(|w| w.enable().set_bit())
    }
}

impl watchdog::Watchdog for Watchdog {
    fn feed(&mut self) {
        self.load_counter(self.delay_ms)
    }
}

impl watchdog::WatchdogEnable for Watchdog {
    type Time = duration::Microseconds;

    /// Due to a logic error, the watchdog decrements by 2 and
    /// value must be compensated; see RP2040-E1
    fn start<T: Into<Self::Time>>(&mut self, period: T) {
        let delay_ms = period
            .into()
            .integer() * 2;

        self.load_counter(delay_ms);
        self.enable();
    }
}

impl watchdog::WatchdogDisable for Watchdog {
    fn disable(&mut self) {
        self.watchdog
            .ctrl
            .write(|w| w.enable().clear_bit())
    }
}
