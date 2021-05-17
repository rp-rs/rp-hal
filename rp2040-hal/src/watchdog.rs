//! Watchdog
// See [Chapter 4 Section 7](https://datasheets.raspberrypi.org/rp2040/rp2040_datasheet.pdf) for more details
use crate::pac::WATCHDOG;
use embedded_hal::watchdog;
use embedded_time::{duration, fixed_point::FixedPoint};

///
pub struct Watchdog {
    watchdog: WATCHDOG,
    delay_ms: u32,
}

impl Watchdog {
    ///
    pub fn new(watchdog: WATCHDOG) -> Self {
        Self {
            watchdog,
            delay_ms: 0,
        }
    }

    ///
    pub fn enable_tick_generation(&mut self, cycles: u8) {
        const WATCHDOG_TICK_ENABLE_BITS: u32 = 0x200;

        self.watchdog
            .tick
            .write(|w| unsafe { w.bits(WATCHDOG_TICK_ENABLE_BITS | cycles as u32) })
    }

    ///
    pub fn pause_on_debug(&mut self, pause: bool) {
        self.watchdog.ctrl.write(|w| {
            w.pause_dbg0()
                .bit(pause)
                .pause_dbg1()
                .bit(pause)
                .pause_jtag()
                .bit(pause)
        })
    }

    fn load_counter(&self, counter: u32) {
        self.watchdog.load.write(|w| unsafe { w.bits(counter) });
    }

    fn enable(&self, bit: bool) {
        self.watchdog.ctrl.write(|w| w.enable().bit(bit))
    }
}

impl watchdog::Watchdog for Watchdog {
    fn feed(&mut self) {
        self.load_counter(self.delay_ms)
    }
}

impl watchdog::WatchdogEnable for Watchdog {
    type Time = duration::Microseconds;

    fn start<T: Into<Self::Time>>(&mut self, period: T) {
        const MAX_PERIOD: u32 = 0x7FFFFF;

        // Due to a logic error, the watchdog decrements by 2 and
        // the load value must be compensated; see RP2040-E1
        self.delay_ms = period.into().integer() * 2;

        if self.delay_ms > MAX_PERIOD {
            panic!("Period cannot exceed maximum load value of {}", MAX_PERIOD);
        }

        self.enable(false);
        self.load_counter(self.delay_ms);
        self.enable(true);
    }
}

impl watchdog::WatchdogDisable for Watchdog {
    fn disable(&mut self) {
        self.enable(false)
    }
}
