//! Watchdog
//!
//! The watchdog is a countdown timer that can restart parts of the chip if it reaches zero. This can be used to restart the
//! processor if software gets stuck in an infinite loop. The programmer must periodically write a value to the watchdog to
//! stop it from reaching zero.
//!
//! See [Chapter 4 Section 7](https://datasheets.raspberrypi.org/rp2040/rp2040_datasheet.pdf) of the datasheet for more details
//!
//! ## Usage
//! ```no_run
//! use cortex_m::prelude::{_embedded_hal_watchdog_Watchdog, _embedded_hal_watchdog_WatchdogEnable};
//! use embedded_time::duration::units::*;
//! use rp2040_hal::{clocks::init_clocks_and_plls, pac, watchdog::Watchdog};
//! let mut pac = pac::Peripherals::take().unwrap();
//! let mut watchdog = Watchdog::new(pac.WATCHDOG);
//! let _clocks = init_clocks_and_plls(
//!     12_000_000,
//!     pac.XOSC,
//!     pac.CLOCKS,
//!     pac.PLL_SYS,
//!     pac.PLL_USB,
//!     &mut pac.RESETS,
//!     &mut watchdog,
//! ).ok().unwrap();
//! // Set to watchdog to reset if it's not reloaded within 1.05 seconds, and start it
//! watchdog.start(1_050_000.microseconds());
//! // Feed the watchdog once per cycle to avoid reset
//! for _ in 1..=10000 {
//!     cortex_m::asm::delay(100_000);
//!     watchdog.feed();
//! }
//! // Stop feeding, now we'll reset
//! loop {}
//! ```
//! See [examples/watchdog.rs](https://github.com/rp-rs/rp-hal/tree/main/rp2040-hal/examples/watchdog.rs) for a more complete example

use crate::pac::WATCHDOG;
use embedded_hal::watchdog;
use embedded_time::{duration, fixed_point::FixedPoint};

/// Watchdog peripheral
pub struct Watchdog {
    watchdog: WATCHDOG,
    delay_ms: u32,
}

impl Watchdog {
    /// Create a new [`Watchdog`]
    pub fn new(watchdog: WATCHDOG) -> Self {
        Self {
            watchdog,
            delay_ms: 0,
        }
    }

    /// Starts tick generation on clk_tick which is driven from clk_ref.
    ///
    /// # Arguments
    ///
    /// * `cycles` - Total number of tick cycles before the next tick is generated.
    pub fn enable_tick_generation(&mut self, cycles: u8) {
        const WATCHDOG_TICK_ENABLE_BITS: u32 = 0x200;

        self.watchdog
            .tick
            .write(|w| unsafe { w.bits(WATCHDOG_TICK_ENABLE_BITS | cycles as u32) })
    }

    /// Defines whether or not the watchdog timer should be paused when processor(s) are in debug mode
    /// or when JTAG is accessing bus fabric
    ///
    /// # Arguments
    ///
    /// * `pause` - If true, watchdog timer will be paused
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

    /// Configure which hardware will be reset by the watchdog
    /// the default is everything except ROSC, XOSC
    ///
    /// Safety: ensure no other device is writing to psm.wdsel
    /// This is easy at the moment, since nothing else uses PSM
    unsafe fn configure_wdog_reset_triggers(&self) {
        let psm = &*pac::PSM::ptr();
        psm.wdsel.write_with_zero(|w| {
            w.bits(0x0001ffff);
            w.xosc().clear_bit();
            w.rosc().clear_bit();
            w
        });
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
        const MAX_PERIOD: u32 = 0xFFFFFF;

        // Due to a logic error, the watchdog decrements by 2 and
        // the load value must be compensated; see RP2040-E1
        self.delay_ms = period.into().integer() * 2;

        if self.delay_ms > MAX_PERIOD {
            panic!("Period cannot exceed maximum load value of {}", MAX_PERIOD);
        }

        self.enable(false);
        unsafe {
            self.configure_wdog_reset_triggers();
        }
        self.load_counter(self.delay_ms);
        self.enable(true);
    }
}

impl watchdog::WatchdogDisable for Watchdog {
    fn disable(&mut self) {
        self.enable(false)
    }
}
