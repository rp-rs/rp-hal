//! Watchdog
//!
//! The watchdog is a countdown timer that can restart parts of the chip if it reaches zero. This can be used to restart the
//! processor if software gets stuck in an infinite loop. The programmer must periodically write a value to the watchdog to
//! stop it from reaching zero.
//!
//! See [Section 12.9](https://rptl.io/rp2350-datasheet) of the datasheet for more details
//!
//! ## Usage
//! ```no_run
//! use fugit::ExtU32;
//! use rp235x_hal::{self as hal, clocks::init_clocks_and_plls, watchdog::Watchdog};
//! let mut pac = hal::pac::Peripherals::take().unwrap();
//! let mut watchdog = Watchdog::new(pac.WATCHDOG);
//! let _clocks = init_clocks_and_plls(
//!     12_000_000,
//!     pac.XOSC,
//!     pac.CLOCKS,
//!     pac.PLL_SYS,
//!     pac.PLL_USB,
//!     &mut pac.RESETS,
//!     &mut watchdog,
//! )
//! .ok()
//! .unwrap();
//! // Set to watchdog to reset if it's not reloaded within 1.05 seconds, and start it
//! watchdog.start(1_050_000.micros());
//! // Feed the watchdog once per cycle to avoid reset
//! for _ in 1..=10000 {
//!     hal::arch::delay(100_000);
//!     watchdog.feed();
//! }
//! // Stop feeding, now we'll reset
//! loop {}
//! ```
//! See [examples/watchdog.rs](https://github.com/rp-rs/rp-hal/tree/main/rp235x-hal-examples/src/bin/watchdog.rs) for a more complete example

// Embedded HAL 1.0.0 doesn't have an ADC trait, so use the one from 0.2
use embedded_hal_0_2::watchdog;
use fugit::MicrosDurationU32;

use crate::pac::{self, WATCHDOG};

/// Watchdog peripheral
pub struct Watchdog {
    watchdog: WATCHDOG,
    load_value: u32, // decremented by 2 per tick (µs)
}

#[derive(Debug)]
#[allow(missing_docs)]
/// Scratch registers of the watchdog peripheral
pub enum ScratchRegister {
    Scratch0,
    Scratch1,
    Scratch2,
    Scratch3,
    Scratch4,
    Scratch5,
    Scratch6,
    Scratch7,
}

impl Watchdog {
    /// Create a new [`Watchdog`]
    pub fn new(watchdog: WATCHDOG) -> Self {
        Self {
            watchdog,
            load_value: 0,
        }
    }

    /// Starts tick generation on all the ticks.
    ///
    /// # Arguments
    ///
    /// * `cycles` - Total number of tick cycles before the next tick is generated.
    ///   It is expected to be the frequency in MHz of clk_ref.
    pub fn enable_tick_generation(&mut self, cycles: u16) {
        // now we have separate ticks for every destination
        // we own the watchdog, so no-one else can be writing to this register
        let ticks = unsafe { &*pac::TICKS::ptr() };
        for ticker in ticks.tick_iter() {
            // TODO: work out how to rename proc0_cycles to cycles in the SVD patch YAML
            ticker
                .cycles()
                .write(|w| unsafe { w.proc0_cycles().bits(cycles) });
            ticker.ctrl().write(|w| w.enable().set_bit());
        }
    }

    /// Defines whether or not the watchdog timer should be paused when processor(s) are in debug mode
    /// or when JTAG is accessing bus fabric
    ///
    /// # Arguments
    ///
    /// * `pause` - If true, watchdog timer will be paused
    pub fn pause_on_debug(&mut self, pause: bool) {
        self.watchdog.ctrl().write(|w| {
            w.pause_dbg0()
                .bit(pause)
                .pause_dbg1()
                .bit(pause)
                .pause_jtag()
                .bit(pause)
        })
    }

    fn load_counter(&self, counter: u32) {
        self.watchdog.load().write(|w| unsafe { w.bits(counter) });
    }

    fn enable(&self, bit: bool) {
        self.watchdog.ctrl().write(|w| w.enable().bit(bit))
    }

    /// Read a scratch register
    pub fn read_scratch(&self, reg: ScratchRegister) -> u32 {
        match reg {
            ScratchRegister::Scratch0 => self.watchdog.scratch0().read().bits(),
            ScratchRegister::Scratch1 => self.watchdog.scratch1().read().bits(),
            ScratchRegister::Scratch2 => self.watchdog.scratch2().read().bits(),
            ScratchRegister::Scratch3 => self.watchdog.scratch3().read().bits(),
            ScratchRegister::Scratch4 => self.watchdog.scratch4().read().bits(),
            ScratchRegister::Scratch5 => self.watchdog.scratch5().read().bits(),
            ScratchRegister::Scratch6 => self.watchdog.scratch6().read().bits(),
            ScratchRegister::Scratch7 => self.watchdog.scratch7().read().bits(),
        }
    }

    /// Write a scratch register
    pub fn write_scratch(&mut self, reg: ScratchRegister, value: u32) {
        match reg {
            ScratchRegister::Scratch0 => {
                self.watchdog.scratch0().write(|w| unsafe { w.bits(value) })
            }
            ScratchRegister::Scratch1 => {
                self.watchdog.scratch1().write(|w| unsafe { w.bits(value) })
            }
            ScratchRegister::Scratch2 => {
                self.watchdog.scratch2().write(|w| unsafe { w.bits(value) })
            }
            ScratchRegister::Scratch3 => {
                self.watchdog.scratch3().write(|w| unsafe { w.bits(value) })
            }
            ScratchRegister::Scratch4 => {
                self.watchdog.scratch4().write(|w| unsafe { w.bits(value) })
            }
            ScratchRegister::Scratch5 => {
                self.watchdog.scratch5().write(|w| unsafe { w.bits(value) })
            }
            ScratchRegister::Scratch6 => {
                self.watchdog.scratch6().write(|w| unsafe { w.bits(value) })
            }
            ScratchRegister::Scratch7 => {
                self.watchdog.scratch7().write(|w| unsafe { w.bits(value) })
            }
        }
    }

    /// Configure which hardware will be reset by the watchdog
    /// the default is everything except ROSC, XOSC
    ///
    /// Safety: ensure no other device is writing to psm.wdsel
    /// This is easy at the moment, since nothing else uses PSM
    unsafe fn configure_wdog_reset_triggers(&self) {
        let psm = &*pac::PSM::ptr();
        psm.wdsel().write_with_zero(|w| {
            w.bits(0x0001ffff);
            w.xosc().clear_bit();
            w.rosc().clear_bit();
            w
        });
    }

    /// Set the watchdog counter back to its load value, making sure
    /// that the watchdog reboot will not be triggered for the configured
    /// period.
    pub fn feed(&self) {
        self.load_counter(self.load_value)
    }

    /// Start the watchdog. This enables a timer which will reboot the
    /// rp2350 if [`Watchdog::feed()`] does not get called for the configured period.
    pub fn start<T: Into<MicrosDurationU32>>(&mut self, period: T) {
        const MAX_PERIOD: u32 = 0xFFFFFF;

        let delay_us = period.into().to_micros();
        if delay_us > MAX_PERIOD / 2 {
            panic!(
                "Period cannot exceed maximum load value of {} ({} microseconds))",
                MAX_PERIOD,
                MAX_PERIOD / 2
            );
        }
        self.load_value = delay_us;

        self.enable(false);
        unsafe {
            self.configure_wdog_reset_triggers();
        }
        self.load_counter(self.load_value);
        self.enable(true);
    }

    /// Disable the watchdog timer.
    pub fn disable(&self) {
        self.enable(false)
    }
}

impl watchdog::Watchdog for Watchdog {
    fn feed(&mut self) {
        (*self).feed()
    }
}

impl watchdog::WatchdogEnable for Watchdog {
    type Time = MicrosDurationU32;

    fn start<T: Into<Self::Time>>(&mut self, period: T) {
        self.start(period)
    }
}

impl watchdog::WatchdogDisable for Watchdog {
    fn disable(&mut self) {
        (*self).disable()
    }
}
