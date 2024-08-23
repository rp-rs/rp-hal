//! Low Power Oscillator (ROSC)
//!
//! See [Section
//! 8.4](https://rptl.io/rp2350-datasheet) for
//! more details
//!
//! The on-chip 32kHz Low Power Oscillator requires no external
//! components. It starts automatically when the always-on domain is powered and
//! provides a clock for the power manager and a tick for the Real-Time Clock
//! (RTC) when the switched-core power domain is powered off.

use crate::{pac::powman::LPOSC, typelevel::Sealed};

/// A Low Power Oscillator.
pub struct LowPowerOscillator {
    device: LPOSC,
}

impl LowPowerOscillator {
    /// Creates a new LowPowerOscillator from the underlying device.
    pub fn new(dev: LPOSC) -> Self {
        LowPowerOscillator { device: dev }
    }

    /// Releases the underlying device.
    pub fn free(self) -> LPOSC {
        self.device
    }
}

impl Sealed for LowPowerOscillator {}
