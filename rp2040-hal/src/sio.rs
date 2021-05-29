//! Single Cycle Input and Output (SIO)
//!
//! To be able to partition parts of the SIO block to other modules:
//!
//! ```rust
//! let sio = Sio::new(pac.SIO);
//! ```
//!
//! And then for example
//!
//! ```rust
//! let pins = pac.IO_BANK0.split(pac.PADS_BANK0, sio.gpio_bank0, &mut pac.RESETS);
//! ```
use super::*;
use core::marker::PhantomData;

/// Marker struct for ownership of SIO gpio bank0
pub struct SioGpioBank0 {
    _private: PhantomData<u32>,
}

/// Marker struct for ownership of divide/modulo
pub struct HwDivider {
    _private: PhantomData<u32>,
}

/// Result from divide/modulo operation
pub struct DivResult<T> {
    /// The remainder result from divide/modulo operation
    pub remainder: T,
    /// The quotient result from divide/modulo operation
    pub quotient: T,
}

/// Struct containing ownership markers for managing ownership of the SIO registers.
pub struct Sio {
    _sio: pac::SIO,
    /// GPIO Bank 0 registers
    pub gpio_bank0: SioGpioBank0,
    /// 8-cycle hardware divide/modulo module
    pub hwdivider: HwDivider,
    // we can hand out other things here, for example:
    // gpio_qspi
    // interp0
    // interp1
}
impl Sio {
    /// Create `Sio` from the PAC.
    pub fn new(sio: pac::SIO) -> Self {
        Self {
            _sio: sio,

            gpio_bank0: SioGpioBank0 {
                _private: PhantomData,
            },

            hwdivider: HwDivider {
                _private: PhantomData,
            },
        }
    }
}

/// Perform hardware unsigned divide/module operation
impl HwDivider {
    pub fn unsigned(&self, dividend: u32, divisor: u32) -> DivResult<u32> {
        let sio = unsafe { &(*pac::SIO::ptr()) };
        sio.div_sdividend.write(|w| unsafe { w.bits(dividend) });

        sio.div_sdivisor.write(|w| unsafe { w.bits(divisor) });

        cortex_m::asm::delay(8);

        // Note: quotient must be read last
        let remainder = sio.div_remainder.read().bits();
        let quotient = sio.div_quotient.read().bits();

        DivResult {
            remainder,
            quotient,
        }
    }

/// Perform hardware signed divide/module operation
    pub fn signed(&self, dividend: i32, divisor: i32) -> DivResult<i32> {
        let sio = unsafe { &(*pac::SIO::ptr()) };
        sio.div_sdividend
            .write(|w| unsafe { w.bits(dividend as u32) });

        sio.div_sdivisor
            .write(|w| unsafe { w.bits(divisor as u32) });

        cortex_m::asm::delay(8);

        // Note: quotient must be read last
        let remainder = sio.div_remainder.read().bits() as i32;
        let quotient = sio.div_quotient.read().bits() as i32;

        DivResult {
            remainder,
            quotient,
        }
    }
}
