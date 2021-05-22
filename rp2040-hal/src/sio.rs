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

/// Struct containing ownership markers for managing ownership of the SIO registers.
pub struct Sio {
    _sio: pac::SIO,

    /// GPIO Bank 0 registers
    pub gpio_bank0: SioGpioBank0,
    // we can hand out other things here, for example:
    // gpio_qspi
    // divider
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
        }
    }
}

/// 8-cycle unsigned divide/modulo
pub struct DivModUnsigned {
    dividend: u32,
    divisor: u32,
}

/// 8-cycle signed divide/modulo
pub struct DivModSigned {
    dividend: i32,
    divisor: i32,
}

/// Struct containing result from unsigned divide/modulo operation
pub struct DivModUnsignedResult {
    /// The remainder result from unsigned divide/modulo operation
    pub remainder: u32,
    /// The quotient result from unsigned divide/modulo operation
    pub quotient: u32,
}

/// Struct containing result from signed divide/modulo operation
pub struct DivModSignedResult {
    /// The remainder result from signed divide/modulo operation
    pub remainder: i32,
    /// The quotient result from signed divide/modulo operation
    pub quotient: i32,
}

impl DivModUnsigned {
    fn sio(&self) -> &pac::sio::RegisterBlock {
        unsafe { &(*pac::SIO::ptr()) }
    }

    /// Create a new [`DivModUnsigned`]
    pub fn new(dividend: u32, divisor: u32) -> Self {
        Self { dividend, divisor }
    }

    /// Perform unsigned divide/module operation
    /// ```rust
    /// let hwdivmod = DivModUnsigned::new(500, 7);
    /// let div_mod_result = hwdivmod.unsigned_div_mod();
    /// let remainder = div_mod_result.remainder;
    /// let quotient = div_mod_result.quotient;
    /// ```
    pub fn unsigned_div_mod(&self) -> DivModUnsignedResult {
        self.sio()
            .div_udividend
            .write(|w| unsafe { w.bits(self.dividend) });

        self.sio()
            .div_udivisor
            .write(|w| unsafe { w.bits(self.divisor) });

        cortex_m::asm::delay(8);

        // Note: quotient must be read last
        let remainder = self.sio().div_remainder.read().bits();
        let quotient = self.sio().div_quotient.read().bits();

        DivModUnsignedResult {
            remainder,
            quotient,
        }
    }
}

impl DivModSigned {
    fn sio(&self) -> &pac::sio::RegisterBlock {
        unsafe { &(*pac::SIO::ptr()) }
    }

    /// Create a new [`DivModSigned`]
    pub fn new(dividend: i32, divisor: i32) -> Self {
        Self { dividend, divisor }
    }

    /// Perform signed divide/module operation
    /// ```rust
    /// let hwdivmod = DivModSigned::new(-500, 7);
    /// let div_mod_result = hwdivmod.signed_div_mod();
    /// let remainder = div_mod_result.remainder;
    /// let quotient = div_mod_result.quotient;
    /// ```
    pub fn signed_div_mod(&self) -> DivModSignedResult {
        self.sio()
            .div_sdividend
            .write(|w| unsafe { w.bits(self.dividend as u32) });

        self.sio()
            .div_sdivisor
            .write(|w| unsafe { w.bits(self.divisor as u32) });

        cortex_m::asm::delay(8);

        // Note: quotient must be read last
        let remainder = self.sio().div_remainder.read().bits() as i32;
        let quotient = self.sio().div_quotient.read().bits() as i32;

        DivModSignedResult {
            remainder,
            quotient,
        }
    }
}
