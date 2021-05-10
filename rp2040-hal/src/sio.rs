//! Single Cycle Input and Output (CIO)
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
pub struct SioGpioBank0 { _private: PhantomData<u32> }

/// Struct containing ownership markers for managing ownership of the SIO registers.
pub struct Sio {
    _sio : pac::SIO,

    /// GPIO Bank 0 registers
    pub gpio_bank0 : SioGpioBank0,
    // we can hand out other things here, for example:
    // gpio_qspi
    // divider
    // interp0
    // interp1
}
impl Sio {
    /// Create `Sio` from the PAC.
    pub fn new(sio : pac::SIO) -> Self {
        Self {
            _sio: sio,

            gpio_bank0: SioGpioBank0 { _private: PhantomData }
        }
    }
}