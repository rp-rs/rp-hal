//! Single Cycle Input and Output (SIO)
//!
//! To be able to partition parts of the SIO block to other modules:
//!
//! ```no_run
//! use rp2040_hal::{gpio::Pins, pac, sio::Sio};
//!
//! let mut peripherals = pac::Peripherals::take().unwrap();
//! let sio = Sio::new(peripherals.SIO);
//! ```
//!
//! And then for example
//!
//! ```no_run
//! # use rp2040_hal::{gpio::Pins, pac, sio::Sio};
//! # let mut peripherals = pac::Peripherals::take().unwrap();
//! # let sio = Sio::new(peripherals.SIO);
//! let pins = Pins::new(peripherals.IO_BANK0, peripherals.PADS_BANK0, sio.gpio_bank0, &mut peripherals.RESETS);
//! ```

use super::*;
use core::convert::Infallible;

/// Marker struct for ownership of SIO gpio bank0
pub struct SioGpioBank0 {
    _private: (),
}

/// Marker struct for ownership of SIO gpio qspi
pub struct SioGpioQspi {
    _private: (),
}

/// Marker struct for ownership of divide/modulo module
pub struct HwDivider {
    _private: (),
}

/// Result of divide/modulo operation
pub struct DivResult<T> {
    /// The remainder of divide/modulo operation
    pub remainder: T,
    /// The quotient of divide/modulo operation
    pub quotient: T,
}

/// Struct containing ownership markers for managing ownership of the SIO registers.
pub struct Sio {
    _sio: pac::SIO,
    /// GPIO Bank 0 registers
    pub gpio_bank0: SioGpioBank0,
    /// GPIO QSPI registers
    pub gpio_qspi: SioGpioQspi,
    /// 8-cycle hardware divide/modulo module
    pub hwdivider: HwDivider,
    // we can hand out other things here, for example:
    // interp0
    // interp1
}
impl Sio {
    /// Create `Sio` from the PAC.
    pub fn new(sio: pac::SIO) -> Self {
        Self {
            _sio: sio,

            gpio_bank0: SioGpioBank0 { _private: () },
            gpio_qspi: SioGpioQspi { _private: () },

            hwdivider: HwDivider { _private: () },
        }
    }
}

impl HwDivider {
    /// Perform hardware unsigned divide/modulo operation
    pub fn unsigned(&self, dividend: u32, divisor: u32) -> DivResult<u32> {
        let sio = unsafe { &(*pac::SIO::ptr()) };
        sio.div_udividend.write(|w| unsafe { w.bits(dividend) });

        sio.div_udivisor.write(|w| unsafe { w.bits(divisor) });

        cortex_m::asm::delay(8);

        // Note: quotient must be read last
        let remainder = sio.div_remainder.read().bits();
        let quotient = sio.div_quotient.read().bits();

        DivResult {
            remainder,
            quotient,
        }
    }

    /// Perform hardware signed divide/modulo operation
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

/// Trait for all the spinlock. See the documentation of e.g. [`Spinlock0`] for more information
pub trait Spinlock: typelevel::Sealed + Sized {
    /// Try to claim the spinlock. Will return `Some(Self)` if the lock is obtained, and `None` if the lock is
    /// already in use somewhere else.
    fn try_claim() -> Option<Self>;

    /// Claim the spinlock, will block the current thread until the lock is available.
    ///
    /// Note that calling this multiple times in a row will cause a deadlock
    fn claim() -> Self {
        loop {
            if let Some(result) = Self::try_claim() {
                break result;
            }
        }
    }

    /// Try to claim the spinlock. Will return `WouldBlock` until the spinlock is available.
    fn claim_async() -> nb::Result<Self, Infallible> {
        Self::try_claim().ok_or(nb::Error::WouldBlock)
    }
}
macro_rules! impl_spinlock {
    ($($spinlock_name:ident => $register:ident,)*) => {
        $(
            /// Hardware based spinlock.
            ///
            /// You can claim this lock by calling either [`claim`], [`try_claim`] or [`claim_async`].
            /// This will automatically lock ALL spinlocks of type `
            #[doc = stringify!($spinlock_name)]
            /// `.
            ///
            /// When the obtained spinlock goes out of scope, it is automatically unlocked.
            ///
            /// **warning**: These spinlocks are not re-entrant, meaning that the following code will cause a deadlock:
            ///
            /// ```no_run
            /// use rp2040_hal::sio::{Spinlock0, Spinlock};
            /// let lock_1 = Spinlock0::claim();
            /// let lock_2 = Spinlock0::claim(); // deadlock here
            /// ```
            ///
            /// [`claim`]: #method.claim
            /// [`try_claim`]: #method.try_claim
            /// [`claim_async`]: #method.claim_async
            pub struct $spinlock_name(core::marker::PhantomData<()>);

            impl Spinlock for $spinlock_name {
                fn try_claim() -> Option<$spinlock_name> {
                    // Safety: We're only reading from this register
                    let sio = unsafe { &*pac::SIO::ptr() };
                    let lock = sio.$register.read().bits();
                    if lock > 0 {
                        Some(Self(core::marker::PhantomData))
                    } else {
                        None
                    }
                }
            }

            impl typelevel::Sealed for $spinlock_name {}

            impl Drop for $spinlock_name {
                fn drop(&mut self) {
                    // Safety: At this point we should be the only one accessing this spinlock register
                    // so writing to this address is fine
                    let sio = unsafe { &*pac::SIO::ptr() };

                    // Write (any value): release the lock
                    sio.$register.write(|b| unsafe { b.bits(1) });
                }
            }
        )*
    }
}

impl_spinlock! {
    Spinlock0 => spinlock0,
    Spinlock1 => spinlock1,
    Spinlock2 => spinlock2,
    Spinlock3 => spinlock3,
    Spinlock4 => spinlock4,
    Spinlock5 => spinlock5,
    Spinlock6 => spinlock6,
    Spinlock7 => spinlock7,
    Spinlock8 => spinlock8,
    Spinlock9 => spinlock9,
    Spinlock10 => spinlock10,
    Spinlock11 => spinlock11,
    Spinlock12 => spinlock12,
    Spinlock13 => spinlock13,
    Spinlock14 => spinlock14,
    Spinlock15 => spinlock15,
    Spinlock16 => spinlock16,
    Spinlock17 => spinlock17,
    Spinlock18 => spinlock18,
    Spinlock19 => spinlock19,
    Spinlock20 => spinlock20,
    Spinlock21 => spinlock21,
    Spinlock22 => spinlock22,
    Spinlock23 => spinlock23,
    Spinlock24 => spinlock24,
    Spinlock25 => spinlock25,
    Spinlock26 => spinlock26,
    Spinlock27 => spinlock27,
    Spinlock28 => spinlock28,
    Spinlock29 => spinlock29,
    Spinlock30 => spinlock30,
    Spinlock31 => spinlock31,
}

/// Returns the current state of the spinlocks. Each index corresponds to the associated spinlock, e.g. if index `5` is set to `true`, it means that [`Spinlock5`] is currently locked.
///
/// Note that spinlocks can be claimed or released at any point, so this function cannot guarantee the spinlock is actually available right after calling this function. This function is mainly intended for debugging.
pub fn spinlock_state() -> [bool; 32] {
    // Safety: we're only reading from a register
    let sio = unsafe { &*pac::SIO::ptr() };
    // A bitmap containing the state of all 32 spinlocks (1=locked).
    let register = sio.spinlock_st.read().bits();
    let mut result = [false; 32];
    #[allow(clippy::needless_range_loop)]
    for i in 0..32 {
        result[i] = (register & (1 << i)) > 0;
    }
    result
}
