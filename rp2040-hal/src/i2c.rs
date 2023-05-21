//! Inter-Integrated Circuit (I2C) bus
//!
//! See [Chapter 4 Section 3](https://datasheets.raspberrypi.org/rp2040/rp2040_datasheet.pdf) for more details
//!
//! ## Usage
//! ```no_run
//! use fugit::RateExtU32;
//! use rp2040_hal::{i2c::I2C, gpio::Pins, pac, Sio};
//! let mut peripherals = pac::Peripherals::take().unwrap();
//! let sio = Sio::new(peripherals.SIO);
//! let pins = Pins::new(peripherals.IO_BANK0, peripherals.PADS_BANK0, sio.gpio_bank0, &mut peripherals.RESETS);
//!
//! let mut i2c = I2C::i2c1(
//!     peripherals.I2C1,
//!     pins.gpio18.into_mode(), // sda
//!     pins.gpio19.into_mode(), // scl
//!     400.kHz(),
//!     &mut peripherals.RESETS,
//!     125_000_000.Hz(),
//! );
//!
//! // Scan for devices on the bus by attempting to read from them
//! use embedded_hal::prelude::_embedded_hal_blocking_i2c_Read;
//! for i in 0..=127 {
//!     let mut readbuf: [u8; 1] = [0; 1];
//!     let result = i2c.read(i, &mut readbuf);
//!     if let Ok(d) = result {
//!         // Do whatever work you want to do with found devices
//!         // writeln!(uart, "Device found at address{:?}", i).unwrap();
//!     }
//! }
//!
//! // Write some data to a device at 0x2c
//! use embedded_hal::prelude::_embedded_hal_blocking_i2c_Write;
//! i2c.write(0x2c, &[1, 2, 3]).unwrap();
//!
//! // Write and then read from a device at 0x3a
//! use embedded_hal::prelude::_embedded_hal_blocking_i2c_WriteRead;
//! let mut readbuf: [u8; 1] = [0; 1];
//! i2c.write_read(0x2c, &[1, 2, 3], &mut readbuf).unwrap();
//! ```
//!
//! See [examples/i2c.rs](https://github.com/rp-rs/rp-hal/tree/main/rp2040-hal/examples/i2c.rs)
//! for a complete example

use core::{marker::PhantomData, ops::Deref};

use crate::{
    gpio::{bank0::*, pin::pin_sealed::TypeLevelPinId, AnyPin, FunctionI2c},
    resets::SubsystemReset,
    typelevel::Sealed,
};
use fugit::HertzU32;
use pac::{i2c0::RegisterBlock as I2CBlock, I2C0, I2C1, RESETS};

/// Controller implementaion
pub mod controller;
/// Peripheral implementation
pub mod peripheral;

/// Pac I2C device
pub trait I2cDevice: Deref<Target = pac::i2c0::RegisterBlock> + SubsystemReset + Sealed {
    /// Index of the peripheral.
    const ID: usize;
}
impl Sealed for pac::I2C0 {}
impl I2cDevice for pac::I2C0 {
    const ID: usize = 0;
}
impl Sealed for pac::I2C1 {}
impl I2cDevice for pac::I2C1 {
    const ID: usize = 1;
}

/// I2C error
#[non_exhaustive]
#[cfg_attr(not(feature = "eh1_0_alpha"), derive(Debug))]
#[cfg_attr(
    all(feature = "defmt", not(feature = "eh1_0_alpha")),
    derive(defmt::Format)
)]
pub enum Error {
    /// I2C abort with error
    Abort(u32),
    /// User passed in a read buffer that was 0 length
    InvalidReadBufferLength,
    /// User passed in a write buffer that was 0 length
    InvalidWriteBufferLength,
    /// Target i2c address is out of range
    AddressOutOfRange(u16),
    /// Target i2c address is reserved
    AddressReserved(u16),
}

#[cfg(feature = "eh1_0_alpha")]
impl core::fmt::Debug for Error {
    fn fmt(&self, fmt: &mut core::fmt::Formatter<'_>) -> core::fmt::Result {
        use eh1_0_alpha::i2c::Error as _;
        match self {
            Error::InvalidReadBufferLength => write!(fmt, "InvalidReadBufferLength"),
            Error::InvalidWriteBufferLength => write!(fmt, "InvalidWriteBufferLength"),
            Error::AddressOutOfRange(addr) => write!(fmt, "AddressOutOfRange({:x})", addr),
            Error::AddressReserved(addr) => write!(fmt, "AddressReserved({:x})", addr),
            Error::Abort(_) => {
                write!(fmt, "{:?}", self.kind())
            }
        }
    }
}

#[cfg(all(feature = "defmt", feature = "eh1_0_alpha"))]
impl defmt::Format for Error {
    fn format(&self, fmt: defmt::Formatter) {
        use eh1_0_alpha::i2c::Error as _;
        match self {
            Error::InvalidReadBufferLength => defmt::write!(fmt, "InvalidReadBufferLength"),
            Error::InvalidWriteBufferLength => defmt::write!(fmt, "InvalidWriteBufferLength"),
            Error::AddressOutOfRange(addr) => defmt::write!(fmt, "AddressOutOfRange({:x})", addr),
            Error::AddressReserved(addr) => defmt::write!(fmt, "AddressReserved({:x})", addr),
            Error::Abort(_) => {
                defmt::write!(fmt, "{}", defmt::Debug2Format(&self.kind()))
            }
        }
    }
}

#[cfg(feature = "eh1_0_alpha")]
impl eh1_0_alpha::i2c::Error for Error {
    fn kind(&self) -> eh1_0_alpha::i2c::ErrorKind {
        match &self {
            Error::Abort(v) if v & 1<<12 != 0 // ARB_LOST
                => eh1_0_alpha::i2c::ErrorKind::ArbitrationLoss,
            Error::Abort(v) if v & 1<<7 != 0 // ABRT_SBYTE_ACKDET
                => eh1_0_alpha::i2c::ErrorKind::Bus,
            Error::Abort(v) if v & 1<<6 != 0 // ABRT_HS_ACKDET
                => eh1_0_alpha::i2c::ErrorKind::Bus,
            Error::Abort(v) if v & 1<<4 != 0 // ABRT_GCALL_NOACK
                => eh1_0_alpha::i2c::ErrorKind::NoAcknowledge(eh1_0_alpha::i2c::NoAcknowledgeSource::Address),
            Error::Abort(v) if v & 1<<3 != 0 // ABRT_TXDATA_NOACK
                => eh1_0_alpha::i2c::ErrorKind::NoAcknowledge(eh1_0_alpha::i2c::NoAcknowledgeSource::Data),
            Error::Abort(v) if v & 1<<2 != 0 // ABRT_10ADDR2_NOACK
                => eh1_0_alpha::i2c::ErrorKind::NoAcknowledge(eh1_0_alpha::i2c::NoAcknowledgeSource::Address),
            Error::Abort(v) if v & 1<<1 != 0 // ABRT_10ADDR1_NOACK
                => eh1_0_alpha::i2c::ErrorKind::NoAcknowledge(eh1_0_alpha::i2c::NoAcknowledgeSource::Address),
            Error::Abort(v) if v & 1<<0 != 0 // ABRT_7B_ADDR_NOACK
                => eh1_0_alpha::i2c::ErrorKind::NoAcknowledge(eh1_0_alpha::i2c::NoAcknowledgeSource::Address),
            _ => eh1_0_alpha::i2c::ErrorKind::Other,
        }
    }
}

macro_rules! pin_validation {
    ($p:ident) => {
        paste::paste!{
            #[doc = "Marker for PinId that can serve as " $p]
            pub trait [<ValidPinId $p>]<I2C>: Sealed {}

            #[doc = "Valid " $p]
            pub trait [<ValidPin $p>]<I2C>: Sealed {}

            impl<T, U: I2cDevice> [<ValidPin $p>]<U> for T
            where
                T: AnyPin<Function = FunctionI2c>,
                T::Id: [<ValidPinId $p>]<U>,
            {
            }

            #[doc = "A runtime validated " $p " pin for I2C."]
            pub struct [<ValidatedPin $p>]<P, I2C>(P, PhantomData<I2C>);
            impl<P, I2C: I2cDevice> Sealed for [<ValidatedPin $p>]<P, I2C> {}
            impl<P, I2C: I2cDevice> [<ValidPin $p>]<I2C> for [<ValidatedPin $p>]<P, I2C> {}
            impl<P, S> [<ValidatedPin $p>]<P, S>
            where
                P: AnyPin<Function = FunctionI2c>,
                S: I2cDevice,
            {
                /// Validate a pin's function on a i2c peripheral.
                ///
                #[doc = "Will err if the pin cannot be used as a " $p " pin for that I2C."]
                pub fn validate(p: P, _u: &S) -> Result<Self, P> {
                    if [<$p:upper>].contains(&(p.borrow().id().num, S::ID)) &&
                        p.borrow().id().bank == crate::gpio::DynBankId::Bank0 {
                        Ok(Self(p, PhantomData))
                    } else {
                        Err(p)
                    }
                }
            }
        }
    };
    ($($p:ident),*) => {
        $(
            pin_validation!($p);
        )*
    };
}

pin_validation!(Scl, Sda);

macro_rules! valid_pins {
    ($($i2c:ident: {
        sda: [$($sda:ident),*],
        scl: [$($scl:ident),*]
    }),*) => {
        $(
            $(impl ValidPinIdSda<$i2c> for $sda {})*
            $(impl ValidPinIdScl<$i2c> for $scl {})*
         )*

        const SDA: &[(u8, usize)] = &[$($(($sda::ID.num, $i2c::ID)),*),*];
        const SCL: &[(u8, usize)] = &[$($(($scl::ID.num, $i2c::ID)),*),*];
    };
}
valid_pins! {
    I2C0: {
        sda: [Gpio0, Gpio4, Gpio8, Gpio12, Gpio16, Gpio20, Gpio24, Gpio28],
        scl: [Gpio1, Gpio5, Gpio9, Gpio13, Gpio17, Gpio21, Gpio25, Gpio29]
    },
    I2C1: {
        sda: [Gpio2, Gpio6, Gpio10, Gpio14, Gpio18, Gpio22, Gpio26],
        scl: [Gpio3, Gpio7, Gpio11, Gpio15, Gpio19, Gpio23, Gpio27]
    }
}

/// Operational mode of the I2C peripheral.
pub trait I2CMode: Sealed {
    /// Indicates whether this mode is Controller or Peripheral.
    const IS_CONTROLLER: bool;
}
/// Marker for an I2C peripheral operating as a controller.
pub enum Controller {}
impl Sealed for Controller {}
impl I2CMode for Controller {
    const IS_CONTROLLER: bool = true;
}
/// Marker for an I2C peripheral operating as a peripehral.
pub enum Peripheral {}
impl Sealed for Peripheral {}
impl I2CMode for Peripheral {
    const IS_CONTROLLER: bool = false;
}

/// I2C peripheral
pub struct I2C<I2C, Pins, Mode = Controller> {
    i2c: I2C,
    pins: Pins,
    mode: PhantomData<Mode>,
}

const TX_FIFO_SIZE: u8 = 16;
const RX_FIFO_SIZE: u8 = 16;

fn i2c_reserved_addr(addr: u16) -> bool {
    (addr & 0x78) == 0 || (addr & 0x78) == 0x78
}

impl<Block, Sda, Scl, Mode> I2C<Block, (Sda, Scl), Mode>
where
    Block: SubsystemReset + Deref<Target = I2CBlock>,
{
    /// Releases the I2C peripheral and associated pins
    #[allow(clippy::type_complexity)]
    pub fn free(self, resets: &mut RESETS) -> (Block, (Sda, Scl)) {
        self.i2c.reset_bring_down(resets);

        (self.i2c, self.pins)
    }
}

impl<Block: Deref<Target = I2CBlock>, PINS, Mode> I2C<Block, PINS, Mode> {
    /// Number of bytes currently in the RX FIFO
    #[inline]
    pub fn rx_fifo_used(&self) -> u8 {
        self.i2c.ic_rxflr.read().rxflr().bits()
    }

    /// Remaining capacity in the RX FIFO
    #[inline]
    pub fn rx_fifo_free(&self) -> u8 {
        RX_FIFO_SIZE - self.rx_fifo_used()
    }

    /// RX FIFO is empty
    #[inline]
    pub fn rx_fifo_empty(&self) -> bool {
        self.rx_fifo_used() == 0
    }

    /// Number of bytes currently in the TX FIFO
    #[inline]
    pub fn tx_fifo_used(&self) -> u8 {
        self.i2c.ic_txflr.read().txflr().bits()
    }

    /// Remaining capacity in the TX FIFO
    #[inline]
    pub fn tx_fifo_free(&self) -> u8 {
        TX_FIFO_SIZE - self.tx_fifo_used()
    }

    /// TX FIFO is at capacity
    #[inline]
    pub fn tx_fifo_full(&self) -> bool {
        self.tx_fifo_free() == 0
    }
}

macro_rules! hal {
    ($($I2CX:ident: ($i2cX:ident),)+) => {
        $(
            impl<Sda, Scl> I2C<$I2CX, (Sda, Scl)> {
                /// Configures the I2C peripheral to work in master mode
                pub fn $i2cX<F, SystemF>(
                    i2c: $I2CX,
                    sda_pin: Sda,
                    scl_pin: Scl,
                    freq: F,
                    resets: &mut RESETS,
                    system_clock: SystemF) -> Self
                where
                    F: Into<HertzU32>,
                    Sda: ValidPinSda<$I2CX>,
                    Scl: ValidPinScl<$I2CX>,
                    SystemF: Into<HertzU32>,
                {
                    Self::new_controller(i2c, sda_pin, scl_pin, freq.into(), resets, system_clock.into())
                }
            }
        )+
    }
}
hal! {
    I2C0: (i2c0),
    I2C1: (i2c1),
}
