//! Inter-Integrated Circuit (I2C) bus
//!
//! See [Section 12.2](https://rptl.io/rp2350-datasheet#section_i2c) for more details
//!
//! ## Usage
//! ```no_run
//! use fugit::RateExtU32;
//! use rp235x_hal::{self as hal, gpio::Pins, i2c::I2C, Sio};
//! let mut peripherals = hal::pac::Peripherals::take().unwrap();
//! let sio = Sio::new(peripherals.SIO);
//! let pins = Pins::new(
//!     peripherals.IO_BANK0,
//!     peripherals.PADS_BANK0,
//!     sio.gpio_bank0,
//!     &mut peripherals.RESETS,
//! );
//!
//! let mut i2c = I2C::i2c1(
//!     peripherals.I2C1,
//!     pins.gpio18.reconfigure(), // sda
//!     pins.gpio19.reconfigure(), // scl
//!     400.kHz(),
//!     &mut peripherals.RESETS,
//!     125_000_000.Hz(),
//! );
//!
//! // Scan for devices on the bus by attempting to read from them
//! use embedded_hal_0_2::prelude::_embedded_hal_blocking_i2c_Read;
//! for i in 0..=127u8 {
//!     let mut readbuf: [u8; 1] = [0; 1];
//!     let result = i2c.read(i, &mut readbuf);
//!     if let Ok(d) = result {
//!         // Do whatever work you want to do with found devices
//!         // writeln!(uart, "Device found at address{:?}", i).unwrap();
//!     }
//! }
//!
//! // Write some data to a device at 0x2c
//! use embedded_hal_0_2::prelude::_embedded_hal_blocking_i2c_Write;
//! i2c.write(0x2Cu8, &[1, 2, 3]).unwrap();
//!
//! // Write and then read from a device at 0x3a
//! use embedded_hal_0_2::prelude::_embedded_hal_blocking_i2c_WriteRead;
//! let mut readbuf: [u8; 1] = [0; 1];
//! i2c.write_read(0x2Cu8, &[1, 2, 3], &mut readbuf).unwrap();
//! ```
//!
//! See [examples/i2c.rs](https://github.com/rp-rs/rp-hal/tree/main/rp235x-hal-examples/src/bin/i2c.rs)
//! for a complete example
//!
//! ## Async Usage
//!
//! See [examples/i2c_async.rs](https://github.com/rp-rs/rp-hal/tree/main/rp235x-hal-examples/src/bin/i2c_async.rs)
//! and [examples/i2c_async_irq.rs](https://github.com/rp-rs/rp-hal/tree/main/rp235x-hal-examples/src/bin/i2c_async_irq.rs)
//! for complete examples.

use core::{marker::PhantomData, ops::Deref};
use fugit::HertzU32;
use rp235x_pac::i2c0::ic_con::IC_10BITADDR_SLAVE_A;

use crate::{
    gpio::{bank0::*, pin::pin_sealed::TypeLevelPinId, AnyPin, FunctionI2c, PullUp},
    pac::{
        i2c0::{ic_con::IC_10BITADDR_MASTER_A, RegisterBlock},
        I2C0, I2C1, RESETS,
    },
    resets::SubsystemReset,
    typelevel::Sealed,
};

mod controller;
pub mod peripheral;

/// Pac I2C device
pub trait I2cDevice: Deref<Target = RegisterBlock> + SubsystemReset + Sealed {
    /// Index of the peripheral.
    const ID: usize;
}
impl Sealed for I2C0 {}
impl I2cDevice for I2C0 {
    const ID: usize = 0;
}
impl Sealed for I2C1 {}
impl I2cDevice for I2C1 {
    const ID: usize = 1;
}

/// Marks valid/supported address types
pub trait ValidAddress:
    Into<u16> + embedded_hal::i2c::AddressMode + embedded_hal_0_2::blocking::i2c::AddressMode + Copy
{
    /// Variant for the IC_CON.10bitaddr_master field
    const BIT_ADDR_M: IC_10BITADDR_MASTER_A;
    /// Variant for the IC_CON.10bitaddr_slave field
    const BIT_ADDR_S: IC_10BITADDR_SLAVE_A;

    /// Validates the address against address ranges supported by the hardware.
    fn is_valid(self) -> Result<(), Error>;
}
impl ValidAddress for u8 {
    const BIT_ADDR_M: IC_10BITADDR_MASTER_A = IC_10BITADDR_MASTER_A::ADDR_7BITS;
    const BIT_ADDR_S: IC_10BITADDR_SLAVE_A = IC_10BITADDR_SLAVE_A::ADDR_7BITS;

    fn is_valid(self) -> Result<(), Error> {
        if self >= 0x80 {
            Err(Error::AddressOutOfRange(self.into()))
        } else {
            Ok(())
        }
    }
}
impl ValidAddress for u16 {
    const BIT_ADDR_M: IC_10BITADDR_MASTER_A = IC_10BITADDR_MASTER_A::ADDR_10BITS;
    const BIT_ADDR_S: IC_10BITADDR_SLAVE_A = IC_10BITADDR_SLAVE_A::ADDR_10BITS;

    fn is_valid(self) -> Result<(), Error> {
        Ok(())
    }
}

/// I2C error
#[non_exhaustive]
pub enum Error {
    /// I2C abort with error
    Abort(u32),
    /// User passed in a read buffer that was 0 length
    ///
    /// This is a limitation of the rp235x I2C peripheral.
    /// If the slave ACKs its address, the I2C peripheral must read
    /// at least one byte before sending the STOP condition.
    InvalidReadBufferLength,
    /// User passed in a write buffer that was 0 length
    ///
    /// This is a limitation of the rp235x I2C peripheral.
    /// If the slave ACKs its address, the I2C peripheral must write
    /// at least one byte before sending the STOP condition.
    InvalidWriteBufferLength,
    /// Target i2c address is out of range
    AddressOutOfRange(u16),
    /// Target i2c address is reserved
    AddressReserved(u16),
}

impl core::fmt::Debug for Error {
    fn fmt(&self, fmt: &mut core::fmt::Formatter<'_>) -> core::fmt::Result {
        use embedded_hal::i2c::Error as _;
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

#[cfg(feature = "defmt")]
impl defmt::Format for Error {
    fn format(&self, fmt: defmt::Formatter) {
        use embedded_hal::i2c::Error as _;
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

impl embedded_hal::i2c::Error for Error {
    fn kind(&self) -> embedded_hal::i2c::ErrorKind {
        match &self {
            Error::Abort(v) if v & (1<<12) != 0 // ARB_LOST
                => embedded_hal::i2c::ErrorKind::ArbitrationLoss,
            Error::Abort(v) if v & (1<<7) != 0 // ABRT_SBYTE_ACKDET
                => embedded_hal::i2c::ErrorKind::Bus,
            Error::Abort(v) if v & (1<<6) != 0 // ABRT_HS_ACKDET
                => embedded_hal::i2c::ErrorKind::Bus,
            Error::Abort(v) if v & (1<<4) != 0 // ABRT_GCALL_NOACK
                => embedded_hal::i2c::ErrorKind::NoAcknowledge(embedded_hal::i2c::NoAcknowledgeSource::Address),
            Error::Abort(v) if v & (1<<3) != 0 // ABRT_TXDATA_NOACK
                => embedded_hal::i2c::ErrorKind::NoAcknowledge(embedded_hal::i2c::NoAcknowledgeSource::Data),
            Error::Abort(v) if v & (1<<2) != 0 // ABRT_10ADDR2_NOACK
                => embedded_hal::i2c::ErrorKind::NoAcknowledge(embedded_hal::i2c::NoAcknowledgeSource::Address),
            Error::Abort(v) if v & (1<<1) != 0 // ABRT_10ADDR1_NOACK
                => embedded_hal::i2c::ErrorKind::NoAcknowledge(embedded_hal::i2c::NoAcknowledgeSource::Address),
            Error::Abort(v) if v & (1<<0) != 0 // ABRT_7B_ADDR_NOACK
                => embedded_hal::i2c::ErrorKind::NoAcknowledge(embedded_hal::i2c::NoAcknowledgeSource::Address),
            _ => embedded_hal::i2c::ErrorKind::Other,
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
pub struct Controller {}
impl Sealed for Controller {}
impl I2CMode for Controller {
    const IS_CONTROLLER: bool = true;
}

/// Marker for an I2C block operating as a peripehral.
pub struct Peripheral {
    state: peripheral::State,
}
impl Sealed for Peripheral {}
impl I2CMode for Peripheral {
    const IS_CONTROLLER: bool = false;
}

/// I2C peripheral
pub struct I2C<I2C, Pins, Mode = Controller> {
    i2c: I2C,
    pins: Pins,
    mode: Mode,
}

impl<Block, Sda, Scl, Mode> I2C<Block, (Sda, Scl), Mode>
where
    Block: SubsystemReset + Deref<Target = RegisterBlock>,
{
    /// Releases the I2C peripheral and associated pins
    #[allow(clippy::type_complexity)]
    pub fn free(self, resets: &mut RESETS) -> (Block, (Sda, Scl)) {
        self.i2c.reset_bring_down(resets);

        (self.i2c, self.pins)
    }
}

impl<Block: Deref<Target = RegisterBlock>, PINS, Mode> I2C<Block, PINS, Mode> {
    /// Depth of the TX FIFO.
    pub const TX_FIFO_DEPTH: u8 = 16;

    /// Depth of the RX FIFO.
    pub const RX_FIFO_DEPTH: u8 = 16;

    /// Number of bytes currently in the RX FIFO
    #[inline]
    pub fn rx_fifo_used(&self) -> u8 {
        self.i2c.ic_rxflr().read().rxflr().bits()
    }

    /// Remaining capacity in the RX FIFO
    #[inline]
    pub fn rx_fifo_available(&self) -> u8 {
        Self::RX_FIFO_DEPTH - self.rx_fifo_used()
    }

    /// RX FIFO is empty
    #[inline]
    pub fn rx_fifo_empty(&self) -> bool {
        self.i2c.ic_status().read().rfne().bit_is_clear()
    }

    /// Number of bytes currently in the TX FIFO
    #[inline]
    pub fn tx_fifo_used(&self) -> u8 {
        self.i2c.ic_txflr().read().txflr().bits()
    }

    /// Remaining capacity in the TX FIFO
    #[inline]
    pub fn tx_fifo_available(&self) -> u8 {
        Self::TX_FIFO_DEPTH - self.tx_fifo_used()
    }

    /// TX FIFO is at capacity
    #[inline]
    pub fn tx_fifo_full(&self) -> bool {
        self.i2c.ic_status().read().tfnf().bit_is_clear()
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
                    Sda: ValidPinSda<$I2CX> + AnyPin<Pull = PullUp>,
                    Scl: ValidPinScl<$I2CX> + AnyPin<Pull = PullUp>,
                    SystemF: Into<HertzU32>,
                {
                    Self::new_controller(i2c, sda_pin, scl_pin, freq.into(), resets, system_clock.into())
                }

                $crate::paste::paste! {
                    /// Configures the I2C peripheral to work in master mode
                    ///
                    /// This function can be called without activating internal pull-ups on the I2C pins.
                    /// It should only be used if external pull-ups are provided.
                    pub fn [<$i2cX _with_external_pull_up>]<F, SystemF>(
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
            }

            impl<P, M> $crate::async_utils::sealed::Wakeable for I2C<$I2CX, P, M> {
                fn waker() -> &'static $crate::async_utils::sealed::IrqWaker {
                    static WAKER: $crate::async_utils::sealed::IrqWaker =
                        $crate::async_utils::sealed::IrqWaker::new();
                    &WAKER
                }
            }
        )+
    }
}
hal! {
    I2C0: (i2c0),
    I2C1: (i2c1),
}
