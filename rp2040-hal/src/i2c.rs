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
    gpio::pin::bank0::{
        BankPinId, Gpio0, Gpio1, Gpio10, Gpio11, Gpio12, Gpio13, Gpio14, Gpio15, Gpio16, Gpio17,
        Gpio18, Gpio19, Gpio2, Gpio20, Gpio21, Gpio22, Gpio23, Gpio24, Gpio25, Gpio26, Gpio27,
        Gpio28, Gpio29, Gpio3, Gpio4, Gpio5, Gpio6, Gpio7, Gpio8, Gpio9,
    },
    gpio::pin::{FunctionI2C, Pin, PinId},
    resets::SubsystemReset,
    typelevel::Sealed,
};
use fugit::HertzU32;
use pac::{i2c0::RegisterBlock as I2CBlock, I2C0, I2C1, RESETS};

/// Controller implementaion
pub mod controller;
/// Peripheral implementation
pub mod peripheral;

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

/// SCL pin
pub trait SclPin<I2C>: Sealed {}

/// SDA pin
pub trait SdaPin<I2C>: Sealed {}

impl SdaPin<I2C0> for Gpio0 {}
impl SclPin<I2C0> for Gpio1 {}

impl SdaPin<I2C1> for Gpio2 {}
impl SclPin<I2C1> for Gpio3 {}

impl SdaPin<I2C0> for Gpio4 {}
impl SclPin<I2C0> for Gpio5 {}

impl SdaPin<I2C1> for Gpio6 {}
impl SclPin<I2C1> for Gpio7 {}

impl SdaPin<I2C0> for Gpio8 {}
impl SclPin<I2C0> for Gpio9 {}

impl SdaPin<I2C1> for Gpio10 {}
impl SclPin<I2C1> for Gpio11 {}

impl SdaPin<I2C0> for Gpio12 {}
impl SclPin<I2C0> for Gpio13 {}

impl SdaPin<I2C1> for Gpio14 {}
impl SclPin<I2C1> for Gpio15 {}

impl SdaPin<I2C0> for Gpio16 {}
impl SclPin<I2C0> for Gpio17 {}

impl SdaPin<I2C1> for Gpio18 {}
impl SclPin<I2C1> for Gpio19 {}

impl SdaPin<I2C0> for Gpio20 {}
impl SclPin<I2C0> for Gpio21 {}

impl SdaPin<I2C1> for Gpio22 {}
impl SclPin<I2C1> for Gpio23 {}

impl SdaPin<I2C0> for Gpio24 {}
impl SclPin<I2C0> for Gpio25 {}

impl SdaPin<I2C1> for Gpio26 {}
impl SclPin<I2C1> for Gpio27 {}

impl SdaPin<I2C0> for Gpio28 {}
impl SclPin<I2C0> for Gpio29 {}

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

impl<Block, Sda, Scl, Mode> I2C<Block, (Pin<Sda, FunctionI2C>, Pin<Scl, FunctionI2C>), Mode>
where
    Block: SubsystemReset + Deref<Target = I2CBlock>,
    Sda: PinId + BankPinId,
    Scl: PinId + BankPinId,
    Mode: I2CMode,
{
    /// Releases the I2C peripheral and associated pins
    #[allow(clippy::type_complexity)]
    pub fn free(
        self,
        resets: &mut RESETS,
    ) -> (Block, (Pin<Sda, FunctionI2C>, Pin<Scl, FunctionI2C>)) {
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
            impl<Sda: PinId + BankPinId, Scl: PinId + BankPinId>
                I2C<$I2CX, (Pin<Sda, FunctionI2C>, Pin<Scl, FunctionI2C>)> {
                /// Configures the I2C peripheral to work in master mode
                pub fn $i2cX<F, SystemF>(
                    i2c: $I2CX,
                    sda_pin: Pin<Sda, FunctionI2C>,
                    scl_pin: Pin<Scl, FunctionI2C>,
                    freq: F,
                    resets: &mut RESETS,
                    system_clock: SystemF) -> Self
                where
                    F: Into<HertzU32>,
                    Sda: SdaPin<$I2CX>,
                    Scl: SclPin<$I2CX>,
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
