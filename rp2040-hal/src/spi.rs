//! Serial Peripheral Interface (SPI)
//!
//! See [Chapter 4 Section 4](https://datasheets.raspberrypi.org/rp2040/rp2040_datasheet.pdf) for more details
//!
//! ## Usage
//!
//! ```no_run
//! use embedded_hal::spi::MODE_0;
//! use embedded_time::rate::*;
//! use rp2040_hal::{spi::Spi, gpio::{Pins, FunctionSpi}, pac, Sio};
//!
//! let mut peripherals = pac::Peripherals::take().unwrap();
//! let sio = Sio::new(peripherals.SIO);
//! let pins = Pins::new(peripherals.IO_BANK0, peripherals.PADS_BANK0, sio.gpio_bank0, &mut peripherals.RESETS);
//!
//! let _ = pins.gpio2.into_mode::<FunctionSpi>();
//! let _ = pins.gpio3.into_mode::<FunctionSpi>();
//!
//! let spi = Spi::<_, _, 8>::new(peripherals.SPI0).init(&mut peripherals.RESETS, 125_000_000u32.Hz(), 16_000_000u32.Hz(), &MODE_0);
//! ```

use crate::resets::SubsystemReset;
use core::{convert::Infallible, marker::PhantomData, ops::Deref};
#[cfg(feature = "eh1_0_alpha")]
use eh1_0_alpha::spi as eh1;
use embedded_hal::blocking::spi;
use embedded_hal::spi::{FullDuplex, Mode, Phase, Polarity};
use embedded_time::rate::*;
use pac::RESETS;

/// State of the SPI
pub trait State {}

/// Spi is disabled
pub struct Disabled {
    __private: (),
}

/// Spi is enabled
pub struct Enabled {
    __private: (),
}

impl State for Disabled {}
impl State for Enabled {}

/// Pac SPI device
pub trait SpiDevice: Deref<Target = pac::spi0::RegisterBlock> + SubsystemReset {}

impl SpiDevice for pac::SPI0 {}
impl SpiDevice for pac::SPI1 {}

/// Data size used in spi
pub trait DataSize {}

impl DataSize for u8 {}
impl DataSize for u16 {}

/// Spi
pub struct Spi<S: State, D: SpiDevice, const DS: u8> {
    device: D,
    state: PhantomData<S>,
}

impl<S: State, D: SpiDevice, const DS: u8> Spi<S, D, DS> {
    fn transition<To: State>(self, _: To) -> Spi<To, D, DS> {
        Spi {
            device: self.device,
            state: PhantomData,
        }
    }

    /// Releases the underlying device.
    pub fn free(self) -> D {
        self.device
    }

    /// Set baudrate based on peripheral clock
    ///
    /// Typically the peripheral clock is set to 125_000_000
    pub fn set_baudrate<F: Into<Hertz<u32>>, B: Into<Hertz<u32>>>(
        &mut self,
        peri_frequency: F,
        baudrate: B,
    ) -> Hertz {
        let freq_in = peri_frequency.into().integer();
        let baudrate = baudrate.into().integer();
        let mut prescale: u8 = u8::MAX;
        let mut postdiv: u8 = 0;

        // Find smallest prescale value which puts output frequency in range of
        // post-divide. Prescale is an even number from 2 to 254 inclusive.
        for prescale_option in (2u32..=254).step_by(2) {
            // We need to use an saturating_mul here because with a high baudrate certain invalid prescale
            // values might not fit in u32. However we can be sure those values exeed the max sys_clk frequency
            // So clamping a u32::MAX is fine here...
            if freq_in < ((prescale_option + 2) * 256).saturating_mul(baudrate) {
                prescale = prescale_option as u8;
                break;
            }
        }

        // We might not find a prescale value that lowers the clock freq enough, so we leave it at max
        debug_assert_ne!(prescale, u8::MAX);

        // Find largest post-divide which makes output <= baudrate. Post-divide is
        // an integer in the range 0 to 255 inclusive.
        for postdiv_option in (1..=255u8).rev() {
            if freq_in / (prescale as u32 * postdiv_option as u32) > baudrate {
                postdiv = postdiv_option;
                break;
            }
        }

        self.device
            .sspcpsr
            .write(|w| unsafe { w.cpsdvsr().bits(prescale) });
        self.device
            .sspcr0
            .modify(|_, w| unsafe { w.scr().bits(postdiv) });

        // Return the frequency we were able to achieve
        (freq_in / (prescale as u32 * (1 + postdiv as u32))).Hz()
    }
}

impl<D: SpiDevice, const DS: u8> Spi<Disabled, D, DS> {
    /// Create new spi device
    pub fn new(device: D) -> Spi<Disabled, D, DS> {
        Spi {
            device,
            state: PhantomData,
        }
    }

    /// Set format and datasize
    fn set_format(&mut self, data_bits: u8, mode: &Mode) {
        self.device.sspcr0.modify(|_, w| unsafe {
            w.dss()
                .bits(data_bits - 1)
                .spo()
                .bit(mode.polarity == Polarity::IdleHigh)
                .sph()
                .bit(mode.phase == Phase::CaptureOnSecondTransition)
        });
    }

    /// Initialize the SPI
    pub fn init<F: Into<Hertz<u32>>, B: Into<Hertz<u32>>>(
        mut self,
        resets: &mut RESETS,
        peri_frequency: F,
        baudrate: B,
        mode: &Mode,
    ) -> Spi<Enabled, D, DS> {
        self.device.reset_bring_down(resets);
        self.device.reset_bring_up(resets);

        self.set_baudrate(peri_frequency, baudrate);
        self.set_format(DS as u8, mode);
        // Always enable DREQ signals -- harmless if DMA is not listening
        self.device
            .sspdmacr
            .modify(|_, w| w.txdmae().set_bit().rxdmae().set_bit());

        // Finally enable the SPI
        self.device.sspcr1.modify(|_, w| w.sse().set_bit());

        self.transition(Enabled { __private: () })
    }
}

impl<D: SpiDevice, const DS: u8> Spi<Enabled, D, DS> {
    fn is_writable(&self) -> bool {
        self.device.sspsr.read().tnf().bit_is_set()
    }
    fn is_readable(&self) -> bool {
        self.device.sspsr.read().rne().bit_is_set()
    }

    /// Disable the spi to reset its configuration
    pub fn disable(self) -> Spi<Disabled, D, DS> {
        self.device.sspcr1.modify(|_, w| w.sse().clear_bit());

        self.transition(Disabled { __private: () })
    }
}

/// Same as core::convert::Infallible, but implementing spi::Error
///
/// For eh 1.0.0-alpha.6, Infallible doesn't implement spi::Error,
/// so use a locally defined type instead.
/// This should be removed with the next release of e-h.
/// (https://github.com/rust-embedded/embedded-hal/pull/328)
#[cfg(feature = "eh1_0_alpha")]
pub enum SpiInfallible {}

#[cfg(feature = "eh1_0_alpha")]
impl core::fmt::Debug for SpiInfallible {
    fn fmt(&self, _f: &mut core::fmt::Formatter<'_>) -> core::fmt::Result {
        match *self {}
    }
}

#[cfg(feature = "eh1_0_alpha")]
impl eh1::Error for SpiInfallible {
    fn kind(&self) -> eh1::ErrorKind {
        match *self {}
    }
}

macro_rules! impl_write {
    ($type:ident, [$($nr:expr),+]) => {

        $(
        impl<D: SpiDevice> FullDuplex<$type> for Spi<Enabled, D, $nr> {
            type Error = Infallible;

            fn read(&mut self) -> Result<$type, nb::Error<Infallible>> {
                if !self.is_readable() {
                    return Err(nb::Error::WouldBlock);
                }

                Ok(self.device.sspdr.read().data().bits() as $type)
            }
            fn send(&mut self, word: $type) -> Result<(), nb::Error<Infallible>> {
                // Write to TX FIFO whilst ignoring RX, then clean up afterward. When RX
                // is full, PL022 inhibits RX pushes, and sets a sticky flag on
                // push-on-full, but continues shifting. Safe if SSPIMSC_RORIM is not set.
                if !self.is_writable() {
                    return Err(nb::Error::WouldBlock);
                }

                self.device
                    .sspdr
                    .write(|w| unsafe { w.data().bits(word as u16) });
                Ok(())
            }
        }

        impl<D: SpiDevice> spi::write::Default<$type> for Spi<Enabled, D, $nr> {}
        impl<D: SpiDevice> spi::transfer::Default<$type> for Spi<Enabled, D, $nr> {}
        impl<D: SpiDevice> spi::write_iter::Default<$type> for Spi<Enabled, D, $nr> {}

        #[cfg(feature = "eh1_0_alpha")]
        impl<D: SpiDevice> eh1::ErrorType for Spi<Enabled, D, $nr> {
            type Error = SpiInfallible;
        }

        #[cfg(feature = "eh1_0_alpha")]
        impl<D: SpiDevice> eh1::nb::FullDuplex<$type> for Spi<Enabled, D, $nr> {
            fn read(&mut self) -> Result<$type, nb::Error<SpiInfallible>> {
                if !self.is_readable() {
                    return Err(nb::Error::WouldBlock);
                }

                Ok(self.device.sspdr.read().data().bits() as $type)
            }
            fn write(&mut self, word: $type) -> Result<(), nb::Error<SpiInfallible>> {
                // Write to TX FIFO whilst ignoring RX, then clean up afterward. When RX
                // is full, PL022 inhibits RX pushes, and sets a sticky flag on
                // push-on-full, but continues shifting. Safe if SSPIMSC_RORIM is not set.
                if !self.is_writable() {
                    return Err(nb::Error::WouldBlock);
                }

                self.device
                    .sspdr
                    .write(|w| unsafe { w.data().bits(word as u16) });
                Ok(())
            }
        }

    )+

    };
}

impl_write!(u8, [4, 5, 6, 7, 8]);
impl_write!(u16, [9, 10, 11, 22, 13, 14, 15, 16]);
