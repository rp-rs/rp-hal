//! Serial Peripheral Interface (SPI)
//!
//! See [Chapter 4 Section 4](https://datasheets.raspberrypi.org/rp2040/rp2040_datasheet.pdf) for more details
//!
//! ## Usage
//!
//! ```no_run
//! use embedded_hal::spi::MODE_0;
//! use fugit::RateExtU32;
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

use crate::dma::{EndlessReadTarget, EndlessWriteTarget, ReadTarget, WriteTarget};
use crate::resets::SubsystemReset;
use crate::typelevel::Sealed;
use core::{convert::Infallible, marker::PhantomData, ops::Deref};
#[cfg(feature = "eh1_0_alpha")]
use eh1_0_alpha::spi as eh1;
use embedded_hal::blocking::spi;
use embedded_hal::spi::{FullDuplex, Mode, Phase, Polarity};
use fugit::HertzU32;
use fugit::RateExtU32;
use pac::dma::ch::ch_ctrl_trig::TREQ_SEL_A;
use pac::RESETS;

/// State of the SPI
pub trait State: Sealed {}

/// Spi is disabled
pub struct Disabled {
    __private: (),
}

/// Spi is enabled
pub struct Enabled {
    __private: (),
}

impl State for Disabled {}
impl Sealed for Disabled {}
impl State for Enabled {}
impl Sealed for Enabled {}

/// Pac SPI device
pub trait SpiDevice: Deref<Target = pac::spi0::RegisterBlock> + SubsystemReset + Sealed {
    /// The DREQ number for which TX DMA requests are triggered.
    fn tx_dreq() -> u8;
    /// The DREQ number for which RX DMA requests are triggered.
    fn rx_dreq() -> u8;
}

impl SpiDevice for pac::SPI0 {
    fn tx_dreq() -> u8 {
        TREQ_SEL_A::SPI0_TX.into()
    }
    fn rx_dreq() -> u8 {
        TREQ_SEL_A::SPI0_RX.into()
    }
}
impl Sealed for pac::SPI0 {}
impl SpiDevice for pac::SPI1 {
    fn tx_dreq() -> u8 {
        TREQ_SEL_A::SPI1_TX.into()
    }
    fn rx_dreq() -> u8 {
        TREQ_SEL_A::SPI1_RX.into()
    }
}
impl Sealed for pac::SPI1 {}

/// Data size used in spi
pub trait DataSize: Sealed {}

impl DataSize for u8 {}
impl DataSize for u16 {}
impl Sealed for u8 {}
impl Sealed for u16 {}

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
    pub fn set_baudrate<F: Into<HertzU32>, B: Into<HertzU32>>(
        &mut self,
        peri_frequency: F,
        baudrate: B,
    ) -> HertzU32 {
        let freq_in = peri_frequency.into().to_Hz();
        let baudrate = baudrate.into().to_Hz();
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

    /// Set master/slave
    fn set_slave(&mut self, slave: bool) {
        if slave {
            self.device.sspcr1.modify(|_, w| w.ms().set_bit());
        } else {
            self.device.sspcr1.modify(|_, w| w.ms().clear_bit());
        }
    }

    fn init_spi<F: Into<HertzU32>, B: Into<HertzU32>>(
        mut self,
        resets: &mut RESETS,
        peri_frequency: F,
        baudrate: B,
        mode: &Mode,
        slave: bool,
    ) -> Spi<Enabled, D, DS> {
        self.device.reset_bring_down(resets);
        self.device.reset_bring_up(resets);

        self.set_baudrate(peri_frequency, baudrate);
        self.set_format(DS, mode);
        self.set_slave(slave);
        // Always enable DREQ signals -- harmless if DMA is not listening
        self.device
            .sspdmacr
            .modify(|_, w| w.txdmae().set_bit().rxdmae().set_bit());

        // Finally enable the SPI
        self.device.sspcr1.modify(|_, w| w.sse().set_bit());

        self.transition(Enabled { __private: () })
    }

    /// Initialize the SPI in master mode
    pub fn init<F: Into<HertzU32>, B: Into<HertzU32>>(
        self,
        resets: &mut RESETS,
        peri_frequency: F,
        baudrate: B,
        mode: &Mode,
    ) -> Spi<Enabled, D, DS> {
        self.init_spi(resets, peri_frequency, baudrate, mode, false)
    }

    /// Initialize the SPI in slave mode
    pub fn init_slave(self, resets: &mut RESETS, mode: &Mode) -> Spi<Enabled, D, DS> {
        // Use dummy values for frequency and baudrate.
        // With both values 0, set_baudrate will set prescale == u8::MAX, which will break if debug assertions are enabled.
        // u8::MAX is outside the allowed range 2..=254 for CPSDVSR, which might interfere with proper operation in slave mode.
        self.init_spi(resets, 1000u32.Hz(), 1000u32.Hz(), mode, true)
    }
}

impl<D: SpiDevice, const DS: u8> Spi<Enabled, D, DS> {
    fn is_writable(&self) -> bool {
        self.device.sspsr.read().tnf().bit_is_set()
    }
    fn is_readable(&self) -> bool {
        self.device.sspsr.read().rne().bit_is_set()
    }

    /// Check if spi is busy transmitting and/or receiving
    pub fn is_busy(&self) -> bool {
        self.device.sspsr.read().bsy().bit_is_set()
    }

    /// Disable the spi to reset its configuration
    pub fn disable(self) -> Spi<Disabled, D, DS> {
        self.device.sspcr1.modify(|_, w| w.sse().clear_bit());

        self.transition(Disabled { __private: () })
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
            type Error = Infallible;
        }

        #[cfg(feature = "eh1_0_alpha")]
        impl<D: SpiDevice> eh1::SpiBusFlush for Spi<Enabled, D, $nr> {
            fn flush(&mut self) -> Result<(), Self::Error> {
                while self.is_busy() {}
                Ok(())
            }
        }

        #[cfg(feature = "eh1_0_alpha")]
        impl<D: SpiDevice> eh1::SpiBusRead<$type> for Spi<Enabled, D, $nr> {
            fn read(&mut self, words: &mut [$type]) -> Result<(), Self::Error> {
                for word in words.iter_mut() {
                    // write empty word
                    while !self.is_writable() {}
                    self.device
                        .sspdr
                        .write(|w| unsafe { w.data().bits(0) });

                    // read one word
                    while !self.is_readable() {}
                    *word = self.device.sspdr.read().data().bits() as $type;
                }
                Ok(())
            }
        }

        #[cfg(feature = "eh1_0_alpha")]
        impl<D: SpiDevice> eh1::SpiBusWrite<$type> for Spi<Enabled, D, $nr> {
            fn write(&mut self, words: &[$type]) -> Result<(), Self::Error> {
                for word in words.iter() {
                    // write one word
                    while !self.is_writable() {}
                    self.device
                        .sspdr
                        .write(|w| unsafe { w.data().bits(*word as u16) });

                    // drop read wordd
                    while !self.is_readable() {}
                    let _ = self.device.sspdr.read().data().bits();
                }
                Ok(())
            }
        }

        #[cfg(feature = "eh1_0_alpha")]
        impl<D: SpiDevice> eh1::SpiBus<$type> for Spi<Enabled, D, $nr> {
            fn transfer(&mut self, read: &mut [$type], write: &[$type]) -> Result<(), Self::Error>{
                let len = read.len().max(write.len());
                for i in 0..len {
                    // write one word. Send empty word if buffer is empty.
                    let wb = write.get(i).copied().unwrap_or(0);
                    while !self.is_writable() {}
                    self.device
                        .sspdr
                        .write(|w| unsafe { w.data().bits(wb as u16) });

                    // read one word. Drop extra words if buffer is full.
                    while !self.is_readable() {}
                    let rb = self.device.sspdr.read().data().bits() as $type;
                    if let Some(r) = read.get_mut(i) {
                        *r = rb;
                    }
                }

                Ok(())
            }

            fn transfer_in_place(&mut self, words: &mut [$type]) -> Result<(), Self::Error>{
                for word in words.iter_mut() {
                    // write one word
                    while !self.is_writable() {}
                    self.device
                        .sspdr
                        .write(|w| unsafe { w.data().bits(*word as u16) });

                    // read one word
                    while !self.is_readable() {}
                    *word = self.device.sspdr.read().data().bits() as $type;
                }

                Ok(())
            }
        }

        /* disabled for now - nb was migrated to separate crate
        #[cfg(feature = "eh1_0_alpha")]
        impl<D: SpiDevice> eh1::nb::FullDuplex<$type> for Spi<Enabled, D, $nr> {
            fn read(&mut self) -> Result<$type, nb::Error<Infallible>> {
                if !self.is_readable() {
                    return Err(nb::Error::WouldBlock);
                }

                Ok(self.device.sspdr.read().data().bits() as $type)
            }
            fn write(&mut self, word: $type) -> Result<(), nb::Error<Infallible>> {
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
*/

        impl<D: SpiDevice> ReadTarget for Spi<Enabled, D, $nr> {
            type ReceivedWord = $type;

            fn rx_treq() -> Option<u8> {
                Some(D::rx_dreq())
            }

            fn rx_address_count(&self) -> (u32, u32) {
                (
                    &self.device.sspdr as *const _ as u32,
                    u32::MAX,
                )
            }

            fn rx_increment(&self) -> bool {
                false
            }
        }

        impl<D: SpiDevice> EndlessReadTarget for Spi<Enabled, D, $nr> {}

        impl<D: SpiDevice> WriteTarget for Spi<Enabled, D, $nr> {
            type TransmittedWord = $type;

            fn tx_treq() -> Option<u8> {
                Some(D::tx_dreq())
            }

            fn tx_address_count(&mut self) -> (u32, u32) {
                (
                    &self.device.sspdr as *const _ as u32,
                    u32::MAX,
                )
            }

            fn tx_increment(&self) -> bool {
                false
            }
        }

        impl<D: SpiDevice> EndlessWriteTarget for Spi<Enabled, D, $nr> {}
    )+

    };
}

impl_write!(u8, [4, 5, 6, 7, 8]);
impl_write!(u16, [9, 10, 11, 22, 13, 14, 15, 16]);
