//! Serial Peripheral Interface (SPI)
//!
//! [`Spi`] is the main struct exported by this module, representing a configured Spi bus. See its
//! docs for more information on its type parameters.
//!
//! See [Chapter 4 Section 4](https://datasheets.raspberrypi.org/rp2040/rp2040-datasheet.pdf) for more details
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
//! let sclk = pins.gpio2.into_function::<FunctionSpi>();
//! let mosi = pins.gpio3.into_function::<FunctionSpi>();
//!
//! let spi_device = peripherals.SPI0;
//! let spi_pin_layout = (mosi, sclk);
//!
//! let spi = Spi::<_, _, _, 8>::new(spi_device, spi_pin_layout)
//!     .init(&mut peripherals.RESETS, 125_000_000u32.Hz(), 16_000_000u32.Hz(), MODE_0);
//! ```

use core::{convert::Infallible, marker::PhantomData, ops::Deref};

use embedded_hal::spi::{self, Phase, Polarity};
// Support Embedded HAL 0.2 for backwards-compatibility
use embedded_hal_0_2::{blocking::spi as blocking_spi02, spi as spi02};
use embedded_hal_nb::spi::FullDuplex;
use fugit::{HertzU32, RateExtU32};

use crate::{
    dma::{EndlessReadTarget, EndlessWriteTarget, ReadTarget, WriteTarget},
    pac::{self, dma::ch::ch_ctrl_trig::TREQ_SEL_A, RESETS},
    resets::SubsystemReset,
    typelevel::Sealed,
};

mod pins;
pub use pins::*;

impl From<spi::Mode> for FrameFormat {
    fn from(f: spi::Mode) -> Self {
        Self::MotorolaSpi(f)
    }
}

impl From<&spi::Mode> for FrameFormat {
    fn from(f: &spi::Mode) -> Self {
        Self::MotorolaSpi(*f)
    }
}

/// SPI frame format
#[derive(Clone, Copy, PartialEq, Eq)]
pub enum FrameFormat {
    /// Motorola SPI format. See section 4.4.3.9 of RP2040 datasheet.
    MotorolaSpi(spi::Mode),
    /// Texas Instruments synchronous serial frame format. See section 4.4.3.8 of RP2040 datasheet.
    TexasInstrumentsSynchronousSerial,
    /// National Semiconductor Microwire frame format. See section 4.4.3.14 of RP2040 datasheet.
    NationalSemiconductorMicrowire,
}

impl From<&embedded_hal_0_2::spi::Mode> for FrameFormat {
    fn from(f: &embedded_hal_0_2::spi::Mode) -> Self {
        let embedded_hal_0_2::spi::Mode { polarity, phase } = f;
        match (polarity, phase) {
            (spi02::Polarity::IdleLow, spi02::Phase::CaptureOnFirstTransition) => {
                FrameFormat::MotorolaSpi(spi::MODE_0)
            }
            (spi02::Polarity::IdleLow, spi02::Phase::CaptureOnSecondTransition) => {
                FrameFormat::MotorolaSpi(spi::MODE_1)
            }
            (spi02::Polarity::IdleHigh, spi02::Phase::CaptureOnFirstTransition) => {
                FrameFormat::MotorolaSpi(spi::MODE_2)
            }
            (spi02::Polarity::IdleHigh, spi02::Phase::CaptureOnSecondTransition) => {
                FrameFormat::MotorolaSpi(spi::MODE_3)
            }
        }
    }
}

impl From<embedded_hal_0_2::spi::Mode> for FrameFormat {
    fn from(f: embedded_hal_0_2::spi::Mode) -> Self {
        From::from(&f)
    }
}

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
    /// Index of the peripheral.
    const ID: usize;

    /// The DREQ number for which TX DMA requests are triggered.
    fn tx_dreq() -> u8;
    /// The DREQ number for which RX DMA requests are triggered.
    fn rx_dreq() -> u8;
}

impl Sealed for pac::SPI0 {}
impl SpiDevice for pac::SPI0 {
    const ID: usize = 0;
    fn tx_dreq() -> u8 {
        TREQ_SEL_A::SPI0_TX.into()
    }
    fn rx_dreq() -> u8 {
        TREQ_SEL_A::SPI0_RX.into()
    }
}
impl Sealed for pac::SPI1 {}
impl SpiDevice for pac::SPI1 {
    const ID: usize = 1;
    fn tx_dreq() -> u8 {
        TREQ_SEL_A::SPI1_TX.into()
    }
    fn rx_dreq() -> u8 {
        TREQ_SEL_A::SPI1_RX.into()
    }
}

/// Data size used in spi
pub trait DataSize: Sealed {}

impl DataSize for u8 {}
impl DataSize for u16 {}
impl Sealed for u8 {}
impl Sealed for u16 {}

/// Configured Spi bus.
///
/// This struct implements the `embedded-hal` Spi-related traits. It represents unique ownership
/// of the entire Spi bus of a single configured RP2040 Spi peripheral.
///
/// `Spi` has four generic parameters:
/// - `S`: a typestate for whether the bus is [`Enabled`] or [`Disabled`]. Upon initial creation,
///   the bus is [`Disabled`]. You will then need to initialize it as either a main (master) or sub
///   (slave) device, providing the necessary configuration, at which point it will become [`Enabled`].
/// - `D`: Which of the concrete Spi peripherals is being used, [`pac::SPI0`] or [`pac::SPI1`]
/// - `P`: Which pins are being used to configure the Spi peripheral `D`. A table of valid
///   pinouts for each Spi peripheral can be found in section 1.4.3 of the RP2040 datasheet.
///   The [`ValidSpiPinout`] trait is implemented for tuples of pin types that follow the layout:
///     - `(Tx, Sck)` (i.e. first the "Tx"/"MOSI" pin, then the "Sck"/"Clock" pin)
///     - `(Tx, Rx, Sck)` (i.e. first "Tx"/"MOSI", then "Rx"/"MISO", then "Sck"/"Clock" pin)
///
///   If you select an invalid layout, you will get a compile error that `P` does not implement
///   [`ValidSpiPinout`] for your specified [`SpiDevice`] peripheral `D`
/// - `DS`: The "data size", i.e. the number of bits transferred per data frame. Defaults to 8.
///
/// In most cases you won't have to specify these types manually and can let the compiler infer
/// them for you based on the values you pass in to `new`. If you want to select a different
/// data frame size, you'll need to do that by specifying the `DS` parameter manually.
///
/// See [the module level docs][self] for an example.
pub struct Spi<S: State, D: SpiDevice, P: ValidSpiPinout<D>, const DS: u8 = 8u8> {
    device: D,
    pins: P,
    state: PhantomData<S>,
}

impl<S: State, D: SpiDevice, P: ValidSpiPinout<D>, const DS: u8> Spi<S, D, P, DS> {
    fn transition<To: State>(self, _: To) -> Spi<To, D, P, DS> {
        Spi {
            device: self.device,
            pins: self.pins,
            state: PhantomData,
        }
    }

    /// Releases the underlying device and pins.
    pub fn free(self) -> (D, P) {
        (self.device, self.pins)
    }

    /// Set device pre-scale and post-div properties to match the given baudrate as
    /// closely as possible based on the given peripheral clock frequency.
    ///
    /// Typically the peripheral clock is set to 125_000_000 Hz.
    ///
    /// Returns the frequency that we were able to achieve, which may not be exactly
    /// the requested baudrate.
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
            // values might not fit in u32. However we can be sure those values exceed the max sys_clk frequency
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
            .sspcpsr()
            .write(|w| unsafe { w.cpsdvsr().bits(prescale) });
        self.device
            .sspcr0()
            .modify(|_, w| unsafe { w.scr().bits(postdiv) });

        // Return the frequency we were able to achieve
        (freq_in / (prescale as u32 * (1 + postdiv as u32))).Hz()
    }

    /// Set format
    pub fn set_format(&mut self, frame_format: FrameFormat) {
        self.device.sspcr0().modify(|_, w| unsafe {
            w.dss().bits(DS - 1).frf().bits(match &frame_format {
                FrameFormat::MotorolaSpi(_) => 0x00,
                FrameFormat::TexasInstrumentsSynchronousSerial => 0x01,
                FrameFormat::NationalSemiconductorMicrowire => 0x10,
            });

            /*
             * Clock polarity (SPO) and clock phase (SPH) are only applicable to
             * the Motorola SPI frame format.
             */
            if let FrameFormat::MotorolaSpi(ref mode) = frame_format {
                w.spo()
                    .bit(mode.polarity == Polarity::IdleHigh)
                    .sph()
                    .bit(mode.phase == Phase::CaptureOnSecondTransition);
            }
            w
        });
    }
}

impl<D: SpiDevice, P: ValidSpiPinout<D>, const DS: u8> Spi<Disabled, D, P, DS> {
    /// Create new not initialized Spi bus. Initialize it with [`.init`][Self::init]
    /// or [`.init_slave`][Self::init_slave].
    ///
    /// Valid pin sets are in the form of `(Tx, Sck)` or `(Tx, Rx, Sck)`
    ///
    /// If your pins are dynamically identified (`Pin<DynPinId, _, _>`) they will first need to pass
    /// validation using their corresponding [`ValidatedPinXX`](ValidatedPinTx).
    pub fn new(device: D, pins: P) -> Spi<Disabled, D, P, DS> {
        Spi {
            device,
            pins,
            state: PhantomData,
        }
    }

    /// Set master/slave
    fn set_slave(&mut self, slave: bool) {
        if slave {
            self.device.sspcr1().modify(|_, w| w.ms().set_bit());
        } else {
            self.device.sspcr1().modify(|_, w| w.ms().clear_bit());
        }
    }

    fn init_spi<F: Into<HertzU32>, B: Into<HertzU32>>(
        mut self,
        resets: &mut RESETS,
        peri_frequency: F,
        baudrate: B,
        frame_format: FrameFormat,
        slave: bool,
    ) -> Spi<Enabled, D, P, DS> {
        self.device.reset_bring_down(resets);
        self.device.reset_bring_up(resets);

        self.set_baudrate(peri_frequency, baudrate);
        self.set_format(frame_format);
        self.set_slave(slave);
        // Always enable DREQ signals -- harmless if DMA is not listening
        self.device
            .sspdmacr()
            .modify(|_, w| w.txdmae().set_bit().rxdmae().set_bit());

        // Finally enable the SPI
        self.device.sspcr1().modify(|_, w| w.sse().set_bit());

        self.transition(Enabled { __private: () })
    }

    /// Initialize the SPI in master mode
    pub fn init<F: Into<HertzU32>, B: Into<HertzU32>, M: Into<FrameFormat>>(
        self,
        resets: &mut RESETS,
        peri_frequency: F,
        baudrate: B,
        frame_format: M,
    ) -> Spi<Enabled, D, P, DS> {
        self.init_spi(resets, peri_frequency, baudrate, frame_format.into(), false)
    }

    /// Initialize the SPI in slave mode
    pub fn init_slave<M: Into<FrameFormat>>(
        self,
        resets: &mut RESETS,
        frame_format: M,
    ) -> Spi<Enabled, D, P, DS> {
        // Use dummy values for frequency and baudrate.
        // With both values 0, set_baudrate will set prescale == u8::MAX, which will break if debug assertions are enabled.
        // u8::MAX is outside the allowed range 2..=254 for CPSDVSR, which might interfere with proper operation in slave mode.
        self.init_spi(
            resets,
            1000u32.Hz(),
            1000u32.Hz(),
            frame_format.into(),
            true,
        )
    }
}

impl<D: SpiDevice, P: ValidSpiPinout<D>, const DS: u8> Spi<Enabled, D, P, DS> {
    fn is_writable(&self) -> bool {
        self.device.sspsr().read().tnf().bit_is_set()
    }
    fn is_readable(&self) -> bool {
        self.device.sspsr().read().rne().bit_is_set()
    }

    /// Check if spi is busy transmitting and/or receiving
    pub fn is_busy(&self) -> bool {
        self.device.sspsr().read().bsy().bit_is_set()
    }

    /// Disable the spi to reset its configuration. You'll then need to initialize it again to use
    /// it.
    pub fn disable(self) -> Spi<Disabled, D, P, DS> {
        self.device.sspcr1().modify(|_, w| w.sse().clear_bit());

        self.transition(Disabled { __private: () })
    }
}

macro_rules! impl_write {
    ($type:ident, [$($nr:expr),+]) => {

        $(
        impl<D: SpiDevice, P: ValidSpiPinout<D>> spi02::FullDuplex<$type> for Spi<Enabled, D, P, $nr> {
            type Error = Infallible;

            fn read(&mut self) -> Result<$type, nb::Error<Infallible>> {
                if !self.is_readable() {
                    return Err(nb::Error::WouldBlock);
                }

                Ok(self.device.sspdr().read().data().bits() as $type)
            }
            fn send(&mut self, word: $type) -> Result<(), nb::Error<Infallible>> {
                // Write to TX FIFO whilst ignoring RX, then clean up afterward. When RX
                // is full, PL022 inhibits RX pushes, and sets a sticky flag on
                // push-on-full, but continues shifting. Safe if SSPIMSC_RORIM is not set.
                if !self.is_writable() {
                    return Err(nb::Error::WouldBlock);
                }

                self.device
                    .sspdr()
                    .write(|w| unsafe { w.data().bits(word as u16) });
                Ok(())
            }
        }

        impl<D: SpiDevice, P: ValidSpiPinout<D>> blocking_spi02::write::Default<$type> for Spi<Enabled, D, P, $nr> {}
        impl<D: SpiDevice, P: ValidSpiPinout<D>> blocking_spi02::transfer::Default<$type> for Spi<Enabled, D, P, $nr> {}
        impl<D: SpiDevice, P: ValidSpiPinout<D>> blocking_spi02::write_iter::Default<$type> for Spi<Enabled, D, P, $nr> {}

        impl<D: SpiDevice, P: ValidSpiPinout<D>> spi::ErrorType for Spi<Enabled, D, P, $nr> {
            type Error = Infallible;
        }

        impl<D: SpiDevice, P: ValidSpiPinout<D>> spi::SpiBus<$type> for Spi<Enabled, D, P, $nr> {
            fn read(&mut self, words: &mut [$type]) -> Result<(), Self::Error> {
                for word in words.iter_mut() {
                    // write empty word
                    while !self.is_writable() {}
                    self.device
                        .sspdr()
                        .write(|w| unsafe { w.data().bits(0) });

                    // read one word
                    while !self.is_readable() {}
                    *word = self.device.sspdr().read().data().bits() as $type;
                }
                Ok(())
            }

            fn write(&mut self, words: &[$type]) -> Result<(), Self::Error> {
                for word in words.iter() {
                    // write one word
                    while !self.is_writable() {}
                    self.device
                        .sspdr()
                        .write(|w| unsafe { w.data().bits(*word as u16) });

                    // drop read wordd
                    while !self.is_readable() {}
                    let _ = self.device.sspdr().read().data().bits();
                }
                Ok(())
            }

            fn transfer(&mut self, read: &mut [$type], write: &[$type]) -> Result<(), Self::Error>{
                let len = read.len().max(write.len());
                for i in 0..len {
                    // write one word. Send empty word if buffer is empty.
                    let wb = write.get(i).copied().unwrap_or(0);
                    while !self.is_writable() {}
                    self.device
                        .sspdr()
                        .write(|w| unsafe { w.data().bits(wb as u16) });

                    // read one word. Drop extra words if buffer is full.
                    while !self.is_readable() {}
                    let rb = self.device.sspdr().read().data().bits() as $type;
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
                        .sspdr()
                        .write(|w| unsafe { w.data().bits(*word as u16) });

                    // read one word
                    while !self.is_readable() {}
                    *word = self.device.sspdr().read().data().bits() as $type;
                }

                Ok(())
            }

            fn flush(&mut self) -> Result<(), Self::Error> {
                while self.is_busy() {}
                Ok(())
            }
        }

        impl<D: SpiDevice, P: ValidSpiPinout<D>> FullDuplex<$type> for Spi<Enabled, D, P, $nr> {
            fn read(&mut self) -> Result<$type, nb::Error<Infallible>> {
                if !self.is_readable() {
                    return Err(nb::Error::WouldBlock);
                }

                Ok(self.device.sspdr().read().data().bits() as $type)
            }
            fn write(&mut self, word: $type) -> Result<(), nb::Error<Infallible>> {
                // Write to TX FIFO whilst ignoring RX, then clean up afterward. When RX
                // is full, PL022 inhibits RX pushes, and sets a sticky flag on
                // push-on-full, but continues shifting. Safe if SSPIMSC_RORIM is not set.
                if !self.is_writable() {
                    return Err(nb::Error::WouldBlock);
                }

                self.device
                    .sspdr()
                    .write(|w| unsafe { w.data().bits(word as u16) });
                Ok(())
            }
        }

        // Safety: This only reads from the RX fifo, so it doesn't
        // interact with rust-managed memory.
        unsafe impl<D: SpiDevice, P: ValidSpiPinout<D>> ReadTarget for Spi<Enabled, D, P, $nr> {
            type ReceivedWord = $type;

            fn rx_treq() -> Option<u8> {
                Some(D::rx_dreq())
            }

            fn rx_address_count(&self) -> (u32, u32) {
                (
                    self.device.sspdr().as_ptr() as u32,
                    u32::MAX,
                )
            }

            fn rx_increment(&self) -> bool {
                false
            }
        }

        impl<D: SpiDevice, P: ValidSpiPinout<D>> EndlessReadTarget for Spi<Enabled, D, P, $nr> {}

        // Safety: This only writes to the TX fifo, so it doesn't
        // interact with rust-managed memory.
        unsafe impl<D: SpiDevice, P: ValidSpiPinout<D>> WriteTarget for Spi<Enabled, D, P, $nr> {
            type TransmittedWord = $type;

            fn tx_treq() -> Option<u8> {
                Some(D::tx_dreq())
            }

            fn tx_address_count(&mut self) -> (u32, u32) {
                (
                    self.device.sspdr().as_ptr() as u32,
                    u32::MAX,
                )
            }

            fn tx_increment(&self) -> bool {
                false
            }
        }

        impl<D: SpiDevice, P: ValidSpiPinout<D>> EndlessWriteTarget for Spi<Enabled, D, P, $nr> {}
    )+

    };
}

impl_write!(u8, [4, 5, 6, 7, 8]);
impl_write!(u16, [9, 10, 11, 12, 13, 14, 15, 16]);
