//! Analog-Digital Converter (ADC)
//!
//! See [Section 12.4](https://rptl.io/rp2350-datasheet#section_adc) of the datasheet for more details
//!
//! ## Usage
//!
//! Capture ADC reading from a pin:

//! ```no_run
//! // Embedded HAL 1.0.0 doesn't have an ADC trait, so use the one from 0.2
//! use embedded_hal_0_2::adc::OneShot;
//! use rp235x_hal::{self as hal, adc::Adc, adc::AdcPin, gpio::Pins, Sio};
//! let mut peripherals = hal::pac::Peripherals::take().unwrap();
//! let sio = Sio::new(peripherals.SIO);
//! let pins = Pins::new(
//!     peripherals.IO_BANK0,
//!     peripherals.PADS_BANK0,
//!     sio.gpio_bank0,
//!     &mut peripherals.RESETS,
//! );
//! // Enable adc
//! let mut adc = Adc::new(peripherals.ADC, &mut peripherals.RESETS);
//! // Configure one of the pins as an ADC input
//! let mut adc_pin_0 = AdcPin::new(pins.gpio26.into_floating_input()).unwrap();
//! // Read the ADC counts from the ADC channel
//! let pin_adc_counts: u16 = adc.read(&mut adc_pin_0).unwrap();
//! ```
//!
//! Capture ADC reading from temperature sensor. Note that this needs conversion to be a real-world temperature.
//!
//! ```no_run
//! // Embedded HAL 1.0.0 doesn't have an ADC trait, so use the one from 0.2
//! use embedded_hal_0_2::adc::OneShot;
//! use rp235x_hal::{self as hal, adc::Adc, gpio::Pins, Sio};
//! let mut peripherals = hal::pac::Peripherals::take().unwrap();
//! let sio = Sio::new(peripherals.SIO);
//! let pins = Pins::new(
//!     peripherals.IO_BANK0,
//!     peripherals.PADS_BANK0,
//!     sio.gpio_bank0,
//!     &mut peripherals.RESETS,
//! );
//! // Enable adc
//! let mut adc = Adc::new(peripherals.ADC, &mut peripherals.RESETS);
//! // Enable the temperature sensor
//! let mut temperature_sensor = adc.take_temp_sensor().unwrap();
//! // Read the ADC counts from the ADC channel
//! let temperature_adc_counts: u16 = adc.read(&mut temperature_sensor).unwrap();
//! ```
//!
//! See [examples/adc.rs](https://github.com/rp-rs/rp-hal/tree/main/rp235x-hal-examples/src/bin/adc.rs) and
//! [pimoroni_pico_explorer_showcase.rs](https://github.com/rp-rs/rp-hal-boards/tree/main/boards/pimoroni-pico-explorer/examples/pimoroni_pico_explorer_showcase.rs) for more complete examples
//!
//! ### Free running mode with FIFO
//!
//! In free-running mode the ADC automatically captures samples in regular intervals.
//! The samples are written to a FIFO, from which they can be retrieved.
//!
//! ```no_run
//! # use rp235x_hal::{self as hal, adc::Adc, gpio::Pins, pac, Sio};
//! let mut peripherals = hal::pac::Peripherals::take().unwrap();
//! let sio = Sio::new(peripherals.SIO);
//! let pins = Pins::new(
//!     peripherals.IO_BANK0,
//!     peripherals.PADS_BANK0,
//!     sio.gpio_bank0,
//!     &mut peripherals.RESETS,
//! );
//! // Enable adc
//! let mut adc = Adc::new(peripherals.ADC, &mut peripherals.RESETS);
//! // Enable the temperature sensor
//! let mut temperature_sensor = adc.take_temp_sensor().unwrap();
//!
//! // Configure & start capturing to the fifo:
//! let mut fifo = adc
//!     .build_fifo()
//!     .clock_divider(0, 0) // sample as fast as possible (500ksps. This is the default)
//!     .set_channel(&mut temperature_sensor)
//!     .start();
//!
//! loop {
//!     if fifo.len() > 0 {
//!         // Read one captured ADC sample from the FIFO:
//!         let temperature_adc_counts: u16 = fifo.read();
//!     }
//! }
//! ```
//! See [examples/adc_fifo_poll.rs](https://github.com/rp-rs/rp-hal/tree/main/rp235x-hal-examples/src/bin/adc_fifo_poll.rs) for a more complete example.
//!
//! ### Using DMA
//!
//! When the ADC is in free-running mode, it's possible to use DMA to transfer data from the FIFO elsewhere, without having to read the FIFO manually.
//!
//! This requires a number of steps:
//! 1. Build an `AdcFifo`, with DMA enabled ([`AdcFifoBuilder::enable_dma`])
//! 2. Use [`AdcFifoBuilder::prepare`] instead of [`AdcFifoBuilder::start`], so that the FIFO is created in `paused` state
//! 3. Start a DMA transfer ([`dma::single_buffer::Transfer`], [`dma::double_buffer::Transfer`], ...), using the [`AdcFifo::dma_read_target`] as the source (`from` parameter)
//! 4. Finally unpause the FIFO by calling [`AdcFifo::resume`], to start capturing
//!
//! Example:
//! ```no_run
//! use rp235x_hal::{self as hal, singleton, adc::Adc, gpio::Pins, pac, Sio, dma::{single_buffer, DMAExt}};
//! let mut peripherals = hal::pac::Peripherals::take().unwrap();
//! let sio = Sio::new(peripherals.SIO);
//! let pins = Pins::new(peripherals.IO_BANK0, peripherals.PADS_BANK0, sio.gpio_bank0, &mut peripherals.RESETS);
//! let dma = peripherals.DMA.split(&mut peripherals.RESETS);
//! // Enable adc
//! let mut adc = Adc::new(peripherals.ADC, &mut peripherals.RESETS);
//! // Enable the temperature sensor
//! let mut temperature_sensor = adc.take_temp_sensor().unwrap();
//!
//! // Configure & start capturing to the fifo:
//! let mut fifo = adc.build_fifo()
//!   .clock_divider(0, 0) // sample as fast as possible (500ksps. This is the default)
//!   .set_channel(&mut temperature_sensor)
//!   .enable_dma()
//!   .prepare();
//!
//! // Set up a buffer, where the samples should be written:
//! let buf = singleton!(: [u16; 500] = [0; 500]).unwrap();
//!
//! // Start DMA transfer
//! let transfer = single_buffer::Config::new(dma.ch0, fifo.dma_read_target(), buf).start();
//!
//! // Resume the FIFO to start capturing
//! fifo.resume();
//!
//! // Wait for the transfer to complete:
//! let (ch, adc_read_target, buf) = transfer.wait();
//!
//! // do something with `buf` (it now contains 500 samples read from the ADC)
//! //...
//! ```
//! //! See [examples/adc_fifo_dma.rs](https://github.com/rp-rs/rp-hal/tree/main/rp235x-hal-examples/src/bin/adc_fifo_dma.rs) for a more complete example.
//!
//! ### Free running mode without FIFO
//!
//! While free-running mode is usually used in combination with a FIFO, there are
//! use cases where it can be used without. For example, if you want to be able to
//! get the latest available sample at any point in time, and without waiting 96 ADC clock
//! cycles (2µs).
//!
//! In this case, you can just enable free-running mode on it's own. The ADC will
//! continuously do ADC conversions. The ones not read will just be discarded, but it's
//! always possible to read the latest value, without additional delay:
//!
//! ```no_run
//! use rp235x_hal::{self as hal, adc::Adc, adc::AdcPin, gpio::Pins, Sio};
//! // Embedded HAL 1.0.0 doesn't have an ADC trait, so use the one from 0.2
//! use embedded_hal_0_2::adc::OneShot;
//! let mut peripherals = hal::pac::Peripherals::take().unwrap();
//! let sio = Sio::new(peripherals.SIO);
//! let pins = Pins::new(
//!     peripherals.IO_BANK0,
//!     peripherals.PADS_BANK0,
//!     sio.gpio_bank0,
//!     &mut peripherals.RESETS,
//! );
//! // Enable adc
//! let mut adc = Adc::new(peripherals.ADC, &mut peripherals.RESETS);
//! // Configure one of the pins as an ADC input
//! let mut adc_pin_0 = AdcPin::new(pins.gpio26.into_floating_input()).unwrap();
//! // Enable free-running mode
//! adc.free_running(&adc_pin_0);
//! // Read the ADC counts from the ADC channel whenever necessary
//! loop {
//!     let pin_adc_counts: u16 = adc.read_single();
//!     // Do time critical stuff
//! }
//! ```

use core::convert::Infallible;
use core::marker::PhantomData;
// Embedded HAL 1.0.0 doesn't have an ADC trait, so use the one from 0.2
use embedded_hal_0_2::adc::{Channel, OneShot};

use crate::{
    dma,
    gpio::{
        bank0::{Gpio26, Gpio27, Gpio28, Gpio29},
        AnyPin, DynBankId, DynPinId, Function, OutputEnableOverride, Pin, PullType, ValidFunction,
    },
    pac::{dma::ch::ch_ctrl_trig::TREQ_SEL_A, ADC, RESETS},
    resets::SubsystemReset,
    typelevel::Sealed,
};

const TEMPERATURE_SENSOR_CHANNEL: u8 = 4;

/// The pin was invalid for the requested operation
#[derive(Debug)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub struct InvalidPinError;

/// A pin locked in use with the ADC.
pub struct AdcPin<P>
where
    P: AnyPin,
{
    pin: P,
    saved_output_disable: bool,
    saved_input_enable: bool,
}

impl<P> AdcPin<P>
where
    P: AnyPin,
{
    /// Captures the pin to be used with an ADC and disables its digital circuitry.
    pub fn new(pin: P) -> Result<Self, InvalidPinError> {
        let pin_id = pin.borrow().id();
        if (26..=29).contains(&pin_id.num) && pin_id.bank == DynBankId::Bank0 {
            let mut p = pin.into();
            let (od, ie) = (p.get_output_disable(), p.get_input_enable());
            p.set_output_enable_override(OutputEnableOverride::Disable);
            p.set_input_enable(false);
            Ok(Self {
                pin: P::from(p),
                saved_output_disable: od,
                saved_input_enable: ie,
            })
        } else {
            Err(InvalidPinError)
        }
    }

    /// Release the pin and restore its digital circuitry's state.
    pub fn release(self) -> P {
        let mut p = self.pin.into();
        p.set_output_disable(self.saved_output_disable);
        p.set_input_enable(self.saved_input_enable);
        P::from(p)
    }

    /// Returns the ADC channel of this AdcPin.
    pub fn channel(&self) -> u8 {
        let pin_id = self.pin.borrow().id();
        // Self::new() makes sure that this is a valid channel number
        pin_id.num - 26
    }
}

/// Trait for entities that can be used as ADC channels.
///
/// This is implemented by [`AdcPin`] and by [`TempSense`].
/// The trait is sealed and can't be implemented in other crates.
pub trait AdcChannel: Sealed {
    /// Get the channel id used to configure the ADC peripheral.
    fn channel(&self) -> u8;
}

impl<P: AnyPin> Sealed for AdcPin<P> {}
impl<P: AnyPin> AdcChannel for AdcPin<P> {
    fn channel(&self) -> u8 {
        self.channel()
    }
}

impl Sealed for TempSense {}
impl AdcChannel for TempSense {
    fn channel(&self) -> u8 {
        TEMPERATURE_SENSOR_CHANNEL
    }
}

macro_rules! channel {
    ($pin:ident, $channel:expr) => {
        impl<F: Function, M: PullType> Channel<Adc> for AdcPin<Pin<$pin, F, M>>
        where
            $pin: crate::gpio::ValidFunction<F>,
        {
            type ID = u8; // ADC channels are identified numerically

            fn channel() -> u8 {
                $channel
            }
        }
    };
}

channel!(Gpio26, 0);
channel!(Gpio27, 1);
channel!(Gpio28, 2);
channel!(Gpio29, 3);

impl<F: Function, M: PullType> Channel<Adc> for AdcPin<Pin<DynPinId, F, M>>
where
    DynPinId: crate::gpio::ValidFunction<F>,
{
    type ID = (); // ADC channels are identified at run time
    fn channel() {}
}

/// Internal temperature sensor type
pub struct TempSense {
    __private: (),
}

impl Channel<Adc> for TempSense {
    type ID = u8; // ADC channels are identified numerically

    fn channel() -> u8 {
        TEMPERATURE_SENSOR_CHANNEL
    }
}

/// Analog to Digital Convertor (ADC).
///
/// Represents an ADC within the RP2350. Each ADC has multiple channels, and each
/// channel is either associated with a specific GPIO pin or attached to the internal
/// temperature sensor. You should put the relevant pin into ADC mode by creating an
/// [`AdcPin`] object with it, or you can put the ADC into `Temperature Sensing Mode`
/// by calling [`Adc::take_temp_sensor()`]. Either way, the resulting objects can be
/// passed to the [`OneShot::read()`][a] trait method to actually do the read.
///
/// [a]: embedded_hal_0_2::adc::OneShot::read
pub struct Adc {
    device: ADC,
}

impl Adc {
    /// Create new adc struct and bring up adc
    pub fn new(device: ADC, resets: &mut RESETS) -> Self {
        device.reset_bring_down(resets);
        device.reset_bring_up(resets);

        // Enable adc
        device.cs().write(|w| w.en().set_bit());

        // Wait for adc ready
        while !device.cs().read().ready().bit_is_set() {}

        Self { device }
    }

    /// Free underlying register block
    pub fn free(self) -> ADC {
        self.device
    }

    /// Read the most recently sampled ADC value
    ///
    /// This function does not wait for the current conversion to finish.
    /// If a conversion is still in progress, it returns the result of the
    /// previous one.
    ///
    /// It also doesn't trigger a new conversion.
    pub fn read_single(&self) -> u16 {
        self.device.result().read().result().bits()
    }

    /// Enable temperature sensor, returns a channel to use.
    ///
    /// This can only be done once before calling [`Adc::disable_temp_sensor()`]. If the sensor has already
    /// been enabled, this method will panic.
    #[deprecated(
        note = "This method may panic, use `take_temp_sensor()` instead.",
        since = "0.9.0"
    )]
    pub fn enable_temp_sensor(&mut self) -> TempSense {
        self.take_temp_sensor()
            .expect("Temp sensor is already enabled.")
    }

    /// Enable temperature sensor, returns a channel to use
    ///
    /// If the sensor has already been enabled, this method returns `None`.
    pub fn take_temp_sensor(&mut self) -> Option<TempSense> {
        let mut disabled = false;
        self.device.cs().modify(|r, w| {
            disabled = r.ts_en().bit_is_clear();
            // if bit was already set, this is a nop
            w.ts_en().set_bit()
        });
        disabled.then_some(TempSense { __private: () })
    }

    /// Disable temperature sensor, consumes channel
    pub fn disable_temp_sensor(&mut self, _: TempSense) {
        self.device.cs().modify(|_, w| w.ts_en().clear_bit());
    }

    /// Start configuring free-running mode, and set up the FIFO
    ///
    /// The [`AdcFifoBuilder`] returned by this method can be used
    /// to configure capture options, like sample rate, channels to
    /// capture from etc.
    ///
    /// Capturing is started by calling [`AdcFifoBuilder::start`], which
    /// returns an [`AdcFifo`] to read from.
    pub fn build_fifo(&mut self) -> AdcFifoBuilder<'_, u16> {
        AdcFifoBuilder {
            adc: self,
            marker: PhantomData,
        }
    }

    /// Enable free-running mode by setting the start_many flag.
    pub fn free_running(&mut self, pin: &dyn AdcChannel) {
        self.device.cs().modify(|_, w| {
            unsafe {
                w.ainsel().bits(pin.channel());
            }
            w.start_many().set_bit();
            w
        });
    }

    /// Disable free-running mode by unsetting the start_many flag.
    pub fn stop(&mut self) {
        self.device.cs().modify(|_, w| w.start_many().clear_bit());
    }

    fn inner_read(&mut self, chan: u8) -> u16 {
        self.wait_ready();

        self.device
            .cs()
            .modify(|_, w| unsafe { w.ainsel().bits(chan).start_once().set_bit() });

        self.wait_ready();

        self.read_single()
    }

    /// Wait for the ADC to become ready.
    ///
    /// Also returns immediately if start_many is set, to avoid indefinite blocking.
    pub fn wait_ready(&self) {
        while !self.is_ready_or_free_running() {
            core::hint::spin_loop();
        }
    }

    fn is_ready_or_free_running(&self) -> bool {
        let cs = self.device.cs().read();
        cs.ready().bit_is_set() || cs.start_many().bit_is_set()
    }

    /// Returns true if the ADC is ready for the next conversion.
    ///
    /// This implies that any previous conversion has finished.
    pub fn is_ready(&self) -> bool {
        self.device.cs().read().ready().bit_is_set()
    }
}

// Implementation for TempSense and type-checked pins
impl<WORD, SRC> OneShot<Adc, WORD, SRC> for Adc
where
    WORD: From<u16>,
    SRC: Channel<Adc, ID = u8>,
{
    type Error = Infallible;

    fn read(&mut self, _pin: &mut SRC) -> nb::Result<WORD, Self::Error> {
        let chan = SRC::channel();

        Ok(self.inner_read(chan).into())
    }
}

// Implementation for dyn-pins
impl<WORD, F, M> OneShot<Adc, WORD, AdcPin<Pin<DynPinId, F, M>>> for Adc
where
    WORD: From<u16>,
    F: Function,
    M: PullType,
    DynPinId: ValidFunction<F>,
    AdcPin<Pin<DynPinId, F, M>>: Channel<Adc, ID = ()>,
{
    type Error = Infallible;

    fn read(&mut self, pin: &mut AdcPin<Pin<DynPinId, F, M>>) -> nb::Result<WORD, Self::Error> {
        Ok(self.inner_read(pin.channel()).into())
    }
}

/// Used to configure & build an [`AdcFifo`]
///
/// See [`Adc::build_fifo`] for details, as well as the `adc_fifo_*` [examples](https://github.com/rp-rs/rp-hal/tree/main/rp235x-hal/examples).
pub struct AdcFifoBuilder<'a, Word> {
    adc: &'a mut Adc,
    marker: PhantomData<Word>,
}

impl<'a, Word> AdcFifoBuilder<'a, Word> {
    /// Manually set clock divider to control sample rate
    ///
    /// The ADC is tied to the USB clock, normally running at 48MHz. ADC
    /// conversion happens at 96 cycles per sample, so with the dividers both
    /// set to 0 (the default) the sample rate will be `48MHz / 96 = 500ksps`.
    ///
    /// Setting the `int` and / or `frac` dividers will hold off between
    /// samples, leading to an effective rate of:
    ///
    /// ```text
    ///  rate = 48MHz / (1 + int + (frac / 256))
    /// ```
    ///
    /// To determine the required `int` and `frac` values for a given target
    /// rate, use these equations:
    ///
    /// ```text
    ///  int = floor((48MHz / rate) - 1)
    ///  frac = round(256 * ((48MHz / rate) - 1 - int))
    /// ```
    ///
    /// Some examples:
    ///
    /// | Target rate | `int`   | `frac` |
    /// |-------------|---------|--------|
    /// | 1000sps     | `47999` |    `0` |
    /// | 1024sps     | `46874` |    `0` |
    /// | 1337sps     | `35900` |   `70` |
    /// | 4096sps     | `11717` |  `192` |
    /// | 96ksps      |   `499` |    `0` |
    ///
    /// Since each conversion takes 96 cycles, setting `int` to anything below
    /// 96 does not make a difference, and leads to the same result as setting
    /// it to 0.
    ///
    /// The lowest possible rate is 732.41Hz, attainable by setting `int =
    /// 0xFFFF, frac = 0xFF`.
    ///
    /// For more details, please refer to [Section
    /// 12.4.3.2](https://rptl.io/rp2350-datasheet#section_adc) in the RP2350
    /// datasheet.
    pub fn clock_divider(self, int: u16, frac: u8) -> Self {
        self.adc
            .device
            .div()
            .modify(|_, w| unsafe { w.int().bits(int).frac().bits(frac) });
        self
    }

    /// Select ADC input channel to sample from
    ///
    /// If round-robin mode is used, this will only affect the first sample.
    ///
    /// The given `pin` can either be one of the ADC inputs (GPIO26-28) or the
    /// internal temperature sensor (retrieved via [`Adc::take_temp_sensor`]).
    pub fn set_channel<P: AdcChannel>(self, pin: &mut P) -> Self {
        self.adc
            .device
            .cs()
            .modify(|_, w| unsafe { w.ainsel().bits(pin.channel()) });
        self
    }

    /// Set channels to use for round-robin mode
    ///
    /// Takes a tuple of channels, like `(&mut adc_pin, &mut temp_sense)`.
    ///
    /// **NOTE:** *The order in which the channels are specified has no effect!
    /// Channels are always sampled in increasing order, by their channel number (Channel 0, Channel 1, ...).*
    pub fn round_robin<T: Into<RoundRobin>>(self, selected_channels: T) -> Self {
        let RoundRobin(bits) = selected_channels.into();
        self.adc
            .device
            .cs()
            .modify(|_, w| unsafe { w.rrobin().bits(bits as u16) });
        self
    }

    /// Enable the FIFO interrupt ([`ADC_IRQ_FIFO`](crate::pac::Interrupt::ADC_IRQ_FIFO))
    ///
    /// It will be triggered whenever there are at least `threshold` samples waiting in the FIFO.
    pub fn enable_interrupt(self, threshold: u8) -> Self {
        self.adc.device.inte().modify(|_, w| w.fifo().set_bit());
        self.adc
            .device
            .fcs()
            .modify(|_, w| unsafe { w.thresh().bits(threshold) });
        self
    }

    /// Shift values to produce 8 bit samples (discarding the lower 4 bits).
    ///
    /// Normally the ADC uses 12 bits of precision, packed into a u16.
    /// Shifting the values loses some precision, but produces smaller samples.
    ///
    /// When this method has been called, the resulting fifo's `read` method returns u8.
    pub fn shift_8bit(self) -> AdcFifoBuilder<'a, u8> {
        self.adc.device.fcs().modify(|_, w| w.shift().set_bit());
        AdcFifoBuilder {
            adc: self.adc,
            marker: PhantomData,
        }
    }

    /// Enable DMA for the FIFO.
    ///
    /// This must be called to be able to transfer data from the ADC using a DMA transfer.
    ///
    /// **NOTE:** *this method sets the FIFO interrupt threshold to `1`, which is required for DMA transfers to work.
    /// The threshold is the same one as set by [`AdcFifoBuilder::enable_interrupt`]. If you want to enable FIFO
    /// interrupts, but also use DMA, the `threshold` parameter passed to `enable_interrupt` *must* be set to `1` as well.*
    pub fn enable_dma(self) -> Self {
        self.adc
            .device
            .fcs()
            .modify(|_, w| unsafe { w.dreq_en().set_bit().thresh().bits(1) });
        self
    }

    /// Enable ADC FIFO and start free-running conversion
    ///
    /// Use the returned [`AdcFifo`] instance to access the captured data.
    ///
    /// To stop capturing, call [`AdcFifo::stop`].
    ///
    /// Note: if you plan to use the FIFO for DMA transfers, [`AdcFifoBuilder::prepare`] instead.
    pub fn start(self) -> AdcFifo<'a, Word> {
        self.adc.device.fcs().modify(|_, w| w.en().set_bit());
        self.adc.device.cs().modify(|_, w| w.start_many().set_bit());
        AdcFifo {
            adc: self.adc,
            marker: PhantomData,
        }
    }

    /// Enable ADC FIFO, but do not start conversion yet
    ///
    /// Same as [`AdcFifoBuilder::start`], except the FIFO is initially paused.
    ///
    /// Use [`AdcFifo::resume`] to start conversion.
    pub fn start_paused(self) -> AdcFifo<'a, Word> {
        self.adc.device.fcs().modify(|_, w| w.en().set_bit());
        self.adc
            .device
            .cs()
            .modify(|_, w| w.start_many().clear_bit());
        AdcFifo {
            adc: self.adc,
            marker: PhantomData,
        }
    }

    /// Alias for [`AdcFifoBuilder::start_paused`].
    #[deprecated(note = "Use `start_paused()` instead.", since = "0.10.0")]
    pub fn prepare(self) -> AdcFifo<'a, Word> {
        self.start_paused()
    }
}

/// Represents the ADC fifo
///
/// Constructed by [`AdcFifoBuilder::start`], which is accessible through [`Adc::build_fifo`].
pub struct AdcFifo<'a, Word> {
    adc: &'a mut Adc,
    marker: PhantomData<Word>,
}

impl<'a, Word> AdcFifo<'a, Word> {
    #[allow(clippy::len_without_is_empty)]
    /// Returns the number of elements currently in the fifo
    pub fn len(&mut self) -> u8 {
        self.adc.device.fcs().read().level().bits()
    }

    /// Check if there was a fifo overrun
    ///
    /// An overrun happens when the fifo is filled up faster than `read` is called to consume it.
    ///
    /// This function also clears the `over` bit if it was set.
    pub fn is_over(&mut self) -> bool {
        let over = self.adc.device.fcs().read().over().bit();
        if over {
            self.adc
                .device
                .fcs()
                .modify(|_, w| w.over().clear_bit_by_one());
        }
        over
    }

    /// Check if there was a fifo underrun
    ///
    /// An underrun happens when `read` is called on an empty fifo (`len() == 0`).
    ///
    /// This function also clears the `under` bit if it was set.
    pub fn is_under(&mut self) -> bool {
        let under = self.adc.device.fcs().read().under().bit();
        if under {
            self.adc
                .device
                .fcs()
                .modify(|_, w| w.under().clear_bit_by_one());
        }
        under
    }

    /// Read the most recently sampled ADC value
    ///
    /// Returns the most recently sampled value, bypassing the FIFO.
    ///
    /// This can be used if you want to read samples occasionally, but don't
    /// want to incur the 96 cycle delay of a one-off read.
    ///
    /// Example:
    /// ```ignore
    /// // start continuously sampling values:
    /// let mut fifo = adc.build_fifo().set_channel(&mut adc_pin).start();
    ///
    /// loop {
    ///   do_something_timing_critical();
    ///
    ///   // read the most recent value:
    ///   if fifo.read_single() > THRESHOLD {
    ///     led.set_high().unwrap();
    ///   } else {
    ///     led.set_low().unwrap();
    ///   }
    /// }
    ///
    /// // stop sampling, when it's no longer needed
    /// fifo.stop();
    /// ```
    ///
    /// Note that when round-robin sampling is used, there is no way
    /// to tell from which channel this sample came.
    pub fn read_single(&mut self) -> u16 {
        self.adc.read_single()
    }

    /// Returns `true` if conversion is currently paused.
    ///
    /// While paused, no samples will be added to the FIFO.
    ///
    /// There may be existing samples in the FIFO though, or a conversion may still be in progress.
    pub fn is_paused(&mut self) -> bool {
        self.adc.device.cs().read().start_many().bit_is_clear()
    }

    /// Temporarily pause conversion
    ///
    /// This method stops ADC conversion, but leaves everything else configured.
    ///
    /// No new samples are captured until [`AdcFifo::resume`] is called.
    ///
    /// Note that existing samples can still be read from the FIFO, and can possibly
    /// cause interrupts and DMA transfer progress until the FIFO is emptied.
    pub fn pause(&mut self) {
        self.adc
            .device
            .cs()
            .modify(|_, w| w.start_many().clear_bit());
    }

    /// Resume conversion after it was paused
    ///
    /// There are two situations when it makes sense to use this method:
    /// - After having called [`AdcFifo::pause`] on an AdcFifo
    /// - If the FIFO was initialized using [`AdcFifoBuilder::prepare`].
    ///
    /// Calling this method when conversion is already running has no effect.
    pub fn resume(&mut self) {
        self.adc.device.cs().modify(|_, w| w.start_many().set_bit());
    }

    /// Clears the FIFO, removing all values
    ///
    /// Reads and discards values from the FIFO until it is empty.
    ///
    /// This only makes sense to use while the FIFO is paused (see [`AdcFifo::pause`]).
    pub fn clear(&mut self) {
        while self.len() > 0 {
            self.read_from_fifo();
        }
    }

    /// Stop capturing in free running mode.
    ///
    /// Resets all capture options that can be set via [`AdcFifoBuilder`] to
    /// their defaults.
    ///
    /// Returns the underlying [`Adc`], to be reused.
    pub fn stop(mut self) -> &'a mut Adc {
        // stop capture and clear channel selection
        self.adc
            .device
            .cs()
            .modify(|_, w| unsafe { w.start_many().clear_bit().rrobin().bits(0).ainsel().bits(0) });
        // disable fifo interrupt
        self.adc.device.inte().modify(|_, w| w.fifo().clear_bit());
        // Wait for one more conversion, then drain remaining values from fifo.
        // This MUST happen *after* the interrupt is disabled, but
        // *before* `thresh` is modified. Otherwise if `INTS.FIFO = 1`,
        // the interrupt will be fired one more time.
        // The only way to clear `INTS.FIFO` is for `FCS.LEVEL` to go
        // below `FCS.THRESH`, which requires `FCS.THRESH` not to be 0.
        while self.adc.device.cs().read().ready().bit_is_clear() {}
        self.clear();
        // disable fifo, reset threshold to 0 and disable DMA
        self.adc
            .device
            .fcs()
            .modify(|_, w| unsafe { w.en().clear_bit().thresh().bits(0).dreq_en().clear_bit() });
        // reset clock divider
        self.adc
            .device
            .div()
            .modify(|_, w| unsafe { w.int().bits(0).frac().bits(0) });
        self.adc
    }

    /// Block until a ADC_IRQ_FIFO interrupt occurs
    ///
    /// Interrupts must be enabled ([`AdcFifoBuilder::enable_interrupt`]), or else this methods blocks forever.
    pub fn wait_for_interrupt(&mut self) {
        while self.adc.device.intr().read().fifo().bit_is_clear() {}
    }

    fn read_from_fifo(&mut self) -> u16 {
        self.adc.device.fifo().read().val().bits()
    }

    /// Returns a read-target for initiating DMA transfers
    ///
    /// The [`DmaReadTarget`] returned by this function can be used to initiate DMA transfers
    /// reading from the ADC.
    pub fn dma_read_target(&self) -> DmaReadTarget<Word> {
        DmaReadTarget(self.adc.device.fifo().as_ptr() as u32, PhantomData)
    }

    /// Trigger a single conversion
    ///
    /// Ignored when in [`Adc::free_running`] mode.
    pub fn trigger(&mut self) {
        self.adc.device.cs().modify(|_, w| w.start_once().set_bit());
    }

    /// Check if ADC is ready for the next conversion trigger
    ///
    /// Not useful when in [`Adc::free_running`] mode.
    pub fn is_ready(&self) -> bool {
        self.adc.device.cs().read().ready().bit_is_set()
    }
}

impl AdcFifo<'_, u16> {
    /// Read a single value from the fifo (u16 version, not shifted)
    pub fn read(&mut self) -> u16 {
        self.read_from_fifo()
    }
}

impl AdcFifo<'_, u8> {
    /// Read a single value from the fifo (u8 version, shifted)
    ///
    /// Also see [`AdcFifoBuilder::shift_8bit`].
    pub fn read(&mut self) -> u8 {
        self.read_from_fifo() as u8
    }
}

/// Represents a [`dma::ReadTarget`] for the [`AdcFifo`]
///
/// If [`AdcFifoBuilder::shift_8bit`] was called when constructing the FIFO,
/// `Word` will be `u8`, otherwise it will be `u16`.
pub struct DmaReadTarget<Word>(u32, PhantomData<Word>);

/// Safety: rx_address_count points to a register which is always a valid
/// read target.
unsafe impl<Word> dma::ReadTarget for DmaReadTarget<Word> {
    type ReceivedWord = Word;

    fn rx_treq() -> Option<u8> {
        Some(TREQ_SEL_A::ADC.into())
    }

    fn rx_address_count(&self) -> (u32, u32) {
        (self.0, u32::MAX)
    }

    fn rx_increment(&self) -> bool {
        false
    }
}

impl<Word> dma::EndlessReadTarget for DmaReadTarget<Word> {}

/// Internal struct representing values for the `CS.RROBIN` register.
///
/// See [`AdcFifoBuilder::round_robin`], for usage example.
pub struct RoundRobin(u8);

impl<PIN: AdcChannel> From<&PIN> for RoundRobin {
    fn from(pin: &PIN) -> Self {
        Self(1 << pin.channel())
    }
}

impl<A, B> From<(&A, &B)> for RoundRobin
where
    A: AdcChannel,
    B: AdcChannel,
{
    fn from(pins: (&A, &B)) -> Self {
        Self((1 << pins.0.channel()) | (1 << pins.1.channel()))
    }
}

impl<A, B, C> From<(&A, &B, &C)> for RoundRobin
where
    A: AdcChannel,
    B: AdcChannel,
    C: AdcChannel,
{
    fn from(pins: (&A, &B, &C)) -> Self {
        Self((1 << pins.0.channel()) | (1 << pins.1.channel()) | (1 << pins.2.channel()))
    }
}

impl<A, B, C, D> From<(&A, &B, &C, &D)> for RoundRobin
where
    A: AdcChannel,
    B: AdcChannel,
    C: AdcChannel,
    D: AdcChannel,
{
    fn from(pins: (&A, &B, &C, &D)) -> Self {
        Self(
            (1 << pins.0.channel())
                | (1 << pins.1.channel())
                | (1 << pins.2.channel())
                | (1 << pins.3.channel()),
        )
    }
}

impl<A, B, C, D, E> From<(&A, &B, &C, &D, &E)> for RoundRobin
where
    A: AdcChannel,
    B: AdcChannel,
    C: AdcChannel,
    D: AdcChannel,
    E: AdcChannel,
{
    fn from(pins: (&A, &B, &C, &D, &E)) -> Self {
        Self(
            (1 << pins.0.channel())
                | (1 << pins.1.channel())
                | (1 << pins.2.channel())
                | (1 << pins.3.channel())
                | (1 << pins.4.channel()),
        )
    }
}
