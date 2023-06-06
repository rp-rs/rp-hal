//! Analog-Digital Converter (ADC)
//!
//! See [Chapter 4 Section 9](https://datasheets.raspberrypi.org/rp2040/rp2040_datasheet.pdf) of the datasheet for more details
//!
//! ## Usage
//!
//! Capture ADC reading from a pin
//! ```no_run
//! use embedded_hal::adc::OneShot;
//! use rp2040_hal::{adc::Adc, adc::AdcPin, gpio::Pins, pac, Sio};
//! let mut peripherals = pac::Peripherals::take().unwrap();
//! let sio = Sio::new(peripherals.SIO);
//! let pins = Pins::new(peripherals.IO_BANK0, peripherals.PADS_BANK0, sio.gpio_bank0, &mut peripherals.RESETS);
//! // Enable adc
//! let mut adc = Adc::new(peripherals.ADC, &mut peripherals.RESETS);
//! // Configure one of the pins as an ADC input
//! let mut adc_pin_0 = AdcPin::new(pins.gpio26.into_floating_input());
//! // Read the ADC counts from the ADC channel
//! let pin_adc_counts: u16 = adc.read(&mut adc_pin_0).unwrap();
//! ```
//!
//! Capture ADC reading from temperature sensor. Note that this needs conversion to be a real-world temperature.
//! ```no_run
//! use embedded_hal::adc::OneShot;
//! use rp2040_hal::{adc::Adc, gpio::Pins, pac, Sio};
//! let mut peripherals = pac::Peripherals::take().unwrap();
//! let sio = Sio::new(peripherals.SIO);
//! let pins = Pins::new(peripherals.IO_BANK0, peripherals.PADS_BANK0, sio.gpio_bank0, &mut peripherals.RESETS);
//! // Enable adc
//! let mut adc = Adc::new(peripherals.ADC, &mut peripherals.RESETS);
//! // Enable the temperature sensor
//! let mut temperature_sensor = adc.take_temp_sensor().unwrap();
//! // Read the ADC counts from the ADC channel
//! let temperature_adc_counts: u16 = adc.read(&mut temperature_sensor).unwrap();
//! ```
//!
//! See [examples/adc.rs](https://github.com/rp-rs/rp-hal/tree/main/rp2040-hal/examples/adc.rs) and
//! [pimoroni_pico_explorer_showcase.rs](https://github.com/rp-rs/rp-hal-boards/tree/main/boards/pimoroni-pico-explorer/examples/pimoroni_pico_explorer_showcase.rs) for more complete examples
//!
//! ### Free running mode
//!
//! In free-running mode the ADC automatically captures samples in regular intervals.
//! The samples are written to a FIFO, from which they can be retrieved.
//!
//! ```no_run
//! use rp2040_hal::{adc::Adc, gpio::Pins, pac, Sio};
//! let mut peripherals = pac::Peripherals::take().unwrap();
//! let sio = Sio::new(peripherals.SIO);
//! let pins = Pins::new(peripherals.IO_BANK0, peripherals.PADS_BANK0, sio.gpio_bank0, &mut peripherals.RESETS);
//! // Enable adc
//! let mut adc = Adc::new(peripherals.ADC, &mut peripherals.RESETS);
//! // Enable the temperature sensor
//! let mut temperature_sensor = adc.enable_temp_sensor();
//!
//! // Configure & start capturing to the fifo:
//! let fifo = adc.build_fifo()
//!   .clock_divider(0, 0) // sample as fast as possible (500ksps. This is the default)
//!   .set_channel(&mut temperature_sensor)
//!   .start();
//!
//! loop {
//!   if fifo.len() > 0 {
//!     // Read one captured ADC sample from the FIFO:
//!     let temperature_adc_counts: u16 = fifo.read();
//!   }
//! }
//! ```
//! See [examples/adc_fifo_poll.rs](https://github.com/rp-rs/rp-hal/tree/main/rp2040-hal/examples/adc.rs) for a more complete example.
//!

use core::convert::Infallible;

use hal::adc::{Channel, OneShot};
use pac::{ADC, RESETS};

use crate::{
    gpio::{
        bank0::{Gpio26, Gpio27, Gpio28, Gpio29},
        AnyPin, DynPinId, Function, OutputEnableOverride, Pin, PullType, ValidFunction,
    },
    resets::SubsystemReset,
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
    /// Captures the pin to be used with an ADC and disables its digital circuitery.
    pub fn new(pin: P) -> Self {
        let mut p = pin.into();
        let (od, ie) = (p.get_output_disable(), p.get_input_enable());
        p.set_output_enable_override(OutputEnableOverride::Disable);
        p.set_input_enable(false);
        Self {
            pin: P::from(p),
            saved_output_disable: od,
            saved_input_enable: ie,
        }
    }

    /// Release the pin and restore its digital circuitery's state.
    pub fn release(self) -> P {
        let mut p = self.pin.into();
        p.set_output_disable(self.saved_output_disable);
        p.set_input_enable(self.saved_input_enable);
        P::from(p)
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
/// Represents an ADC within the RP2040. Each ADC has multiple channels, and each
/// channel is either associated with a specific GPIO pin or attached to the internal
/// temperature sensor. You should put the relevant pin into ADC mode by creating an
/// [`AdcPin`] object with it, or you can put the ADC into `Temperature Sensing Mode`
/// by calling [`Adc::take_temp_sensor()`]. Either way, the resulting objects can be
/// passed to the [`OneShot::read()`][a] trait method to actually do the read.
///
/// [a]: embedded_hal::adc::OneShot::read
pub struct Adc {
    device: ADC,
}

impl Adc {
    /// Create new adc struct and bring up adc
    pub fn new(device: ADC, resets: &mut RESETS) -> Self {
        device.reset_bring_down(resets);
        device.reset_bring_up(resets);

        // Enable adc
        device.cs.write(|w| w.en().set_bit());

        // Wait for adc ready
        while !device.cs.read().ready().bit_is_set() {}

        Self { device }
    }

    /// Free underlying register block
    pub fn free(self) -> ADC {
        self.device
    }

    /// Read single
    pub fn read_single(&self) -> u16 {
        self.device.result.read().result().bits()
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
        self.device.cs.modify(|r, w| {
            disabled = r.ts_en().bit_is_clear();
            // if bit was already set, this is a nop
            w.ts_en().set_bit()
        });
        disabled.then_some(TempSense { __private: () })
    }

    /// Disable temperature sensor, consumes channel
    pub fn disable_temp_sensor(&mut self, _: TempSense) {
        self.device.cs.modify(|_, w| w.ts_en().clear_bit());
    }

    /// Start configuring free-running mode, and set up the FIFO
    ///
    /// The [`AdcFifoBuilder`] returned by this method can be used
    /// to configure capture options, like sample rate, channels to
    /// capture from etc.
    ///
    /// Capturing is started by calling [`AdcFifoBuilder::start`], which
    /// returns an [`AdcFifo`] to read from.
    pub fn build_fifo<'a>(&'a mut self) -> AdcFifoBuilder<'a> {
        AdcFifoBuilder { adc: self }
    }

    fn read(&mut self, chan: u8) -> u16 {
        while !self.device.cs.read().ready().bit_is_set() {
            cortex_m::asm::nop();
        }

        self.device
            .cs
            .modify(|_, w| unsafe { w.ainsel().bits(chan).start_once().set_bit() });

        while !self.device.cs.read().ready().bit_is_set() {
            cortex_m::asm::nop();
        }

        self.device.result.read().result().bits()
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

        Ok(self.read(chan).into())
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
    type Error = InvalidPinError;

    fn read(&mut self, _pin: &mut AdcPin<Pin<DynPinId, F, M>>) -> nb::Result<WORD, Self::Error> {
        use crate::gpio::DynBankId;
        let pin_id = _pin.pin.id();
        let chan = if (26..=29).contains(&pin_id.num) && pin_id.bank == DynBankId::Bank0 {
            pin_id.num - 26
        } else {
            return Err(nb::Error::Other(InvalidPinError));
        };

        Ok(self.read(chan).into())
    }
}

/// Used to configure & build an [`AdcFifo`]
///
/// See [`Adc::build_fifo`] for details, as well as the `adc_fifo_*` [examples](https://github.com/rp-rs/rp-hal/tree/main/rp2040-hal/examples).
pub struct AdcFifoBuilder<'a> {
    adc: &'a mut Adc,
}

impl<'a> AdcFifoBuilder<'a> {
    /// Manually set clock divider integral and fractional parts
    ///
    /// Default: 0, 0
    pub fn clock_divider(self, int: u16, frac: u8) -> Self {
        self.adc.device.div.modify(|_, w| unsafe { w.int().bits(int).frac().bits(frac) });
        self
    }

    /// Select ADC input channel to sample from
    ///
    /// If round-robin mode is used, this will only affect the first sample.
    ///
    /// The given `pin` can either be one of the ADC inputs (GPIO26-28) or the
    /// internal temperature sensor (retrieved via [`Adc::enable_temp_sensor`]).
    pub fn set_channel<PIN: Channel<Adc, ID = u8>>(self, _pin: &mut PIN) -> Self {
        self.adc.device.cs.modify(|_, w| unsafe { w.ainsel().bits(PIN::channel()) });
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
        self.adc.device.cs.modify(|_, w| unsafe { w.rrobin().bits(bits) });
        self
    }

    /// Enable the FIFO interrupt ([`ADC_IRQ_FIFO`](pac::Interrupt::ADC_IRQ_FIFO))
    ///
    /// It will be triggered whenever there are at least `threshold` samples waiting in the FIFO.
    ///
    pub fn enable_interrupt(self, threshold: u8) -> Self {
        self.adc.device.inte.modify(|_, w| w.fifo().set_bit());
        self.adc.device.fcs.modify(|_, w| unsafe { w.thresh().bits(threshold) });
        self
    }

    /// Enable ADC FIFO and start free-running conversion
    ///
    /// Use the returned [`AdcFifo`] instance to access the captured data.
    ///
    /// To stop capturing, call [`AdcFifo::stop`].
    pub fn start(self) -> AdcFifo<'a> {
        self.adc.device.fcs.modify(|_, w| w.en().set_bit());
        self.adc.device.cs.modify(|_, w| w.start_many().set_bit());
        AdcFifo { adc: self.adc }
    }
}

/// Represents the ADC fifo, when used in free running mode
///
/// Constructed by [`AdcFifoBuilder::start`], which is accessible through [`Adc::build_fifo`].
///
pub struct AdcFifo<'a> {
    adc: &'a mut Adc,
}

impl<'a> AdcFifo<'a> {
    /// Returns the number of elements currently in the fifo
    pub fn len(&mut self) -> u8 {
        self.adc.device.fcs.read().level().bits()
    }

    /// Check if there was a fifo overrun
    ///
    /// An overrun happens when the fifo is filled up faster than `read` is called to consume it.
    ///
    /// This function also clears the `over` bit if it was set.
    pub fn is_over(&mut self) -> bool {
        let over = self.adc.device.fcs.read().over().bit();
        if over {
            self.adc.device.fcs.modify(|_, w| w.over().set_bit());
        }
        over
    }

    /// Check if there was a fifo underrun
    ///
    /// An underrun happens when `read` is called on an empty fifo (`len() == 0`).
    ///
    /// This function also clears the `under` bit if it was set.
    pub fn is_under(&mut self) -> bool {
        let under = self.adc.device.fcs.read().under().bit();
        if under {
            self.adc.device.fcs.modify(|_, w| w.under().set_bit());
        }
        under
    }

    /// Read a single value from the fifo
    pub fn read(&mut self) -> u16 {
        self.adc.device.fifo.read().val().bits()
    }

    /// Stop capturing in free running mode.
    ///
    /// Resets all capture options that can be set via [`AdcFifoBuilder`] to
    /// their defaults.
    ///
    /// Returns the underlying [`Adc`], to be reused.
    pub fn stop(mut self) -> &'a mut Adc {
        // stop capture and clear channel selection
        self.adc.device.cs.modify(|_, w| unsafe {
            w.start_many().clear_bit()
                .rrobin().bits(0)
                .ainsel().bits(0)
        });
        // disable fifo interrupt
        self.adc.device.inte.modify(|_, w| w.fifo().clear_bit());
        // drain remaining values from fifo.
        // This MUST happen *after* the interrupt is disabled, but
        // *before* `thresh` is modified. Otherwise if `INTS.FIFO = 1`,
        // the interrupt fill be fired one more time.
        // The only way to clear `INTS.FIFO` is for `FCS.LEVEL` to go
        // below `FCS.THRESH`, which requires `FCS.THRESH` not to be 0.
        while self.len() > 0 {
            self.read();
        }
        // disable fifo and reset threshold to 0
        self.adc.device.fcs.modify(|_, w| unsafe {
            w.en().clear_bit()
                .thresh().bits(0)
        });
        // reset clock divider
        self.adc.device.div.modify(|_, w| unsafe { w.int().bits(0).frac().bits(0) });
        self.adc
    }

    /// Block until a ADC_IRQ_FIFO interrupt occurs
    ///
    /// Interrupts must be enabled ([`AdcFifoBuilder::enable_interrupt`]), or else this methods blocks forever.
    pub fn wait_for_interrupt(&mut self) {
        while self.adc.device.intr.read().fifo().bit_is_clear() {}
    }
}

/// Internal struct representing values for the `CS.RROBIN` register.
///
/// See [`AdcFreeRunning.round_robin`], for usage example.
pub struct RoundRobin(u8);

impl<PIN: Channel<Adc, ID = u8>> From<PIN> for RoundRobin {
    fn from(_: PIN) -> Self {
        Self(1 << PIN::channel())
    }
}

impl<A, B> From<(&mut A, &mut B)> for RoundRobin
where
    A: Channel<Adc, ID = u8>,
    B: Channel<Adc, ID = u8>,
{
    fn from(_: (&mut A, &mut B)) -> Self {
        Self(1 << A::channel() | 1 << B::channel())
    }
}

impl<A, B, C> From<(&mut A, &mut B, &mut C)> for RoundRobin
where
    A: Channel<Adc, ID = u8>,
    B: Channel<Adc, ID = u8>,
    C: Channel<Adc, ID = u8>,
{
    fn from(_: (&mut A, &mut B, &mut C)) -> Self {
        Self(1 << A::channel() | 1 << B::channel() | 1 << C::channel())
    }
}

impl<A, B, C, D> From<(&mut A, &mut B, &mut C, &mut D)> for RoundRobin
where
    A: Channel<Adc, ID = u8>,
    B: Channel<Adc, ID = u8>,
    C: Channel<Adc, ID = u8>,
    D: Channel<Adc, ID = u8>,
{
    fn from(_: (&mut A, &mut B, &mut C, &mut D)) -> Self {
        Self(1 << A::channel() | 1 << B::channel() | 1 << C::channel() | 1 << D::channel())
    }
}

impl<A, B, C, D, E> From<(&mut A, &mut B, &mut C, &mut D, &mut E)> for RoundRobin
where
    A: Channel<Adc, ID = u8>,
    B: Channel<Adc, ID = u8>,
    C: Channel<Adc, ID = u8>,
    D: Channel<Adc, ID = u8>,
    E: Channel<Adc, ID = u8>,
{
    fn from(_: (&mut A, &mut B, &mut C, &mut D, &mut E)) -> Self {
        Self(1 << A::channel() | 1 << B::channel() | 1 << C::channel() | 1 << D::channel() | 1 << E::channel())
    }
}
