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
    output_disable: bool,
    input_enable: bool,
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
            output_disable: od,
            input_enable: ie,
        }
    }

    /// Release the pin and restore its digital circuitery's state.
    pub fn release(self) -> P {
        let mut p = self.pin.into();
        p.set_output_disable(self.output_disable);
        p.set_input_enable(self.input_enable);
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
