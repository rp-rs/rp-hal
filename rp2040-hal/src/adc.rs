//! Analog-Digital Converter (ADC)
//!
//! See [Chapter 4 Section 9](https://datasheets.raspberrypi.org/rp2040/rp2040_datasheet.pdf) of the datasheet for more details
//!
//! ## Usage
//!
//! Capture ADC reading from a pin
//! ```no_run
//! use embedded_hal::adc::OneShot;
//! use rp2040_hal::{adc::Adc, gpio::Pins, pac, Sio};
//! let mut peripherals = pac::Peripherals::take().unwrap();
//! let sio = Sio::new(peripherals.SIO);
//! let pins = Pins::new(peripherals.IO_BANK0, peripherals.PADS_BANK0, sio.gpio_bank0, &mut peripherals.RESETS);
//! // Enable adc
//! let mut adc = Adc::new(peripherals.ADC, &mut peripherals.RESETS);
//! // Configure one of the pins as an ADC input
//! let mut adc_pin_0 = pins.gpio26.into_floating_input();
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
//! let mut temperature_sensor = adc.enable_temp_sensor();
//! // Read the ADC counts from the ADC channel
//! let temperature_adc_counts: u16 = adc.read(&mut temperature_sensor).unwrap();
//! ```
//!
//! See [examples/adc.rs](https://github.com/rp-rs/rp-hal/tree/main/rp2040-hal/examples/adc.rs) and
//! [pimoroni_pico_explorer_showcase.rs](https://github.com/rp-rs/rp-hal/tree/main/boards/pimoroni_pico_explorer/examples/pimoroni_pico_explorer_showcase.rs) for more complete examples

use hal::adc::{Channel, OneShot};
use pac::{ADC, RESETS};

use crate::{
    gpio::Pin,
    gpio::{
        bank0::{Gpio26, Gpio27, Gpio28, Gpio29},
        FloatingInput,
    },
    resets::SubsystemReset,
};

const TEMPERATURE_SENSOR_CHANNEL: u8 = 4;

/// Adc
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

    /// Enable temperature sensor, returns a channel to use
    pub fn enable_temp_sensor(&mut self) -> TempSense {
        self.device.cs.modify(|_, w| w.ts_en().set_bit());

        TempSense { __private: () }
    }

    /// Disable temperature sensor, consumes channel
    pub fn disable_temp_sensor(&mut self, _: TempSense) {
        self.device.cs.modify(|_, w| w.ts_en().clear_bit());
    }
}

macro_rules! channel {
    ($pin:ident, $channel:expr) => {
        impl Channel<Adc> for Pin<$pin, FloatingInput> {
            type ID = u8; // ADC channels are identified numerically

            fn channel() -> u8 {
                $channel
            }
        }

        #[cfg(feature = "eh1_0_alpha")]
        impl eh1_0_alpha::adc::nb::Channel<Adc> for Pin<$pin, FloatingInput> {
            type ID = u8; // ADC channels are identified numerically

            fn channel(&self) -> u8 {
                $channel
            }
        }
    };
}

channel!(Gpio26, 0);
channel!(Gpio27, 1);
channel!(Gpio28, 2);
channel!(Gpio29, 3);

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

#[cfg(feature = "eh1_0_alpha")]
impl eh1_0_alpha::adc::nb::Channel<Adc> for TempSense {
    type ID = u8; // ADC channels are identified numerically

    fn channel(&self) -> u8 {
        TEMPERATURE_SENSOR_CHANNEL
    }
}

impl<WORD, PIN> OneShot<Adc, WORD, PIN> for Adc
where
    WORD: From<u16>,
    PIN: Channel<Adc, ID = u8>,
{
    type Error = ();

    fn read(&mut self, _pin: &mut PIN) -> nb::Result<WORD, Self::Error> {
        let chan = PIN::channel();

        if chan == 4 {
            self.device.cs.modify(|_, w| w.ts_en().set_bit())
        }

        while !self.device.cs.read().ready().bit_is_set() {
            cortex_m::asm::nop();
        }

        self.device
            .cs
            .modify(|_, w| unsafe { w.ainsel().bits(chan).start_once().set_bit() });

        while !self.device.cs.read().ready().bit_is_set() {
            cortex_m::asm::nop();
        }

        Ok(self.device.result.read().result().bits().into())
    }
}

#[cfg(feature = "eh1_0_alpha")]
impl<WORD, PIN> eh1_0_alpha::adc::nb::OneShot<Adc, WORD, PIN> for Adc
where
    WORD: From<u16>,
    PIN: eh1_0_alpha::adc::nb::Channel<Adc, ID = u8>,
{
    type Error = ();

    fn read(&mut self, pin: &mut PIN) -> nb::Result<WORD, Self::Error> {
        let chan = PIN::channel(pin);

        if chan == 4 {
            self.device.cs.modify(|_, w| w.ts_en().set_bit())
        }

        while !self.device.cs.read().ready().bit_is_set() {
            cortex_m::asm::nop();
        }

        self.device
            .cs
            .modify(|_, w| unsafe { w.ainsel().bits(chan).start_once().set_bit() });

        while !self.device.cs.read().ready().bit_is_set() {
            cortex_m::asm::nop();
        }

        Ok(self.device.result.read().result().bits().into())
    }
}
