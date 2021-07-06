//! Analog-Digital Converter (ADC)
// See [Chapter 4 Section 9](https://datasheets.raspberrypi.org/rp2040/rp2040_datasheet.pdf) for more details
// TODO

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
    pub fn enable_temp_sensor(self) -> TempSense {
        self.device.cs.modify(|_, w| w.ts_en().set_bit());

        TempSense { __private: () }
    }

    /// Disable temperature sensor, consumes channel
    pub fn disable_temp_sensor(self, _: TempSense) {
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

        if self.device.cs.read().ready().bit_is_set() {
            self.device
                .cs
                .modify(|_, w| unsafe { w.ainsel().bits(chan).start_once().set_bit() });
        };
        if !self.device.cs.read().ready().bit_is_set() {
            // Can't return WouldBlock here since that would take to long and next call conversion would be over
            cortex_m::asm::nop();
        };

        Ok(self.device.result.read().result().bits().into())
    }
}
