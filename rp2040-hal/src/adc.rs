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

use core::convert::TryInto;
use hal::adc::{Channel, OneShot};
use num_traits::float::FloatCore;
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

    /// Start an ADC capture. call read_single() to get the result once
    pub fn start_single(&self, chan: u8) {
        self.device
            .cs
            .modify(|_, w| unsafe { w.ainsel().bits(chan).start_once().set_bit() });
    }

    /// Start the ADC in Round Robin mode - read samples from the interrupt
    /// Conversion always takes 96 cycles regardless of ADC sample frequency
    /// If the sample frequency is greater than 500 kilosamples/s, conversion will be back-to-back with no gap
    /// otherwise, each conversion is scheduled on this interval.
    /// Note: divisor is *at most* 65536, so the minimum sampling frequency is 732.42 hz.
    /// if you're going to do low frequency sampling, one-shot is probably easier.
    pub fn start_many_round_robin(&self, channel_bitfield: u8, conversion_frequency_hz: u32) {
        // The first ADC channel is the least significant bit
        let first_channel = channel_bitfield.trailing_zeros();
        let number_of_channels = channel_bitfield.count_ones().try_into().unwrap();
        // The clock divider specifies how regularly *one* sample will be taken.
        // Since the user specified a number of channels and a frequency, they will expect that number
        // of samples per channel per second.
        // Scale up our conversion rate so that it makes sense.
        let conversion_frequency_hz = conversion_frequency_hz * number_of_channels as u32;
        assert!(first_channel < 4);
        assert!(number_of_channels < 5);
        // Stop any sampling that is currently happening
        self.device.cs.modify(|_, w| w.en().clear_bit());
        // Drain the FIFO
        while self.device.fcs.read().empty().bit_is_clear() {
            let _ = self.device.fifo.read();
        }
        // According to the docs, ADC clock *must* be 48MHz to be in spec - so we'll assume that frequency.
        let adc_clock_frequency_hz = 48_000_000 as f32;
        // TODO: expose sio::divider_unsigned (if we can safely do so) so we can calculate this without resorting to floats.
        // Can't easily do this with fixed-point u32 math since 48M * 256 will exceed the range of u32.
        // let denom = 48_000_000 / conversion_frequency_hz;
        let clock_divider = adc_clock_frequency_hz / (conversion_frequency_hz as f32);
        // Division is actually performed on denominator + 1
        // so if requested freq == 48MHz, clock_divider would be 1 but we need
        // to subtract 1 for the HW to do what we want
        let clock_divider_int = clock_divider.trunc() as i32 - 1;
        // if we wrapped around due to subtraction, use 0 instead
        let clock_divider_int = clock_divider_int.max(0);
        // get the fractional parts out of the divider, scale them so 1.0 == 255
        let clock_divider_frac = (u8::MAX as f32 * clock_divider.fract()).trunc() as u32;
        // clamp clock_divider_int to u16::MAX before casting into u16
        let mut clock_divider_int = clock_divider_int.min(u16::MAX as i32) as u16;
        // clamp max fractional value to u16::MAX
        let mut clock_divider_frac = clock_divider_frac.min(u8::MAX as u32) as u8;
        // Don't use the divider if it would fire while still sampling - back-to-back makes more sense there
        if clock_divider_int <= 96 {
            clock_divider_int = 0;
            clock_divider_frac = 0;
        }
        // Finally configure for the sample frequency that we just calculated
        self.device.div.modify(|_, w| unsafe {
            w.int()
                .bits(clock_divider_int)
                .frac()
                .bits(clock_divider_frac)
        });
        // Set up our interrupts before enabling the ADC
        self.enable_fifo_interrupt(number_of_channels);
        // Configure for round-robin sampling, start at the first channel
        self.device.cs.modify(|_, w| unsafe {
            // Set the first sampled channel to the lowest one
            w.ainsel().bits(first_channel.try_into().unwrap());
            // Enable round-robin sampling of the channels
            w.rrobin().bits(channel_bitfield);
            // Clear sticky error bit
            w.err_sticky().set_bit();
            // Continously perform conversions
            w.start_many().set_bit();
            // Enable the ADC
            w.en().set_bit();
            w
        });
    }

    /// Read an ADC value value from the FIFO
    pub fn read_fifo(&mut self) -> Option<u16> {
        if self.device.fcs.read().empty().bit_is_clear() {
            // if we read fifo.read().err() that consumes the fifo entry
            // so fifo.read().val() is n*2 what we expect.
            // that means we need to manually mask out the bits, unfortunately
            let read = self.device.fifo.read().bits();
            if read & 1u32 << 15 != 0 {
                None
            } else {
                Some((read & 0b11_1111_1111) as u16)
            }
        } else {
            None
        }
    }

    /// Number of entries waiting in FIFO
    pub fn fifo_len(&mut self) -> u8 {
        self.device.fcs.read().level().bits()
    }

    /// Number of entries waiting in FIFO
    pub fn interrupt_pending(&mut self) -> bool {
        self.device.ints.read().fifo().bit_is_set()
    }

    /// Return the index of the first channel to be sampled in round-robin mode
    pub fn first_channel(&mut self) -> u8 {
        self.device.cs.read().rrobin().bits().trailing_zeros() as u8
    }

    /// Return the index of the last channel to be sampled in round-robin mode
    pub fn last_channel(&mut self) -> u8 {
        self.device.cs.read().rrobin().bits().leading_zeros() as u8
    }

    /// Return the channel that is currently being captured
    pub fn current_channel(&mut self) -> u8 {
        self.device.cs.read().ainsel().bits()
    }

    /// Return the current state of the ADC::FCS register
    pub fn fcs(&self) -> u32 {
        self.device.fcs.read().bits()
    }

    /// Return the current state of the ADC::CS register
    pub fn cs(&self) -> u32 {
        self.device.cs.read().bits()
    }

    /// Set up FIFO interrupt generation and enable the interrupt
    pub fn enable_fifo_interrupt(&self, threshold: u8) {
        self.device.fcs.modify(|_, w| unsafe {
            // Clear underflow, overflow bits
            w.over().set_bit();
            w.under().set_bit();
            // Set the FIFO threshold that will trigger an interrupt
            w.thresh().bits(threshold);
            // Write the ADC capture result to the FIFO when ready
            w.en().set_bit();
            w
        });
        self.device.inte.modify(|_, w| {
            // Enable interrupt when FIFO reaches threshold
            w.fifo().set_bit()
        });
    }

    /// ADC conversion is complete
    pub fn is_busy(&self) -> bool {
        self.device.cs.read().ready().bit_is_set()
    }

    /// Read last ADC result
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
