//! Simple driver for the gSPI connection between rp2040 and CYW43439 wifi chip.
//!
//! This is just a quick bit-banging hack to get pico-w working and should be replaced
//! by a PIO based driver.

use defmt::*;

use core::convert::Infallible;

// A shorter alias for the Hardware Abstraction Layer, which provides
// higher-level drivers.
use crate::hal;

use embedded_hal::digital::v2::InputPin;
use embedded_hal::digital::v2::OutputPin;
use embedded_hal_1::spi::SpiBusFlush;
use embedded_hal_1::spi::{SpiBusRead, SpiBusWrite};
use embedded_hal_1::spi::ErrorType;

pub struct GSpi {
    /// SPI clock
    clk: hal::gpio::Pin<hal::gpio::bank0::Gpio29, hal::gpio::Output<hal::gpio::PushPull>>,

    /// 4 signals, all in one!!
    /// - SPI MISO
    /// - SPI MOSI
    /// - IRQ
    /// - strap to set to gSPI mode on boot.
    dio: hal::gpio::dynpin::DynPin,
}

impl GSpi {
    pub fn new(
        clk: hal::gpio::Pin<hal::gpio::bank0::Gpio29, hal::gpio::Output<hal::gpio::PushPull>>,
        dio: hal::gpio::dynpin::DynPin,
    ) -> Self {
        Self { clk, dio }
    }
}

impl ErrorType for GSpi {
    type Error = Infallible;
}

impl SpiBusFlush for GSpi {
    fn flush(&mut self) -> Result<(), Self::Error> {
        Ok(())
    }
}

impl SpiBusRead<u32> for GSpi {
    fn read<'a>(&'a mut self, words: &'a mut [u32]) -> Result<(), Self::Error> {
        trace!("spi read {}", words.len());
        self.dio.into_floating_input();
        for word in words.iter_mut() {
            let mut w = 0;
            for _ in 0..32 {
                w <<= 1;

                cortex_m::asm::nop();
                // rising edge, sample data
                if self.dio.is_high().unwrap() {
                    w |= 0x01;
                }
                self.clk.set_high().unwrap();

                cortex_m::asm::nop();
                // falling edge
                self.clk.set_low().unwrap();
            }
            *word = w
        }

        trace!("spi read result: {:x}", words);
        Ok(())
    }
}

impl SpiBusWrite<u32> for GSpi {
    fn write<'a>(&'a mut self, words: &'a [u32]) -> Result<(), Self::Error> {
        trace!("spi write {:x}", words);
        self.dio.into_push_pull_output();
        for word in words {
            let mut word = *word;
            for _ in 0..32 {
                // falling edge, setup data
                cortex_m::asm::nop();
                self.clk.set_low().unwrap();
                if word & 0x8000_0000 == 0 {
                    self.dio.set_low().unwrap();
                } else {
                    self.dio.set_high().unwrap();
                }

                cortex_m::asm::nop();
                // rising edge
                self.clk.set_high().unwrap();

                word <<= 1;
            }
        }
        self.clk.set_low().unwrap();

        self.dio.into_floating_input();
        Ok(())
    }
}
