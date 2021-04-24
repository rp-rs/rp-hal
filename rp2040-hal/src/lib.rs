//! HAL for the RP2040 microcontroller
//!
//! This is an implementation of the [`embedded-hal`] traits for the RP2040 microcontroller
//! NOTE This HAL is still under active development. This API will remain volatile until 1.0.0

#![deny(missing_docs)]
#![deny(warnings)]
#![no_std]

extern crate cortex_m;
extern crate embedded_hal as hal;
extern crate nb;
extern crate embedded_time;

pub extern crate rp2040_pac as pac;

pub mod adc;
pub mod i2c;
pub mod prelude;
pub mod pwm;
pub mod rom_data;
pub mod rtc;
pub mod spi;
pub mod ssi;
pub mod timer;
pub mod uart;
pub mod usb;
pub mod watchdog;
pub mod xosc;
