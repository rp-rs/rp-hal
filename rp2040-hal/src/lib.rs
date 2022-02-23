//! HAL for the RP2040 microcontroller
//!
//! This is an implementation of the [`embedded-hal`] traits for the RP2040 microcontroller
//! NOTE This HAL is still under active development. This API will remain volatile until 1.0.0

#![warn(missing_docs)]
#![no_std]
#![cfg_attr(feature = "embassy-traits", feature(generic_associated_types))]
#![cfg_attr(feature = "embassy-traits", feature(type_alias_impl_trait))]

extern crate cortex_m;
extern crate embedded_hal as hal;
extern crate nb;
pub use paste;

pub extern crate rp2040_pac as pac;

#[macro_use]
mod intrinsics;

pub mod adc;
pub(crate) mod atomic_register_access;
pub mod clocks;
mod critical_section_impl;
pub mod dma;
mod float;
pub mod gpio;
pub mod i2c;
pub mod multicore;
pub mod pio;
pub mod pll;
pub mod prelude;
pub mod pwm;
pub mod resets;
pub mod rom_data;
pub mod rosc;
pub mod rtc;
pub mod sio;
pub mod spi;
pub mod ssi;
pub mod timer;
pub mod typelevel;
pub mod uart;
pub mod usb;
pub mod watchdog;
pub mod xosc;

// Provide access to common datastructures to avoid repeating ourselves
pub use adc::Adc;
pub use clocks::Clock;
pub use i2c::I2C;
/// Attribute to declare the entry point of the program
///
/// This is based on and can be used like the [entry attribute from
/// cortex-m-rt](https://docs.rs/cortex-m-rt/latest/cortex_m_rt/attr.entry.html).
///
/// It extends that macro with code to unlock all spinlocks at the beginning
/// of `main`. As spinlocks are not automatically unlocked on software resets,
/// this can prevent unexpected deadlocks when running from a debugger.
pub use rp2040_hal_macros::entry;
pub use sio::Sio;
pub use spi::Spi;
pub use timer::Timer;
pub use watchdog::Watchdog;
