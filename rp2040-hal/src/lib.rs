//! HAL for the RP2040 microcontroller
//!
//! This is an implementation of the [`embedded-hal`](https://crates.io/crates/embedded-hal)
//! traits for the RP2040 microcontroller
//!
//! NOTE This HAL is still under active development. This API will remain volatile until 1.0.0
//!
//! # Crate features
//!
//! * **chrono** -
//!   Modifies some RTC access functions to use chrono types instead of a rp2040-hal specific
//!   DateTime type
//! * **critical-section-impl** -
//!   critical section that is safe for multicore use
//! * **defmt** -
//!   Implement `defmt::Format` for several types.
//! * **disable-intrinsics** -
//!   Disable automatic mapping of language features (like floating point math) to ROM functions
//! * **eh1_0_alpha** -
//!   Support alpha release of embedded-hal
//! * **rom-func-cache** -
//!   Memoize(cache) ROM function pointers on first use to improve performance
//! * **rt** -
//!   Minimal startup / runtime for Cortex-M microcontrollers
//! * **rom-v2-intrinsics** -
//!   This enables ROM functions for f64 math that were not present in the earliest RP2040s
//! * **rp2040-e5** -
//!   This enables a fix for USB errata 5: USB device fails to exit RESET state on busy USB bus.
//!   Only required for RP2040 B0 and RP2040 B1, but it also works for RP2040 B2 and above.
//!   **Note that the workaround takes control of pin 15 (bank0) during usb reset so the bank needs
//!   to be taken out of reset before calling `UsbBus::new`**.
//!   Using `let pins = Pins::new(peripherals.IO_BANK0, peripherals.PADS_BANK0, sio.gpio_bank0, &mut peripherals.RESETS);`
//!   is enough to take the Bank 0 out of reset.
//! * **rtic-monotonic** -
//!   Implement
//!   `rtic_monotonic::Monotonic` based on the RP2040 timer peripheral

#![warn(missing_docs)]
#![no_std]

pub use paste;

/// Re-export of the PAC
pub use rp2040_pac as pac;

#[macro_use]
mod intrinsics;

pub mod adc;
pub(crate) mod atomic_register_access;
pub mod clocks;
#[cfg(feature = "critical-section-impl")]
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
pub mod vector_table;
pub mod watchdog;
pub mod xosc;

// Provide access to common datastructures to avoid repeating ourselves
pub use adc::Adc;
pub use clocks::Clock;
pub use i2c::{DynI2c, I2C};
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
