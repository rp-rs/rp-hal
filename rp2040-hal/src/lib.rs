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
//!   Add conversion functions between chrono types and the rp2040-hal specific DateTime type
//! * **critical-section-impl** -
//!   critical section that is safe for multicore use
//! * **defmt** -
//!   Implement `defmt::Format` for several types.
//! * **disable-intrinsics** -
//!   Disable automatic mapping of language features (like floating point math) to ROM functions
//! * **embedded_hal_1** -
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
//! * **i2c-write-iter** -
//!   Implement `i2c_write_iter` traits for `I2C<_, _, Controller>`.
//! * **binary-info** -
//!   Include a `static` variable containing picotool compatible binary info.

#![warn(missing_docs)]
#![no_std]

#[doc(hidden)]
pub use paste;

/// Re-export of the PAC
pub use rp2040_pac as pac;

#[macro_use]
mod intrinsics;

pub mod adc;
pub mod arch;
#[macro_use]
pub mod async_utils;
pub(crate) mod atomic_register_access;
pub use rp_binary_info as binary_info;
pub mod clocks;
#[cfg(feature = "critical-section-impl")]
mod critical_section_impl;
pub mod dma;
mod float;
pub mod gpin;
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
pub mod vreg;
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
use sio::CoreId;
pub use sio::Sio;
pub use spi::Spi;
pub use timer::Timer;
pub use watchdog::Watchdog;
// Re-export crates used in rp2040-hal's public API
pub extern crate fugit;

/// Trigger full reset of the RP2040.
///
/// Uses the watchdog and the power-on state machine (PSM) to reset all on-chip components.
pub fn reset() -> ! {
    unsafe {
        cortex_m::interrupt::disable();
        (*pac::PSM::PTR).wdsel().write(|w| w.bits(0x0001ffff));
        (*pac::WATCHDOG::PTR)
            .ctrl()
            .write(|w| w.trigger().set_bit());
        #[allow(clippy::empty_loop)]
        loop {}
    }
}

/// Halt the RP2040.
///
/// Disables the other core, and parks the current core in an
/// infinite loop with interrupts disabled.
///
/// Doesn't stop other subsystems, like the DMA controller.
///
/// When called from core1, core0 will be kept forced off, which
/// likely breaks debug connections. You may need to reboot with
/// BOOTSEL pressed to reboot into a debuggable state.
pub fn halt() -> ! {
    unsafe {
        cortex_m::interrupt::disable();
        // Stop other core
        match crate::Sio::core() {
            CoreId::Core0 => {
                // Stop core 1.
                (*pac::PSM::PTR)
                    .frce_off()
                    .modify(|_, w| w.proc1().set_bit());
                while !(*pac::PSM::PTR).frce_off().read().proc1().bit_is_set() {
                    cortex_m::asm::nop();
                }
                // Restart core 1. Without this, most debuggers will fail connecting.
                // It will loop indefinitely in BOOTROM, as nothing
                // will trigger the wakeup sequence.
                (*pac::PSM::PTR)
                    .frce_off()
                    .modify(|_, w| w.proc1().clear_bit());
            }
            CoreId::Core1 => {
                // Stop core 0.
                (*pac::PSM::PTR)
                    .frce_off()
                    .modify(|_, w| w.proc0().set_bit());
                // We cannot restart core 0 here, as it would just boot into main.
                // So the best we can do is to keep core 0 disabled, which may break
                // further debug connections.
            }
        };

        // Keep current core running, so debugging stays possible
        loop {
            cortex_m::asm::wfe()
        }
    }
}
