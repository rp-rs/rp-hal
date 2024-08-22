//! HAL for the Raspberry Pi RP235x microcontrollers
//!
//! This is an implementation of the [`embedded-hal`](https://crates.io/crates/embedded-hal)
//! traits for the RP235x microcontrollers
//!
//! NOTE This HAL is still under active development. This API will remain volatile until 1.0.0
//!
//! # Crate features
//!
//! * **critical-section-impl** -
//!   critical section that is safe for multicore use
//! * **defmt** -
//!   Implement `defmt::Format` for several types.
//! * **embedded_hal_1** -
//!   Support alpha release of embedded-hal
//! * **rom-func-cache** -
//!   Memoize(cache) ROM function pointers on first use to improve performance
//! * **rt** -
//!   Minimal startup / runtime for Cortex-M microcontrollers
//! * **rtic-monotonic** -
//!   Implement `rtic_monotonic::Monotonic` based on the RP2350 timer peripheral
//! * **i2c-write-iter** -
//!   Implement `i2c_write_iter` traits for `I2C<_, _, Controller>`.
//! * **binary-info** -
//!   Include a `static` variable containing picotool compatible binary info.

#![recursion_limit = "256"]
#![warn(missing_docs)]
#![no_std]

#[doc(hidden)]
pub use paste;

/// Re-export of the PAC
pub use rp235x_pac as pac;

pub mod adc;
pub mod arch;
#[macro_use]
pub mod async_utils;
pub(crate) mod atomic_register_access;
pub use rp_binary_info as binary_info;
pub mod block;
pub mod clocks;
#[cfg(feature = "critical-section-impl")]
mod critical_section_impl;
#[cfg(all(target_arch = "arm", target_os = "none"))]
pub mod dcp;
pub mod dma;
pub mod gpio;
pub mod i2c;
pub mod lposc;
#[cfg(all(target_arch = "arm", target_os = "none"))]
pub mod multicore;
pub mod otp;
pub mod pio;
pub mod pll;
pub mod powman;
pub mod prelude;
pub mod pwm;
pub mod reboot;
pub mod resets;
pub mod rom_data;
pub mod rosc;
pub mod sio;
pub mod spi;
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
pub use i2c::I2C;

/// Attribute to declare the entry point of the program
///
/// This is based on and can be used like the [entry attribute from
/// cortex-m-rt](https://docs.rs/cortex-m-rt/latest/cortex_m_rt/attr.entry.html).
///
/// It extends that macro with code to unlock all spinlocks at the beginning of
/// `main`. As spinlocks are not automatically unlocked on software resets, this
/// can prevent unexpected deadlocks when running from a debugger. The macro
/// also enables the FPU (CP10) and the Double-Co-Processor (CP4) before we hit
/// main.
pub use rp235x_hal_macros::entry;

/// Called by the rp235x-specific entry macro
#[cfg(all(target_arch = "arm", target_os = "none"))]
pub use cortex_m_rt::entry as arch_entry;

/// Called by the rp235x-specific entry macro
#[cfg(all(target_arch = "riscv32", target_os = "none"))]
pub use riscv_rt::entry as arch_entry;

use sio::CoreId;
pub use sio::Sio;
pub use spi::Spi;
pub use timer::Timer;
pub use watchdog::Watchdog;
// Re-export crates used in rp235x-hal's public API
pub extern crate fugit;

/// Trigger full reset of the rp235x.
///
/// Uses the watchdog and the power-on state machine (PSM) to reset all on-chip components.
pub fn reset() -> ! {
    unsafe {
        crate::arch::interrupt_disable();
        (*pac::PSM::PTR).wdsel().write(|w| w.bits(0x0001ffff));
        (*pac::WATCHDOG::PTR)
            .ctrl()
            .write(|w| w.trigger().set_bit());
        #[allow(clippy::empty_loop)]
        loop {}
    }
}

/// Halt the rp235x.
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
        crate::arch::interrupt_disable();
        // Stop other core
        match crate::Sio::core() {
            CoreId::Core0 => {
                // Stop core 1.
                (*pac::PSM::PTR)
                    .frce_off()
                    .modify(|_, w| w.proc1().set_bit());
                while !(*pac::PSM::PTR).frce_off().read().proc1().bit_is_set() {
                    crate::arch::nop();
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
            crate::arch::wfe()
        }
    }
}
