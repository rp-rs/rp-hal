//! Portable in-line assembly
//!
//! On the RP235x, this is useful to write code portable between ARM and RISC-V cores.
//! While there's no such choice on the RP2040, providing the same functions helps writing
//! code that works on both RP2040 and RP235x.

#[cfg(all(target_arch = "arm", target_os = "none"))]
mod inner {
    pub use cortex_m::asm::{delay, dsb, nop, sev, wfe, wfi};
    pub use cortex_m::interrupt::{disable as interrupt_disable, enable as interrupt_enable};

    /// Are interrupts current enabled?
    pub fn interrupts_enabled() -> bool {
        cortex_m::register::primask::read().is_active()
    }

    /// Run the closure without interrupts
    ///
    /// No critical-section token because we haven't blocked the second core
    pub fn interrupt_free<T, F>(f: F) -> T
    where
        F: FnOnce() -> T,
    {
        let active = interrupts_enabled();
        if active {
            interrupt_disable();
        }
        let t = f();
        if active {
            unsafe {
                interrupt_enable();
            }
        }
        t
    }
}

#[cfg(not(all(any(target_arch = "arm", target_arch = "riscv32"), target_os = "none")))]
mod inner {
    /// Placeholder function to disable interrupts
    pub fn interrupt_disable() {}
    /// Placeholder function to enable interrupts
    pub fn interrupt_enable() {}
    /// Placeholder function to check if interrupts are enabled
    pub fn interrupts_enabled() -> bool {
        false
    }
    /// Placeholder function to wait for an event
    pub fn wfe() {}
    /// Placeholder function to do nothing
    pub fn nop() {}
    /// Placeholder function to emit a data synchronisation barrier
    pub fn dsb() {}
    /// Placeholder function to run a closure with interrupts disabled
    pub fn interrupt_free<T, F>(f: F) -> T
    where
        F: FnOnce() -> T,
    {
        f()
    }
    /// Placeholder function to wait for some clock cycles
    pub fn delay(_: u32) {}
    /// Placeholder function to emit an event
    pub fn sev() {}
}

pub use inner::*;
