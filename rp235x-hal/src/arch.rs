//! Portable in-line assembly
//!
//! Replaces `cortex_m::asm` with things that work on RISC-V and Arm.

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

    /// Enable co-processors.
    ///
    /// For core0, this is done by the `#[entry]` macro. Fore core1, this function is called
    /// from `multicore::Core::spawn`.
    ///
    /// # Safety
    ///
    /// Must only be called immediately after starting up a core.
    pub unsafe fn enable_coprocessors() {
        unsafe {
            (*cortex_m::peripheral::SCB::PTR)
                .cpacr
                .modify(|value| value | 3 | (3 << 8) | (3 << 20) | (3 << 22))
        }
    }
}

#[cfg(all(target_arch = "riscv32", target_os = "none"))]
mod inner {
    pub use riscv::asm::{delay, nop, wfi};
    pub use riscv::interrupt::machine::{
        disable as interrupt_disable, enable as interrupt_enable, free as interrupt_free,
    };

    /// Send Event
    #[inline(always)]
    pub fn sev() {
        unsafe {
            // This is how h3.unblock is encoded.
            core::arch::asm!("slt x0, x0, x1");
        }
    }

    /// Wait for Event
    ///
    /// This is the interrupt-safe version of WFI.
    pub fn wfe() {
        let active = interrupts_enabled();
        if active {
            interrupt_disable();
        }
        wfi();
        if active {
            unsafe {
                interrupt_enable();
            }
        }
    }

    /// Data Synchronization Barrier
    #[inline(always)]
    pub fn dsb() {
        core::sync::atomic::compiler_fence(core::sync::atomic::Ordering::SeqCst);
        unsafe { core::arch::asm!("fence", options(nostack, preserves_flags)) };
        core::sync::atomic::compiler_fence(core::sync::atomic::Ordering::SeqCst);
    }

    /// Are interrupts current enabled?
    #[inline(always)]
    pub fn interrupts_enabled() -> bool {
        riscv::register::mstatus::read().mie()
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

/// Create a static variable which we can grab a mutable reference to exactly once.
#[macro_export]
macro_rules! singleton {
    ($name:ident: $ty:ty = $expr:expr) => {{
        static mut $name: (::core::mem::MaybeUninit<$ty>, ::core::sync::atomic::AtomicBool) =
            (::core::mem::MaybeUninit::uninit(), ::core::sync::atomic::AtomicBool::new(false));

        #[allow(unsafe_code)]
        if unsafe { $name.1.compare_exchange(false, true, ::core::sync::atomic::Ordering::SeqCst, ::core::sync::atomic::Ordering::SeqCst).is_ok() } {
            // If we get here, the bool was false and we were the ones who set it to true.
            // So we have exclusive access.
            let expr = $expr;
            #[allow(unsafe_code)]
            unsafe {
                $name.0 = ::core::mem::MaybeUninit::new(expr);
                Some(&mut *$name.0.as_mut_ptr())
            }
        } else {
            None
        }
    }};
    (: $ty:ty = $expr:expr) => {
        $crate::singleton!(VAR: $ty = $expr)
    };
}
