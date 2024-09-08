//! Multicore support
//!
//! This module handles setup of the 2nd cpu core on the rp235x, which we refer to as core1.
//! It provides functionality for setting up the stack, and starting core1.
//!
//! The entrypoint for core1 can be any function that never returns, including closures.
//!
//! # Usage
//!
//! ```no_run
//! use rp235x_hal::{
//!     gpio::Pins,
//!     multicore::{Multicore, Stack},
//!     pac,
//!     sio::Sio,
//! };
//!
//! static mut CORE1_STACK: Stack<4096> = Stack::new();
//!
//! fn core1_task() {
//!     loop {}
//! }
//!
//! fn main() -> ! {
//!     let mut pac = hal::pac::Peripherals::take().unwrap();
//!     let mut sio = Sio::new(pac.SIO);
//!     // Other init code above this line
//!     let mut mc = Multicore::new(&mut pac.PSM, &mut pac.PPB, &mut sio.fifo);
//!     let cores = mc.cores();
//!     let core1 = &mut cores[1];
//!     let _test = core1.spawn(unsafe { &mut CORE1_STACK.mem }, core1_task);
//!     // The rest of your application below this line
//!     # loop {}
//! }
//! ```
//!
//! For inter-processor communications, see [`crate::sio::SioFifo`] and [`crate::sio::Spinlock0`]
//!
//! For a detailed example, see [examples/multicore_fifo_blink.rs](https://github.com/rp-rs/rp-hal/tree/main/rp235x-hal-examples/src/bin/multicore_fifo_blink.rs)

use core::mem::ManuallyDrop;
use core::sync::atomic::compiler_fence;
use core::sync::atomic::Ordering;

use crate::pac;
use crate::Sio;

/// Errors for multicore operations.
#[derive(Debug)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum Error {
    /// Operation is invalid on this core.
    InvalidCore,
    /// Core was unresponsive to commands.
    Unresponsive,
}

#[inline(always)]
fn install_stack_guard(_stack_limit: *mut usize) {
    // TBD Cortex-M33 / Hazard3 MPU stack guard stuff. See the Pico SDK for
    // ideas as to what to do here.
}

#[inline(always)]
fn core1_setup(stack_limit: *mut usize) {
    install_stack_guard(stack_limit);
}

/// Multicore execution management.
pub struct Multicore<'p> {
    cores: [Core<'p>; 2],
}

/// Data type for a properly aligned stack of N 32-bit (usize) words
#[repr(C, align(32))]
pub struct Stack<const SIZE: usize> {
    /// Memory to be used for the stack
    pub mem: [usize; SIZE],
}

impl<const SIZE: usize> Default for Stack<SIZE> {
    fn default() -> Self {
        Self::new()
    }
}

impl<const SIZE: usize> Stack<SIZE> {
    /// Construct a stack of length SIZE, initialized to 0
    pub const fn new() -> Stack<SIZE> {
        Stack { mem: [0; SIZE] }
    }
}

impl<'p> Multicore<'p> {
    /// Create a new |Multicore| instance.
    pub fn new(
        psm: &'p mut pac::PSM,
        ppb: &'p mut pac::PPB,
        sio: &'p mut crate::sio::SioFifo,
    ) -> Self {
        Self {
            cores: [
                Core { inner: None },
                Core {
                    inner: Some((psm, ppb, sio)),
                },
            ],
        }
    }

    /// Get the available |Core| instances.
    pub fn cores(&mut self) -> &'p mut [Core] {
        &mut self.cores
    }
}

extern "C" {
    fn _core1_trampoline() -> !;
}

#[cfg(target_arch = "arm")]
core::arch::global_asm!(
    ".global _core1_trampoline",
    ".thumb_func",
    "_core1_trampoline:",
    "pop {{r0, r1, pc}}",
);

#[cfg(not(target_arch = "arm"))]
core::arch::global_asm!(
    ".align 4",
    ".section .text, \"ax\"",
    ".global _core1_trampoline",
    "_core1_trampoline:",
    // load the pointer to the closure
    "lw a0, 0(sp)",
    // load the stack limit
    "lw a1, 4(sp)",
    // load the pointer to core1_startup
    "lw a2, 8(sp)",
    // load and set the GP
    "lw gp, 12(sp)",
    // move the stack pointer back 4 words
    "addi sp, sp, 16",
    // core1_startup(entry, stack_limit)
    "jr a2",
);

/// A handle for controlling a logical core.
pub struct Core<'p> {
    inner: Option<(
        &'p mut pac::PSM,
        &'p mut pac::PPB,
        &'p mut crate::sio::SioFifo,
    )>,
}

impl<'p> Core<'p> {
    /// Get the id of this core.
    pub fn id(&self) -> u8 {
        match self.inner {
            None => 0,
            Some(..) => 1,
        }
    }

    /// Spawn a function on this core.
    ///
    /// The closure should not return. It is currently defined as `-> ()` because `-> !` is not yet
    /// stable.
    ///
    /// Core 1 will be reset from core 0 in order to spawn another task.
    ///
    /// Resetting a single core of a running program can have undesired consequences. Deadlocks are
    /// likely if the core being reset happens to be inside a critical section.
    /// It may even break safety assumptions of some unsafe code. So, be careful when calling this method
    /// more than once.
    pub fn spawn<F>(&mut self, stack: &'static mut [usize], entry: F) -> Result<(), Error>
    where
        F: FnOnce() + Send + 'static,
    {
        if let Some((psm, ppb, fifo)) = self.inner.as_mut() {
            /// Called from the asm `_core1_trampoline` function.
            extern "C" fn core1_startup<F: FnOnce()>(
                entry: *mut ManuallyDrop<F>,
                stack_limit: *mut usize,
            ) -> ! {
                core1_setup(stack_limit);

                let entry = unsafe { ManuallyDrop::take(&mut *entry) };

                // make sure the preceding read doesn't get reordered past the following fifo write
                compiler_fence(Ordering::SeqCst);

                // Signal that it's safe for core 0 to get rid of the original value now.
                //
                // We don't have any way to get at core 1's SIO without using `Peripherals::steal` right now,
                // since svd2rust doesn't really support multiple cores properly.
                let peripherals = unsafe { pac::Peripherals::steal() };
                let mut sio = Sio::new(peripherals.SIO);
                sio.fifo.write_blocking(1);

                entry();
                loop {
                    crate::arch::wfe()
                }
            }

            // Reset the core
            // TODO: resetting without prior check that the core is actually stowed is not great.
            // But there does not seem to be any obvious way to check that. A marker flag could be
            // set from this method and cleared for the wrapper after `entry` returned. But doing
            // so wouldn't be zero cost.
            psm.frce_off().modify(|_, w| w.proc1().set_bit());
            while !psm.frce_off().read().proc1().bit_is_set() {
                crate::arch::nop();
            }
            psm.frce_off().modify(|_, w| w.proc1().clear_bit());

            // Set up the stack
            // AAPCS requires in 6.2.1.2 that the stack is 8bytes aligned., we may need to trim the
            // array size to guarantee that the base of the stack (the end of the array) meets that requirement.
            // The start of the array does not need to be aligned.

            let raw_stack_top = stack.as_mut_ptr_range().end;

            // on arm, usize are 4 bytes, so align_offset(8) on a *mut usize returns either 0 or 1.
            let misalignment_offset = raw_stack_top.align_offset(8);

            let aligned_stack_top = unsafe { raw_stack_top.sub(misalignment_offset) };

            // We don't want to drop this, since it's getting moved to the other core.
            let mut entry = ManuallyDrop::new(entry);

            let mut stack_ptr = aligned_stack_top;

            // Push `core1_startup` and its the arguments to the stack.
            //
            // Our stack grows downwards. We want `entry` at the lowest address,
            // which is the first to be popped.
            unsafe {
                #[cfg(target_arch = "riscv32")]
                {
                    // Push `GP`.
                    //
                    // The trampoline puts this back in gp.
                    let mut gp: usize;
                    core::arch::asm!("mv {0},gp", out(reg) gp);
                    stack_ptr = stack_ptr.sub(1);
                    stack_ptr.write(gp);
                }

                // Push extern "C" wrapper function.
                //
                // It will get the next two values as its args thanks to
                // our trampoline.
                //
                // This ends up in pc/a2.
                stack_ptr = stack_ptr.sub(1);
                stack_ptr.write(core1_startup::<F> as usize);

                // Push `stack_limit`.
                // This ends up in r1/a1.
                stack_ptr = stack_ptr.sub(1);
                stack_ptr.write(stack.as_mut_ptr() as usize);

                // Push `entry`.
                // This ends up in r0/a0.
                stack_ptr = stack_ptr.sub(1);
                stack_ptr.write(&mut entry as *mut ManuallyDrop<F> as usize);
            }

            // Make sure the compiler does not reorder the stack writes after to after the
            // below FIFO writes, which would result in them not being seen by the second
            // core.
            //
            // From the compiler perspective, this doesn't guarantee that the second core
            // actually sees those writes. However, we know that the rp235x doesn't have
            // memory caches, and writes happen in-order.
            compiler_fence(Ordering::Release);

            let vector_table = ppb.vtor().read().bits();

            // After reset, core 1 is waiting to receive commands over FIFO.
            // This is the sequence to have it jump to some code.
            let cmd_seq = [
                0,
                0,
                1,
                vector_table as usize,
                // This points at where we put stuff on the stack. As the
                // trampoline pops it off, the address will end up back at
                // aligned_stack_top.
                stack_ptr as usize,
                _core1_trampoline as usize,
            ];

            let mut seq = 0;
            let mut fails = 0;
            loop {
                let cmd = cmd_seq[seq] as u32;
                if cmd == 0 {
                    fifo.drain();
                    crate::arch::sev();
                }
                fifo.write_blocking(cmd);
                let response = fifo.read_blocking();
                if cmd == response {
                    seq += 1;
                } else {
                    seq = 0;
                    fails += 1;
                    if fails > 16 {
                        // The second core isn't responding, and isn't going to take the entrypoint,
                        // so we have to drop it ourselves.
                        drop(ManuallyDrop::into_inner(entry));
                        return Err(Error::Unresponsive);
                    }
                }
                if seq >= cmd_seq.len() {
                    break;
                }
            }

            // Wait until the other core has copied `entry` before returning.
            fifo.read_blocking();

            Ok(())
        } else {
            Err(Error::InvalidCore)
        }
    }
}
