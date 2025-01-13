//! Multicore support
//!
//! This module handles setup of the 2nd cpu core on the rp2040, which we refer to as core1.
//! It provides functionality for setting up the stack, and starting core1.
//!
//! The entrypoint for core1 can be any function that never returns, including closures.
//!
//! # Usage
//!
//! ```no_run
//! use rp2040_hal::{pac, gpio::Pins, sio::Sio, multicore::{Multicore, Stack}};
//!
//! static CORE1_STACK: Stack<4096> = Stack::new();
//!
//! fn core1_task() {
//!     loop {}
//! }
//!
//! fn main() -> ! {
//!     let mut pac = pac::Peripherals::take().unwrap();
//!     let mut sio = Sio::new(pac.SIO);
//!     // Other init code above this line
//!     let mut mc = Multicore::new(&mut pac.PSM, &mut pac.PPB, &mut sio.fifo);
//!     let cores = mc.cores();
//!     let core1 = &mut cores[1];
//!     let _test = core1.spawn(CORE1_STACK.take().unwrap(), core1_task);
//!     // The rest of your application below this line
//!     # loop {}
//! }
//!
//! ```
//!
//! For inter-processor communications, see [`crate::sio::SioFifo`] and [`crate::sio::Spinlock0`]
//!
//! For a detailed example, see [examples/multicore_fifo_blink.rs](https://github.com/rp-rs/rp-hal/tree/main/rp2040-hal-examples/src/bin/multicore_fifo_blink.rs)

use core::cell::Cell;
use core::cell::UnsafeCell;
use core::mem::ManuallyDrop;
use core::ops::Range;
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
fn install_stack_guard(stack_limit: *mut usize) {
    let core = unsafe { pac::CorePeripherals::steal() };

    // Trap if MPU is already configured
    if core.MPU.ctrl.read() != 0 {
        cortex_m::asm::udf();
    }

    // The minimum we can protect is 32 bytes on a 32 byte boundary, so round up which will
    // just shorten the valid stack range a tad.
    let addr = (stack_limit as u32 + 31) & !31;
    // Mask is 1 bit per 32 bytes of the 256 byte range... clear the bit for the segment we want
    let subregion_select = 0xff ^ (1 << ((addr >> 5) & 7));
    unsafe {
        core.MPU.ctrl.write(5); // enable mpu with background default map
        const RBAR_VALID: u32 = 0x10;
        core.MPU.rbar.write((addr & !0xff) | RBAR_VALID);
        core.MPU.rasr.write(
            1 // enable region
               | (0x7 << 1) // size 2^(7 + 1) = 256
               | (subregion_select << 8)
               | 0x10000000, // XN = disable instruction fetch; no other bits means no permissions
        );
    }
}

#[inline(always)]
fn core1_setup(stack_limit: *mut usize) {
    install_stack_guard(stack_limit);
    // TODO: irq priorities
}

/// Multicore execution management.
pub struct Multicore<'p> {
    cores: [Core<'p>; 2],
}

/// Data type for a properly aligned stack of N 32-bit (usize) words
#[repr(C, align(32))]
pub struct Stack<const SIZE: usize> {
    /// Memory to be used for the stack
    mem: UnsafeCell<[usize; SIZE]>,
    taken: Cell<bool>,
}

impl<const SIZE: usize> Default for Stack<SIZE> {
    fn default() -> Self {
        Self::new()
    }
}

// Safety: Only one thread can `take` access to contents of the
// struct, guarded by a critical section.
unsafe impl<const SIZE: usize> Sync for Stack<SIZE> {}

impl<const SIZE: usize> Stack<SIZE> {
    /// Construct a stack of length SIZE, initialized to 0
    ///
    /// The minimum allowed SIZE is 64 bytes, but most programs
    /// will need a significantly larger stack.
    pub const fn new() -> Stack<SIZE> {
        const { assert!(SIZE >= 64, "Stack too small") };
        Stack {
            mem: UnsafeCell::new([0; SIZE]),
            taken: Cell::new(false),
        }
    }

    /// Take the StackAllocation out of this Stack.
    ///
    /// This returns None if the stack is already taken.
    pub fn take(&self) -> Option<StackAllocation> {
        let taken = critical_section::with(|_| self.taken.replace(true));
        if taken {
            None
        } else {
            // Safety: We know the size of this allocation
            unsafe {
                let start = self.mem.get() as *mut usize;
                let end = start.add(SIZE);
                Some(StackAllocation::from_raw_parts(start, end))
            }
        }
    }

    /// Reset the taken flag of the stack area
    ///
    /// # Safety
    ///
    /// The caller must ensure that the stack is no longer in use, eg. because
    /// the core that used it was reset. This method doesn't do any synchronization
    /// so it must not be called from multiple threads concurrently.
    pub unsafe fn reset(&self) {
        self.taken.replace(false);
    }
}

/// This object represents a memory area which can be used for a stack.
///
/// It is essentially a range of pointers with these additional guarantees:
/// The begin / end pointers must define a stack
/// with proper alignment (at least 8 bytes, preferably 32 bytes)
/// and sufficient size (64 bytes would be sound but much too little for
/// most real-world workloads). The underlying memory must
/// have a static lifetime and must be owned by the object exclusively.
/// No mutable references to that memory must exist.
/// Therefore, a function that gets passed such an object is free to write
/// to arbitrary memory locations in the range.
#[derive(Debug)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub struct StackAllocation {
    /// Start and end pointer of the StackAllocation as a Range
    mem: Range<*mut usize>,
}

impl StackAllocation {
    fn get(&self) -> Range<*mut usize> {
        self.mem.clone()
    }

    /// Unsafely construct a stack allocation
    ///
    /// This is mainly useful to construct a stack allocation in some memory region
    /// defined in a linker script, for example to place the stack in the SRAM4/5 regions.
    ///
    /// # Safety
    ///
    /// The caller must ensure that the guarantees that a StackAllocation provides
    /// are upheld.
    pub unsafe fn from_raw_parts(start: *mut usize, end: *mut usize) -> Self {
        StackAllocation { mem: start..end }
    }
}

impl<const SIZE: usize> From<&Stack<SIZE>> for Option<StackAllocation> {
    fn from(stack: &Stack<SIZE>) -> Self {
        let taken = critical_section::with(|_| stack.taken.replace(true));
        if taken {
            None
        } else {
            // Safety: We know the size of this allocation
            unsafe {
                let start = stack.mem.get() as *mut usize;
                let end = start.add(SIZE);
                Some(StackAllocation::from_raw_parts(start, end))
            }
        }
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

/// A handle for controlling a logical core.
pub struct Core<'p> {
    inner: Option<(
        &'p mut pac::PSM,
        &'p mut pac::PPB,
        &'p mut crate::sio::SioFifo,
    )>,
}

impl Core<'_> {
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
    pub fn spawn<F>(&mut self, stack: StackAllocation, entry: F) -> Result<(), Error>
    where
        F: FnOnce() + Send + 'static,
    {
        if let Some((psm, ppb, fifo)) = self.inner.as_mut() {
            // The first two ignored `u64` parameters are there to take up all of the registers,
            // which means that the rest of the arguments are taken from the stack,
            // where we're able to put them from core 0.
            extern "C" fn core1_startup<F: FnOnce()>(
                _: u64,
                _: u64,
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
                    cortex_m::asm::wfe()
                }
            }

            // Reset the core
            // TODO: resetting without prior check that the core is actually stowed is not great.
            // But there does not seem to be any obvious way to check that. A marker flag could be
            // set from this method and cleared for the wrapper after `entry` returned. But doing
            // so wouldn't be zero cost.
            psm.frce_off().modify(|_, w| w.proc1().set_bit());
            while !psm.frce_off().read().proc1().bit_is_set() {
                cortex_m::asm::nop();
            }
            psm.frce_off().modify(|_, w| w.proc1().clear_bit());

            // Set up the stack
            // AAPCS requires in 6.2.1.2 that the stack is 8bytes aligned., we may need to trim the
            // array size to guaranty that the base of the stack (the end of the array) meets that requirement.
            // The start of the array does not need to be aligned.

            let stack = stack.get();
            let mut stack_ptr = stack.end;
            // on rp2040, usize are 4 bytes, so align_offset(8) on a *mut usize returns either 0 or 1.
            let misalignment_offset = stack_ptr.align_offset(8);

            // We don't want to drop this, since it's getting moved to the other core.
            let mut entry = ManuallyDrop::new(entry);

            // Push the arguments to `core1_startup` onto the stack.
            unsafe {
                stack_ptr = stack_ptr.sub(misalignment_offset);

                // Push `stack_limit`.
                stack_ptr = stack_ptr.sub(1);
                stack_ptr.cast::<*mut usize>().write(stack.start);

                // Push `entry`.
                stack_ptr = stack_ptr.sub(1);
                stack_ptr.cast::<*mut ManuallyDrop<F>>().write(&mut entry);
            }

            // Make sure the compiler does not reorder the stack writes after to after the
            // below FIFO writes, which would result in them not being seen by the second
            // core.
            //
            // From the compiler perspective, this doesn't guarantee that the second core
            // actually sees those writes. However, we know that the RP2040 doesn't have
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
                stack_ptr as usize,
                core1_startup::<F> as usize,
            ];

            let mut seq = 0;
            let mut fails = 0;
            loop {
                let cmd = cmd_seq[seq] as u32;
                if cmd == 0 {
                    fifo.drain();
                    cortex_m::asm::sev();
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
