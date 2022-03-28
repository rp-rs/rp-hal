//! Multicore support
//!
//! This module handles setup of the 2nd cpu core on the rp2040, which we refer to as core1.
//! It provides functionality for setting up the stack, and starting core1.
//!
//! The options for an entrypoint for core1 are
//! - a function that never returns - eg
//! `fn core1_task() -> ! { loop{} }; `
//! - a lambda (note: This requires a global allocator which requires a nightly compiler. Not recommended for beginners)
//!
//! # Usage
//!
//! ```no_run
//! static mut CORE1_STACK: Stack<4096> = Stack::new();
//! fn core1_task() -> ! {
//!     loop{}
//! }
//! // fn main() -> ! {
//!     use rp2040_hal::{pac, gpio::Pins, sio::Sio, multicore::{Multicore, Stack}};
//!     let mut pac = pac::Peripherals::take().unwrap();
//!     let mut sio = Sio::new(pac.SIO);
//!     // Other init code above this line
//!     let mut mc = Multicore::new(&mut pac.PSM, &mut pac.PPB, &mut sio);
//!     let cores = mc.cores();
//!     let core1 = &mut cores[1];
//!     let _test = core1.spawn(core1_task, unsafe { &mut CORE1_STACK.mem });
//!     // The rest of your application below this line
//! //}
//!
//! ```
//!
//! For inter-processor communications, see [`crate::sio::SioFifo`] and [`crate::sio::Spinlock0`]
//!
//! For a detailed example, see [examples/multicore_fifo_blink.rs](https://github.com/rp-rs/rp-hal/tree/main/rp2040-hal/examples/multicore_fifo_blink.rs)

use crate::pac;

#[cfg(feature = "alloc")]
extern crate alloc;

/// Errors for multicore operations.
#[derive(Debug)]
pub enum Error {
    /// Operation is invalid on this core.
    InvalidCore,
    /// Core was unresposive to commands.
    Unresponsive,
    /// The vector table did not meet cortex-m vector table alignment requirements
    /// - Vector table must be 32 word (128 byte) aligned
    InvalidVectorTableAlignment,
    /// Vector table is not in SRAM, XIP SRAM or Flash
    InvalidProgramLocation,
    /// Invalid stack pointer alignment
    InvalidStackPointerAlignment,
    /// Stack pointer address is not inside of RAM
    InvalidStackPointerAddress,
    /// Invalid program entry address - is before the start of the program
    InvalidEntryAddressBelow,
    /// Invalid program entry address - program in SRAM but entry point is not
    InvalidEntryAddressAboveRAM,
    /// Invalid program entry address - program in XIP SRAM but entry point is not
    InvalidEntryAddressAboveXIPRAM,
    /// Invalid program entry address - program in Flash but entry point is not
    InvalidEntryAddressAboveFLASH,
}

// We pass data to cores via the stack, so we read
// the data off the stack and into parameters that
// rust can read here. Ideally this would be a
// #[naked] function but that is not stable yet.
static MULTICORE_TRAMPOLINE: [u16; 2] = [
    0xbd03, // pop {r0, r1, pc} - call wrapper (pc) with r0 and r1
    0x46c0, // nop - pad this out to 32 bits long
];

#[inline(always)]
fn install_stack_guard(stack_bottom: *mut usize) {
    let core = unsafe { pac::CorePeripherals::steal() };

    // Trap if MPU is already configured
    if core.MPU.ctrl.read() != 0 {
        cortex_m::asm::udf();
    }

    // The minimum we can protect is 32 bytes on a 32 byte boundary, so round up which will
    // just shorten the valid stack range a tad.
    let addr = (stack_bottom as u32 + 31) & !31;
    // Mask is 1 bit per 32 bytes of the 256 byte range... clear the bit for the segment we want
    let subregion_select = 0xff ^ (1 << ((addr >> 5) & 7));
    unsafe {
        core.MPU.ctrl.write(5); // enable mpu with background default map
        core.MPU.rbar.write((addr & !0xff) | 0x8);
        core.MPU.rasr.write(
            1 // enable region
               | (0x7 << 1) // size 2^(7 + 1) = 256
               | (subregion_select << 8)
               | 0x10000000, // XN = disable instruction fetch; no other bits means no permissions
        );
    }
}

#[inline(always)]
fn core1_setup(stack_bottom: *mut usize) {
    install_stack_guard(stack_bottom);
    // TODO: irq priorities
}

/// Perform some basic validation of the program payload
///
/// Validation performed:
/// - Check `vector_table_addr` is correctly aligned. See
/// [CM0+ user guide](https://developer.arm.com/documentation/dui0662/b/The-Cortex-M0--Processor/Exception-model/Vector-table)
/// for reference
/// - Check that `vector_table_addr` is in a valid memory type (SRAM, XIP_SRAM, Flash)
/// - Check `entry_addr` is after `vector_table_addr`
/// - Check that `entry_addr` is not beyond the end of the memory type
/// (SRAM, XIP_SRAM, Flash) that the `vector_table_addr` is in.
fn validate_bootstrap_payload(
    vector_table_addr: usize,
    stack_addr: usize,
    entry_addr: usize,
) -> Result<(), Error> {
    if vector_table_addr & (0x100 - 1) != 0 {
        // Vector table was not 64 word (256 byte) aligned
        return Err(Error::InvalidVectorTableAlignment);
    }
    if stack_addr & (0x4 - 1) != 0 {
        // Stack pointer must be 4 byte aligned - invalid vector table?
        return Err(Error::InvalidStackPointerAlignment);
    }
    if entry_addr <= vector_table_addr {
        // Reset vector pointed to before program to bootload
        return Err(Error::InvalidEntryAddressBelow);
    }

    // Since we didn't pass in the size of the program we don't really know where it ends
    // We can still check that we're within the memory segment the program is loaded into
    if vector_table_addr & 0x2000_0000 == 0x2000_0000 {
        // Program is in RAM
        if entry_addr >= 0x2004_2000 {
            // Reset vector pointed off the end of RAM
            return Err(Error::InvalidEntryAddressAboveRAM);
        }
    } else if vector_table_addr & 0x1500_0000 == 0x1500_0000 {
        // Program is in XIP RAM
        if entry_addr >= 0x15004000 {
            // Reset vector pointed off the end of XIP RAM
            return Err(Error::InvalidEntryAddressAboveXIPRAM);
        }
    } else if vector_table_addr & 0x1000_0000 == 0x1000_0000 {
        // Program is in Flash
        if entry_addr >= 0x1100_0000 {
            // Reset vector pointed off the end of Flash
            return Err(Error::InvalidEntryAddressAboveFLASH);
        }
    } else {
        return Err(Error::InvalidProgramLocation);
    }

    // Verify stack pointer is in RAM
    if !(0x2000_0000..=0x2004_2000).contains(&stack_addr) {
        return Err(Error::InvalidStackPointerAddress);
    }
    // If we haven't hit any of the previous guard clauses,
    // we have validated successfully
    Ok(())
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

impl<const SIZE: usize> Stack<SIZE> {
    /// Construct a stack of length SIZE, initialized to 0
    pub const fn new() -> Stack<SIZE> {
        Stack { mem: [0; SIZE] }
    }
}

impl<'p> Multicore<'p> {
    /// Create a new |Multicore| instance.
    pub fn new(psm: &'p mut pac::PSM, ppb: &'p mut pac::PPB, sio: &'p mut crate::Sio) -> Self {
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
    inner: Option<(&'p mut pac::PSM, &'p mut pac::PPB, &'p mut crate::Sio)>,
}

impl<'p> Core<'p> {
    /// Get the id of this core.
    pub fn id(&self) -> u8 {
        match self.inner {
            None => 0,
            Some(..) => 1,
        }
    }

    fn inner_bootstrap(
        &mut self,
        vector_table: usize,
        stack_ptr: usize,
        entry: usize,
    ) -> Result<(), Error> {
        if let Some((psm, _ppb, sio)) = self.inner.as_mut() {
            // Reset the core
            psm.frce_off.modify(|_, w| w.proc1().set_bit());
            while !psm.frce_off.read().proc1().bit_is_set() {
                cortex_m::asm::nop();
            }
            psm.frce_off.modify(|_, w| w.proc1().clear_bit());
            // After reset, core 1 is waiting to receive commands over FIFO.
            // This is the sequence to have it jump to some code.
            let cmd_seq = [0, 0, 1, vector_table, stack_ptr, entry];

            let mut seq = 0;
            let mut fails = 0;
            loop {
                let cmd = cmd_seq[seq] as u32;
                // Always drain the READ FIFO (from core 1) before sending a 0
                if cmd == 0 {
                    sio.fifo.drain();
                    // Execute a SEV as core 1 may be waiting for FIFO space via WFE
                    cortex_m::asm::sev();
                }
                sio.fifo.write_blocking(cmd);
                let response = sio.fifo.read_blocking();
                if cmd == response {
                    // Move to next state on correct response (core1 replied with cmd value)
                    seq += 1;
                } else {
                    // otherwise start over
                    seq = 0;
                    fails += 1;
                    if fails > 16 {
                        return Err(Error::Unresponsive);
                    }
                }
                if seq >= cmd_seq.len() {
                    break;
                }
            }

            Ok(())
        } else {
            Err(Error::InvalidCore)
        }
    }

    fn inner_spawn(
        &mut self,
        wrapper: *mut (),
        entry: *mut (),
        stack: &'static mut [usize],
    ) -> Result<(), Error> {
        if let Some((_psm, ppb, _sio)) = self.inner.as_mut() {
            // Set up the stack
            let mut stack_ptr = unsafe { stack.as_mut_ptr().add(stack.len()) };

            let mut push = |v: usize| unsafe {
                stack_ptr = stack_ptr.sub(1);
                stack_ptr.write(v);
            };

            push(wrapper as usize);
            push(stack.as_mut_ptr() as usize);
            push(entry as usize);

            let vector_table = ppb.vtor.read().bits();

            self.inner_bootstrap(
                vector_table as usize,
                stack_ptr as usize,
                MULTICORE_TRAMPOLINE.as_ptr() as usize + 1,
            )
        } else {
            Err(Error::InvalidCore)
        }
    }

    /// Spawn a function on this core.
    #[cfg(not(feature = "alloc"))]
    pub fn spawn(&mut self, entry: fn() -> !, stack: &'static mut [usize]) -> Result<(), Error> {
        #[allow(improper_ctypes_definitions)]
        extern "C" fn core1_no_alloc(entry: fn() -> !, stack_bottom: *mut usize) -> ! {
            core1_setup(stack_bottom);
            entry();
        }

        self.inner_spawn(core1_no_alloc as _, entry as _, stack)
    }

    /// Spawn a function on this core.
    #[cfg(feature = "alloc")]
    pub fn spawn<F>(&mut self, entry: F, stack: &'static mut [usize]) -> Result<(), Error>
    where
        F: FnOnce() -> bad::Never,
        F: Send + 'static,
    {
        use alloc::boxed::Box;

        let main: Box<dyn FnOnce() -> bad::Never> = Box::new(move || entry());
        let p = Box::into_raw(Box::new(main));

        extern "C" fn core1_alloc(entry: *mut (), stack_bottom: *mut usize) -> ! {
            core1_setup(stack_bottom);
            let main = unsafe { Box::from_raw(entry as *mut Box<dyn FnOnce() -> bad::Never>) };
            main();
        }

        self.inner_spawn(core1_alloc as _, p as _, stack)
    }

    /// Bootload a Rust program at address `core1_prog_addr` on this core
    ///
    /// Reads the initial stack pointer value and reset vector from
    /// the provided vector table address, then bootstraps core1 using
    /// this information
    ///
    /// # Safety
    /// - You need to compile your program to have prog_addr as start of
    /// flash, so that the vector table is at prog_addr + 0x0
    /// - The vector table must be valid, with a valid stack pointer as the first word and
    /// a valid reset vector as the second word.
    pub unsafe fn bootload(&mut self, prog_addr: usize) -> Result<(), Error> {
        let prog = prog_addr as *const usize;
        // Stack pointer is u32 at offset 0 of the vector table
        let stack_ptr = prog.read_volatile();
        // Reset vector is u32 at offset 1 of the vector table
        let reset_vector = prog.offset(1).read_volatile();
        // Check if our program's addresses make sense
        let validated = validate_bootstrap_payload(prog_addr, stack_ptr, reset_vector);

        // The entry point of a Rust cortex-m program is the reset vector
        if validated.is_ok() {
            self.inner_bootstrap(prog_addr, stack_ptr, reset_vector)
        } else {
            validated
        }
    }
}

// https://github.com/nvzqz/bad-rs/blob/master/src/never.rs
#[cfg(feature = "alloc")]
mod bad {
    pub(crate) type Never = <F as HasOutput>::Output;

    pub trait HasOutput {
        type Output;
    }

    impl<O> HasOutput for fn() -> O {
        type Output = O;
    }

    type F = fn() -> !;
}
