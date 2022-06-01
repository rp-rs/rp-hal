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
//! static mut CORE1_STACK: Stack<4096> = Stack::new();
//!
//! fn core1_task() -> ! {
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
//!     let _test = core1.spawn(unsafe { &mut CORE1_STACK.mem }, core1_task);
//!     // The rest of your application below this line
//!     # loop {}
//! }
//!
//! ```
//!
//! For inter-processor communications, see [`crate::sio::SioFifo`] and [`crate::sio::Spinlock0`]
//!
//! For a detailed example, see [examples/multicore_fifo_blink.rs](https://github.com/rp-rs/rp-hal/tree/main/rp2040-hal/examples/multicore_fifo_blink.rs)

use core::mem::ManuallyDrop;
use core::sync::atomic::compiler_fence;
use core::sync::atomic::Ordering;

use crate::pac;
use crate::Sio;

/// Errors for multicore operations.
#[derive(Debug)]
pub enum Error {
    /// Operation is invalid on this core.
    InvalidCore,
    /// Core was unresponsive to commands.
    Unresponsive,
}

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

/// Entry for a Vector in the Interrupt Vector Table.
///
/// Each entry in the Vector table is a union with usize to allow it to be 0 initialized via const initializer
///
/// Implementation borrowed from https://docs.rs/cortex-m-rt/0.7.1/cortex_m_rt/index.html#__interrupts
#[derive(Clone, Copy)]
pub union Vector {
    handler: extern "C" fn(),
    reserved: usize,
}

/// Data type for a properly aligned interrupt vector table
///
/// The VTOR register can only point to a 256 byte offsets - see
/// [Cortex-M0+ Devices Generic User Guide](https://developer.arm.com/documentation/dui0662/b/The-Cortex-M0--Processor/Exception-model/Vector-table) -
/// so that is our required alignment.
/// The vector table length depends on the number of interrupts the system supports.
/// The first 16 words are defined in the ARM Cortex-M spec.
/// The M0+ cores on RP2040 have 32 interrupts, of which only 26 are wired to external interrupt
/// signals - but the last 6 can be used for software interrupts so leave room for them
#[repr(C, align(256))]
pub struct VectorTable {
    /// SP + Reset vector + 14 exceptions + 32 interrupts = 48 entries (192 bytes) in an RP2040 core's VectorTable
    table: [Vector; 48],
}

impl VectorTable {
    /// Create a new vector table. All entries will point to 0 - you must call init()
    /// on this to copy the current vector table before setting it as active
    pub const fn new() -> VectorTable {
        VectorTable {
            table: [Vector { reserved: 0 }; 48],
        }
    }

    /// Initialise our vector table by copying the current table on top of it
    pub fn init(&mut self, ppb: &mut pac::PPB) {
        let vector_table = ppb.vtor.read().bits();
        unsafe {
            crate::rom_data::memcpy44(
                &mut self.table as *mut _ as *mut u32,
                vector_table as *const u32,
                core::mem::size_of::<VectorTable>() as u32,
            )
        };
    }

    /// Dynamically register a function as being an interrupt handler
    pub fn register_handler(&mut self, interrupt_idx: usize, interrupt_fn: extern "C" fn()) {
        self.table[16 + interrupt_idx].handler = interrupt_fn;
    }

    /// Set the stack pointer address in a VectorTable. This will be used on Reset
    ///
    /// # Safety
    /// There is no checking whether this is a valid stack pointer address
    pub unsafe fn set_sp(&mut self, stack_pointer_address: usize) {
        self.table[0].reserved = stack_pointer_address;
    }

    /// Set the entry-point address in a VectorTable. This will be used on Reset
    ///
    /// # Safety
    /// There is no checking whether this is a valid entry point
    pub unsafe fn set_entry(&mut self, entry_address: usize) {
        self.table[1].reserved = entry_address;
    }

    /// Switch the current core to use this Interrupt Vector Table
    ///
    /// # Safety
    /// Until the vector table has valid entries, activating it will cause an unhandled hardfault!
    /// You must call init() first.
    pub unsafe fn activate(&mut self, ppb: &mut pac::PPB) {
        ppb.vtor
            .write(|w| w.bits(&mut self.table as *mut _ as *mut u32 as u32));
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

impl<'p> Core<'p> {
    /// Get the id of this core.
    pub fn id(&self) -> u8 {
        match self.inner {
            None => 0,
            Some(..) => 1,
        }
    }

    /// Spawn a function on this core.
    pub fn spawn<F>(&mut self, stack: &'static mut [usize], entry: F) -> Result<(), Error>
    where
        F: FnOnce() -> bad::Never + Send + 'static,
    {
        if let Some((psm, ppb, fifo)) = self.inner.as_mut() {
            // The first two ignored `u64` parameters are there to take up all of the registers,
            // which means that the rest of the arguments are taken from the stack,
            // where we're able to put them from core 0.
            extern "C" fn core1_startup<F: FnOnce() -> bad::Never>(
                _: u64,
                _: u64,
                entry: &mut ManuallyDrop<F>,
                stack_bottom: *mut usize,
            ) -> ! {
                core1_setup(stack_bottom);

                let entry = unsafe { ManuallyDrop::take(entry) };

                // Signal that it's safe for core 0 to get rid of the original value now.
                //
                // We don't have any way to get at core 1's SIO without using `Peripherals::steal` right now,
                // since svd2rust doesn't really support multiple cores properly.
                let peripherals = unsafe { pac::Peripherals::steal() };
                let mut sio = Sio::new(peripherals.SIO);
                sio.fifo.write_blocking(1);

                entry()
            }

            // Reset the core
            psm.frce_off.modify(|_, w| w.proc1().set_bit());
            while !psm.frce_off.read().proc1().bit_is_set() {
                cortex_m::asm::nop();
            }
            psm.frce_off.modify(|_, w| w.proc1().clear_bit());

            // Set up the stack
            let mut stack_ptr = unsafe { stack.as_mut_ptr().add(stack.len()) };

            // We don't want to drop this, since it's getting moved to the other core.
            let mut entry = ManuallyDrop::new(entry);

            // Push the arguments to `core1_startup` onto the stack.
            unsafe {
                // Push `stack_bottom`.
                stack_ptr = stack_ptr.sub(1);
                stack_ptr.cast::<*mut usize>().write(stack.as_mut_ptr());

                // Push `entry`.
                stack_ptr = stack_ptr.sub(1);
                stack_ptr.cast::<&mut ManuallyDrop<F>>().write(&mut entry);
            }

            // Make sure the compiler does not reorder the stack writes after to after the
            // below FIFO writes, which would result in them not being seen by the second
            // core.
            //
            // From the compiler perspective, this doesn't guarantee that the second core
            // actually sees those writes. However, we know that the RP2040 doesn't have
            // memory caches, and writes happen in-order.
            compiler_fence(Ordering::Release);

            let vector_table = ppb.vtor.read().bits();

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

// https://github.com/nvzqz/bad-rs/blob/master/src/never.rs
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
