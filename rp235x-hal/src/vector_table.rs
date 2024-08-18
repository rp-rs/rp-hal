//! Interrupt vector table utilities
//!
//! Provide functionality to switch to another vector table using the
//! Vector Table Offset Register (VTOR) of the Cortex-33
//! Also provides types and utilities for copying a vector table into RAM

/// Entry for a Vector in the Interrupt Vector Table.
///
/// Each entry in the Vector table is a union with usize to allow it to be 0 initialized via const initializer
///
/// Implementation borrowed from https://docs.rs/cortex-m-rt/0.7.1/cortex_m_rt/index.html#__interrupts
#[derive(Clone, Copy)]
union Vector {
    handler: extern "C" fn(),
    reserved: usize,
}

/// Data type for a properly aligned interrupt vector table
///
/// The VTOR register can only point to a 128 byte offsets - see
/// [Cortex-33 Devices Generic User Guide](https://developer.arm.com/documentation/100235/0004/the-cortex-m33-peripherals/system-control-block/vector-table-offset-register?lang=en) -
/// so that is our required alignment.
/// The vector table length depends on the number of interrupts the system supports.
/// The first 16 words are defined in the ARM Cortex-M spec.
/// The Cortex-M33 cores on RP235x have 52 interrupts, of which only 47 are wired to external interrupt
/// signals - but the last 6 can be used for software interrupts so leave room for them
#[repr(C, align(128))]
pub struct VectorTable {
    /// SP + Reset vector + 14 exceptions + 52 interrupts = 68 entries (272 bytes) in an rp235x core's VectorTable
    table: [Vector; 68],
}

impl Default for VectorTable {
    fn default() -> Self {
        Self::new()
    }
}

impl VectorTable {
    /// Create a new vector table. All entries will point to 0 - you must call init()
    /// on this to copy the current vector table before setting it as active
    pub const fn new() -> VectorTable {
        VectorTable {
            table: [Vector { reserved: 0 }; 68],
        }
    }

    /// Initialise our vector table by copying the current table on top of it
    #[allow(unknown_lints)]
    #[allow(clippy::needless_pass_by_ref_mut)]
    pub fn init(&mut self, ppb: &mut crate::pac::PPB) {
        let mut vector_table = ppb.vtor().read().bits() as *const usize;
        for entry in self.table.iter_mut() {
            // Safety:
            //
            // This value must be valid because it's in the current vector table.
            *entry = Vector {
                reserved: unsafe { vector_table.read() },
            };
            // Safety:
            //
            // We are iterating through our copy of the vector table, which we
            // know is the same size as the real vector table.
            unsafe {
                vector_table = vector_table.add(1);
            }
        }
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
    #[allow(unknown_lints)]
    #[allow(clippy::needless_pass_by_ref_mut)]
    pub unsafe fn activate(&mut self, ppb: &mut crate::pac::PPB) {
        ppb.vtor()
            .write(|w| w.bits(&mut self.table as *mut _ as *mut u32 as u32));
    }
}
