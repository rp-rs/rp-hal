//! Interrupt vector table utilities
//!
//! Provide functionality to switch to another vector table using the
//! Vector Table Offset Register (VTOR) of the Cortex-M0+
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
                192,
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
