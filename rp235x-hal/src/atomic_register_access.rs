//! Provide atomic access to peripheral registers
//!
//! This feature is not available for all peripherals.
//! See [Section 2.1.3][section_2_1_3] of the RP2350 datasheet for details.
//!
//! [section_2_1_3]: https://rptl.io/rp2350-datasheet#atomic-rwtype

use core::ptr::write_volatile;

/// Perform atomic bitmask set operation on register
///
/// See [Section 2.1.3][section_2_1_3] of the RP2350 datasheet for details.
///
/// [section_2_1_3]: https://rptl.io/rp2350-datasheet#atomic-rwtype
///
/// # Safety
///
/// In addition to the requirements of [core::ptr::write_volatile],
/// `register` must point to a register providing atomic aliases.
#[inline]
pub(crate) unsafe fn write_bitmask_set(register: *mut u32, bits: u32) {
    let alias = (register as usize + 0x2000) as *mut u32;
    write_volatile(alias, bits);
}

/// Perform atomic bitmask clear operation on register
///
/// See [Section 2.1.3][section_2_1_3] of the RP2350 datasheet for details.
///
/// [section_2_1_3]: https://rptl.io/rp2350-datasheet#atomic-rwtype
///
/// # Safety
///
/// In addition to the requirements of [core::ptr::write_volatile],
/// `register` must point to a register providing atomic aliases.
#[inline]
pub(crate) unsafe fn write_bitmask_clear(register: *mut u32, bits: u32) {
    let alias = (register as usize + 0x3000) as *mut u32;
    write_volatile(alias, bits);
}
