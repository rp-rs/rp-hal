//! Provide atomic access to peripheral registers
//!
//! This feature is not available for all peripherals.
//! See [section 2.1.2 of the RP2040 datasheet][section_2_1_2] for details.
//!
//! [section_2_1_2]: https://datasheets.raspberrypi.com/rp2040/rp2040-datasheet.pdf#atomic-rwtype

use core::ptr::write_volatile;

/// Perform atomic bitmask set operation on register
///
/// See [section 2.1.2 of the RP2040 datasheet][section_2_1_2] for details.
///
/// [section_2_1_2]: https://datasheets.raspberrypi.com/rp2040/rp2040-datasheet.pdf#atomic-rwtype
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
/// See [section 2.1.2 of the RP2040 datasheet][section_2_1_2] for details.
///
/// [section_2_1_2]: https://datasheets.raspberrypi.com/rp2040/rp2040-datasheet.pdf#atomic-rwtype
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
