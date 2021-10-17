//! Provide atomic access to registers, as described in
//! [section 2.1.2 of the reference manual](https://datasheets.raspberrypi.com/rp2040/rp2040-datasheet.pdf#atomic-rwtype)

use core::ptr::write_volatile;

/// Perform atomic XOR operation on register
///
/// # Safety
///
/// In addition to the requirements of [core::ptr::write_volatile],
/// reg must point to a register providing atomic aliases.
#[inline]
pub unsafe fn write_xor(reg: *mut u32, bits: u32) {
    let alias = (reg as usize + 0x1000) as *mut u32;
    write_volatile(alias, bits);
}

/// Perform atomic bitmask set operation on register
///
/// # Safety
///
/// In addition to the requirements of [core::ptr::write_volatile],
/// reg must point to a register providing atomic aliases.
#[inline]
pub unsafe fn write_bitmask_set(reg: *mut u32, bits: u32) {
    let alias = (reg as usize + 0x2000) as *mut u32;
    write_volatile(alias, bits);
}

/// Perform atomic bitmask clear operation on register
///
/// # Safety
///
/// In addition to the requirements of [core::ptr::write_volatile],
/// reg must point to a register providing atomic aliases.
#[inline]
pub unsafe fn write_bitmask_clear(reg: *mut u32, bits: u32) {
    let alias = (reg as usize + 0x3000) as *mut u32;
    write_volatile(alias, bits);
}
