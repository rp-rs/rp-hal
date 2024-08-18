//! Interface to the RP2350's One Time Programmable Memory

/// The ways in which we can fail to read OTP
#[derive(Debug, Clone)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum Error {
    /// The user passed an invalid index to a function.
    InvalidIndex,
    /// The hardware refused to let us read this word, probably due to
    /// read lock set earlier in the boot process.
    InvalidPermissions,
}

/// OTP read address, using automatic Error Correction.
///
/// A 32-bit read returns the ECC-corrected data for two neighbouring rows, or
/// all-ones on permission failure. Only the first 8 KiB is populated.
pub const OTP_DATA_BASE: *const u32 = 0x4013_0000 as *const u32;

/// OTP read address, without using any automatic Error Correction.
///
/// A 32-bit read returns 24-bits of raw data from the OTP word.
pub const OTP_DATA_RAW_BASE: *const u32 = 0x4013_4000 as *const u32;

/// How many pages in OTP (post error-correction)
pub const NUM_PAGES: usize = 64;

/// How many rows in one page in OTP (post error-correction)
pub const NUM_ROWS_PER_PAGE: usize = 64;

/// How many rows in OTP (post error-correction)
pub const NUM_ROWS: usize = NUM_PAGES * NUM_ROWS_PER_PAGE;

/// Read one ECC protected word from the OTP
pub fn read_ecc_word(row: usize) -> Result<u16, Error> {
    if row >= NUM_ROWS {
        return Err(Error::InvalidIndex);
    }
    // First do a raw read to check permissions
    let _ = read_raw_word(row)?;
    // One 32-bit read gets us two rows
    let offset = row >> 1;
    // # Safety
    //
    // We checked this offset was in range already.
    let value = unsafe { OTP_DATA_BASE.add(offset).read() };
    if (row & 1) == 0 {
        Ok(value as u16)
    } else {
        Ok((value >> 16) as u16)
    }
}

/// Read one raw word from the OTP
///
/// You get the 24-bit raw value in the lower part of the 32-bit result.
pub fn read_raw_word(row: usize) -> Result<u32, Error> {
    if row >= NUM_ROWS {
        return Err(Error::InvalidIndex);
    }
    // One 32-bit read gets us one row
    // # Safety
    //
    // We checked this offset was in range already.
    let value = unsafe { OTP_DATA_RAW_BASE.add(row).read() };
    if value == 0xFFFF_FFFF {
        Err(Error::InvalidPermissions)
    } else {
        Ok(value)
    }
}
