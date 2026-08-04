//! True Random Number Generator (TRNG)
//!
//! This module provides a HAL abstraction for the RP235x TrustZone Random Number Generator.
//!
//! See [RP2350 Datasheet Section 12.12](https://rptl.io/rp2350-datasheet#section_trng) for more details.
//!
//! ## Usage
//!
//! ```no_run
//! use rp235x_hal::{self as hal, trng::Trng};
//! use rand_core::RngCore;
//!
//! let mut pac = hal::pac::Peripherals::take().unwrap();
//!
//! // Create and initialize TRNG
//! let mut trng = Trng::new(pac.TRNG, &mut pac.RESETS);
//!
//! // Generate random numbers
//! let random_u32: u32 = trng.next_u32();
//! let random_u64: u64 = trng.next_u64();
//! ```

use crate::{
    pac::{self, TRNG},
    resets::SubsystemReset,
    typelevel::Sealed,
};

/// TrustZone Random Number Generator peripheral
pub struct Trng {
    trng: TRNG,
}

impl Trng {
    /// Creates a new TRNG instance from the PAC peripheral and initializes it.
    ///
    /// This function:
    /// - Brings the TRNG out of reset
    /// - Enables the random source
    ///
    /// # Arguments
    ///
    /// * `trng` - The TRNG peripheral
    /// * `resets` - The RESETS peripheral
    pub fn new(trng: TRNG, resets: &mut pac::RESETS) -> Self {
        let out = Self { trng };
        out.trng.reset_bring_up(resets);
        out.source_enable();
        out
    }

    /// Enable the TRNG random source so it starts collecting entropy.
    pub fn source_enable(&self) {
        self.trng
            .rnd_source_enable()
            .write(|w| w.rnd_src_en().set_bit());
    }

    /// Disable the TRNG random source so it stops collecting entropy.
    pub fn source_disable(&self) {
        self.trng
            .rnd_source_enable()
            .write(|w| w.rnd_src_en().clear_bit());
    }

    /// Check if the TRNG is busy generating random data.
    pub fn is_busy(&self) -> bool {
        self.trng.trng_busy().read().trng_busy().bit_is_set()
    }

    /// Wait for TRNG to have valid data available.
    ///
    /// This blocks until the TRNG has collected 192 bits of valid random data.
    pub fn wait_for_valid(&self) {
        while self.trng.trng_valid().read().ehr_valid().bit_is_clear() {}
    }

    /// Read all 6 EHR_DATA registers (192 bits / 24 bytes total).
    ///
    /// This function waits for valid data to be available before reading.
    /// Reading the last register (ehr_data5) clears all EHR_DATA registers,
    /// ensuring data is not reused.
    pub fn read_192(&self) -> [u8; 24] {
        self.wait_for_valid();
        let data = [
            self.trng.ehr_data0().read().ehr_data0().bits(),
            self.trng.ehr_data1().read().ehr_data1().bits(),
            self.trng.ehr_data2().read().ehr_data2().bits(),
            self.trng.ehr_data3().read().ehr_data3().bits(),
            self.trng.ehr_data4().read().ehr_data4().bits(),
            self.trng.ehr_data5().read().ehr_data5().bits(),
        ];

        // Convert 6 u32 values to 24 bytes (192 bits)
        let mut out = [0u8; 24];
        for (i, &word) in data.iter().enumerate() {
            let bytes = word.to_le_bytes();
            out[i * 4..(i + 1) * 4].copy_from_slice(&bytes);
        }
        out
    }

    /// Perform a software reset of the TRNG.
    pub fn software_reset(&self) {
        self.trng
            .trng_sw_reset()
            .write(|w| w.trng_sw_reset().set_bit());
    }

    /// Release the underlying PAC peripheral.
    pub fn free(self) -> TRNG {
        self.trng
    }
}

impl Sealed for Trng {}

impl rand_core::RngCore for Trng {
    /// Generate 32 bits of random data.
    ///
    /// Uses `next_u32_via_fill` which fills a 4-byte buffer.
    fn next_u32(&mut self) -> u32 {
        rand_core::impls::next_u32_via_fill(self)
    }

    /// Generate 64 bits of random data.
    ///
    /// Uses `next_u64_via_fill` which fills an 8-byte buffer.
    fn next_u64(&mut self) -> u64 {
        rand_core::impls::next_u64_via_fill(self)
    }

    /// Fill a buffer with random bytes.
    ///
    /// This reads 192-bit chunks from the TRNG until the buffer is filled.
    /// Each 192-bit read is guaranteed to be fresh data since reading the
    /// last register clears all EHR_DATA registers.
    fn fill_bytes(&mut self, dest: &mut [u8]) {
        let mut offset = 0;
        while offset < dest.len() {
            let chunk = self.read_192();
            let remaining = dest.len() - offset;
            let to_copy = chunk.len().min(remaining);
            dest[offset..offset + to_copy].copy_from_slice(&chunk[..to_copy]);
            offset += to_copy;
        }
    }
}

impl rand_core::CryptoRng for Trng {}
