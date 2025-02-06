//! This test needs a connection between:
//!
//! | from GPIO (pico Pin) | to GPIO (pico Pin) |
//! | -------------------- | ------------------ |
//! |         0 (1)        |       2 (4)        |
//! |         1 (2)        |       3 (5)        |

#![no_std]
#![no_main]
#![cfg(test)]

use defmt_rtt as _; // defmt transport
use defmt_test as _;
use panic_probe as _;
#[cfg(feature = "rp2040")]
use rp2040_hal as hal; // memory layout // panic handler
#[cfg(feature = "rp235x")]
use rp235x_hal as hal;

use hal::{async_utils::AsyncPeripheral, pac::interrupt};

/// The linker will place this boot block at the start of our program image. We
/// need this to help the ROM bootloader get our code up and running.
/// Note: This boot block is not necessary when using a rp-hal based BSP
/// as the BSPs already perform this step.
#[cfg(feature = "rp2040")]
#[link_section = ".boot2"]
#[used]
pub static BOOT2: [u8; 256] = rp2040_boot2::BOOT_LOADER_GENERIC_03H;

/// Tell the Boot ROM about our application
#[cfg(feature = "rp235x")]
#[link_section = ".start_block"]
#[used]
pub static IMAGE_DEF: hal::block::ImageDef = hal::block::ImageDef::secure_exe();

/// External high-speed crystal on the Raspberry Pi Pico board is 12 MHz. Adjust
/// if your board has a different frequency
const XTAL_FREQ_HZ: u32 = 12_000_000u32;

pub mod i2c_tests;
mod init;

#[interrupt]
unsafe fn I2C0_IRQ() {
    i2c_tests::Controller::on_interrupt();
}

#[interrupt]
unsafe fn I2C1_IRQ() {
    i2c_tests::Target::on_interrupt();
}

#[defmt_test::tests]
mod tests {
    use crate::i2c_tests::{
        non_blocking::{self, run_test, State},
        ADDR_10BIT, ADDR_7BIT,
    };

    #[init]
    fn setup() -> State {
        unsafe { crate::init::reset_cleanup() };
        non_blocking::setup(super::XTAL_FREQ_HZ, ADDR_7BIT)
    }

    #[test]
    fn embedded_hal(state: &mut State) {
        run_test(non_blocking::embedded_hal(state, ADDR_7BIT, 2..=2));
        run_test(non_blocking::embedded_hal(state, ADDR_10BIT, 2..=7));
    }

    #[test]
    fn transactions_iter(state: &mut State) {
        run_test(non_blocking::transaction(state, ADDR_7BIT, 7..=9));
        run_test(non_blocking::transaction(state, ADDR_10BIT, 7..=14));
    }

    #[test]
    fn i2c_write_iter(state: &mut State) {
        run_test(non_blocking::transaction_iter(state, ADDR_7BIT));
        run_test(non_blocking::transaction_iter(state, ADDR_10BIT));
    }

    // Sad paths:
    // invalid tx buf on write
    // invalid rx buf on read
    //
    // invalid (rx/tx) buf in transactions
    //
    // Peripheral Nack
    //
    // Arbritration conflict
}
