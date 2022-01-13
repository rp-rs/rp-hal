#![no_std]

pub extern crate rp2040_hal as hal;

#[cfg(feature = "rt")]
extern crate cortex_m_rt;
#[cfg(feature = "rt")]
pub use cortex_m_rt::entry;

/// The linker will place this boot block at the start of our program image. We
/// need this to help the ROM bootloader get our code up and running.
#[cfg(feature = "boot2")]
#[link_section = ".boot2"]
#[no_mangle]
#[used]
pub static BOOT2_FIRMWARE: [u8; 256] = rp2040_boot2::BOOT_LOADER_W25Q080;

pub use hal::pac;

hal::bsp_pins!(
    Gpio12 { name: button },
    Gpio16 {
        name: sda,
        aliases: { FunctionI2C: Sda }
    },
    Gpio17 {
        name: scl,
        aliases: { FunctionI2C: Scl }
    },
    Gpio27 { name: neopixel },
);

pub const XOSC_CRYSTAL_FREQ: u32 = 12_000_000;
