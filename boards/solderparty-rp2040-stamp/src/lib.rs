#![no_std]

pub use rp2040_hal as hal;

#[cfg(feature = "rt")]
extern crate cortex_m_rt;
#[cfg(feature = "rt")]
pub use hal::entry;

/// The linker will place this boot block at the start of our program image. We
/// need this to help the ROM bootloader get our code up and running.
#[cfg(feature = "boot2")]
#[link_section = ".boot2"]
#[no_mangle]
#[used]
pub static BOOT2_FIRMWARE: [u8; 256] = rp2040_boot2::BOOT_LOADER_GD25Q64CS;

pub use hal::pac;

hal::bsp_pins!(
    Gpio0 { name: gpio0 },
    Gpio1 { name: gpio1 },
    Gpio2 { name: gpio2 },
    Gpio3 { name: gpio3 },
    Gpio4 { name: gpio4 },
    Gpio5 { name: gpio5 },
    Gpio6 { name: gpio6 },
    Gpio7 { name: gpio7 },
    Gpio8 { name: gpio8 },
    Gpio9 { name: gpio9 },
    Gpio10 { name: gpio10 },
    Gpio11 { name: gpio11 },
    Gpio12 { name: gpio12 },
    Gpio13 { name: gpio13 },
    Gpio14 { name: gpio14 },
    Gpio15 { name: gpio15 },
    Gpio16 { name: gpio16 },
    Gpio17 { name: gpio17 },
    Gpio18 { name: gpio18 },
    Gpio19 { name: gpio19 },
    Gpio20 { name: gpio20 },
    Gpio21 { name: neopixel },
    Gpio22 { name: gpio22 },
    Gpio23 { name: gpio23 },
    Gpio24 { name: gpio24 },
    Gpio25 { name: gpio25 },
    Gpio26 { name: gpio26 },
    Gpio27 { name: gpio27 },
    Gpio28 { name: gpio28 },
    Gpio29 { name: gpio29 },
);

pub const XOSC_CRYSTAL_FREQ: u32 = 12_000_000;
