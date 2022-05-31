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
pub static BOOT2_FIRMWARE: [u8; 256] = rp2040_boot2::BOOT_LOADER_W25Q080;

pub use hal::pac;

hal::bsp_pins!(
    Gpio0 { name: tx0 },
    Gpio1 { name: rx0 },
    Gpio2 { name: gpio2 },
    Gpio3 { name: gpio3 },
    Gpio4 { name: gpio4 },
    Gpio5 { name: gpio5 },
    Gpio6 { name: gpio6 },
    Gpio7 { name: gpio7 },
    Gpio8 { name: tx1 },
    Gpio9 { name: rx1 },
    Gpio16 { name: sda },
    Gpio17 { name: scl },
    Gpio20 { name: cipo },
    Gpio21 { name: ncs },
    Gpio22 { name: sck },
    Gpio23 { name: copi },
    Gpio25 { name: led },
    Gpio26 { name: adc0 },
    Gpio27 { name: adc1 },
    Gpio28 { name: adc2 },
    Gpio29 { name: adc3 },
);

pub const XOSC_CRYSTAL_FREQ: u32 = 12_000_000;
