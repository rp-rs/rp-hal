#![no_std]

pub use rp2040_hal as hal;
#[cfg(feature = "rt")]
pub use rp2040_hal::entry;

/// The linker will place this boot block at the start of our program image. We
/// need this to help the ROM bootloader get our code up and running.
#[cfg(feature = "boot2")]
#[link_section = ".boot2"]
#[no_mangle]
#[used]
pub static BOOT2_FIRMWARE: [u8; 256] = rp2040_boot2::BOOT_LOADER_W25Q080;

pub use hal::pac;

hal::bsp_pins!(
    Gpio0 { name: tx },
    Gpio1 { name: rx },
    Gpio2 { name: sck },
    Gpio3 { name: copi },
    Gpio4 { name: cipo },
    Gpio6 { name: sda },
    Gpio7 { name: scl },
    Gpio8 { name: ws2812 },
    Gpio16 { name: gpio16 },
    Gpio17 { name: gpio17 },
    Gpio18 { name: gpio18 },
    Gpio19 { name: gpio19 },
    Gpio20 { name: gpio20 },
    Gpio21 { name: gpio21 },
    Gpio22 { name: gpio22 },
    Gpio25 { name: led },
    Gpio26 { name: adc0 },
    Gpio27 { name: adc1 },
    Gpio28 { name: adc2 },
    Gpio29 { name: adc3 },
);

pub const XOSC_CRYSTAL_FREQ: u32 = 12_000_000;
