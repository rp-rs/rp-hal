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
pub static BOOT2_FIRMWARE: [u8; 256] = rp2040_boot2::BOOT_LOADER_GD25Q64CS;

pub use hal::pac;

hal::bsp_pins!(
    Gpio3 {
        name: mosi,
        aliases: { FunctionSpi: Mosi }
    },
    Gpio4 {
        name: miso
        aliases: { FunctionSpi: Miso }
    },
    Gpio5 {
        name: rx,
        aliases: { FunctionUart: UartRx }
    },
    Gpio6 {
        name: sclk,
        aliases: { FunctionSpi: Sclk }
    },
    Gpio11 { name: neopixel_power },
    Gpio12 { name: neopixel_data },
    Gpio20 {
        name: tx,
        aliases: { FunctionUart: UartTx }
    },
    Gpio21 {
        name: button
    },
    Gpio22 {
        name: sda1,
        aliases: { FunctionI2C: Sda1 }
    },
    Gpio23 {
        name: scl1,
        aliases: { FunctionI2C: Scl1 }
    },
    Gpio24 {
        name: sda,
        aliases: { FunctionI2C: Sda }
    },
    Gpio25 {
        name: scl,
        aliases: { FunctionI2C: Scl }
    },
    Gpio26 { name: a3 },
    Gpio27 { name: a2 },
    Gpio28 { name: a1 },
    Gpio29 { name: a0 },
);

pub const XOSC_CRYSTAL_FREQ: u32 = 12_000_000;
