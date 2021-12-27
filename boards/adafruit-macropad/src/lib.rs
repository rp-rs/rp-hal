#![no_std]

pub use rp2040_hal as hal;

#[cfg(feature = "rt")]
extern crate cortex_m_rt;
#[cfg(feature = "rt")]
pub use cortex_m_rt::entry;

// Adafruit macropad uses W25Q64JVxQ flash chip. Should work with BOOT_LOADER_W25Q080 (untested)

/// The linker will place this boot block at the start of our program image. We
/// need this to help the ROM bootloader get our code up and running.
#[cfg(feature = "boot2")]
#[link_section = ".boot2"]
#[no_mangle]
#[used]
pub static BOOT2_FIRMWARE: [u8; 256] = rp2040_boot2::BOOT_LOADER_W25Q080;

pub use hal::pac;

hal::bsp_pins!(
    Gpio0 { name: button },
    Gpio1 { name: key1 },
    Gpio2 { name: key2 },
    Gpio3 { name: key3 },
    Gpio4 { name: key4 },
    Gpio5 { name: key5 },
    Gpio6 { name: key6 },
    Gpio7 { name: key7 },
    Gpio8 { name: key8 },
    Gpio9 { name: key9 },
    Gpio10 { name: key10 },
    Gpio11 { name: key11 },
    Gpio12 { name: key12 },
    Gpio13 { name: led },
    Gpio14 {
        name: speaker_shutdown
    },
    Gpio15 { name: speaker },
    Gpio17 {
        name: encoder_rota,
        aliases: { PullUpInput: RotaryA }
    },
    Gpio18 {
        name: encoder_rotb,
        aliases: { PullUpInput: RotaryB }
    },
    Gpio19 { name: neopixel },
    Gpio20 {
        name: sda,
        aliases: { FunctionI2C: Sda }
    },
    Gpio21 {
        name: scl,
        aliases: { FunctionI2C: Scl }
    },
    // This CS is on the wrong SPI channel so cannot be controlled by the peripheral
    Gpio22 { name: oled_cs },
    Gpio23 { name: oled_reset },
    Gpio24 { name: oled_dc },
    Gpio26 {
        name: sclk,
        aliases: { FunctionSpi: Sclk }
    },
    Gpio27 {
        name: mosi,
        aliases: { FunctionSpi: Mosi }
    },
    Gpio28 {
        name: miso,
        aliases: { FunctionSpi: Miso }
    },
);
