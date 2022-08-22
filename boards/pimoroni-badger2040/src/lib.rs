#![no_std]

pub extern crate rp2040_hal as hal;

pub use hal::pac;

#[cfg(feature = "rt")]
pub use rp2040_hal::entry;

/// The linker will place this boot block at the start of our program image. We
/// need this to help the ROM bootloader get our code up and running.
#[cfg(feature = "boot2")]
#[link_section = ".boot2"]
#[no_mangle]
#[used]
pub static BOOT2_FIRMWARE: [u8; 256] = rp2040_boot2::BOOT_LOADER_W25Q080;

hal::bsp_pins!(
    Gpio0 {
        name: gpio0,
        aliases: {
            /// UART Function alias for pin [crate::Pins::gpio0].
            FunctionUart: UartTx
        }
    },
    Gpio1 {
        name: gpio1,
        aliases: {
            /// UART Function alias for pin [crate::Pins::gpio1].
            FunctionUart: UartRx
        }
    },
    Gpio3 { name: i2c_int },
    Gpio4 {
        name: gpio4,
        aliases: {
            /// I2C Function alias for pin [crate::Pins::gpio4].
            FunctionI2C: I2cSda
        }
    },
    Gpio5 {
        name: gpio5,
        aliases: {
            /// I2C Function alias for pin [crate::Pins::gpio5].
            FunctionI2C: I2cScl
        }
    },
    Gpio10 { name: p3v3_en },
    Gpio11 { name: sw_down },
    Gpio12 { name: sw_a },
    Gpio13 { name: sw_b },
    Gpio14 { name: sw_c },
    Gpio15 { name: sw_up },
    Gpio16 {
        name: miso,
        aliases: {
            /// SPI Function alias for pin [crate::Pins::gpio16].
            FunctionSpi: Miso
        }
    },
    Gpio17 {
        name: inky_cs_gpio,
        aliases: {
            /// SPI Function alias for pin [crate::Pins::gpio17].
            FunctionSpi: InkyCs
        }
    },
    Gpio18 {
        name: sclk,
        aliases: {
            /// SPI Function alias for pin [crate::Pins::gpio18].
            FunctionSpi: Sclk
        }
    },
    Gpio19 {
        name: mosi,
        aliases: {
            /// SPI Function alias for pin [crate::Pins::gpio19].
            FunctionSpi: Mosi
        }
    },
    Gpio20 { name: inky_dc },
    Gpio21 { name: inky_res },
    Gpio23 { name: user_sw },
    /// GPIO 24 is connected to vbus_detect of the badger2040.
    Gpio24 { name: vbus_detect },
    /// GPIO 25 is connected to activity LED of the badger2040.
    Gpio25 { name: led },
    Gpio26 { name: inky_busy },
    Gpio27 { name: vref_power },
    Gpio28 { name: vref_1v24 },
    /// GPIO 29 is connected to battery monitor of the badger2040
    Gpio29 { name: vbat_sense },
);

pub const XOSC_CRYSTAL_FREQ: u32 = 12_000_000;
