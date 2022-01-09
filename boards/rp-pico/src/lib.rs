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
    /// GPIO 0 supports following functions:
    /// - `SPI0 RX`
    /// - `UART0 TX`
    /// - `I2C0 SDA`
    /// - `PWM0 A`
    /// - `SIO`, `PIO0`, `PIO1`
    Gpio0 { name: gpio0 },

    /// GPIO 1 supports following functions:
    /// - `SPI0 CSn`
    /// - `UART0 RX`
    /// - `I2C0 SCL`
    /// - `PWM0 B`
    /// - `SIO`, `PIO0`, `PIO1`
    Gpio1 { name: gpio1 },

    /// GPIO 2 supports following functions:
    /// - `SPI0 SCK`
    /// - `UART0 CTS`
    /// - `I2C1 SDA`
    /// - `PWM1 A`
    /// - `SIO`, `PIO0`, `PIO1`
    Gpio2 { name: gpio2 },

    /// GPIO 3 supports following functions:
    /// - `SPI0 TX`
    /// - `UART0 RTS`
    /// - `I2C1 SCL`
    /// - `PWM1 B`
    /// - `SIO`, `PIO0`, `PIO1`
    Gpio3 { name: gpio3 },

    /// GPIO 4 supports following functions:
    /// - `SPI0 RX`
    /// - `UART1 TX`
    /// - `I2C0 SDA`
    /// - `PWM2 A`
    /// - `SIO`, `PIO0`, `PIO1`
    Gpio4 { name: gpio4 },

    /// GPIO 5 supports following functions:
    /// - `SPI0 CSn`
    /// - `UART1 RX`
    /// - `I2C0 SCL`
    /// - `PWM2 B`
    /// - `SIO`, `PIO0`, `PIO1`
    Gpio5 { name: gpio5 },

    /// GPIO 6 supports following functions:
    /// - `SPI0 SCK`
    /// - `UART1 CTS`
    /// - `I2C1 SDA`
    /// - `PWM3 A`
    /// - `SIO`, `PIO0`, `PIO1`
    Gpio6 { name: gpio6 },

    /// GPIO 7 supports following functions:
    /// - `SPI0 TX`
    /// - `UART1 RTS`
    /// - `I2C1 SCL`
    /// - `PWM3 B`
    /// - `SIO`, `PIO0`, `PIO1`
    Gpio7 { name: gpio7 },

    /// GPIO 8 supports following functions:
    /// - `SPI1 RX`
    /// - `UART1 TX`
    /// - `I2C0 SDA`
    /// - `PWM4 A`
    /// - `SIO`, `PIO0`, `PIO1`
    Gpio8 { name: gpio8 },

    /// GPIO 9 supports following functions:
    /// - `SPI1 CSn`
    /// - `UART1 RX`
    /// - `I2C0 SCL`
    /// - `PWM4 B`
    /// - `SIO`, `PIO0`, `PIO1`
    Gpio9 { name: gpio9 },

    /// GPIO 10 supports following functions:
    /// - `SPI1 SCK`
    /// - `UART1 CTS`
    /// - `I2C1 SDA`
    /// - `PWM5 A`
    /// - `SIO`, `PIO0`, `PIO1`
    Gpio10 { name: gpio10 },

    /// GPIO 11 supports following functions:
    /// - `SPI1 TX`
    /// - `UART1 RTS`
    /// - `I2C1 SCL`
    /// - `PWM5 B`
    /// - `SIO`, `PIO0`, `PIO1`
    Gpio11 { name: gpio11 },

    /// GPIO 12 supports following functions:
    /// - `SPI1 RX`
    /// - `UART0 TX`
    /// - `I2C0 SDA`
    /// - `PWM6 A`
    /// - `SIO`, `PIO0`, `PIO1`
    Gpio12 { name: gpio12 },

    /// GPIO 13 supports following functions:
    /// - `SPI1 CSn`
    /// - `UART0 RX`
    /// - `I2C0 SCL`
    /// - `PWM6 B`
    /// - `SIO`, `PIO0`, `PIO1`
    Gpio13 { name: gpio13 },

    /// GPIO 14 supports following functions:
    /// - `SPI1 SCK`
    /// - `UART0 CTS`
    /// - `I2C1 SDA`
    /// - `PWM7 A`
    /// - `SIO`, `PIO0`, `PIO1`
    Gpio14 { name: gpio14 },

    /// GPIO 15 supports following functions:
    /// - `SPI1 TX`
    /// - `UART0 RTS`
    /// - `I2C1 SCL`
    /// - `PWM7 B`
    /// - `SIO`, `PIO0`, `PIO1`
    Gpio15 { name: gpio15 },

    /// GPIO 16 supports following functions:
    /// - `SPI0 RX`
    /// - `UART0 TX`
    /// - `I2C0 SDA`
    /// - `PWM0 A`
    /// - `SIO`, `PIO0`, `PIO1`
    Gpio16 { name: gpio16 },

    /// GPIO 17 supports following functions:
    /// - `SPI0 CSn`
    /// - `UART0 RX`
    /// - `I2C0 SCL`
    /// - `PWM0 B`
    /// - `SIO`, `PIO0`, `PIO1`
    Gpio17 { name: gpio17 },

    /// GPIO 18 supports following functions:
    /// - `SPI0 SCK`
    /// - `UART0 CTS`
    /// - `I2C1 SDA`
    /// - `PWM1 A`
    /// - `SIO`, `PIO0`, `PIO1`
    Gpio18 { name: gpio18 },

    /// GPIO 19 supports following functions:
    /// - `SPI0 TX`
    /// - `UART0 RTS`
    /// - `I2C1 SCL`
    /// - `PWM1 B`
    /// - `SIO`, `PIO0`, `PIO1`
    Gpio19 { name: gpio19 },

    /// GPIO 20 supports following functions:
    /// - `SPI0 RX`
    /// - `UART1 TX`
    /// - `I2C0 SDA`
    /// - `PWM2 A`
    /// - `SIO`, `PIO0`, `PIO1`
    Gpio20 { name: gpio20 },

    /// GPIO 21 supports following functions:
    /// - `SPI0 CSn`
    /// - `UART1 RX`
    /// - `I2C0 SCL`
    /// - `PWM2 B`
    /// - `SIO`, `PIO0`, `PIO1`
    Gpio21 { name: gpio21 },

    /// GPIO 22 supports following functions:
    /// - `SPI0 SCK`
    /// - `UART1 CTS`
    /// - `I2C1 SDA`
    /// - `PWM3 A`
    /// - `SIO`, `PIO0`, `PIO1`
    Gpio22 { name: gpio22 },

    /// GPIO 23 is connected to b_power_save of the Raspberry Pi Pico board.
    Gpio23 { name: b_power_save },

    /// GPIO 24 is connected to vbus_detect of the Raspberry Pi Pico board.
    Gpio24 { name: vbus_detect },

    /// GPIO 25 is connected to the LED of the Raspberry Pi Pico board.
    Gpio25 { name: led },

    /// GPIO 26 supports following functions:
    /// - `SPI1 SCK`
    /// - `UART1 CTS`
    /// - `I2C1 SDA`
    /// - `PWM5 A`
    /// - `SIO`, `PIO0`, `PIO1`
    Gpio26 { name: gpio26 },

    /// GPIO 27 supports following functions:
    /// - `SPI1 TX`
    /// - `UART1 RTS`
    /// - `I2C1 SCL`
    /// - `PWM5 B`
    /// - `SIO`, `PIO0`, `PIO1`
    Gpio27 { name: gpio27 },

    /// GPIO 28 supports following functions:
    /// - `SPI1 RX`
    /// - `UART0 TX`
    /// - `I2C0 SDA`
    /// - `PWM6 A`
    /// - `SIO`, `PIO0`, `PIO1`
    Gpio28 { name: gpio28 },

    /// GPIO 29 is connected to voltage_monitor of the Raspberry Pi Pico board.
    Gpio29 { name: voltage_monitor },
);

pub const XOSC_CRYSTAL_FREQ: u32 = 12_000_000;
