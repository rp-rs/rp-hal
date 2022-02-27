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
    ///
    /// | Function     | Alias with applied function |
    /// |--------------|-----------------------------|
    /// | `SPI0 RX`    | [crate::Gp0Spi0Rx]          |
    /// | `UART0 TX`   | [crate::Gp0Uart0Tx]         |
    /// | `I2C0 SDA`   | [crate::Gp0I2C0Sda]         |
    /// | `PWM0 A`     | [crate::Gp0Pwm0A]           |
    /// | `PIO0`       | [crate::Gp0Pio0]            |
    /// | `PIO1`       | [crate::Gp0Pio1]            |
    Gpio0 {
        name: gpio0,
        aliases: {
            /// UART Function alias for pin [crate::Pins::gpio0].
            FunctionUart: Gp0Uart0Tx,
            /// SPI Function alias for pin [crate::Pins::gpio0].
            FunctionSpi: Gp0Spi0Rx,
            /// I2C Function alias for pin [crate::Pins::gpio0].
            FunctionI2C: Gp0I2C0Sda,
            /// PWM Function alias for pin [crate::Pins::gpio0].
            FunctionPwm: Gp0Pwm0A,
            /// PIO0 Function alias for pin [crate::Pins::gpio0].
            FunctionPio0: Gp0Pio0,
            /// PIO1 Function alias for pin [crate::Pins::gpio0].
            FunctionPio1: Gp0Pio1
        }
    },

    /// GPIO 1 supports following functions:
    ///
    /// | Function     | Alias with applied function |
    /// |--------------|-----------------------------|
    /// | `SPI0 CSn`   | [crate::Gp1Spi0Csn]         |
    /// | `UART0 RX`   | [crate::Gp1Uart0Rx]         |
    /// | `I2C0 SCL`   | [crate::Gp1I2C0Scl]         |
    /// | `PWM0 B`     | [crate::Gp1Pwm0B]           |
    /// | `PIO0`       | [crate::Gp1Pio0]            |
    /// | `PIO1`       | [crate::Gp1Pio1]            |
    Gpio1 {
        name: gpio1,
        aliases: {
            /// UART Function alias for pin [crate::Pins::gpio1].
            FunctionUart: Gp1Uart0Rx,
            /// SPI Function alias for pin [crate::Pins::gpio1].
            FunctionSpi: Gp1Spi0Csn,
            /// I2C Function alias for pin [crate::Pins::gpio1].
            FunctionI2C: Gp1I2C0Scl,
            /// PWM Function alias for pin [crate::Pins::gpio1].
            FunctionPwm: Gp1Pwm0B,
            /// PIO0 Function alias for pin [crate::Pins::gpio1].
            FunctionPio0: Gp1Pio0,
            /// PIO1 Function alias for pin [crate::Pins::gpio1].
            FunctionPio1: Gp1Pio1
        }
    },

    /// GPIO 2 supports following functions:
    ///
    /// | Function     | Alias with applied function |
    /// |--------------|-----------------------------|
    /// | `SPI0 SCK`   | [crate::Gp2Spi0Sck]         |
    /// | `UART0 CTS`  | [crate::Gp2Uart0Cts]        |
    /// | `I2C1 SDA`   | [crate::Gp2I2C1Sda]         |
    /// | `PWM1 A`     | [crate::Gp2Pwm1A]           |
    /// | `PIO0`       | [crate::Gp2Pio0]            |
    /// | `PIO1`       | [crate::Gp2Pio1]            |
    Gpio2 {
        name: gpio2,
        aliases: {
            /// UART Function alias for pin [crate::Pins::gpio2].
            FunctionUart: Gp2Uart0Cts,
            /// SPI Function alias for pin [crate::Pins::gpio2].
            FunctionSpi: Gp2Spi0Sck,
            /// I2C Function alias for pin [crate::Pins::gpio2].
            FunctionI2C: Gp2I2C1Sda,
            /// PWM Function alias for pin [crate::Pins::gpio2].
            FunctionPwm: Gp2Pwm1A,
            /// PIO0 Function alias for pin [crate::Pins::gpio2].
            FunctionPio0: Gp2Pio0,
            /// PIO1 Function alias for pin [crate::Pins::gpio2].
            FunctionPio1: Gp2Pio1
        }
    },

    /// GPIO 3 supports following functions:
    ///
    /// | Function     | Alias with applied function |
    /// |--------------|-----------------------------|
    /// | `SPI0 TX`    | [crate::Gp3Spi0Tx]          |
    /// | `UART0 RTS`  | [crate::Gp3Uart0Rts]        |
    /// | `I2C1 SCL`   | [crate::Gp3I2C1Scl]         |
    /// | `PWM1 B`     | [crate::Gp3Pwm1B]           |
    /// | `PIO0`       | [crate::Gp3Pio0]            |
    /// | `PIO1`       | [crate::Gp3Pio1]            |
    Gpio3 {
        name: gpio3,
        aliases: {
            /// UART Function alias for pin [crate::Pins::gpio3].
            FunctionUart: Gp3Uart0Rts,
            /// SPI Function alias for pin [crate::Pins::gpio3].
            FunctionSpi: Gp3Spi0Tx,
            /// I2C Function alias for pin [crate::Pins::gpio3].
            FunctionI2C: Gp3I2C1Scl,
            /// PWM Function alias for pin [crate::Pins::gpio3].
            FunctionPwm: Gp3Pwm1B,
            /// PIO0 Function alias for pin [crate::Pins::gpio3].
            FunctionPio0: Gp3Pio0,
            /// PIO1 Function alias for pin [crate::Pins::gpio3].
            FunctionPio1: Gp3Pio1
        }
    },

    /// GPIO 4 supports following functions:
    ///
    /// | Function     | Alias with applied function |
    /// |--------------|-----------------------------|
    /// | `SPI0 RX`    | [crate::Gp4Spi0Rx]          |
    /// | `UART1 TX`   | [crate::Gp4Uart1Tx]         |
    /// | `I2C0 SDA`   | [crate::Gp4I2C0Sda]         |
    /// | `PWM2 A`     | [crate::Gp4Pwm2A]           |
    /// | `PIO0`       | [crate::Gp4Pio0]            |
    /// | `PIO1`       | [crate::Gp4Pio1]            |
    Gpio4 {
        name: gpio4,
        aliases: {
            /// UART Function alias for pin [crate::Pins::gpio4].
            FunctionUart: Gp4Uart1Tx,
            /// SPI Function alias for pin [crate::Pins::gpio4].
            FunctionSpi: Gp4Spi0Rx,
            /// I2C Function alias for pin [crate::Pins::gpio4].
            FunctionI2C: Gp4I2C0Sda,
            /// PWM Function alias for pin [crate::Pins::gpio4].
            FunctionPwm: Gp4Pwm2A,
            /// PIO0 Function alias for pin [crate::Pins::gpio4].
            FunctionPio0: Gp4Pio0,
            /// PIO1 Function alias for pin [crate::Pins::gpio4].
            FunctionPio1: Gp4Pio1
        }
    },

    /// GPIO 5 supports following functions:
    ///
    /// | Function     | Alias with applied function |
    /// |--------------|-----------------------------|
    /// | `SPI0 CSn`   | [crate::Gp5Spi0Csn]         |
    /// | `UART1 RX`   | [crate::Gp5Uart1Rx]         |
    /// | `I2C0 SCL`   | [crate::Gp5I2C0Scl]         |
    /// | `PWM2 B`     | [crate::Gp5Pwm2B]           |
    /// | `PIO0`       | [crate::Gp5Pio0]            |
    /// | `PIO1`       | [crate::Gp5Pio1]            |
    Gpio5 {
        name: gpio5,
        aliases: {
            /// UART Function alias for pin [crate::Pins::gpio5].
            FunctionUart: Gp5Uart1Rx,
            /// SPI Function alias for pin [crate::Pins::gpio5].
            FunctionSpi: Gp5Spi0Csn,
            /// I2C Function alias for pin [crate::Pins::gpio5].
            FunctionI2C: Gp5I2C0Scl,
            /// PWM Function alias for pin [crate::Pins::gpio5].
            FunctionPwm: Gp5Pwm2B,
            /// PIO0 Function alias for pin [crate::Pins::gpio5].
            FunctionPio0: Gp5Pio0,
            /// PIO1 Function alias for pin [crate::Pins::gpio5].
            FunctionPio1: Gp5Pio1
        }
    },

    /// GPIO 6 supports following functions:
    ///
    /// | Function     | Alias with applied function |
    /// |--------------|-----------------------------|
    /// | `SPI0 SCK`   | [crate::Gp6Spi0Sck]         |
    /// | `UART1 CTS`  | [crate::Gp6Uart1Cts]        |
    /// | `I2C1 SDA`   | [crate::Gp6I2C1Sda]         |
    /// | `PWM3 A`     | [crate::Gp6Pwm3A]           |
    /// | `PIO0`       | [crate::Gp6Pio0]            |
    /// | `PIO1`       | [crate::Gp6Pio1]            |
    Gpio6 {
        name: gpio6,
        aliases: {
            /// UART Function alias for pin [crate::Pins::gpio6].
            FunctionUart: Gp6Uart1Cts,
            /// SPI Function alias for pin [crate::Pins::gpio6].
            FunctionSpi: Gp6Spi0Sck,
            /// I2C Function alias for pin [crate::Pins::gpio6].
            FunctionI2C: Gp6I2C1Sda,
            /// PWM Function alias for pin [crate::Pins::gpio6].
            FunctionPwm: Gp6Pwm3A,
            /// PIO0 Function alias for pin [crate::Pins::gpio6].
            FunctionPio0: Gp6Pio0,
            /// PIO1 Function alias for pin [crate::Pins::gpio6].
            FunctionPio1: Gp6Pio1
        }
    },

    /// GPIO 7 supports following functions:
    ///
    /// | Function     | Alias with applied function |
    /// |--------------|-----------------------------|
    /// | `SPI0 TX`    | [crate::Gp7Spi0Tx]          |
    /// | `UART1 RTS`  | [crate::Gp7Uart1Rts]        |
    /// | `I2C1 SCL`   | [crate::Gp7I2C1Scl]         |
    /// | `PWM3 B`     | [crate::Gp7Pwm3B]           |
    /// | `PIO0`       | [crate::Gp7Pio0]            |
    /// | `PIO1`       | [crate::Gp7Pio1]            |
    Gpio7 {
        name: gpio7,
        aliases: {
            /// UART Function alias for pin [crate::Pins::gpio7].
            FunctionUart: Gp7Uart1Rts,
            /// SPI Function alias for pin [crate::Pins::gpio7].
            FunctionSpi: Gp7Spi0Tx,
            /// I2C Function alias for pin [crate::Pins::gpio7].
            FunctionI2C: Gp7I2C1Scl,
            /// PWM Function alias for pin [crate::Pins::gpio7].
            FunctionPwm: Gp7Pwm3B,
            /// PIO0 Function alias for pin [crate::Pins::gpio7].
            FunctionPio0: Gp7Pio0,
            /// PIO1 Function alias for pin [crate::Pins::gpio7].
            FunctionPio1: Gp7Pio1
        }
    },

    /// GPIO 18 is red LED, active low
    Gpio18 {
        name: led_red
    },

    /// GPIO 19 is green LED, active low
    Gpio19 {
        name: led_green
    },

    /// GPIO 20 is blue LED, active low
    Gpio20 {
        name: led_blue
    },

    /// GPIO 23 is connected to bootsel button, active low
    Gpio23 {
        name: bootsel
    },

    /// GPIO 26 supports following functions:
    ///
    /// | Function     | Alias with applied function |
    /// |--------------|-----------------------------|
    /// | `SPI1 SCK`   | [crate::Gp26Spi1Sck]        |
    /// | `UART1 CTS`  | [crate::Gp26Uart1Cts]       |
    /// | `I2C1 SDA`   | [crate::Gp26I2C1Sda]        |
    /// | `PWM5 A`     | [crate::Gp26Pwm5A]          |
    /// | `PIO0`       | [crate::Gp26Pio0]           |
    /// | `PIO1`       | [crate::Gp26Pio1]           |
    Gpio26 {
        name: gpio26,
        aliases: {
            /// UART Function alias for pin [crate::Pins::gpio26].
            FunctionUart: Gp26Uart1Cts,
            /// SPI Function alias for pin [crate::Pins::gpio26].
            FunctionSpi: Gp26Spi1Sck,
            /// I2C Function alias for pin [crate::Pins::gpio26].
            FunctionI2C: Gp26I2C1Sda,
            /// PWM Function alias for pin [crate::Pins::gpio26].
            FunctionPwm: Gp26Pwm5A,
            /// PIO0 Function alias for pin [crate::Pins::gpio26].
            FunctionPio0: Gp26Pio0,
            /// PIO1 Function alias for pin [crate::Pins::gpio26].
            FunctionPio1: Gp26Pio1
        }
    },

    /// GPIO 27 supports following functions:
    ///
    /// | Function     | Alias with applied function |
    /// |--------------|-----------------------------|
    /// | `SPI1 TX`    | [crate::Gp27Spi1Tx]         |
    /// | `UART1 RTS`  | [crate::Gp27Uart1Rts]       |
    /// | `I2C1 SCL`   | [crate::Gp27I2C1Scl]        |
    /// | `PWM5 B`     | [crate::Gp27Pwm5B]          |
    /// | `PIO0`       | [crate::Gp27Pio0]           |
    /// | `PIO1`       | [crate::Gp27Pio1]           |
    Gpio27 {
        name: gpio27,
        aliases: {
            /// UART Function alias for pin [crate::Pins::gpio27].
            FunctionUart: Gp27Uart1Rts,
            /// SPI Function alias for pin [crate::Pins::gpio27].
            FunctionSpi: Gp27Spi1Tx,
            /// I2C Function alias for pin [crate::Pins::gpio27].
            FunctionI2C: Gp27I2C1Scl,
            /// PWM Function alias for pin [crate::Pins::gpio27].
            FunctionPwm: Gp27Pwm5B,
            /// PIO0 Function alias for pin [crate::Pins::gpio27].
            FunctionPio0: Gp27Pio0,
            /// PIO1 Function alias for pin [crate::Pins::gpio27].
            FunctionPio1: Gp27Pio1
        }
    },

    /// GPIO 28 supports following functions:
    ///
    /// | Function     | Alias with applied function |
    /// |--------------|-----------------------------|
    /// | `SPI1 RX`    | [crate::Gp28Spi1Rx]         |
    /// | `UART0 TX`   | [crate::Gp28Uart0Tx]        |
    /// | `I2C0 SDA`   | [crate::Gp28I2C0Sda]        |
    /// | `PWM6 A`     | [crate::Gp28Pwm6A]          |
    /// | `PIO0`       | [crate::Gp28Pio0]           |
    /// | `PIO1`       | [crate::Gp28Pio1]           |
    Gpio28 {
        name: gpio28,
        aliases: {
            /// UART Function alias for pin [crate::Pins::gpio28].
            FunctionUart: Gp28Uart0Tx,
            /// SPI Function alias for pin [crate::Pins::gpio28].
            FunctionSpi: Gp28Spi1Rx,
            /// I2C Function alias for pin [crate::Pins::gpio28].
            FunctionI2C: Gp28I2C0Sda,
            /// PWM Function alias for pin [crate::Pins::gpio28].
            FunctionPwm: Gp28Pwm6A,
            /// PIO0 Function alias for pin [crate::Pins::gpio28].
            FunctionPio0: Gp28Pio0,
            /// PIO1 Function alias for pin [crate::Pins::gpio28].
            FunctionPio1: Gp28Pio1
        }
    },

    /// GPIO 29 supports following functions:
    ///
    /// | Function     | Alias with applied function |
    /// |--------------|-----------------------------|
    /// | `SPI1 CSn`   | [crate::Gp29Spi1Csn]        |
    /// | `UART0 RX`   | [crate::Gp29Uart0Rx]        |
    /// | `I2C0 SCL`   | [crate::Gp29I2C0Scl]        |
    /// | `PWM6 B`     | [crate::Gp29Pwm6B]          |
    /// | `PIO0`       | [crate::Gp29Pio0]           |
    /// | `PIO1`       | [crate::Gp29Pio1]           |
    Gpio29 {
        name: gpio29,
        aliases: {
            /// UART Function alias for pin [crate::Pins::gpio28].
            FunctionUart: Gp29Uart0Rx,
            /// SPI Function alias for pin [crate::Pins::gpio28].
            FunctionSpi: Gp29Spi1Csn,
            /// I2C Function alias for pin [crate::Pins::gpio28].
            FunctionI2C: Gp29I2C0Scl,
            /// PWM Function alias for pin [crate::Pins::gpio28].
            FunctionPwm: Gp29Pwm6B,
            /// PIO0 Function alias for pin [crate::Pins::gpio28].
            FunctionPio0: Gp29Pio0,
            /// PIO1 Function alias for pin [crate::Pins::gpio28].
            FunctionPio1: Gp29Pio1
        }
    },
);

pub const XOSC_CRYSTAL_FREQ: u32 = 12_000_000;
