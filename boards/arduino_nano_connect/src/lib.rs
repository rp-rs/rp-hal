#![no_std]

pub extern crate rp2040_hal as hal;

#[cfg(feature = "rt")]
pub use rp2040_hal::entry;

//// The linker will place this boot block at the start of our program image. We
//// need this to help the ROM bootloader get our code up and running.
#[cfg(feature = "boot2")]
#[link_section = ".boot2"]
#[no_mangle]
#[used]
pub static BOOT2_FIRMWARE: [u8; 256] = rp2040_boot2::BOOT_LOADER_AT25SF128A;

pub use hal::pac;

// borrowed some pin defs from rp-pico from a dicussion on the bsp_pins! macro
// stripped out functions from connected lines that are no available through
// any of the header pins
hal::bsp_pins!(
    /// GPIO 0 supports following functions:
    /// | Default      | UART0 TX (arduino nano connect)
    /// | Function     | Alias with applied function |
    /// |--------------|-----------------------------|
    /// | `SPI0 RX`    | [crate::Gp0Spi0Rx]          |
    /// | `UART0 TX`   | [crate::Gp0Uart0Tx]         |
    /// | `I2C0 SDA`   | [crate::Gp0I2C0Sda]         |
    /// | `PWM0 A`     | [crate::Gp0Pwm0A]           |
    /// | `PIO0`       | [crate::Gp0Pio0]            |
    /// | `PIO1`       | [crate::Gp0Pio1]            |
    Gpio0   {
        name: tx,
        aliases:
            {
                /// UART Function alias for pin [crate::Pins::gpio0].
                FunctionUart:   Gp0Uart0Tx
            }
    },

    /// GPIO 1 supports following functions:
    /// | Default      | UART0 RX (arduino nano connect)
    /// | Function     | Alias with applied function |
    /// |--------------|-----------------------------|
    /// | `SPI0 CSn`   | [crate::Gp1Spi0Csn]         |
    /// | `UART0 RX`   | [crate::Gp1Uart0Rx]         |
    /// | `I2C0 SCL`   | [crate::Gp1I2C0Scl]         |
    /// | `PWM0 B`     | [crate::Gp1Pwm0B]           |
    /// | `PIO0`       | [crate::Gp1Pio0]            |
    /// | `PIO1`       | [crate::Gp1Pio1]            |
    Gpio1   {
        name: rx,
        aliases: {
            /// UART Function alias for pin [crate::Pins::gpio1].
            FunctionUart:   Gp1Uart0Rx
        }
    },

    /// GPIO 2 supports following functions:
    /// | Default      | GPIO0 on nina, relates to BLE CTS
    /// | Function     | Alias with applied function |
    /// |--------------|-----------------------------|
    /// | `SPI0 SCK`   | [crate::Gp2Spi0Sck]         |
    /// | `UART0 CTS`  | [crate::Gp2Uart0Cts]        |
    /// | `I2C1 SDA`   | [crate::Gp2I2C1Sda]         |
    /// | `PWM1 A`     | [crate::Gp2Pwm1A]           |
    /// | `PIO0`       | [crate::Gp2Pio0]            |
    /// | `PIO1`       | [crate::Gp2Pio1]            |
    Gpio2   {
        // name indicates BLE CTS on nina module, so outgoing line is bleRts using UART
        name: ble_rts,
        aliases: {
            /// UART Function alias for pin [crate::Pins::gpio2].
            FunctionUart:   Gp2Uart0Cts
        }
    },

    /// GPIO 3 supports following functions:
    /// | Default      | Line for reset of Nina Wifi Module
    /// | Function     | Alias with applied function |
    /// |--------------|-----------------------------|
    /// | `SPI0 TX`    | [crate::Gp3Spi0Tx]          |
    /// | `UART0 RTS`  | [crate::Gp3Uart0Rts]        |
    /// | `I2C1 SCL`   | [crate::Gp3I2C1Scl]         |
    /// | `PWM1 B`     | [crate::Gp3Pwm1B]           |
    /// | `PIO0`       | [crate::Gp3Pio0]            |
    /// | `PIO1`       | [crate::Gp3Pio1]            |
    Gpio3   {
        name: nina_reset_n
    },

    /// GPIO 4 supports following functions:
    /// | Default      | SpiRx
    /// | Function     | Alias with applied function |
    /// |--------------|-----------------------------|
    /// | `SPI0 RX`    | [crate::Gp4Spi0Rx]          |
    /// | `UART1 TX`   | [crate::Gp4Uart1Tx]         |
    /// | `I2C0 SDA`   | [crate::Gp4I2C0Sda]         |
    /// | `PWM2 A`     | [crate::Gp4Pwm2A]           |
    /// | `PIO0`       | [crate::Gp4Pio0]            |
    /// | `PIO1`       | [crate::Gp4Pio1]            |
    Gpio4   {
        // Conflicting information between schematic and 'latest' pinnout diagram.
        // MISO in schematic, CIPO in 'latest' arduino pinnout diagram. same thing really
        // SPIRX in schematic line label.
        name: cipo,
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
    /// | Default      | Generic Digital pin D10
    /// | Function     | Alias with applied function |
    /// |--------------|-----------------------------|
    /// | `SPI0 CSn`   | [crate::Gp5Spi0Csn]         |
    /// | `UART1 RX`   | [crate::Gp5Uart1Rx]         |
    /// | `I2C0 SCL`   | [crate::Gp5I2C0Scl]         |
    /// | `PWM2 B`     | [crate::Gp5Pwm2B]           |
    /// | `PIO0`       | [crate::Gp5Pio0]            |
    /// | `PIO1`       | [crate::Gp5Pio1]            |
    Gpio5   {
        name: d10,
        aliases: {
            /// UART Function alias for pin [crate::Pins::gpio5].
            FunctionUart:   Gp5Uart1Rx,
            /// SPI Function alias for pin [crate::Pins::gpio5].
            FunctionSpi:    Gp5Spi0Csn,
            /// I2C Function alias for pin [crate::Pins::gpio5].
            FunctionI2C:    Gp5I2C0Scl,
            /// PWM Function alias for pin [crate::Pins::gpio5].
            FunctionPwm:    Gp5Pwm2B,
            /// PIO0 Function alias for pin [crate::Pins::gpio5].
            FunctionPio0:   Gp5Pio0,
            /// PIO1 Function alias for pin [crate::Pins::gpio5].
            FunctionPio1:   Gp5Pio1
        }
    },

    /// GPIO 6 supports following functions:
    /// | Default      | SPI0 SCK (tied to on board led)
    /// | Function     | Alias with applied function |
    /// |--------------|-----------------------------|
    /// | `SPI0 SCK`   | [crate::Gp6Spi0Sck]         |
    /// | `UART1 CTS`  | [crate::Gp6Uart1Cts]        |
    /// | `I2C1 SDA`   | [crate::Gp6I2C1Sda]         |
    /// | `PWM3 A`     | [crate::Gp6Pwm3A]           |
    /// | `PIO0`       | [crate::Gp6Pio0]            |
    /// | `PIO1`       | [crate::Gp6Pio1]            |
    Gpio6   {
        // also tied to on board led, on pin 13
        name: sck0,
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
    /// | Default      | SPI0 TX
    /// | Function     | Alias with applied function |
    /// |--------------|-----------------------------|
    /// | `SPI0 TX`    | [crate::Gp7Spi0Tx]          |
    /// | `UART1 RTS`  | [crate::Gp7Uart1Rts]        |
    /// | `I2C1 SCL`   | [crate::Gp7I2C1Scl]         |
    /// | `PWM3 B`     | [crate::Gp7Pwm3B]           |
    /// | `PIO0`       | [crate::Gp7Pio0]            |
    /// | `PIO1`       | [crate::Gp7Pio1]            |
    Gpio7   {
        // Conflicting information between schematic and 'latest' pinnout diagram.
        // MOSI in schematic, COPI in 'latest' arduino pinnout diagram.
        // SPITX in schematic line label.
        name: copi,
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

    /// GPIO 8 supports following functions:
    /// | Default      | SPI1 CIPO / UART1 TX connection to Nina Module
    /// | Function     | Alias with applied function |
    /// |--------------|-----------------------------|
    /// | `SPI1 RX`    | [crate::Gp8Spi1Rx]          |
    /// | `UART1 TX`   | [crate::Gp8Uart1Tx]         |
    /// | `I2C0 SDA`   | [crate::Gp8I2C0Sda]         |
    /// | `PWM4 A`     | [crate::Gp8Pwm4A]           |
    /// | `PIO0`       | [crate::Gp8Pio0]            |
    /// | `PIO1`       | [crate::Gp8Pio1]            |
    Gpio8 {
        // name indicates port to BLE RX on nina module and SPI CIPO for wifi
        name: ble_tx_cipo,
        aliases: {
            /// UART Function alias for pin [crate::Pins::gpio8].
            FunctionUart:   Gp8Uart1Tx,
            /// SPI Function alias for pin [crate::Pins::gpio8].
            FunctionSpi:    Gp8Spi1Rx
        }
    },

    /// GPIO 9 supports following functions:
    /// | Default      | SPI1 CS / UART1 RX
    /// | Function     | Alias with applied function |
    /// |--------------|-----------------------------|
    /// | `SPI1 CSn`   | [crate::Gp9Spi1Csn]         |
    /// | `UART1 RX`   | [crate::Gp9Uart1Rx]         |
    /// | `I2C0 SCL`   | [crate::Gp9I2C0Scl]         |
    /// | `PWM4 B`     | [crate::Gp9Pwm4B]           |
    /// | `PIO0`       | [crate::Gp9Pio0]            |
    /// | `PIO1`       | [crate::Gp9Pio1]            |
    Gpio9 {
        // name indicates BLE TX on nina module and spi1 cs signal to wifi
        name: ble_rx_cs,
        aliases: {
            /// UART Function alias for pin [crate::Pins::gpio9].
            FunctionUart:   Gp9Uart1Rx,
            /// SPI Function alias for pin [crate::Pins::gpio9].
            FunctionSpi:    Gp9Spi1Csn
        }
    },

    /// GPIO 10 supports following functions:
    /// | Default      | SPI1 ACK / UART1 CTS
    /// | Function     | Alias with applied function |
    /// |--------------|-----------------------------|
    /// | `SPI1 SCK`   | [crate::Gp10Spi1Sck]        |
    /// | `UART1 CTS`  | [crate::Gp10Uart1Cts]       |
    /// | `I2C1 SDA`   | [crate::Gp10I2C1Sda]        |
    /// | `PWM5 A`     | [crate::Gp10Pwm5A]          |
    /// | `PIO0`       | [crate::Gp10Pio0]           |
    /// | `PIO1`       | [crate::Gp10Pio1]           |
    Gpio10 {
        // name indicates BLE RTS on nina module and spi1 ack
        name: ble_cts_ack,
        aliases: {
            /// UART Function alias for pin [crate::Pins::gpio10].
            FunctionUart:   Gp10Uart1Cts,
            /// SPI Function alias for pin [crate::Pins::gpio10].
            FunctionSpi:    Gp10Spi1Sck
        }
    },

    /// GPIO 11 supports following functions:
    /// | Default      | SPI1 COPI / UART1 RTS
    /// | Function     | Alias with applied function |
    /// |--------------|-----------------------------|
    /// | `SPI1 TX`    | [crate::Gp11Spi1Tx]         |
    /// | `UART1 RTS`  | [crate::Gp11Uart1Rts]       |
    /// | `I2C1 SCL`   | [crate::Gp11I2C1Scl]        |
    /// | `PWM5 B`     | [crate::Gp11Pwm5B]          |
    /// | `PIO0`       | [crate::Gp11Pio0]           |
    /// | `PIO1`       | [crate::Gp11Pio1]           |
    Gpio11 {
        // ninaCOPI is
        name: nina_copi,
        aliases: {
            /// UART Function alias for pin [crate::Pins::gpio11].
            FunctionUart:   Gp11Uart1Rts,
            /// SPI Function alias for pin [crate::Pins::gpio11].
            FunctionSpi:    Gp11Spi1Tx
        }
    },

    /// GPIO 12 supports following functions:
    /// | Default      | I2C0 SDA / A4 ~ goes to pullups and auth
    /// | Function     | Alias with applied function |
    /// |--------------|-----------------------------|
    /// | `SPI1 RX`    | [crate::Gp12Spi1Rx]         |
    /// | `UART0 TX`   | [crate::Gp12Uart0Tx]        |
    /// | `I2C0 SDA`   | [crate::Gp12I2C0Sda]        |
    /// | `PWM6 A`     | [crate::Gp12Pwm6A]          |
    /// | `PIO0`       | [crate::Gp12Pio0]           |
    /// | `PIO1`       | [crate::Gp12Pio1]           |
    Gpio12  {
        // Also SDA Crypto
        name: a4,
        aliases: {
            /// UART Function alias for pin [crate::Pins::gpio12].
            FunctionUart: Gp12Uart0Tx,
            /// SPI Function alias for pin [crate::Pins::gpio12].
            FunctionSpi: Gp12Spi1Rx,
            /// I2C Function alias for pin [crate::Pins::gpio12].
            FunctionI2C: Gp12I2C0Sda,
            /// PWM Function alias for pin [crate::Pins::gpio12].
            FunctionPwm: Gp12Pwm6A,
            /// PIO0 Function alias for pin [crate::Pins::gpio12].
            FunctionPio0: Gp12Pio0,
            /// PIO1 Function alias for pin [crate::Pins::gpio12].
            FunctionPio1: Gp12Pio1
        }
    },

    /// GPIO 13 supports following functions:
    /// | Default      | I2C0 SCL / A5 ~ goes to pullups
    /// | Function     | Alias with applied function |
    /// |--------------|-----------------------------|
    /// | `SPI1 CSn`   | [crate::Gp13Spi1Csn]        |
    /// | `UART0 RX`   | [crate::Gp13Uart0Rx]        |
    /// | `I2C0 SCL`   | [crate::Gp13I2C0Scl]        |
    /// | `PWM6 B`     | [crate::Gp13Pwm6B]          |
    /// | `PIO0`       | [crate::Gp13Pio0]           |
    /// | `PIO1`       | [crate::Gp13Pio1]           |
    Gpio13  {
        // Also SCL Crypto
        name: a5,
        aliases: {
            /// UART Function alias for pin [crate::Pins::gpio13].
            FunctionUart: Gp13Uart0Rx,
            /// SPI Function alias for pin [crate::Pins::gpio13].
            FunctionSpi: Gp13Spi1Csn,
            /// I2C Function alias for pin [crate::Pins::gpio13].
            FunctionI2C: Gp13I2C0Scl,
            /// PWM Function alias for pin [crate::Pins::gpio13].
            FunctionPwm: Gp13Pwm6B,
            /// PIO0 Function alias for pin [crate::Pins::gpio13].
            FunctionPio0: Gp13Pio0,
            /// PIO1 Function alias for pin [crate::Pins::gpio13].
            FunctionPio1: Gp13Pio1
        }
    },

    /// GPIO 14 supports following functions:
    /// | Default      | SPI1 SCK ~ nina SCK
    /// | Function     | Alias with applied function |
    /// |--------------|-----------------------------|
    /// | `SPI1 SCK`   | [crate::Gp14Spi1Sck]        |
    /// | `UART0 CTS`  | [crate::Gp14Uart0Cts]       |
    /// | `I2C1 SDA`   | [crate::Gp14I2C1Sda]        |
    /// | `PWM7 A`     | [crate::Gp14Pwm7A]          |
    /// | `PIO0`       | [crate::Gp14Pio0]           |
    /// | `PIO1`       | [crate::Gp14Pio1]           |
    Gpio14 {
        name: nina_sck,
        aliases: {
            /// SPI Function alias for pin [crate::Pins::gpio14].
            FunctionSpi:    Gp14Spi1Sck
        }
    },

    /// GPIO 15 supports following functions:
    /// | Default      | General Digital pin D3
    /// | Function     | Alias with applied function |
    /// |--------------|-----------------------------|
    /// | `SPI1 TX`    | [crate::Gp15Spi1Tx]         |
    /// | `UART0 RTS`  | [crate::Gp15Uart0Rts]       |
    /// | `I2C1 SCL`   | [crate::Gp15I2C1Scl]        |
    /// | `PWM7 B`     | [crate::Gp15Pwm7B]          |
    /// | `PIO0`       | [crate::Gp15Pio0]           |
    /// | `PIO1`       | [crate::Gp15Pio1]           |
    Gpio15  {
        name: d3,
        aliases: {
            /// UART Function alias for pin [crate::Pins::gpio15].
            FunctionUart:   Gp15Uart0Rts,
            /// SPI Function alias for pin [crate::Pins::gpio15].
            FunctionSpi:    Gp15Spi1Tx,
            /// I2C Function alias for pin [crate::Pins::gpio15].
            FunctionI2C:    Gp15I2C1Scl,
            /// PWM Function alias for pin [crate::Pins::gpio15].
            FunctionPwm:    Gp15Pwm7B,
            /// PIO0 Function alias for pin [crate::Pins::gpio15].
            FunctionPio0:   Gp15Pio0,
            /// PIO1 Function alias for pin [crate::Pins::gpio15].
            FunctionPio1:   Gp15Pio1
        }
    },

    /// GPIO 16 supports following functions:
    /// | Default      | General Digital pin D4
    /// | Function     | Alias with applied function |
    /// |--------------|-----------------------------|
    /// | `SPI0 RX`    | [crate::Gp16Spi0Rx]         |
    /// | `UART0 TX`   | [crate::Gp16Uart0Tx]        |
    /// | `I2C0 SDA`   | [crate::Gp16I2C0Sda]        |
    /// | `PWM0 A`     | [crate::Gp16Pwm0A]          |
    /// | `PIO0`       | [crate::Gp16Pio0]           |
    /// | `PIO1`       | [crate::Gp16Pio1]           |
    Gpio16  {
        name: d4,
        aliases: {
            /// UART Function alias for pin [crate::Pins::gpio16].
            FunctionUart:   Gp16Uart0Tx,
            /// SPI Function alias for pin [crate::Pins::gpio16].
            FunctionSpi:    Gp16Spi0Rx,
            /// I2C Function alias for pin [crate::Pins::gpio16].
            FunctionI2C:    Gp16I2C0Sda,
            /// PWM Function alias for pin [crate::Pins::gpio16].
            FunctionPwm:    Gp16Pwm0A,
            /// PIO0 Function alias for pin [crate::Pins::gpio16].
            FunctionPio0:   Gp16Pio0,
            /// PIO1 Function alias for pin [crate::Pins::gpio16].
            FunctionPio1:   Gp16Pio1
        }
    },

    /// GPIO 17 supports following functions:
    /// | Default      | General Digital pin D5
    /// | Function     | Alias with applied function |
    /// |--------------|-----------------------------|
    /// | `SPI0 CSn`   | [crate::Gp17Spi0Csn]        |
    /// | `UART0 RX`   | [crate::Gp17Uart0Rx]        |
    /// | `I2C0 SCL`   | [crate::Gp17I2C0Scl]        |
    /// | `PWM0 B`     | [crate::Gp17Pwm0B]          |
    /// | `PIO0`       | [crate::Gp17Pio0]           |
    /// | `PIO1`       | [crate::Gp17Pio1]           |
    Gpio17  {
        name: d5,
        aliases: {
            /// UART Function alias for pin [crate::Pins::gpio17].
            FunctionUart:   Gp17Uart0Rx,
            /// SPI Function alias for pin [crate::Pins::gpio17].
            FunctionSpi:    Gp17Spi0Csn,
            /// I2C Function alias for pin [crate::Pins::gpio17].
            FunctionI2C:    Gp17I2C0Scl,
            /// PWM Function alias for pin [crate::Pins::gpio17].
            FunctionPwm:    Gp17Pwm0B,
            /// PIO0 Function alias for pin [crate::Pins::gpio17].
            FunctionPio0:   Gp17Pio0,
            /// PIO1 Function alias for pin [crate::Pins::gpio17].
            FunctionPio1:   Gp17Pio1
        }
    },

    /// GPIO 18 supports following functions:
    /// | Default      | General Digital pin D6
    /// | Function     | Alias with applied function |
    /// |--------------|-----------------------------|
    /// | `SPI0 SCK`   | [crate::Gp18Spi0Sck]        |
    /// | `UART0 CTS`  | [crate::Gp18Uart0Cts]       |
    /// | `I2C1 SDA`   | [crate::Gp18I2C1Sda]        |
    /// | `PWM1 A`     | [crate::Gp18Pwm1A]          |
    /// | `PIO0`       | [crate::Gp18Pio0]           |
    /// | `PIO1`       | [crate::Gp18Pio1]           |
    Gpio18  {
        name: d6,
        aliases: {
            /// UART Function alias for pin [crate::Pins::gpio18].
            FunctionUart:   Gp18Uart0Cts,
            /// SPI Function alias for pin [crate::Pins::gpio18].
            FunctionSpi:    Gp18Spi0Sck,
            /// I2C Function alias for pin [crate::Pins::gpio18].
            FunctionI2C:    Gp18I2C1Sda,
            /// PWM Function alias for pin [crate::Pins::gpio18].
            FunctionPwm:    Gp18Pwm1A,
            /// PIO0 Function alias for pin [crate::Pins::gpio18].
            FunctionPio0:   Gp18Pio0,
            /// PIO1 Function alias for pin [crate::Pins::gpio18].
            FunctionPio1:   Gp18Pio1
        }
    },

    /// GPIO 19 supports following functions:
    /// | Default      | General Digital Pin D7
    /// | Function     | Alias with applied function |
    /// |--------------|-----------------------------|
    /// | `SPI0 TX`    | [crate::Gp19Spi0Tx]         |
    /// | `UART0 RTS`  | [crate::Gp19Uart0Rts]       |
    /// | `I2C1 SCL`   | [crate::Gp19I2C1Scl]        |
    /// | `PWM1 B`     | [crate::Gp19Pwm1B]          |
    /// | `PIO0`       | [crate::Gp19Pio0]           |
    /// | `PIO1`       | [crate::Gp19Pio1]           |
    Gpio19  {
        name: d7,
        aliases: {
            /// UART Function alias for pin [crate::Pins::gpio19].
            FunctionUart:   Gp19Uart0Rts,
            /// SPI Function alias for pin [crate::Pins::gpio19].
            FunctionSpi:    Gp19Spi0Tx,
            /// I2C Function alias for pin [crate::Pins::gpio19].
            FunctionI2C:    Gp19I2C1Scl,
            /// PWM Function alias for pin [crate::Pins::gpio19].
            FunctionPwm:    Gp19Pwm1B,
            /// PIO0 Function alias for pin [crate::Pins::gpio19].
            FunctionPio0:   Gp19Pio0,
            /// PIO1 Function alias for pin [crate::Pins::gpio19].
            FunctionPio1:   Gp19Pio1
        }
    },

    /// GPIO 20 supports following functions:
    /// | Default      | General Digital pin D8
    /// | Function     | Alias with applied function |
    /// |--------------|-----------------------------|
    /// | `SPI0 RX`    | [crate::Gp20Spi0Rx]         |
    /// | `UART1 TX`   | [crate::Gp20Uart1Tx]        |
    /// | `I2C0 SDA`   | [crate::Gp20I2C0Sda]        |
    /// | `PWM2 A`     | [crate::Gp20Pwm2A]          |
    /// | `PIO0`       | [crate::Gp20Pio0]           |
    /// | `PIO1`       | [crate::Gp20Pio1]           |
    Gpio20  {
        name: d8,
        aliases: {
            /// UART Function alias for pin [crate::Pins::gpio20].
            FunctionUart:   Gp20Uart1Tx,
            /// SPI Function alias for pin [crate::Pins::gpio20].
            FunctionSpi:    Gp20Spi0Rx,
            /// I2C Function alias for pin [crate::Pins::gpio20].
            FunctionI2C:    Gp20I2C0Sda,
            /// PWM Function alias for pin [crate::Pins::gpio20].
            FunctionPwm:    Gp20Pwm2A,
            /// PIO0 Function alias for pin [crate::Pins::gpio20].
            FunctionPio0:   Gp20Pio0,
            /// PIO1 Function alias for pin [crate::Pins::gpio20].
            FunctionPio1:   Gp20Pio1
        }
    },

    /// GPIO 21 supports following functions:
    /// | Default      | General Digital pin D9
    /// | Function     | Alias with applied function |
    /// |--------------|-----------------------------|
    /// | `SPI0 CSn`   | [crate::Gp21Spi0Csn]        |
    /// | `UART1 RX`   | [crate::Gp21Uart1Rx]        |
    /// | `I2C0 SCL`   | [crate::Gp21I2C0Scl]        |
    /// | `PWM2 B`     | [crate::Gp21Pwm2B]          |
    /// | `PIO0`       | [crate::Gp21Pio0]           |
    /// | `PIO1`       | [crate::Gp21Pio1]           |
    Gpio21  {
        name: d9,
        aliases: {
            /// UART Function alias for pin [crate::Pins::gpio21].
            FunctionUart:   Gp21Uart1Rx,
            /// SPI Function alias for pin [crate::Pins::gpio21].
            FunctionSpi:    Gp21Spi0Csn,
            /// I2C Function alias for pin [crate::Pins::gpio21].
            FunctionI2C:    Gp21I2C0Scl,
            /// PWM Function alias for pin [crate::Pins::gpio21].
            FunctionPwm:    Gp21Pwm2B,
            /// PIO0 Function alias for pin [crate::Pins::gpio21].
            FunctionPio0:   Gp21Pio0,
            /// PIO1 Function alias for pin [crate::Pins::gpio21].
            FunctionPio1:   Gp21Pio1
        }
    },

    /// GPIO 22 supports following functions:
    /// | Default      | PDMDIN
    /// | Function     | Alias with applied function |
    /// |--------------|-----------------------------|
    /// | `SPI0 SCK`   | [crate::Gp22Spi0Sck]        |
    /// | `UART1 CTS`  | [crate::Gp22Uart1Cts]       |
    /// | `I2C1 SDA`   | [crate::Gp22I2C1Sda]        |
    /// | `PWM3 A`     | [crate::Gp22Pwm3A]          |
    /// | `PIO0`       | [crate::Gp22Pio0]           |
    /// | `PIO1`       | [crate::Gp22Pio1]           |
    Gpio22 {
        // this connects to the microphone module
        name: pdmdin
    },

    /// GPIO 23 supports following functions:
    /// | Default      | PDMCLK
    /// | Function     | Alias with applied function |
    /// |--------------|-----------------------------|
    /// | `SPI0 TX`   | [crate::Gp23Spi0Tx]        |
    /// | `UART1 RTS`  | [crate::Gp23Uart1Rts]       |
    /// | `I2C1 SCL`   | [crate::Gp23I2C1Scl]        |
    /// | `PWM3 B`     | [crate::Gp23Pwm3B]          |
    /// | `PIO0`       | [crate::Gp23Pio0]           |
    /// | `PIO1`       | [crate::Gp23Pio1]           |
    Gpio23 {
        name: pdmclk
    },

    /// GPIO 24 supports following functions:
    /// | Default      | INT1 ~ connected to INT1 on the IMU
    /// | Function     | Alias with applied function |
    /// |--------------|-----------------------------|
    /// | `SPI1 RX`   | [crate::Gp24Spi1Rx]        |
    /// | `UART1 TX`  | [crate::Gp24Uart1Tx]       |
    /// | `I2C0 SDA`   | [crate::Gp24I2C0Sda]        |
    /// | `PWM4 A`     | [crate::Gp24Pwm4A]          |
    /// | `PIO0`       | [crate::Gp24Pio0]           |
    /// | `PIO1`       | [crate::Gp24Pio1]           |
    Gpio24 {
        name: int1
    },

    /// GPIO 25 supports following functions:
    /// | Default      |
    /// | Function     | Alias with applied function |
    /// |--------------|-----------------------------|
    /// | `SPI1 CSn`   | [crate::Gp25Spi1Csn]        |
    /// | `UART1 RX`  | [crate::Gp25Uart1Rx]       |
    /// | `I2C0 SCL`   | [crate::Gp25I2C0Scl]        |
    /// | `PWM4 B`     | [crate::Gp25Pwm4B]          |
    /// | `PIO0`       | [crate::Gp25Pio0]           |
    /// | `PIO1`       | [crate::Gp25Pio1]           |
    Gpio25  {
        name: d2,
        aliases: {
            /// UART Function alias for pin [crate::Pins::gpio25].
            FunctionUart:   Gp25Uart1Rx,
            /// SPI Function alias for pin [crate::Pins::gpio25].
            FunctionSpi:    Gp25Spi1Csn,
            /// I2C Function alias for pin [crate::Pins::gpio25].
            FunctionI2C:    Gp25I2C0Scl,
            /// PWM Function alias for pin [crate::Pins::gpio25].
            FunctionPwm:    Gp25Pwm4B,
            /// PIO0 Function alias for pin [crate::Pins::gpio25].
            FunctionPio0:   Gp25Pio0,
            /// PIO1 Function alias for pin [crate::Pins::gpio25].
            FunctionPio1:   Gp25Pio1
        }
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
    Gpio26  {
        name: a0,
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
    Gpio27  {
        name: a1,
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
    Gpio28  {
        name: a2,
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
    /// | `SPI1 CS`    | [crate::Gp29Spi1CSn]         |
    /// | `UART0 RX`   | [crate::Gp29Uart0Rx]        |
    /// | `I2C0 SCL`   | [crate::Gp29I2C0Scl]        |
    /// | `PWM6 B`     | [crate::Gp29Pwm6B]          |
    /// | `PIO0`       | [crate::Gp29Pio0]           |
    /// | `PIO1`       | [crate::Gp29Pio1]           |
    Gpio29  {
        name: a3,
        aliases: {
            /// UART Function alias for pin [crate::Pins::gpio28].
            FunctionUart: Gp29Uart0Rx,
            /// SPI Function alias for pin [crate::Pins::gpio28].
            FunctionSpi: Gp29Spi1CSn,
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
