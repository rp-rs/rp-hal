#![no_std]

pub extern crate rp2040_hal as hal;

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

pub const NUM_SERVOS: u8 = 18;
pub const NUM_SENSORS: u8 = 6;
pub const NUM_LEDS: u8 = 6;

pub const SENSOR_1_ADDR: u8 = 0b_0000;
pub const SENSOR_2_ADDR: u8 = 0b_0001;
pub const SENSOR_3_ADDR: u8 = 0b_0010;
pub const SENSOR_4_ADDR: u8 = 0b_0011;
pub const SENSOR_5_ADDR: u8 = 0b_0100;
pub const SENSOR_6_ADDR: u8 = 0b_0101;
pub const VOLTAGE_SENSE_ADDR: u8 = 0b_0110;
pub const CURRENT_SENSE_ADDR: u8 = 0b_0111;

pub const VOLTAGE_GAIN: f32 = 0.28058;
pub const SHUNT_RESISTOR: f32 = 0.003;
pub const CURRENT_GAIN: u8 = 69;
pub const CURRENT_OFFSET: f32 = -0.02;

pub use hal::pac;

hal::bsp_pins!(
    /// GPIO 0 supports following functions:
    ///
    /// | Function     | Alias with applied function |
    /// |--------------|-----------------------------|
    /// | `PWM0 A`     | [crate::Servo1Pwm0A]           |
    /// | `PIO0`       | [crate::Servo1Pio0]            |
    /// | `PIO1`       | [crate::Servo1Pio1]            |
    Gpio0 {
        name: servo1,
        aliases: {
            /// PWM Function alias for pin [crate::Pins::servo1].
            FunctionPwm: Servo1Pwm0A,
            /// PIO0 Function alias for pin [crate::Pins::servo1].
            FunctionPio0: Servo1Pio0,
            /// PIO1 Function alias for pin [crate::Pins::servo1].
            FunctionPio1: Servo1Pio1
        }
    },

    /// GPIO 1 supports following functions:
    ///
    /// | Function     | Alias with applied function |
    /// |--------------|-----------------------------|
    /// | `PWM0 B`     | [crate::Servo2Pwm0B]           |
    /// | `PIO0`       | [crate::Servo2Pio0]            |
    /// | `PIO1`       | [crate::Servo2Pio1]            |
    Gpio1 {
        name: servo2,
        aliases: {
            /// PWM Function alias for pin [crate::Pins::servo2].
            FunctionPwm: Servo2Pwm0B,
            /// PIO0 Function alias for pin [crate::Pins::servo2].
            FunctionPio0: Servo2Pio0,
            /// PIO1 Function alias for pin [crate::Pins::servo2].
            FunctionPio1: Servo2Pio1
        }
    },

    /// GPIO 2 supports following functions:
    ///
    /// | Function     | Alias with applied function |
    /// |--------------|-----------------------------|
    /// | `PWM1 A`     | [crate::Servo3Pwm1A]           |
    /// | `PIO0`       | [crate::Servo3Pio0]            |
    /// | `PIO1`       | [crate::Servo3Pio1]            |
    Gpio2 {
        name: servo3,
        aliases: {
            /// PWM Function alias for pin [crate::Pins::servo3].
            FunctionPwm: Servo3Pwm1A,
            /// PIO0 Function alias for pin [crate::Pins::servo3].
            FunctionPio0: Servo3Pio0,
            /// PIO1 Function alias for pin [crate::Pins::servo3].
            FunctionPio1: Servo3Pio1
        }
    },

    /// GPIO 3 supports following functions:
    ///
    /// | Function     | Alias with applied function |
    /// |--------------|-----------------------------|
    /// | `PWM1 B`     | [crate::Servo4Pwm1B]           |
    /// | `PIO0`       | [crate::Servo4Pio0]            |
    /// | `PIO1`       | [crate::Servo4Pio1]            |
    Gpio3 {
        name: servo4,
        aliases: {
            /// PWM Function alias for pin [crate::Pins::servo4].
            FunctionPwm: Servo4Pwm1B,
            /// PIO0 Function alias for pin [crate::Pins::servo4].
            FunctionPio0: Servo4Pio0,
            /// PIO1 Function alias for pin [crate::Pins::servo4].
            FunctionPio1: Servo4Pio1
        }
    },

    /// GPIO 4 supports following functions:
    ///
    /// | Function     | Alias with applied function |
    /// |--------------|-----------------------------|
    /// | `PWM2 A`     | [crate::Servo5Pwm2A]           |
    /// | `PIO0`       | [crate::Servo5Pio0]            |
    /// | `PIO1`       | [crate::Servo5Pio1]            |
    Gpio4 {
        name: servo5,
        aliases: {
            /// PWM Function alias for pin [crate::Pins::servo5].
            FunctionPwm: Servo5Pwm2A,
            /// PIO0 Function alias for pin [crate::Pins::servo5].
            FunctionPio0: Servo5Pio0,
            /// PIO1 Function alias for pin [crate::Pins::servo5].
            FunctionPio1: Servo5Pio1
        }
    },

    /// GPIO 5 supports following functions:
    ///
    /// | Function     | Alias with applied function |
    /// |--------------|-----------------------------|
    /// | `PWM2 B`     | [crate::Servo6Pwm2B]           |
    /// | `PIO0`       | [crate::Servo6Pio0]            |
    /// | `PIO1`       | [crate::Servo6Pio1]            |
    Gpio5 {
        name: servo6,
        aliases: {
            /// PWM Function alias for pin [crate::Pins::servo6].
            FunctionPwm: Servo6Pwm2B,
            /// PIO0 Function alias for pin [crate::Pins::servo6].
            FunctionPio0: Servo6Pio0,
            /// PIO1 Function alias for pin [crate::Pins::servo6].
            FunctionPio1: Servo6Pio1
        }
    },

    /// GPIO 6 supports following functions:
    ///
    /// | Function     | Alias with applied function |
    /// |--------------|-----------------------------|
    /// | `PWM3 A`     | [crate::Servo7Pwm3A]           |
    /// | `PIO0`       | [crate::Servo7Pio0]            |
    /// | `PIO1`      | [crate::Servo7Pio1]            |
    Gpio6 {
        name: servo7,
        aliases: {
            /// PWM Function alias for pin [crate::Pins::servo7].
            FunctionPwm: Servo7Pwm3A,
            /// PIO0 Function alias for pin [crate::Pins::servo7].
            FunctionPio0: Servo7Pio0,
            /// PIO1 Function alias for pin [crate::Pins::servo7].
            FunctionPio1: Servo7Pio1
        }
    },

    /// GPIO 7 supports following functions:
    ///
    /// | Function     | Alias with applied function |
    /// |--------------|-----------------------------|
    /// | `PWM3 B`     | [crate::Servo8Pwm3B]           |
    /// | `PIO0`       | [crate::Servo8Pio0]            |
    /// | `PIO1`       | [crate::Servo8Pio1]            |
    Gpio7 {
        name: servo8,
        aliases: {
            /// PWM Function alias for pin [crate::Pins::servo8].
            FunctionPwm: Servo8Pwm3B,
            /// PIO0 Function alias for pin [crate::Pins::servo8].
            FunctionPio0: Servo8Pio0,
            /// PIO1 Function alias for pin [crate::Pins::servo8].
            FunctionPio1: Servo8Pio1
        }
    },

    /// GPIO 8 supports following functions:
    ///
    /// | Function     | Alias with applied function |
    /// |--------------|-----------------------------|
    /// | `PWM4 A`     | [crate::Servo9Pwm4A]           |
    /// | `PIO0`       | [crate::Servo9Pio0]            |
    /// | `PIO1`       | [crate::Servo9Pio1]            |
    Gpio8 {
        name: servo9,
        aliases: {
            /// PWM Function alias for pin [crate::Pins::servo9].
            FunctionPwm: Servo9Pwm4A,
            /// PIO0 Function alias for pin [crate::Pins::servo9].
            FunctionPio0: Servo9Pio0,
            /// PIO1 Function alias for pin [crate::Pins::servo9].
            FunctionPio1: Servo9Pio1
        }
    },

    /// GPIO 9 supports following functions:
    ///
    /// | Function     | Alias with applied function |
    /// |--------------|-----------------------------|
    /// | `PWM4 B`     | [crate::Servo10Pwm4B]           |
    /// | `PIO0`       | [crate::Servo10Pio0]            |
    /// | `PIO1`       | [crate::Servo10Pio1]            |
    Gpio9 {
        name: servo10,
        aliases: {
            /// PWM Function alias for pin [crate::Pins::servo10].
            FunctionPwm: Servo10Pwm4B,
            /// PIO0 Function alias for pin [crate::Pins::servo10].
            FunctionPio0: Servo10Pio0,
            /// PIO1 Function alias for pin [crate::Pins::servo10].
            FunctionPio1: Servo10Pio1
        }
    },

    /// GPIO 10 supports following functions:
    ///
    /// | Function     | Alias with applied function |
    /// |--------------|-----------------------------|
    /// | `PWM5 A`     | [crate::Servo11Pwm5A]          |
    /// | `PIO0`       | [crate::Servo11Pio0]           |
    /// | `PIO1`       | [crate::Servo11Pio1]           |
    Gpio10 {
        name: servo11,
        aliases: {
            /// PWM Function alias for pin [crate::Pins::servo11].
            FunctionPwm: Servo11Pwm5A,
            /// PIO0 Function alias for pin [crate::Pins::servo11].
            FunctionPio0: Servo11Pio0,
            /// PIO1 Function alias for pin [crate::Pins::servo11].
            FunctionPio1: Servo11Pio1
        }
    },

    /// GPIO 11 supports following functions:
    ///
    /// | Function     | Alias with applied function |
    /// |--------------|-----------------------------|
    /// | `PWM5 B`     | [crate::Servo12Pwm5B]          |
    /// | `PIO0`       | [crate::Servo12Pio0]           |
    /// | `PIO1`       | [crate::Servo12Pio1]           |
    Gpio11 {
        name: servo12,
        aliases: {
            /// PWM Function alias for pin [crate::Pins::servo12].
            FunctionPwm: Servo12Pwm5B,
            /// PIO0 Function alias for pin [crate::Pins::servo12].
            FunctionPio0: Servo12Pio0,
            /// PIO1 Function alias for pin [crate::Pins::servo12].
            FunctionPio1: Servo12Pio1
        }
    },

    /// GPIO 12 supports following functions:
    ///
    /// | Function     | Alias with applied function |
    /// |--------------|-----------------------------|
    /// | `PWM6 A`     | [crate::Servo13Pwm6A]          |
    /// | `PIO0`       | [crate::Servo13Pio0]           |
    /// | `PIO1`       | [crate::Servo13Pio1]           |
    Gpio12 {
        name: servo13,
        aliases: {
            /// PWM Function alias for pin [crate::Pins::servo13].
            FunctionPwm: Servo13Pwm6A,
            /// PIO0 Function alias for pin [crate::Pins::servo13].
            FunctionPio0: Servo13Pio0,
            /// PIO1 Function alias for pin [crate::Pins::servo13].
            FunctionPio1: Servo13Pio1
        }
    },

    /// GPIO 13 supports following functions:
    ///
    /// | Function     | Alias with applied function |
    /// |--------------|-----------------------------|
    /// | `PWM6 B`     | [crate::Servo14Pwm6B]          |
    /// | `PIO0`       | [crate::Servo14Pio0]           |
    /// | `PIO1`       | [crate::Servo14Pio1]           |
    Gpio13 {
        name: servo14,
        aliases: {
            /// PWM Function alias for pin [crate::Pins::servo14].
            FunctionPwm: Servo14Pwm6B,
            /// PIO0 Function alias for pin [crate::Pins::servo14].
            FunctionPio0: Servo14Pio0,
            /// PIO1 Function alias for pin [crate::Pins::servo14].
            FunctionPio1: Servo14Pio1
        }
    },

    /// GPIO 14 supports following functions:
    ///
    /// | Function     | Alias with applied function |
    /// |--------------|-----------------------------|
    /// | `PWM7 A`     | [crate::Servo15Pwm7A]          |
    /// | `PIO0`       | [crate::Servo15Pio0]           |
    /// | `PIO1`       | [crate::Servo15Pio1]           |
    Gpio14 {
        name: servo15,
        aliases: {
            /// PWM Function alias for pin [crate::Pins::servo15].
            FunctionPwm: Servo15Pwm7A,
            /// PIO0 Function alias for pin [crate::Pins::servo15].
            FunctionPio0: Servo15Pio0,
            /// PIO1 Function alias for pin [crate::Pins::servo15].
            FunctionPio1: Servo15Pio1
        }
    },

    /// GPIO 15 supports following functions:
    ///
    /// | Function     | Alias with applied function |
    /// |--------------|-----------------------------|
    /// | `PWM7 B`     | [crate::Servo16Pwm7B]          |
    /// | `PIO0`       | [crate::Servo16Pio0]           |
    /// | `PIO1`       | [crate::Servo16Pio1]           |
    Gpio15 {
        name: servo16,
        aliases: {
            /// PWM Function alias for pin [crate::Pins::servo16].
            FunctionPwm: Servo16Pwm7B,
            /// PIO0 Function alias for pin [crate::Pins::servo16].
            FunctionPio0: Servo16Pio0,
            /// PIO1 Function alias for pin [crate::Pins::servo16].
            FunctionPio1: Servo16Pio1
        }
    },

    /// GPIO 16 supports following functions:
    ///
    /// | Function     | Alias with applied function |
    /// |--------------|-----------------------------|
    /// | `PWM0 A`     | [crate::Servo17Pwm0A]          |
    /// | `PIO0`       | [crate::Servo17Pio0]           |
    /// | `PIO1`       | [crate::Servo17Pio1]           |
    Gpio16 {
        name: servo17,
        aliases: {
            /// PWM Function alias for pin [crate::Pins::servo17].
            FunctionPwm: Servo17Pwm0A,
            /// PIO0 Function alias for pin [crate::Pins::servo17].
            FunctionPio0: Servo17Pio0,
            /// PIO1 Function alias for pin [crate::Pins::servo17].
            FunctionPio1: Servo17Pio1
        }
    },

    /// GPIO 17 supports following functions:
    ///
    /// | Function     | Alias with applied function |
    /// |--------------|-----------------------------|
    /// | `PWM0 B`     | [crate::Servo18Pwm0B]          |
    /// | `PIO0`       | [crate::Servo18Pio0]           |
    /// | `PIO1`       | [crate::Servo18Pio1]           |
    Gpio17 {
        name: servo18,
        aliases: {
            /// PWM Function alias for pin [crate::Pins::servo18].
            FunctionPwm: Servo18Pwm0B,
            /// PIO0 Function alias for pin [crate::Pins::servo18].
            FunctionPio0: Servo18Pio0,
            /// PIO1 Function alias for pin [crate::Pins::servo18].
            FunctionPio1: Servo18Pio1
        }
    },

    /// GPIO 18 is connected to the leds of the Servo 2040 board.
    Gpio18 {
        name: led_data
    },

    /// GPIO 19 supports following functions:
    ///
    /// | Function     | Alias with applied function |
    /// |--------------|-----------------------------|
    /// | `SPI0 TX`    | [crate::Gp19Spi0Tx]         |
    /// | `UART0 RTS`  | [crate::Gp19Uart0Rts]       |
    /// | `I2C1 SCL`   | [crate::Gp19I2C1Scl]        |
    /// | `PWM1 B`     | [crate::Gp19Pwm1B]          |
    /// | `PIO0`       | [crate::Gp19Pio0]           |
    /// | `PIO1`       | [crate::Gp19Pio1]           |
    Gpio19 {
        name: int_,
        aliases: {
            /// UART Function alias for pin [crate::Pins::int_].
            FunctionUart: Gp19Uart0Rts,
            /// SPI Function alias for pin [crate::Pins::int_].
            FunctionSpi: Gp19Spi0Tx,
            /// I2C Function alias for pin [crate::Pins::int_].
            FunctionI2C: Gp19I2C1Scl,
            /// PWM Function alias for pin [crate::Pins::int_].
            FunctionPwm: Gp19Pwm1B,
            /// PIO0 Function alias for pin [crate::Pins::int_].
            FunctionPio0: Gp19Pio0,
            /// PIO1 Function alias for pin [crate::Pins::int_].
            FunctionPio1: Gp19Pio1
        }
    },

    /// GPIO 20 supports following functions:
    ///
    /// | Function     | Alias with applied function |
    /// |--------------|-----------------------------|
    /// | `SPI0 RX`    | [crate::Gp20Spi0Rx]         |
    /// | `UART1 TX`   | [crate::Gp20Uart1Tx]        |
    /// | `I2C0 SDA`   | [crate::Gp20I2C0Sda]        |
    /// | `PWM2 A`     | [crate::Gp20Pwm2A]          |
    /// | `PIO0`       | [crate::Gp20Pio0]           |
    /// | `PIO1`       | [crate::Gp20Pio1]           |
    Gpio20 {
        name: sda,
        aliases: {
            /// UART Function alias for pin [crate::Pins::sda].
            FunctionUart: Gp20Uart1Tx,
            /// SPI Function alias for pin [crate::Pins::sda].
            FunctionSpi: Gp20Spi0Rx,
            /// I2C Function alias for pin [crate::Pins::sda].
            FunctionI2C: Gp20I2C0Sda,
            /// PWM Function alias for pin [crate::Pins::sda].
            FunctionPwm: Gp20Pwm2A,
            /// PIO0 Function alias for pin [crate::Pins::sda].
            FunctionPio0: Gp20Pio0,
            /// PIO1 Function alias for pin [crate::Pins::sda].
            FunctionPio1: Gp20Pio1
        }
    },

    /// GPIO 21 supports following functions:
    ///
    /// | Function     | Alias with applied function |
    /// |--------------|-----------------------------|
    /// | `SPI0 CSn`   | [crate::Gp21Spi0Csn]        |
    /// | `UART1 RX`   | [crate::Gp21Uart1Rx]        |
    /// | `I2C0 SCL`   | [crate::Gp21I2C0Scl]        |
    /// | `PWM2 B`     | [crate::Gp21Pwm2B]          |
    /// | `PIO0`       | [crate::Gp21Pio0]           |
    /// | `PIO1`       | [crate::Gp21Pio1]           |
    Gpio21 {
        name: scl,
        aliases: {
            /// UART Function alias for pin [crate::Pins::scl].
            FunctionUart: Gp21Uart1Rx,
            /// SPI Function alias for pin [crate::Pins::scl].
            FunctionSpi: Gp21Spi0Csn,
            /// I2C Function alias for pin [crate::Pins::scl].
            FunctionI2C: Gp21I2C0Scl,
            /// PWM Function alias for pin [crate::Pins::scl].
            FunctionPwm: Gp21Pwm2B,
            /// PIO0 Function alias for pin [crate::Pins::scl].
            FunctionPio0: Gp21Pio0,
            /// PIO1 Function alias for pin [crate::Pins::scl].
            FunctionPio1: Gp21Pio1
        }
    },

    /// GPIO 22 is connected to adc_addr_0 of the Servo 2040 board.
    Gpio22 {
        name: adc_addr_0,
    },

    /// GPIO 23 supports following functions:
    ///
    /// | Function     | Alias with applied function |
    /// |--------------|-----------------------------|
    /// | `SPI0 TX`    | [crate::Gp23Spi0Tx]         |
    /// | `UART1 RTS`  | [crate::Gp23Uart1Rts]       |
    /// | `I2C1 SCL`   | [crate::Gp23I2C1Scl]        |
    /// | `PWM3 B`     | [crate::Gp23Pwm3B]          |
    /// | `PIO0`       | [crate::Gp23Pio0]           |
    /// | `PIO1`       | [crate::Gp23Pio1]           |
    Gpio23 {
        name: user_sw,
        aliases: {
            /// UART Function alias for pin [crate::Pins::user_sw].
            FunctionUart: Gp23Uart1Rts,
            /// SPI Function alias for pin [crate::Pins::user_sw].
            FunctionSpi: Gp23Spi0Tx,
            /// I2C Function alias for pin [crate::Pins::user_sw].
            FunctionI2C: Gp23I2C1Scl,
            /// PWM Function alias for pin [crate::Pins::user_sw].
            FunctionPwm: Gp23Pwm3B,
            /// PIO0 Function alias for pin [crate::Pins::user_sw].
            FunctionPio0: Gp23Pio0,
            /// PIO1 Function alias for pin [crate::Pins::user_sw].
            FunctionPio1: Gp23Pio1
        }
    },

    /// GPIO 24 is connected to adc_addr_1 of the Servo 2040 board.
    Gpio24 {
        name: adc_addr_1,
    },

    /// GPIO 25 is connected to adc_addr_2 of the Servo 2040 board.
    Gpio25 {
        name: adc_addr_2,
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
        name: adc0,
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
        name: adc1,
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
        name: adc2,
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

    /// GPIO 29 is connected to shared adc of the Servo2040 board.
    Gpio29 {
        name: shared_adc,
    },
);

pub const XOSC_CRYSTAL_FREQ: u32 = 12_000_000;
