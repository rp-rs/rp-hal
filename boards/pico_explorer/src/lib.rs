#![no_std]

extern crate rp2040_hal as hal;

#[cfg(feature = "rt")]
extern crate cortex_m_rt;
#[cfg(feature = "rt")]
pub use cortex_m_rt::entry;

pub use hal::pac;

hal::bsp_pins!(
    Gpio0 {
        name: gpio0,
        aliases: { FunctionPwm: Audio0 }
    },
    Gpio1 {
        name: gpio1,
        aliases: { FunctionPwm: Audio1 }
    },
    Gpio2 {
        name: gpio2,
        aliases: { FunctionPwm: Audio2 }
    },
    Gpio3 {
        name: gpio3,
        aliases: { FunctionPwm: Audio3 }
    },
    Gpio4 {
        name: gpio4,
        aliases: { FunctionPwm: Audio4 }
    },
    Gpio5 {
        name: gpio5,
        aliases: { FunctionPwm: Audio5 }
    },
    Gpio6 {
        name: gpio6,
        aliases: { FunctionPwm: Audio6 }
    },
    Gpio7 {
        name: gpio7,
        aliases: { FunctionPwm: Audio7 }
    },
    Gpio8 {
        name: motor1_min,
        aliases: { FunctionPwm: Motor1Min }
    },
    Gpio9 {
        name: motor1_plus,
        aliases: { FunctionPwm: Motor1Plus }
    },
    Gpio10 {
        name: motor2_min,
        aliases: { FunctionPwm: Motor2Min }
    },
    Gpio11 {
        name: motor2_plus,
        aliases: { FunctionPwm: Motor2Plus }
    },
    Gpio12 { name: switch_a },
    Gpio13 { name: switch_b },
    Gpio14 { name: switch_x },
    Gpio15 { name: switch_y },
    Gpio16 {
        name: spi_miso,
        aliases: { FunctionSpi: Miso }
    },
    Gpio17 {
        name: lcd_cs,
        aliases: { FunctionSpi: LcdCs }
    },
    Gpio18 {
        name: spi_sclk,
        aliases: { FunctionSpi: Sclk }
    },
    Gpio19 {
        name: spi_mosi,
        aliases: { FunctionSpi: Mosi }
    },
    Gpio20 {
        name: i2c_sda,
        aliases: { FunctionI2C: Sda }
    },
    Gpio21 {
        name: i2c_scl,
        aliases: { FunctionI2C: Scl }
    },
    Gpio22 {
        name: i2c_int,
        aliases: { FunctionI2C: Int }
    },
    Gpio25 { name: led },
    Gpio26 { name: adc0 },
    Gpio27 { name: adc1 },
    Gpio28 { name: adc2 },
);
