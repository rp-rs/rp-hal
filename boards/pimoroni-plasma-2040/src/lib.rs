#![no_std]

pub extern crate rp2040_hal as hal;

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
    /// GPIO 0 is connected to I2C0_SDA
    Gpio0 { name: i2c0_sda },
    /// GPIO 1 is connected to I2C0_SCL
    Gpio1 { name: i2c0_scl },
    /// GPIO 2 is connected to I2C1_SDA
    Gpio2 { name: i2c1_sda },
    /// GPIO 3 is connected to I2C1_SCL
    Gpio3 { name: i2c1_scl },
    Gpio4 { name: gpio4 },
    Gpio5 { name: gpio5 },
    /// GPIO 12 is connected to button A, active low
    Gpio12 { name: button_a },
    /// GPIO 13 is connected to button B, active low
    Gpio13 { name: button_b },
    /// GPIO 14 is connected to CLK for APA102 only
    Gpio14 { name: clk },
    /// GPIO 15 is connected to DAT for Apa102 and Ws2812
    Gpio15 { name: data },
    /// GPIO 16 is red LED, active low
    Gpio16 { name: led_red },
    /// GPIO 17 is green LED, active low
    Gpio17 { name: led_green },
    /// GPIO 18 is blue LED, active low
    Gpio18 { name: led_blue },
    /// GPIO 19 is I2C_INT
    Gpio19 { name: i2c_int },
    /// GPIO 20 is I2C_SDA
    Gpio20 {
        name: i2c_sda,
        aliases: { FunctionI2C: Sda }
    },
    /// GPIO 21 is I2C_SCL
    Gpio21 {
        name: i2c_scl,
        aliases: { FunctionI2C: Scl }
    },
    /// GPIO 23 is connected to the USER_SW, the BOOT button, active low
    Gpio23 { name: user_sw },
    /// GPIO 26 is connected to ADC0
    Gpio26 { name: adc0 },
    /// GPIO 27 is connected to ADC1
    Gpio27 { name: adc1 },
    /// GPIO 28 is connected to ADC2
    Gpio28 { name: adc2 },
    /// GPIO 29 is connected to ADC3 which is used for low side current sensing
    Gpio29 {
        name: current_sense,
    },
);

pub const XOSC_CRYSTAL_FREQ: u32 = 12_000_000;

pub const ADC_GAIN: u32 = 50;
pub const SHUNT_RESISTOR: f32 = 0.015;
