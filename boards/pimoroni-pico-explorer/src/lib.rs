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

use display_interface_spi::SPIInterface;
use embedded_graphics::{
    draw_target::DrawTarget,
    pixelcolor::{Rgb565, RgbColor},
};
use embedded_hal::{
    adc::{Channel, OneShot},
    blocking::delay::DelayUs,
    digital::v2::{InputPin, OutputPin},
    spi::MODE_0,
};
use embedded_time::rate::*;
pub use hal::pac;
use hal::{
    adc::Adc,
    gpio::{
        bank0::{
            Gpio0, Gpio1, Gpio12, Gpio13, Gpio14, Gpio15, Gpio16, Gpio17, Gpio18, Gpio19, Gpio2,
            Gpio20, Gpio21, Gpio22, Gpio23, Gpio24, Gpio25, Gpio26, Gpio27, Gpio28, Gpio29, Gpio3,
            Gpio4, Gpio5, Gpio6, Gpio7,
        },
        FunctionI2C, FunctionPwm, FunctionSpi, Pin, PinId, PullUpInput, PushPullOutput,
    },
    pac::{RESETS, SPI0},
    sio::SioGpioBank0,
    spi::{Enabled, Spi},
};
use st7789::ST7789;

mod internal_pins {
    hal::bsp_pins!(
        Gpio0 { name: gpio0 },
        Gpio1 { name: gpio1 },
        Gpio2 { name: gpio2 },
        Gpio3 { name: gpio3 },
        Gpio4 { name: gpio4 },
        Gpio5 { name: gpio5 },
        Gpio6 { name: gpio6 },
        Gpio7 { name: gpio7 },
        Gpio8 {
            name: motor1_neg,
            aliases: { FunctionPwm: Motor1Neg }
        },
        Gpio9 {
            name: motor1_pos,
            aliases: { FunctionPwm: Motor1Pos }
        },
        Gpio10 {
            name: motor2_neg,
            aliases: { FunctionPwm: Motor2Neg }
        },
        Gpio11 {
            name: motor2_pos,
            aliases: { FunctionPwm: Motor2Pos }
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
        Gpio22 { name: i2c_int },
        Gpio23 { name: b_power_save },
        Gpio24 { name: vbus_detect },
        Gpio25 { name: led },
        Gpio26 { name: adc0 },
        Gpio27 { name: adc1 },
        Gpio28 { name: adc2 },
        Gpio29 {
            name: voltage_monitor
        },
    );
}

pub struct Pins {
    pub gpio0: Pin<Gpio0, <Gpio0 as PinId>::Reset>,
    pub gpio1: Pin<Gpio1, <Gpio1 as PinId>::Reset>,
    pub gpio2: Pin<Gpio2, <Gpio2 as PinId>::Reset>,
    pub gpio3: Pin<Gpio3, <Gpio3 as PinId>::Reset>,
    pub gpio4: Pin<Gpio4, <Gpio4 as PinId>::Reset>,
    pub gpio5: Pin<Gpio5, <Gpio5 as PinId>::Reset>,
    pub gpio6: Pin<Gpio6, <Gpio6 as PinId>::Reset>,
    pub gpio7: Pin<Gpio7, <Gpio7 as PinId>::Reset>,
    pub spi_sclk: Pin<Gpio18, FunctionSpi>,
    pub spi_mosi: Pin<Gpio19, FunctionSpi>,
    pub i2c_sda: Pin<Gpio20, FunctionI2C>,
    pub i2c_scl: Pin<Gpio21, FunctionI2C>,
    pub i2c_int: Pin<Gpio22, FunctionI2C>,
    pub b_power_save: Pin<Gpio23, <Gpio23 as PinId>::Reset>,
    pub vbus_detect: Pin<Gpio24, <Gpio24 as PinId>::Reset>,
    pub led: Pin<Gpio25, <Gpio25 as PinId>::Reset>,
    pub adc0: Pin<Gpio26, <Gpio26 as PinId>::Reset>,
    pub adc1: Pin<Gpio27, <Gpio27 as PinId>::Reset>,
    pub adc2: Pin<Gpio28, <Gpio28 as PinId>::Reset>,
    pub voltage_monitor: Pin<Gpio29, <Gpio29 as PinId>::Reset>,
}

pub const XOSC_CRYSTAL_FREQ: u32 = 12_000_000;
pub enum Button {
    A,
    B,
    X,
    Y,
}

pub enum Motor {
    _1,
    _2,
}

pub enum MotorAction {
    Forward(f32),
    Reverse(f32),
    Stop,
}

pub type Screen = ST7789<
    SPIInterface<Spi<Enabled, SPI0, 8>, Pin<Gpio16, PushPullOutput>, Pin<Gpio17, PushPullOutput>>,
    DummyPin,
>;

pub struct PicoExplorer {
    a: Pin<Gpio12, PullUpInput>,
    b: Pin<Gpio13, PullUpInput>,
    x: Pin<Gpio14, PullUpInput>,
    y: Pin<Gpio15, PullUpInput>,
    adc: Adc,
    pub screen: Screen,
}

pub struct DummyPin;

impl OutputPin for DummyPin {
    type Error = ();
    fn set_high(&mut self) -> Result<(), Self::Error> {
        Ok(())
    }
    fn set_low(&mut self) -> Result<(), Self::Error> {
        Ok(())
    }
}

impl PicoExplorer {
    pub fn new(
        io: pac::IO_BANK0,
        pads: pac::PADS_BANK0,
        sio: SioGpioBank0,
        spi0: SPI0,
        adc: Adc,
        resets: &mut RESETS,
        delay: &mut impl DelayUs<u32>,
    ) -> (Self, Pins) {
        let internal_pins = internal_pins::Pins::new(io, pads, sio, resets);

        let a = internal_pins.switch_a.into_pull_up_input();
        let b = internal_pins.switch_b.into_pull_up_input();
        let x = internal_pins.switch_x.into_pull_up_input();
        let y = internal_pins.switch_y.into_pull_up_input();

        internal_pins.motor1_pos.into_mode::<FunctionPwm>();
        internal_pins.motor1_neg.into_mode::<FunctionPwm>();
        internal_pins.motor2_pos.into_mode::<FunctionPwm>();
        internal_pins.motor2_neg.into_mode::<FunctionPwm>();

        let dc = internal_pins.spi_miso.into_push_pull_output();
        let cs = internal_pins.lcd_cs.into_push_pull_output();
        let spi_sclk = internal_pins.spi_sclk.into_mode::<FunctionSpi>();
        let spi_mosi = internal_pins.spi_mosi.into_mode::<FunctionSpi>();

        let spi_screen = Spi::<_, _, 8>::new(spi0).init(
            resets,
            125_000_000u32.Hz(),
            16_000_000u32.Hz(),
            &MODE_0,
        );

        let spii_screen = SPIInterface::new(spi_screen, dc, cs);

        let mut screen = ST7789::new(spii_screen, DummyPin, 240, 240);

        screen.init(delay).unwrap();
        screen
            .set_orientation(st7789::Orientation::Portrait)
            .unwrap();
        screen.clear(Rgb565::BLACK).unwrap();

        (
            PicoExplorer {
                a,
                b,
                x,
                y,
                adc,
                screen,
            },
            Pins {
                gpio0: internal_pins.gpio0,
                gpio1: internal_pins.gpio1,
                gpio2: internal_pins.gpio2,
                gpio3: internal_pins.gpio3,
                gpio4: internal_pins.gpio4,
                gpio5: internal_pins.gpio5,
                gpio6: internal_pins.gpio6,
                gpio7: internal_pins.gpio7,
                spi_sclk,
                spi_mosi,
                i2c_sda: internal_pins.i2c_sda.into_mode(),
                i2c_scl: internal_pins.i2c_scl.into_mode(),
                i2c_int: internal_pins.i2c_int.into_mode(),
                b_power_save: internal_pins.b_power_save,
                vbus_detect: internal_pins.vbus_detect,
                led: internal_pins.led,
                adc0: internal_pins.adc0,
                adc1: internal_pins.adc1,
                adc2: internal_pins.adc2,
                voltage_monitor: internal_pins.voltage_monitor,
            },
        )
    }

    pub fn is_pressed(&self, button: Button) -> bool {
        use Button::*;
        match button {
            A => self.a.is_low().unwrap(),
            B => self.b.is_low().unwrap(),
            X => self.x.is_low().unwrap(),
            Y => self.y.is_low().unwrap(),
        }
    }

    pub fn get_adc<Pin: Channel<Adc, ID = u8>>(&mut self, channel: &mut Pin) -> f32 {
        // scale raw 12-bit adc value to 0 .. 1 float
        let adc_value: u16 = self.adc.read(channel).unwrap();
        let mut result: f32 = f32::from(adc_value) / f32::from(1u16 << 12);
        // clamp result to 0 .. 1
        if result > 1.0 {
            result = 1.0
        }

        if result < 0.0 {
            result = 0.0
        }
        result
    }
}
