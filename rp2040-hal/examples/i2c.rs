//! Sends a message using i2c
#![no_std]
#![no_main]

use cortex_m_rt::entry;
use embedded_hal::blocking::i2c::Write;
use embedded_time::rate::Extensions;
use hal::gpio::FunctionI2C;
use hal::i2c::I2C;
use hal::pac;
use hal::sio::Sio;
use panic_halt as _;
use rp2040_hal as hal;

#[link_section = ".boot2"]
#[used]
pub static BOOT2: [u8; 256] = rp2040_boot2::BOOT_LOADER;

const SYS_HZ: u32 = 125_000_000_u32;

#[entry]
fn main() -> ! {
    let mut pac = pac::Peripherals::take().unwrap();

    let sio = Sio::new(pac.SIO);
    let pins = hal::gpio::Pins::new(
        pac.IO_BANK0,
        pac.PADS_BANK0,
        sio.gpio_bank0,
        &mut pac.RESETS,
    );

    let sda_pin = pins.gpio18.into_mode::<FunctionI2C>();
    let scl_pin = pins.gpio19.into_mode::<FunctionI2C>();

    let mut i2c = I2C::i2c1(
        pac.I2C1,
        sda_pin,
        scl_pin,
        400.kHz(),
        &mut pac.RESETS,
        SYS_HZ.Hz(),
    );

    i2c.write(0x2c, &[1, 2, 3]).unwrap();

    #[allow(clippy::empty_loop)]
    loop {}
}
