//! Send data from an I2C controller to an I2C peripheral.
//!
//! This will transmit data between I2C0 on pins GP0, GP1 acting as a controller to I2C1 on pins
//! GP2, GP3 acting as a peripheral.
#![no_std]
#![no_main]

use cortex_m_rt::entry;
use embedded_time::rate::Extensions;
use hal::{
    clocks::{init_clocks_and_plls, Clock},
    pac,
    sio::Sio,
    watchdog::Watchdog,
};
use pico::{hal, Pins, XOSC_CRYSTAL_FREQ};

use panic_halt as _;

mod controller;
mod peripheral;

#[link_section = ".boot2"]
#[used]
pub static BOOT2: [u8; 256] = rp2040_boot2::BOOT_LOADER;

const ADDRESS: u16 = 0x55;

#[entry]
fn main() -> ! {
    let runtime = nostd_async::Runtime::new();

    let mut pac = pac::Peripherals::take().unwrap();
    let mut watchdog = Watchdog::new(pac.WATCHDOG);

    let clocks = init_clocks_and_plls(
        XOSC_CRYSTAL_FREQ,
        pac.XOSC,
        pac.CLOCKS,
        pac.PLL_SYS,
        pac.PLL_USB,
        &mut pac.RESETS,
        &mut watchdog,
    )
    .ok()
    .unwrap();

    let pins = Pins::new(
        pac.IO_BANK0,
        pac.PADS_BANK0,
        Sio::new(pac.SIO).gpio_bank0,
        &mut pac.RESETS,
    );
    let mut i2c0 = hal::i2c::I2C::new_controller(
        pac.I2C0,
        pins.gpio0.into_mode(),
        pins.gpio1.into_mode(),
        1_000.kHz(),
        &mut pac.RESETS,
        clocks.system_clock.freq(),
    );

    let mut i2c1 = hal::i2c::I2C::new_peripheral(
        pac.I2C1,
        pins.gpio2.into_mode(),
        pins.gpio3.into_mode(),
        &mut pac.RESETS,
        ADDRESS,
    )
    .into_async();

    let mut ctrl = nostd_async::Task::new(async { controller::run_demo(&mut i2c0).await });
    let mut prph = nostd_async::Task::new(async { peripheral::run_demo(&mut i2c1).await });

    let h1 = ctrl.spawn(&runtime);
    let h2 = prph.spawn(&runtime);

    h1.join().unwrap();
    h2.join().unwrap();

    loop {
        cortex_m::asm::nop()
    }
}
