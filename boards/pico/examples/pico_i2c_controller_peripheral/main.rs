//! Send data from an I2C controller to an I2C peripheral.
//!
//! This will transmit data between I2C0 on pins GP0, GP1 acting as a controller to I2C1 on pins
//! GP2, GP3 acting as a peripheral.
//!
//! A pull up is required on SCL & SDA lines in order to reach the expected 1MHz. Although it
//! depends on the hardware context (wire length, impedance & capacitance), a typical value of 2KOhm
//! should generally work fine.
#![no_std]
#![no_main]
#![feature(type_alias_impl_trait)]

use embassy::{executor::Executor, util::Forever};
use embedded_time::rate::Extensions;
use hal::{
    clocks::{init_clocks_and_plls, Clock},
    gpio::{bank0, FunctionI2C, Pin},
    i2c::{peripheral::I2CPeripheralEventIterator, I2C},
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
pub static BOOT2: [u8; 256] = rp2040_boot2::BOOT_LOADER_W25Q080;

const ADDRESS: u16 = 0x55;

#[embassy::task]
async fn ctrl_demo(
    mut i2c: I2C<
        pac::I2C0,
        (
            Pin<bank0::Gpio0, FunctionI2C>,
            Pin<bank0::Gpio1, FunctionI2C>,
        ),
    >,
) {
    controller::run_demo(&mut i2c).await.expect("Demo failed")
}

#[embassy::task]
async fn prph_demo(
    mut i2c: I2CPeripheralEventIterator<
        pac::I2C1,
        (
            Pin<bank0::Gpio2, FunctionI2C>,
            Pin<bank0::Gpio3, FunctionI2C>,
        ),
    >,
) {
    peripheral::run_demo(&mut i2c).await.expect("Demo failed")
}

#[cortex_m_rt::entry]
fn main() -> ! {
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
    let i2c0 = I2C::new_controller(
        pac.I2C0,
        pins.gpio0.into_mode(),
        pins.gpio1.into_mode(),
        1_000.kHz(),
        &mut pac.RESETS,
        clocks.system_clock.freq(),
    );

    let i2c1 = I2C::new_peripheral_event_iterator(
        pac.I2C1,
        pins.gpio2.into_mode(),
        pins.gpio3.into_mode(),
        &mut pac.RESETS,
        ADDRESS,
    );

    static EXECUTOR: Forever<Executor> = Forever::new();
    let executor = EXECUTOR.put(Executor::new());

    executor.run(|spawner| {
        spawner.spawn(ctrl_demo(i2c0)).unwrap();
        spawner.spawn(prph_demo(i2c1)).unwrap();
    });
}
