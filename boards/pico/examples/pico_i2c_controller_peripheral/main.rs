//! I2C controller and I2C peripheral async demo.
//!
//! This example demonstrates use of both I2C peripherals (I2C0 and I2C1) at the same time on a single Pico using [Embassy](https://github.com/embassy-rs/embassy), an async executor.
//!
//! Each peripheral is passed to an async task, which allows them to operate independently of each other:
//!   - The controller task (ctrl_demo) uses I2C0. It calls the demo controller code in `controller.rs`
//!   - The peripheral task (prph_demo) uses I2C1. It calls the demo peripheral code in `peripheral.rs`
//!
//! ### Wiring notes:
//!
//! I2C0 uses pin `GP0` for `SDA`, and `GP1` for `SCL`.
//!
//! I2C1 uses `GP2` for `SDA`, and `GP3` for `SCL`.
//!
//! For this demo to function you must connect the `SDA` signals (`GP0` and `GP2`) to each other using wires.
//! You must also connect the `SCL` signals (`GP1` and `GP3`) to each other.
//!
//! A pull up resistor (to 3.3V, which is available on pin `36`) is required on SCL & SDA lines in order to reach the expected 1MHz. Although it
//! depends on the hardware context (wire length, impedance & capacitance), a typical value of 2KOhm
//! should generally work fine.
//!
//! If you do not connect the resistor and instead use the internal pull-ups on the I2C pins, you may need to lower the I2C frequency to avoid transmission errors.
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
    watchdog::Watchdog,
    Sio,
};
use pico::{hal, Pins, XOSC_CRYSTAL_FREQ};

use panic_halt as _;

mod controller;
mod peripheral;

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
