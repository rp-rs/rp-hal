//! Blinks the 3 colour LEDs on a Pimoroni Plasma 2040 in sequence
#![no_std]
#![no_main]

use defmt::*;
use defmt_rtt as _;
use embedded_hal::digital::v2::OutputPin;
use panic_halt as _;

use pimoroni_plasma_2040 as bsp;

use bsp::hal::{
    clocks::{init_clocks_and_plls, Clock},
    gpio::PinState,
    pac,
    sio::Sio,
    watchdog::Watchdog,
};

/// Entry point to our bare-metal application.
///
/// The `#[rp2040_hal::entry]` macro ensures the Cortex-M start-up code calls this function
/// as soon as all global variables and the spinlock are initialised.
#[rp2040_hal::entry]
fn main() -> ! {
    info!("Program start");
    let mut pac = pac::Peripherals::take().unwrap();
    let core = pac::CorePeripherals::take().unwrap();
    let mut watchdog = Watchdog::new(pac.WATCHDOG);
    let sio = Sio::new(pac.SIO);

    let clocks = init_clocks_and_plls(
        bsp::XOSC_CRYSTAL_FREQ,
        pac.XOSC,
        pac.CLOCKS,
        pac.PLL_SYS,
        pac.PLL_USB,
        &mut pac.RESETS,
        &mut watchdog,
    )
    .ok()
    .unwrap();

    let mut delay = cortex_m::delay::Delay::new(core.SYST, clocks.system_clock.freq().to_Hz());

    let pins = bsp::Pins::new(
        pac.IO_BANK0,
        pac.PADS_BANK0,
        sio.gpio_bank0,
        &mut pac.RESETS,
    );

    let mut led_green = pins
        .led_green
        .into_push_pull_output_in_state(PinState::High);
    let mut led_red = pins.led_red.into_push_pull_output_in_state(PinState::High);
    let mut led_blue = pins.led_blue.into_push_pull_output_in_state(PinState::High);

    loop {
        led_green.set_low().unwrap();
        delay.delay_ms(500);
        led_green.set_high().unwrap();
        led_blue.set_low().unwrap();
        delay.delay_ms(500);
        led_blue.set_high().unwrap();
        led_red.set_low().unwrap();
        delay.delay_ms(500);
        led_red.set_high().unwrap();
    }
}

// End of file
