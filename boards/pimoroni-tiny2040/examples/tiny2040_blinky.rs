//! Blinks the 3 colour LEDs on a Tiny2040 in sequence
#![no_std]
#![no_main]

use bsp::entry;
use defmt::*;
use defmt_rtt as _;
use embedded_hal::digital::v2::OutputPin;
use panic_halt as _;

use pimoroni_tiny2040 as bsp;

use bsp::hal::{
    clocks::{init_clocks_and_plls, Clock},
    pac,
    sio::Sio,
    watchdog::Watchdog,
};

#[entry]
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

    let mut led_green = pins.led_green.into_push_pull_output();
    let mut led_red = pins.led_red.into_push_pull_output();
    let mut led_blue = pins.led_blue.into_push_pull_output();
    led_green.set_high().unwrap();
    led_red.set_high().unwrap();
    led_blue.set_high().unwrap();

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
