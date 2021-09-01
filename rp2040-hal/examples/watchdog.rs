//! How to use the watchdog peripheral to reset the system in case something takes too long
#![no_std]
#![no_main]

use cortex_m::prelude::{_embedded_hal_watchdog_Watchdog, _embedded_hal_watchdog_WatchdogEnable};
use cortex_m_rt::entry;
use embedded_hal::digital::v2::OutputPin;
use embedded_time::duration::units::*;
use embedded_time::fixed_point::FixedPoint;
use hal::clocks::{init_clocks_and_plls, Clock};
use hal::gpio::Pins;
use hal::pac;
use hal::sio::Sio;
use hal::watchdog::Watchdog;
use panic_halt as _;
use rp2040_hal as hal;

#[link_section = ".boot2"]
#[used]
pub static BOOT2: [u8; 256] = rp2040_boot2::BOOT_LOADER;

// External high-speed crystal on the pico board is 12Mhz
// Adjust for your board if this isn't the same
const EXTERNAL_XTAL_FREQ_HZ: u32 = 12_000_000;

#[entry]
fn main() -> ! {
    let mut pac = pac::Peripherals::take().unwrap();
    let cp = pac::CorePeripherals::take().unwrap();
    let mut watchdog = Watchdog::new(pac.WATCHDOG);
    let sio = Sio::new(pac.SIO);

    let clocks = init_clocks_and_plls(
        EXTERNAL_XTAL_FREQ_HZ,
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
        sio.gpio_bank0,
        &mut pac.RESETS,
    );

    // We need to accurately delay to give feedback that the watchdog is working correctly
    let mut delay = cortex_m::delay::Delay::new(cp.SYST, clocks.system_clock.freq().integer());

    // Configure an LED so we can show the current state of the watchdog
    let mut led_pin = pins.gpio25.into_push_pull_output();

    // Set the LED high for 2 seconds so we know when we're about to start the watchdog
    led_pin.set_high().unwrap();
    delay.delay_ms(2000);

    // Set to watchdog to reset if it's not reloaded within 1.05 seconds, and start it
    watchdog.start(1_050_000.microseconds());

    // Blink once a second for 5 seconds, refreshing the watchdog timer once a second to avoid a reset
    for _ in 1..=5 {
        led_pin.set_low().unwrap();
        delay.delay_ms(500);
        led_pin.set_high().unwrap();
        delay.delay_ms(500);
        watchdog.feed();
    }

    // Blink 10 times per second, not feeding the watchdog.
    // The processor should reset in 1.05 seconds, or 5 blinks time
    loop {
        led_pin.set_low().unwrap();
        delay.delay_ms(100);
        led_pin.set_high().unwrap();
        delay.delay_ms(100);
    }
}
