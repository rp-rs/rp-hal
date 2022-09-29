//! Animates a rainbow wheel on the 6 color LEDs on a Servo2040 in sequence
#![no_std]
#![no_main]

use bsp::entry;
use bsp::hal::{
    clocks::{init_clocks_and_plls, Clock},
    pac,
    sio::Sio,
    watchdog::Watchdog,
};
use defmt::*;
use defmt_rtt as _;
use embedded_hal::timer::CountDown;
use fugit::ExtU32;
use panic_halt as _;
use pimoroni_servo2040 as bsp;
use rp2040_hal::pio::PIOExt;
use rp2040_hal::Timer;
use smart_leds::{brightness, SmartLedsWrite, RGB8};
use ws2812_pio::Ws2812;

#[entry]
fn main() -> ! {
    info!("Program start");
    let mut pac = pac::Peripherals::take().unwrap();
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

    let pins = bsp::Pins::new(
        pac.IO_BANK0,
        pac.PADS_BANK0,
        sio.gpio_bank0,
        &mut pac.RESETS,
    );

    let timer = Timer::new(pac.TIMER, &mut pac.RESETS);
    let mut delay = timer.count_down();

    // Configure the addressable LED
    let (mut pio, sm0, _, _, _) = pac.PIO0.split(&mut pac.RESETS);

    let mut ws = Ws2812::new(
        pins.led_data.into_mode(),
        &mut pio,
        sm0,
        clocks.peripheral_clock.freq(),
        timer.count_down(),
    );

    // Infinite color wheel loop

    let mut n: u8 = 128;
    let offset = (256u16 / bsp::NUM_LEDS as u16) as u8;
    loop {
        ws.write(brightness(
            IntoIterator::into_iter([
                wheel(n),
                wheel(n.wrapping_add(offset)),
                wheel(n.wrapping_add(offset * 2)),
                wheel(n.wrapping_add(offset * 3)),
                wheel(n.wrapping_add(offset * 4)),
                wheel(n.wrapping_add(offset * 5)),
            ]),
            32,
        ))
        .unwrap();
        n = n.wrapping_add(1);

        delay.start(25.millis());
        let _ = nb::block!(delay.wait());
    }
}

/// Convert a number from `0..=255` to an RGB color triplet.
///
/// The colours are a transition from red, to green, to blue and back to red.
fn wheel(mut wheel_pos: u8) -> RGB8 {
    wheel_pos = 255 - wheel_pos;
    if wheel_pos < 85 {
        // No green in this sector - red and blue only
        (255 - (wheel_pos * 3), 0, wheel_pos * 3).into()
    } else if wheel_pos < 170 {
        // No red in this sector - green and blue only
        wheel_pos -= 85;
        (0, wheel_pos * 3, 255 - (wheel_pos * 3)).into()
    } else {
        // No blue in this sector - red and green only
        wheel_pos -= 170;
        (wheel_pos * 3, 255 - (wheel_pos * 3), 0).into()
    }
}

// End of file
