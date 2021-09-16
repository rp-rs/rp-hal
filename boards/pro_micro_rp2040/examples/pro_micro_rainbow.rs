//! Cycles colors on on the on board addressable LED.
#![no_std]
#![no_main]

use core::iter::once;
use cortex_m_rt::entry;
use embedded_hal::timer::CountDown;
use embedded_time::duration::Extensions;
use panic_halt as _;

use pro_micro_rp2040::{
    hal::{
        clocks::{init_clocks_and_plls, Clock},
        gpio::{FunctionPio0, Pin},
        pac,
        sio::Sio,
        timer::Timer,
        watchdog::Watchdog,
    },
    XOSC_CRYSTAL_FREQ,
};
use smart_leds::{brightness, SmartLedsWrite, RGB8};
use ws2812_pio::Ws2812;

#[link_section = ".boot2"]
#[used]
pub static BOOT2: [u8; 256] = rp2040_boot2::BOOT_LOADER;

#[entry]
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

    let sio = Sio::new(pac.SIO);
    let pins = pro_micro_rp2040::Pins::new(
        pac.IO_BANK0,
        pac.PADS_BANK0,
        sio.gpio_bank0,
        &mut pac.RESETS,
    );
    let _led: Pin<_, FunctionPio0> = pins.led.into_mode();
    let timer = Timer::new(pac.TIMER);
    let mut delay = timer.count_down();

    let mut ws = Ws2812::new(
        25,
        pac.PIO0,
        &mut pac.RESETS,
        clocks.system_clock.freq(),
        timer.count_down(),
    );

    let mut n: u8 = 128;
    loop {
        ws.write(brightness(once(wheel(n)), 32)).unwrap();
        n = n.wrapping_add(1);

        delay.start(25.milliseconds());
        let _ = nb::block!(delay.wait());
    }
}
/// Input a value 0 to 255 to get a color value
/// The colours are a transition r - g - b - back to r.
fn wheel(mut wheel_pos: u8) -> RGB8 {
    wheel_pos = 255 - wheel_pos;
    if wheel_pos < 85 {
        return (255 - wheel_pos * 3, 0, wheel_pos * 3).into();
    }
    if wheel_pos < 170 {
        wheel_pos -= 85;
        return (0, wheel_pos * 3, 255 - wheel_pos * 3).into();
    }
    wheel_pos -= 170;
    (wheel_pos * 3, 255 - wheel_pos * 3, 0).into()
}
