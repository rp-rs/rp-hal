#![no_std]
#![no_main]

use cortex_m_rt::entry;
use hal::pac;
use hal::sio::Sio;
use hd44780_driver as hd44780;
use panic_halt as _;
use rp2040_hal as hal;

#[link_section = ".boot2"]
#[used]
pub static BOOT2: [u8; 256] = rp2040_boot2::BOOT_LOADER;

#[entry]
fn main() -> ! {
    let mut pac = pac::Peripherals::take().unwrap();
    let core = pac::CorePeripherals::take().unwrap();

    // AHL bus speed default
    let mut delay_provider = cortex_m::delay::Delay::new(core.SYST, 12_000_000);

    let sio = Sio::new(pac.SIO);
    let pins = hal::gpio::Pins::new(
        pac.IO_BANK0,
        pac.PADS_BANK0,
        sio.gpio_bank0,
        &mut pac.RESETS,
    );

    let lcd = hd44780::HD44780::new_4bit(
        pins.gpio16.into_push_pull_output(), // Register Select
        pins.gpio17.into_push_pull_output(), // Enable
        pins.gpio18.into_push_pull_output(), // d4
        pins.gpio19.into_push_pull_output(), // d5
        pins.gpio20.into_push_pull_output(), // d6
        pins.gpio21.into_push_pull_output(), // d7
        &mut delay_provider,
    );

    let mut lcd = lcd.unwrap();

    lcd.reset(&mut delay_provider).unwrap();
    lcd.clear(&mut delay_provider).unwrap();
    lcd.write_str("rp-hal on", &mut delay_provider).unwrap();
    lcd.set_cursor_pos(40, &mut delay_provider).unwrap();
    lcd.set_cursor_visibility(hd44780::Cursor::Visible, &mut delay_provider)
        .unwrap();
    lcd.write_str("HD44780!", &mut delay_provider).unwrap();

    #[allow(clippy::empty_loop)]
    loop {}
}
