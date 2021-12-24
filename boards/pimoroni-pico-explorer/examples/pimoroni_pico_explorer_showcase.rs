#![no_std]
#![no_main]

use arrayvec::ArrayString;
use core::fmt::Write;
use cortex_m_rt::entry;
use embedded_graphics::{
    mono_font::{ascii::FONT_10X20, MonoTextStyleBuilder},
    pixelcolor::Rgb565,
    prelude::*,
    text::{Alignment, Text},
};
use embedded_hal::digital::v2::OutputPin;
use embedded_time::rate::*;
use hal::{adc::Adc, clocks::*, watchdog::Watchdog, Sio};
use panic_halt as _;
use pimoroni_pico_explorer::{hal, pac, Button, PicoExplorer, XOSC_CRYSTAL_FREQ};

// See 4.9.5 from RP2040 datasheet
fn calc_temp(adc_value: f32, refv: f64) -> f64 {
    let vbe: f64 = f64::from(adc_value) * refv;
    27f64 - (vbe - 0.706) / 0.001721
}

#[entry]
fn main() -> ! {
    let mut p = pac::Peripherals::take().unwrap();
    let cp = pac::CorePeripherals::take().unwrap();

    // Enable watchdog and clocks
    let mut watchdog = Watchdog::new(p.WATCHDOG);
    let clocks = init_clocks_and_plls(
        XOSC_CRYSTAL_FREQ,
        p.XOSC,
        p.CLOCKS,
        p.PLL_SYS,
        p.PLL_USB,
        &mut p.RESETS,
        &mut watchdog,
    )
    .ok()
    .unwrap();

    let mut delay = cortex_m::delay::Delay::new(cp.SYST, clocks.system_clock.get_freq().integer());

    // Enable adc
    let mut adc = Adc::new(p.ADC, &mut p.RESETS);
    let mut temp_sense = adc.enable_temp_sensor();

    let sio = Sio::new(p.SIO);

    let (mut explorer, pins) = PicoExplorer::new(
        p.IO_BANK0,
        p.PADS_BANK0,
        sio.gpio_bank0,
        p.SPI0,
        adc,
        &mut p.RESETS,
        &mut delay,
    );

    let mut led = pins.led.into_push_pull_output();

    let mut even = true;
    loop {
        delay.delay_ms(500);

        // Set GPIO25 to be low
        led.set_low().unwrap();

        delay.delay_ms(500);

        // Set GPIO25 to be high
        led.set_high().unwrap();

        let adc_value = explorer.get_adc(&mut temp_sense);
        let temp: f64 = calc_temp(adc_value, 3.3);

        // Create a fixed buffer to store screen contents
        let mut buf = ArrayString::<100>::new();

        // Write to buffer
        writeln!(&mut buf, "Hello World {}", if even { '|' } else { '-' }).unwrap();
        writeln!(&mut buf, "Temp: {:.1}", temp).unwrap();
        writeln!(
            &mut buf,
            "A:{:.1} B:{:.1}\nX:{:.1} Y:{:.1}",
            explorer.is_pressed(Button::A),
            explorer.is_pressed(Button::B),
            explorer.is_pressed(Button::X),
            explorer.is_pressed(Button::Y)
        )
        .unwrap();

        // Draw buffer on screen
        let style = MonoTextStyleBuilder::new()
            .font(&FONT_10X20)
            .text_color(Rgb565::GREEN)
            .background_color(Rgb565::BLACK)
            .build();
        Text::with_alignment(&buf, Point::new(20, 30), style, Alignment::Left)
            .draw(&mut explorer.screen)
            .unwrap();

        even = !even;
    }
}
