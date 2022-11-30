//!  Adafruit MacroPad showcase example
//!
//! This example exercises shows off all the functionality on an Adafruit MacroPad board
//! - LCD displays a message
//! - LEDs cycle a colour pattern
//! - Buzzer makes some sound
//! - Blinks the LED
//! TODO:
//! - Poll keys
//! - Poll rotary encoder
//! - Poll rotary encoder button
//! - USB
//! - Interact with external i2c devices connected via qwiic port

#![no_std]
#![no_main]

use adafruit_macropad::{
    hal::{
        self as hal,
        clocks::{init_clocks_and_plls, Clock},
        gpio, pac,
        pio::PIOExt,
        watchdog::Watchdog,
        Sio, Spi, Timer,
    },
    Pins, XOSC_CRYSTAL_FREQ,
};
use cortex_m_rt::entry;
use embedded_hal::digital::v2::{InputPin, ToggleableOutputPin};
use hal::gpio::DynPin;
use panic_halt as _;

// Import useful traits to handle the ws2812 LEDs:
use smart_leds::{brightness, SmartLedsWrite, RGB, RGB8};

// Import the actual crate to handle the Ws2812 protocol:
use embedded_hal::digital::v2::OutputPin;
use fugit::RateExtU32;
use ws2812_pio::Ws2812;

// There are 12 RGB leds in the strip, one for each key
const STRIP_LEN: usize = 12;
// For in the graphics drawing utilities like the font
// and the drawing routines:
use embedded_graphics::{
    mono_font::{ascii::FONT_9X18_BOLD, MonoTextStyleBuilder},
    pixelcolor::BinaryColor,
    prelude::*,
    text::{Baseline, Text},
};

// The display driver:
use sh1106::{prelude::*, Builder};

#[entry]
fn main() -> ! {
    let mut pac = pac::Peripherals::take().unwrap();
    let core = pac::CorePeripherals::take().unwrap();

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

    // Setup a delay for the LED blink signals
    let mut delay = cortex_m::delay::Delay::new(core.SYST, clocks.system_clock.freq().to_Hz());
    // Create a count down timer for the Ws2812 instance
    let timer = Timer::new(pac.TIMER, &mut pac.RESETS);

    let sio = Sio::new(pac.SIO);
    let pins = Pins::new(
        pac.IO_BANK0,
        pac.PADS_BANK0,
        sio.gpio_bank0,
        &mut pac.RESETS,
    );
    let mut led_pin = pins.led.into_push_pull_output();

    let buttons: [DynPin; 12] = [
        pins.key1.into_pull_up_input().into(),
        pins.key2.into_pull_up_input().into(),
        pins.key3.into_pull_up_input().into(),
        pins.key4.into_pull_up_input().into(),
        pins.key5.into_pull_up_input().into(),
        pins.key6.into_pull_up_input().into(),
        pins.key7.into_pull_up_input().into(),
        pins.key8.into_pull_up_input().into(),
        pins.key9.into_pull_up_input().into(),
        pins.key10.into_pull_up_input().into(),
        pins.key11.into_pull_up_input().into(),
        pins.key12.into_pull_up_input().into(),
    ];

    // Split PIO state machine 0 into individual objects, so that Ws2812 can use it
    let (mut pio, sm0, _, _, _) = pac.PIO0.split(&mut pac.RESETS);

    // Instantiate a Ws2812 LED strip
    let mut ws = Ws2812::new(
        // Use neopixel pin (19) on the Adafruit macropad for the LED data output
        pins.neopixel.into_mode(),
        &mut pio,
        sm0,
        clocks.peripheral_clock.freq(),
        timer.count_down(),
    );

    let mut leds: [RGB8; STRIP_LEN] = [(0, 0, 0).into(); STRIP_LEN];
    let mut t = 0.0;

    // Bring down the overall brightness of the LEDs.
    // The onboard LEDs are very bright, they still look pretty good when not as bright.
    // Reducing power might also be helpful for bad cables/connectors.
    let strip_brightness = 64u8; // Limit brightness to 64/255

    // Slow down timer by this factor (0.1 will result in 10 seconds)
    let animation_speed = 0.1;

    // Configure two pins as being I²C for the qwiic port
    // TODO: use qwiic port in example
    let _sda_pin = pins.sda.into_mode::<hal::gpio::FunctionI2C>();
    let _scl_pin = pins.scl.into_mode::<hal::gpio::FunctionI2C>();

    // These are implicitly used by the spi driver if they are in the correct mode
    // Don't need miso for the screen, it isn't connected
    let _spi_sclk = pins.sclk.into_mode::<gpio::FunctionSpi>();
    let _spi_mosi = pins.mosi.into_mode::<gpio::FunctionSpi>();

    let dc = pins.oled_dc.into_push_pull_output();
    let mut oled_reset = pins.oled_reset.into_push_pull_output();
    let cs = pins.oled_cs.into_push_pull_output();

    // Create an SPI driver instance for the SPI0 device
    let spi = Spi::<_, _, 8>::new(pac.SPI1);
    // Exchange the uninitialised SPI driver for an initialised one
    let spi = spi.init(
        &mut pac.RESETS,
        clocks.peripheral_clock.freq(),
        16_000_000u32.Hz(),
        &embedded_hal::spi::MODE_0,
    );

    let mut display: GraphicsMode<_> = Builder::new().connect_spi(spi, dc, cs).into();
    display.reset(&mut oled_reset, &mut delay).unwrap();
    display.init().unwrap();

    // Create a text style for drawing the font
    let text_style = MonoTextStyleBuilder::new()
        .font(&FONT_9X18_BOLD)
        .text_color(BinaryColor::On)
        .build();

    // Clear display before trying to print anything
    display.clear();
    display.flush().unwrap();

    // Set up the buzzer pins
    let mut speaker = pins.speaker.into_push_pull_output();
    let mut speaker_shutdown = pins.speaker_shutdown.into_push_pull_output();

    // Enable buzzer output
    speaker_shutdown.set_high().unwrap();
    let mut speaker_triggered: bool = false;
    loop {
        Text::with_baseline("Hello Rust!", Point::new(0, 16), text_style, Baseline::Top)
            .draw(&mut display)
            .unwrap();
        display.flush().unwrap();
        led_pin.toggle().unwrap();

        // Update the rainbow effect of the key backlight LEDS
        update_leds(t, &mut leds);

        // Here the magic happens and the `leds` buffer is written to the
        // ws2812 LEDs:
        ws.write(brightness(leds.iter().copied(), strip_brightness))
            .unwrap();

        // Wait a bit until calculating the next frame:
        delay.delay_ms(16); // ~60 FPS

        // Increase the time counter variable and make sure it
        // stays inbetween 0.0 to 1.0 range:
        t += (16.0 / 1000.0) * animation_speed;
        while t > 1.0 {
            t -= 1.0;
        }

        // Click the buzzer once if any key is pressed
        if buttons.iter().any(|key| key.is_low().unwrap()) {
            if !speaker_triggered {
                speaker.set_low().unwrap();
            }
            speaker_triggered = true;
        } else {
            speaker_triggered = false;
            speaker.set_high().unwrap();
        }
    }
}

fn update_leds(t: f32, leds: &mut [RGB<u8>; 12]) {
    let sin = adafruit_macropad::hal::rom_data::float_funcs::fsin::ptr();
    for (i, led) in leds.iter_mut().enumerate() {
        // An offset to give each LED:
        let hue_offs = i as f32 / 12.0;

        let sin_11 = sin((t + hue_offs) * 2.0 * core::f32::consts::PI);
        // Bring -1..1 sine range to 0..1 range:
        let sin_01 = (sin_11 + 1.0) * 0.5;

        let hue = 360.0 * sin_01;
        let sat = 1.0;
        let val = 1.0;

        let rgb = hsv2rgb_u8(hue, sat, val);
        *led = rgb.into();
    }
}

pub fn hsv2rgb(hue: f32, sat: f32, val: f32) -> (f32, f32, f32) {
    let c = val * sat;
    let v = (hue / 60.0) % 2.0 - 1.0;
    let v = if v < 0.0 { -v } else { v };
    let x = c * (1.0 - v);
    let m = val - c;
    let (r, g, b) = if hue < 60.0 {
        (c, x, 0.0)
    } else if hue < 120.0 {
        (x, c, 0.0)
    } else if hue < 180.0 {
        (0.0, c, x)
    } else if hue < 240.0 {
        (0.0, x, c)
    } else if hue < 300.0 {
        (x, 0.0, c)
    } else {
        (c, 0.0, x)
    };
    (r + m, g + m, b + m)
}

pub fn hsv2rgb_u8(h: f32, s: f32, v: f32) -> (u8, u8, u8) {
    let r = hsv2rgb(h, s, v);

    (
        (r.0 * 255.0) as u8,
        (r.1 * 255.0) as u8,
        (r.2 * 255.0) as u8,
    )
}
