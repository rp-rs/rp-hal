//!  all the functionality on a Adafruit MacroPad Showcase board
//!
//! This will showcase all the functionality on a Adafruit MacroPad board
#![no_std]
#![no_main]

use adafruit_macropad::{
    hal::{
        self as hal,
        clocks::{init_clocks_and_plls, Clock},
        gpio, pac,
        pio::PIOExt,
        watchdog::Watchdog,
        Sio, Spi, Timer, I2C,
    },
    Pins, XOSC_CRYSTAL_FREQ,
};
use cortex_m_rt::entry;
use embedded_hal::digital::v2::OutputPin;
use embedded_time::rate::*;
use panic_halt as _;

// Import useful traits to handle the ws2812 LEDs:
use smart_leds::{brightness, SmartLedsWrite, RGB8};

// Import the actual crate to handle the Ws2812 protocol:
use ws2812_pio::Ws2812;

// Currently 3 consecutive LEDs are driven by this example
// to keep the power draw compatible with USB:
const STRIP_LEN: usize = 12;

// For string formatting.
use core::fmt::Write;
// For in the graphics drawing utilities like the font
// and the drawing routines:
use embedded_graphics::{
    mono_font::{ascii::FONT_9X18_BOLD, MonoTextStyleBuilder},
    pixelcolor::BinaryColor,
    prelude::*,
    text::{Baseline, Text},
};

// The display driver:
use ssd1306::size::DisplaySize128x64;
use ssd1306::{mode::DisplayConfig, prelude::SPIInterfaceNoCS, Ssd1306};
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

    let mut delay = cortex_m::delay::Delay::new(core.SYST, clocks.system_clock.freq().integer());
    // Setup a delay for the LED blink signals:
    // let mut frame_delay =
    // cortex_m::delay::Delay::new(core.SYST, clocks.system_clock.freq().integer());
    // Create a count down timer for the Ws2812 instance:
    let timer = Timer::new(pac.TIMER, &mut pac.RESETS);
    // Import the `sin` function for a smooth hue animation from the
    // Pico rp2040 ROM:
    let sin = adafruit_macropad::hal::rom_data::float_funcs::fsin::ptr();

    let sio = Sio::new(pac.SIO);
    let pins = Pins::new(
        pac.IO_BANK0,
        pac.PADS_BANK0,
        sio.gpio_bank0,
        &mut pac.RESETS,
    );
    let mut led_pin = pins.led.into_push_pull_output();

    // Split the PIO state machine 0 into individual objects, so that
    // Ws2812 can use it:
    let (mut pio, sm0, _, _, _) = pac.PIO0.split(&mut pac.RESETS);

    // Instanciate a Ws2812 LED strip:
    let mut ws = Ws2812::new(
        // Use neopixel pin (19) on the Adafruit macropad (which is GPIO19 of the rp2040 chip)
        // for the LED data output:
        pins.neopixel.into_mode(),
        &mut pio,
        sm0,
        clocks.peripheral_clock.freq(),
        timer.count_down(),
    );

    let mut leds: [RGB8; STRIP_LEN] = [(0, 0, 0).into(); STRIP_LEN];
    let mut t = 0.0;

    // Bring down the overall brightness of the LEDS
    // Original code here was worried about power consumption
    // my measurements show around 300ma for 12 LEDs at full brightness. we should be good.
    let strip_brightness = 64u8; // Limit brightness to 64/256

    // Slow down timer by this factor (0.1 will result in 10 seconds):
    let animation_speed = 0.1;

    // Configure two pins as being I²C for the qwiic port
    let sda_pin = pins.sda.into_mode::<hal::gpio::FunctionI2C>();
    let scl_pin = pins.scl.into_mode::<hal::gpio::FunctionI2C>();

    // These are implicitly used by the spi driver if they are in the correct mode
    let _spi_sclk = pins.sclk.into_mode::<gpio::FunctionSpi>();
    let _spi_mosi = pins.mosi.into_mode::<gpio::FunctionSpi>();
    let _spi_miso = pins.miso.into_mode::<gpio::FunctionSpi>();
    // let spi_cs = pins.gpio5.into_push_pull_output();

    let dc = pins.oled_dc.into_push_pull_output();
    let oled_reset = pins.oled_reset.into_push_pull_output();
    // Create an SPI driver instance for the SPI0 device
    let spi = Spi::<_, _, 8>::new(pac.SPI1);
    // Exchange the uninitialised SPI driver for an initialised one
    let spi = spi.init(
        &mut pac.RESETS,
        clocks.peripheral_clock.freq(),
        16_000_000u32.Hz(),
        &embedded_hal::spi::MODE_0,
    );

    let interface = display_interface_spi::SPIInterfaceNoCS::new(spi, dc);
    let mut display = Ssd1306::new(interface, DisplaySize128x64, DisplayRotation::Rotate0)
        .into_buffered_graphics_mode();
        // reset<RST, DELAY, PinE>
    display.reset(&mut rst, &mut delay).unwrap();
    display.init().unwrap();
    display.init().unwrap();

    // // Create a text style for drawing the font:
    // let text_style = MonoTextStyleBuilder::new()
    //     .font(&FONT_9X18_BOLD)
    //     .text_color(BinaryColor::On)
    //     .build();

    let mut count = 0;
    let mut buf = FmtBuf::new();
    loop {
        buf.reset();
        // Format some text into a static buffer:
        write!(&mut buf, "counter: {}", count).unwrap();
        count += 1;

        // // Empty the display:
        // display.clear();

        // // Draw 3 lines of text:
        // Text::with_baseline("Hello world!", Point::zero(), text_style, Baseline::Top)
        //     .draw(&mut display)
        //     .unwrap();

        // Text::with_baseline("Hello Rust!", Point::new(0, 16), text_style, Baseline::Top)
        //     .draw(&mut display)
        //     .unwrap();

        // Text::with_baseline(buf.as_str(), Point::new(0, 32), text_style, Baseline::Top)
        //     .draw(&mut display)
        //     .unwrap();

        // display.flush().unwrap();
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

/// This is a very simple buffer to pre format a short line of text
/// limited arbitrarily to 64 bytes.
struct FmtBuf {
    buf: [u8; 64],
    ptr: usize,
}

impl FmtBuf {
    fn new() -> Self {
        Self {
            buf: [0; 64],
            ptr: 0,
        }
    }

    fn reset(&mut self) {
        self.ptr = 0;
    }

    fn as_str(&self) -> &str {
        core::str::from_utf8(&self.buf[0..self.ptr]).unwrap()
    }
}

impl core::fmt::Write for FmtBuf {
    fn write_str(&mut self, s: &str) -> core::fmt::Result {
        let rest_len = self.buf.len() - self.ptr;
        let len = if rest_len < s.len() {
            rest_len
        } else {
            s.len()
        };
        self.buf[self.ptr..(self.ptr + len)].copy_from_slice(&s.as_bytes()[0..len]);
        self.ptr += len;
        Ok(())
    }
}
