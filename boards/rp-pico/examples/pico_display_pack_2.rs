//! # Pimoroni Display Pack 2.0 Example
//!
//! This example makes use of the [Pimoroni Display Pack 2.0](https://shop.pimoroni.com/products/pico-display-pack-2-0?variant=39374122582099)
//!
//! It will demonstrate how to draw on the screen using the embedded_graphics API
//!
//! See the `Cargo.toml` file for Copyright and license details.

#![no_std]
#![no_main]

// The macro for our start-up function
use rp_pico::entry;

// Time handling traits
use fugit::RateExtU32;

// Ensure we halt the program on panic (if we don't mention this crate it won't
// be linked)
use panic_halt as _;

// Pull in any important traits
use rp_pico::hal::prelude::*;

// A shorter alias for the Peripheral Access Crate, which provides low-level
// register access
use rp_pico::hal::pac;

// A shorter alias for the Hardware Abstraction Layer, which provides
// higher-level drivers.
use rp_pico::hal;

// ST7789 driver
use st7789::{Orientation, ST7789};

// SPI display interface
use display_interface_spi::SPIInterface;

// Graphics drawing utilities
use embedded_graphics::pixelcolor::Rgb565;
use embedded_graphics::prelude::*;
use embedded_graphics::primitives::*;

// GPIO traits
use embedded_hal::digital::v2::OutputPin;
use embedded_hal::PwmPin;

/// External high-speed crystal on the Raspberry Pi Pico board is 12 MHz. Adjust
/// if your board has a different frequency
const XTAL_FREQ_HZ: u32 = 12_000_000u32;

#[entry]
fn main() -> ! {
    // Grab our singleton objects
    let mut pac = pac::Peripherals::take().unwrap();
    let core = pac::CorePeripherals::take().unwrap();

    // Set up the watchdog driver - needed by the clock setup code
    let mut watchdog = hal::Watchdog::new(pac.WATCHDOG);

    // Configure the clocks
    let clocks = hal::clocks::init_clocks_and_plls(
        XTAL_FREQ_HZ,
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

    // The single-cycle I/O block controls our GPIO pins
    let sio = hal::Sio::new(pac.SIO);

    // Set the pins up according to their function on this particular board
    let pins = rp_pico::Pins::new(
        pac.IO_BANK0,
        pac.PADS_BANK0,
        sio.gpio_bank0,
        &mut pac.RESETS,
    );

    // Init PWMs
    let mut pwm_slices = hal::pwm::Slices::new(pac.PWM, &mut pac.RESETS);

    // Configure and enable PWMs
    let pwm3 = &mut pwm_slices.pwm3;
    pwm3.set_ph_correct();
    pwm3.enable();
    let pwm4 = &mut pwm_slices.pwm4;
    pwm4.set_ph_correct();
    pwm4.enable();

    // Configure button inputs
    // 12 - A button
    let _a_pin = pins.gpio12.into_pull_up_input();
    // 13 - B button
    let _b_pin = pins.gpio13.into_pull_up_input();
    // 14 - X button
    let _x_pin = pins.gpio14.into_pull_up_input();
    // 15 - Y button
    let _y_pin = pins.gpio15.into_pull_up_input();

    // 6 (PWM 3A) - LED red
    let r_channel = &mut pwm3.channel_a;
    r_channel.output_to(pins.gpio6);
    r_channel.set_inverted();
    r_channel.set_duty(0);
    // 7 (PWM 3B) - LED green
    let g_channel = &mut pwm3.channel_b;
    g_channel.output_to(pins.gpio7);
    g_channel.set_inverted();
    g_channel.set_duty(0);
    // 8 (PWM 4A) - LED blue
    let b_channel = &mut pwm4.channel_a;
    b_channel.output_to(pins.gpio8);
    b_channel.set_inverted();
    b_channel.set_duty(0);

    // 20 - LCD backlight
    pins.gpio20.into_push_pull_output().set_high().unwrap();

    // 22 - LCD hard reset
    let mut rst_pin = pins.gpio22.into_push_pull_output();
    rst_pin.set_low().unwrap();

    // Setup SPI
    let spi_cs = pins.gpio17.into_push_pull_output();
    let spi_miso = pins.gpio16.into_push_pull_output();
    let _spi_sclk = pins.gpio18.into_mode::<hal::gpio::FunctionSpi>();
    let _spi_mosi = pins.gpio19.into_mode::<hal::gpio::FunctionSpi>();

    let spi = hal::Spi::<_, _, 8>::new(pac.SPI0);

    // Exchange the uninitialised SPI driver for an initialised one
    let spi = spi.init(
        &mut pac.RESETS,
        clocks.peripheral_clock.freq(),
        16_000_000u32.Hz(),
        &embedded_hal::spi::MODE_0,
    );

    // display interface abstraction from SPI and DC
    let di = SPIInterface::new(spi, spi_miso, spi_cs);

    // create driver
    let mut display = ST7789::new(di, rst_pin, 320, 240);

    // initialize
    display.init(&mut delay).unwrap();

    // set default orientation
    display.set_orientation(Orientation::Landscape).unwrap();

    // create a filled circle
    let circle =
        Circle::new(Point::new(100, 60), 120).into_styled(PrimitiveStyle::with_fill(Rgb565::RED));

    // draw the circle on a black background
    display.clear(Rgb565::BLACK).unwrap();
    circle.draw(&mut display).unwrap();

    loop {
        cortex_m::asm::wfi();
    }
}
