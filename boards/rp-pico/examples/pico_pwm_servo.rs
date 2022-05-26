//! # Pico PWM Micro Servo Example
//!
//! Moves the micro servo on a Pico board using the PWM peripheral.
//!
//! This will move in different positions the motor attached to GP1.
//!
//! See the `Cargo.toml` file for Copyright and license details.

#![no_std]
#![no_main]

// The macro for our start-up function
use cortex_m_rt::entry;

use cortex_m::prelude::*;

// GPIO traits
use embedded_hal::PwmPin;

// Traits for converting integers to amounts of time
use embedded_time::duration::Extensions;

// Ensure we halt the program on panic (if we don't mention this crate it won't
// be linked)
use panic_halt as _;

// A shorter alias for the Peripheral Access Crate, which provides low-level
// register access
use rp_pico::hal::pac;

// A shorter alias for the Hardware Abstraction Layer, which provides
// higher-level drivers.
use rp_pico::hal;

/// Entry point to our bare-metal application.
///
/// The `#[entry]` macro ensures the Cortex-M start-up code calls this function
/// as soon as all global variables are initialised.
///
/// The function configures the RP2040 peripherals, then fades the LED in an
/// infinite loop.
#[entry]
fn main() -> ! {
    // Grab our singleton objects
    let mut pac = pac::Peripherals::take().unwrap();

    // Set up the watchdog driver - needed by the clock setup code
    let mut watchdog = hal::Watchdog::new(pac.WATCHDOG);

    // Configure the clocks
    //
    // The default is to generate a 125 MHz system clock
    let _clocks = hal::clocks::init_clocks_and_plls(
        rp_pico::XOSC_CRYSTAL_FREQ,
        pac.XOSC,
        pac.CLOCKS,
        pac.PLL_SYS,
        pac.PLL_USB,
        &mut pac.RESETS,
        &mut watchdog,
    )
    .ok()
    .unwrap();

    // Configure the Timer peripheral in count-down mode
    let timer = hal::Timer::new(pac.TIMER, &mut pac.RESETS);
    let mut count_down = timer.count_down();

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

    // Configure PWM0
    let pwm = &mut pwm_slices.pwm0;
    pwm.set_ph_correct();
    pwm.set_div_int(20u8); // 50 hz
    pwm.enable();

    // Output channel B on PWM0 to the GPIO1 pin
    let channel = &mut pwm.channel_b;
    channel.output_to(pins.gpio1);

    // Infinite loop, moving micro servo from one position to another.
    // You may need to adjust the pulse width since several servos from
    // different manufacturers respond differently.
    loop {
        // move to 0°
        channel.set_duty(2500);
        count_down.start(400.milliseconds());
        let _ = nb::block!(count_down.wait());

        // 0° to 90°
        channel.set_duty(3930);
        count_down.start(400.milliseconds());
        let _ = nb::block!(count_down.wait());

        // 90° to 180°
        channel.set_duty(7860);
        count_down.start(400.milliseconds());
        let _ = nb::block!(count_down.wait());

        // 180° to 90°
        channel.set_duty(3930);
        count_down.start(400.milliseconds());
        let _ = nb::block!(count_down.wait());
    }
}

// End of file
