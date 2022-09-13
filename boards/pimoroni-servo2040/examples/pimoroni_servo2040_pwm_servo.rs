//! # Pimoroni Servo2040 PWM Micro Servo Example
//!
//! Moves the micro servo on a Servo2040 board using the PWM peripheral.
//!
//! This will move in different positions the motor attached to GP0.
#![no_std]
#![no_main]

// GPIO traits
use embedded_hal::timer::CountDown;
use embedded_hal::PwmPin;

// Traits for converting integers to amounts of time
use fugit::ExtU64;

// Ensure we halt the program on panic (if we don't mention this crate it won't
// be linked)
use panic_halt as _;

// A shorter alias for the Peripheral Access Crate, which provides low-level
// register access
use pimoroni_servo2040::hal::pac;

// A shorter alias for the Hardware Abstraction Layer, which provides
// higher-level drivers.
use pimoroni_servo2040::hal;

/// Number of microseconds for the pwm signal period.
const PERIOD_US: u32 = 20_000;
/// Max resolution for the pwm signal.
const TOP: u32 = u16::MAX;

#[pimoroni_servo2040::entry]
fn main() -> ! {
    let mut pac = pac::Peripherals::take().unwrap();
    let mut watchdog = hal::Watchdog::new(pac.WATCHDOG);

    let sio = hal::Sio::new(pac.SIO);

    let _clocks = hal::clocks::init_clocks_and_plls(
        pimoroni_servo2040::XOSC_CRYSTAL_FREQ,
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

    let pins = pimoroni_servo2040::Pins::new(
        pac.IO_BANK0,
        pac.PADS_BANK0,
        sio.gpio_bank0,
        &mut pac.RESETS,
    );

    let pwm_slices = hal::pwm::Slices::new(pac.PWM, &mut pac.RESETS);

    const MIN_PULSE: u16 = 1000;
    const MID_PULSE: u16 = 1500;
    const MAX_PULSE: u16 = 2000;

    let mut pwm: hal::pwm::Slice<_, _> = pwm_slices.pwm0;
    pwm.set_ph_correct();
    // 50Hz
    pwm.set_div_int(38);
    pwm.set_div_frac(3);
    pwm.set_top(TOP);
    pwm.enable();

    // Output channel A on PWM0 to the GPIO0/servo1 pin
    let mut channel_a = pwm.channel_a;
    let _channel_a_pin = channel_a.output_to(pins.servo1);
    let movement_delay = 400.millis();

    // Infinite loop, moving micro servo from one position to another.
    // You may need to adjust the pulse width since several servos from
    // different manufacturers respond differently.
    loop {
        // move to 0°
        channel_a.set_duty(us_to_duty(MID_PULSE));
        count_down.start(movement_delay);
        let _ = nb::block!(count_down.wait());

        // 0° to 90°
        channel_a.set_duty(us_to_duty(MAX_PULSE));
        count_down.start(movement_delay);
        let _ = nb::block!(count_down.wait());

        // 90° to 0°
        channel_a.set_duty(us_to_duty(MID_PULSE));
        count_down.start(movement_delay);
        let _ = nb::block!(count_down.wait());

        // 0° to -90°
        channel_a.set_duty(us_to_duty(MIN_PULSE));
        count_down.start(movement_delay);
        let _ = nb::block!(count_down.wait());
    }
}

/// Convert microseconds to duty value.
///
/// This function uses the constants TOP and PERIOD_US defined at the top of the file.
fn us_to_duty(us: u16) -> u16 {
    // Do math in u32 so we maintain higher precision. If we do math in u16, we need to divide first
    // and lose some precision when truncating the remainder.
    (TOP * us as u32 / PERIOD_US) as u16
}
