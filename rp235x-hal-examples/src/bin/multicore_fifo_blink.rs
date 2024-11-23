//! # Multicore FIFO + GPIO 'Blinky' Example
//!
//! This application demonstrates FIFO communication between the CPU cores on the rp235x.
//! Core 0 will calculate and send a delay value to Core 1, which will then wait that long
//! before toggling the LED.
//! Core 0 will wait for Core 1 to complete this task and send an acknowledgement value.
//!
//! It may need to be adapted to your particular board layout and/or pin assignment.
//!
//! See the `Cargo.toml` file for Copyright and license details.

#![no_std]
#![no_main]

// Alias for our HAL crate
use rp235x_hal as hal;

// Some things we need
use embedded_hal::digital::StatefulOutputPin;
use hal::clocks::Clock;
use hal::multicore::{Multicore, Stack};
use hal::sio::Sio;

// Ensure we halt the program on panic (if we don't mention this crate it won't
// be linked)
use panic_halt as _;

/// Tell the Boot ROM about our application
#[link_section = ".start_block"]
#[used]
pub static IMAGE_DEF: hal::block::ImageDef = hal::block::ImageDef::secure_exe();

/// External high-speed crystal on the Raspberry Pi Pico 2 board is 12 MHz.
/// Adjust if your board has a different frequency
const XTAL_FREQ_HZ: u32 = 12_000_000u32;

/// Value to indicate that Core 1 has completed its task
const CORE1_TASK_COMPLETE: u32 = 0xEE;

/// Stack for core 1
///
/// Core 0 gets its stack via the normal route - any memory not used by static values is
/// reserved for stack and initialised by cortex-m-rt.
/// To get the same for Core 1, we would need to compile everything separately and
/// modify the linker file for both programs, and that's quite annoying.
/// So instead, core1.spawn takes a [usize] which gets used for the stack.
/// NOTE: We use the `Stack` struct here to ensure that it has 32-byte alignment, which allows
/// the stack guard to take up the least amount of usable RAM.
static CORE1_STACK: Stack<4096> = Stack::new();

fn core1_task(sys_freq: u32) -> ! {
    let mut pac = unsafe { hal::pac::Peripherals::steal() };
    let core = unsafe { cortex_m::Peripherals::steal() };

    let mut sio = Sio::new(pac.SIO);
    let pins = hal::gpio::Pins::new(
        pac.IO_BANK0,
        pac.PADS_BANK0,
        sio.gpio_bank0,
        &mut pac.RESETS,
    );

    let mut led_pin = pins.gpio25.into_push_pull_output();
    let mut delay = cortex_m::delay::Delay::new(core.SYST, sys_freq);
    loop {
        let input = sio.fifo.read();
        if let Some(word) = input {
            delay.delay_ms(word);
            led_pin.toggle().unwrap();
            sio.fifo.write_blocking(CORE1_TASK_COMPLETE);
        };
    }
}

/// Entry point to our bare-metal application.
///
/// The `#[hal::entry]` macro ensures the Cortex-M start-up code calls this function
/// as soon as all global variables and the spinlock are initialised.
///
/// The function configures the rp235x peripherals, then toggles a GPIO pin in
/// an infinite loop. If there is an LED connected to that pin, it will blink.
#[hal::entry]
fn main() -> ! {
    // Grab our singleton objects
    let mut pac = hal::pac::Peripherals::take().unwrap();

    // Set up the watchdog driver - needed by the clock setup code
    let mut watchdog = hal::watchdog::Watchdog::new(pac.WATCHDOG);

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
    .unwrap();

    let sys_freq = clocks.system_clock.freq().to_Hz();

    // The single-cycle I/O block controls our GPIO pins
    let mut sio = hal::sio::Sio::new(pac.SIO);

    let mut mc = Multicore::new(&mut pac.PSM, &mut pac.PPB, &mut sio.fifo);
    let cores = mc.cores();
    let core1 = &mut cores[1];
    let _test = core1.spawn(CORE1_STACK.take().unwrap(), move || core1_task(sys_freq));

    /// How much we adjust the LED period every cycle
    const LED_PERIOD_INCREMENT: i32 = 2;

    /// The minimum LED toggle interval we allow for.
    const LED_PERIOD_MIN: i32 = 0;

    /// The maximum LED toggle interval period we allow for. Keep it reasonably short so it's easy to see.
    const LED_PERIOD_MAX: i32 = 100;

    // Our current LED period. It starts at the shortest period, which is the highest blink frequency
    let mut led_period: i32 = LED_PERIOD_MIN;

    // The direction we're incrementing our LED period.
    // Since we start at the minimum value, start by counting up
    let mut count_up = true;

    loop {
        if count_up {
            // Increment our period
            led_period += LED_PERIOD_INCREMENT;

            // Change direction of increment if we hit the limit
            if led_period > LED_PERIOD_MAX {
                led_period = LED_PERIOD_MAX;
                count_up = false;
            }
        } else {
            // Decrement our period
            led_period -= LED_PERIOD_INCREMENT;

            // Change direction of increment if we hit the limit
            if led_period < LED_PERIOD_MIN {
                led_period = LED_PERIOD_MIN;
                count_up = true;
            }
        }

        // It should not be possible for led_period to go negative, but let's ensure that.
        if led_period < 0 {
            led_period = 0;
        }

        // Send the new delay time to Core 1. We convert it
        sio.fifo.write(led_period as u32);

        // Sleep until Core 1 sends a message to tell us it is done
        let ack = sio.fifo.read_blocking();
        if ack != CORE1_TASK_COMPLETE {
            // In a real application you might want to handle the case
            // where the CPU sent the wrong message - we're going to
            // ignore it here.
        }
    }
}

/// Program metadata for `picotool info`
#[link_section = ".bi_entries"]
#[used]
pub static PICOTOOL_ENTRIES: [hal::binary_info::EntryAddr; 5] = [
    hal::binary_info::rp_cargo_bin_name!(),
    hal::binary_info::rp_cargo_version!(),
    hal::binary_info::rp_program_description!(c"Multicore FIFO Example"),
    hal::binary_info::rp_cargo_homepage_url!(),
    hal::binary_info::rp_program_build_attribute!(),
];

// End of file
