//! # Architecture Flip example
//!
//! This application demonstrates running both Arm and RISC-V code on the same
//! chip, using partitions.
//!
//! You need a partition table with one partition for Arm (marked NO-BOOT for
//! RISC-V) and one partition for RISC-V (marked NO-BOOT for Arm).
//!
//! It will run for a few seconds, and then reboot into the other. You can tell
//! the difference because they have different blink rates.
//!
//! It may need to be adapted to your particular board layout and/or pin
//! assignment.
//!
//! See the `Cargo.toml` file for Copyright and license details.

#![no_std]
#![no_main]

// Ensure we halt the program on panic (if we don't mention this crate it won't
// be linked)
use panic_halt as _;

// Alias for our HAL crate
use rp235x_hal as hal;

// Some things we need
use embedded_hal::delay::DelayNs;
use embedded_hal::digital::OutputPin;

/// Tell the Boot ROM about our application
#[link_section = ".start_block"]
#[used]
pub static IMAGE_DEF: hal::block::ImageDef = hal::block::ImageDef::secure_exe();

/// External high-speed crystal on the Raspberry Pi Pico 2 board is 12 MHz.
/// Adjust if your board has a different frequency
const XTAL_FREQ_HZ: u32 = 12_000_000u32;

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
    .unwrap();

    let mut timer = hal::Timer::new_timer0(pac.TIMER0, &mut pac.RESETS, &clocks);

    // The single-cycle I/O block controls our GPIO pins
    let sio = hal::Sio::new(pac.SIO);

    // Set the pins to their default state
    let pins = hal::gpio::Pins::new(
        pac.IO_BANK0,
        pac.PADS_BANK0,
        sio.gpio_bank0,
        &mut pac.RESETS,
    );

    #[cfg(target_arch = "arm")]
    let (delay_ms, arch) = (500, hal::reboot::RebootArch::Riscv);

    #[cfg(not(target_arch = "arm"))]
    let (delay_ms, arch) = (250, hal::reboot::RebootArch::Arm);

    // Configure GPIO25 as an output
    let mut led_pin = pins.gpio25.into_push_pull_output();
    for _ in 0..10 {
        led_pin.set_high().unwrap();
        timer.delay_ms(delay_ms);
        led_pin.set_low().unwrap();
        timer.delay_ms(delay_ms);
    }

    // Do an asynchronous reset into the bootloader, but flipping the architecture
    hal::reboot::reboot(hal::reboot::RebootKind::Normal, arch);
}

/// Program metadata for `picotool info`
#[link_section = ".bi_entries"]
#[used]
pub static PICOTOOL_ENTRIES: [hal::binary_info::EntryAddr; 5] = [
    hal::binary_info::rp_cargo_bin_name!(),
    hal::binary_info::rp_cargo_version!(),
    hal::binary_info::rp_program_description!(c"Architecture Flip Example"),
    hal::binary_info::rp_cargo_homepage_url!(),
    hal::binary_info::rp_program_build_attribute!(),
];

// End of file
