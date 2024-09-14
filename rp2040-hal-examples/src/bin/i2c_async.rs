//! # I²C Example
//!
//! This application demonstrates how to talk to I²C devices with an RP2040.
//! in an Async environment.
//!
//! It may need to be adapted to your particular board layout and/or pin assignment.
//!
//! See the top-level `README.md` file for Copyright and license details.

#![no_std]
#![no_main]

// Ensure we halt the program on panic (if we don't mention this crate it won't
// be linked)
use panic_halt as _;

// Alias for our HAL crate
use rp2040_hal as hal;

// Some traits we need
use hal::{
    fugit::RateExtU32,
    gpio::bank0::{Gpio20, Gpio21},
    i2c::Controller,
    I2C,
};

// Import required types & traits.
use embassy_executor::Executor;
use embedded_hal_async::i2c::I2c;
use hal::{
    gpio::{FunctionI2C, Pin, PullUp},
    pac::{self, interrupt},
    Clock,
};
use static_cell::StaticCell;

/// The linker will place this boot block at the start of our program image. We
/// need this to help the ROM bootloader get our code up and running.
/// Note: This boot block is not necessary when using a rp-hal based BSP
/// as the BSPs already perform this step.
#[link_section = ".boot2"]
#[used]
pub static BOOT2: [u8; 256] = rp2040_boot2::BOOT_LOADER_GENERIC_03H;

/// External high-speed crystal on the Raspberry Pi Pico board is 12 MHz. Adjust
/// if your board has a different frequency
const XTAL_FREQ_HZ: u32 = 12_000_000u32;

/// Bind the interrupt handler with the peripheral
#[interrupt]
unsafe fn I2C0_IRQ() {
    use hal::async_utils::AsyncPeripheral;
    I2C::<pac::I2C0, (Gpio20, Gpio21), Controller>::on_interrupt();
}

/// The function configures the RP2040 peripherals, then performs a single I²C
/// write to a fixed address.
#[embassy_executor::task]
async fn demo() {
    let mut pac = pac::Peripherals::take().unwrap();

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

    // The single-cycle I/O block controls our GPIO pins
    let sio = hal::Sio::new(pac.SIO);

    // Set the pins to their default state
    let pins = hal::gpio::Pins::new(
        pac.IO_BANK0,
        pac.PADS_BANK0,
        sio.gpio_bank0,
        &mut pac.RESETS,
    );

    // Configure two pins as being I²C, not GPIO
    let sda_pin: Pin<_, FunctionI2C, PullUp> = pins.gpio20.reconfigure();
    let scl_pin: Pin<_, FunctionI2C, PullUp> = pins.gpio21.reconfigure();

    // Create the I²C drive, using the two pre-configured pins. This will fail
    // at compile time if the pins are in the wrong mode, or if this I²C
    // peripheral isn't available on these pins!
    let mut i2c = hal::I2C::new_controller(
        pac.I2C0,
        sda_pin,
        scl_pin,
        400.kHz(),
        &mut pac.RESETS,
        clocks.system_clock.freq(),
    );

    // Unmask the interrupt in the NVIC to let the core wake up & enter the interrupt handler.
    // Each core has its own NVIC so these needs to executed from the core where the IRQ are
    // expected.
    unsafe {
        pac::NVIC::unpend(hal::pac::Interrupt::I2C0_IRQ);
        pac::NVIC::unmask(hal::pac::Interrupt::I2C0_IRQ);
    }

    // Asynchronously write three bytes to the I²C device with 7-bit address 0x2C
    i2c.write(0x76u8, &[1, 2, 3]).await.unwrap();

    // Demo finish - just loop until reset
    core::future::pending().await
}

/// Entry point to our bare-metal application.
#[rp2040_hal::entry]
fn main() -> ! {
    static EXECUTOR: StaticCell<Executor> = StaticCell::new();
    let executor = EXECUTOR.init(Executor::new());
    executor.run(|spawner| spawner.spawn(demo()).unwrap());
}
