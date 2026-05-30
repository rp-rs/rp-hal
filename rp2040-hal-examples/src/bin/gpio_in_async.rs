//! # GPIO In Async Example
//!
//! This application demonstrates how to wait for GPIO pin event asynchronously an RP2040.
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

// Import required types & traits.
use embassy_executor::Executor;
use embedded_hal::digital::StatefulOutputPin;
use embedded_hal_async::digital::Wait;
use hal::{
    gpio,
    pac::{self, interrupt},
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

// Pin types quickly become very long!
// We'll create some type aliases using `type` to help with that

/// This pin will be our output - it will drive an LED if you run this on a Pico
type LedPin = gpio::Pin<gpio::bank0::Gpio25, gpio::FunctionSioOutput, gpio::PullNone>;

/// This pin will be our interrupt source.
/// It will trigger an interrupt if pulled to ground (via a switch or jumper wire)
type ButtonPin = gpio::Pin<gpio::bank0::Gpio22, gpio::FunctionSioInput, gpio::PullUp>;

/// Bind the interrupt handler with the peripheral
#[interrupt]
unsafe fn IO_IRQ_BANK0() {
    use hal::async_utils::AsyncPeripheral;
    ButtonPin::on_interrupt();
}

/// The function configures the RP2040 peripherals, then performs a single I²C
/// write to a fixed address.
#[embassy_executor::task]
async fn demo() {
    let mut pac = pac::Peripherals::take().unwrap();

    // Set up the watchdog driver - needed by the clock setup code
    let mut watchdog = hal::Watchdog::new(pac.WATCHDOG);

    // Configure the clocks
    let _clocks = hal::clocks::init_clocks_and_plls(
        XTAL_FREQ_HZ,
        pac.XOSC,
        pac.CLOCKS,
        pac.PLL_SYS,
        pac.PLL_USB,
        &mut pac.RESETS,
        &mut watchdog,
    )
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

    // Configure GPIO 25 as an output to drive our LED.
    // we can use reconfigure() instead of into_pull_up_input()
    // since the variable we're pushing it into has that type
    let mut led: LedPin = pins.gpio25.reconfigure();

    // Set up the GPIO pin that will be our input
    let mut in_pin: ButtonPin = pins.gpio22.reconfigure();

    // Unmask the IO_BANK0 IRQ so that the NVIC interrupt controller
    // will jump to the interrupt function when the interrupt occurs.
    // We do this last so that the interrupt can't go off while
    // it is in the middle of being configured
    unsafe {
        pac::NVIC::unmask(hal::pac::Interrupt::IO_IRQ_BANK0);
    }

    loop {
        // wait for button to be pressed down
        let _ = in_pin.wait_for_low().await;

        let _ = led.toggle();

        // wait for button to be released to enter next cycle
        let _ = in_pin.wait_for_high().await;
    }
}

/// Entry point to our bare-metal application.
#[rp2040_hal::entry]
fn main() -> ! {
    static EXECUTOR: StaticCell<Executor> = StaticCell::new();
    let executor = EXECUTOR.init(Executor::new());
    executor.run(|spawner| spawner.spawn(demo()).unwrap());
}
