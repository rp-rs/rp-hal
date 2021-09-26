//! Blinks the LED on a Pico board, using a PIO program
//!
//! This will blink an LED attached to GP25, which is the pin the Pico uses for the on-board LED.
#![no_std]
#![no_main]

use cortex_m_rt::entry;
use hal::gpio::{FunctionPio0, Pin};
use hal::pac;
use hal::sio::Sio;
use panic_halt as _;
use rp2040_hal as hal;

#[link_section = ".boot2"]
#[used]
pub static BOOT2: [u8; 256] = rp2040_boot2::BOOT_LOADER;

#[entry]
fn main() -> ! {
    let mut pac = pac::Peripherals::take().unwrap();

    let sio = Sio::new(pac.SIO);
    let pins = hal::gpio::Pins::new(
        pac.IO_BANK0,
        pac.PADS_BANK0,
        sio.gpio_bank0,
        &mut pac.RESETS,
    );

    // configure LED pin for Pio0.
    let _led: Pin<_, FunctionPio0> = pins.gpio25.into_mode();
    // PIN id for use inside of PIO
    let led_pin_id = 25;

    // Define some simple PIO program.
    const MAX_DELAY: u8 = 31;
    let mut a = pio::Assembler::<32>::new();
    let mut wrap_target = a.label();
    let mut wrap_source = a.label();
    // Set pin as Out
    a.set(pio::SetDestination::PINDIRS, 1);
    // Define begin of program loop
    a.bind(&mut wrap_target);
    // Set pin low
    a.set_with_delay(pio::SetDestination::PINS, 0, MAX_DELAY);
    // Set pin high
    a.set_with_delay(pio::SetDestination::PINS, 1, MAX_DELAY);
    // Define end of program loop
    a.bind(&mut wrap_source);
    // The labels wrap_target and wrap_source, as set above,
    // define a loop which is executed repeatedly by the PIO
    // state machine.
    let program = a.assemble_with_wrap(wrap_source, wrap_target);

    // Initialize and start PIO
    let pio = rp2040_hal::pio::PIO::new(pac.PIO0, &mut pac.RESETS);
    let sm = &pio.state_machines()[0];
    let div = 0f32; // as slow as possible (0 is interpreted as 65536)
    rp2040_hal::pio::PIOBuilder::default()
        .with_program(&program)
        .set_pins(led_pin_id, 1)
        .clock_divisor(div)
        .build(&pio, sm)
        .unwrap();

    // PIO runs in background, independently from CPU
    #[allow(clippy::empty_loop)]
    loop {}
}
