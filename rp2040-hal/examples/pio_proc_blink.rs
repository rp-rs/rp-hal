//! This example toggles the GPIO25 pin, using a PIO program compiled via pio_proc::pio!().
//!
//! If a LED is connected to that pin, like on a Pico board, the LED should blink.
#![no_std]
#![no_main]

use cortex_m_rt::entry;
use hal::gpio::{FunctionPio0, Pin};
use hal::pac;
use hal::pio::PIOExt;
use hal::Sio;
use panic_halt as _;
use rp2040_hal as hal;

#[link_section = ".boot2"]
#[used]
pub static BOOT2: [u8; 256] = rp2040_boot2::BOOT_LOADER_W25Q080;

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
    let program = pio_proc::pio_asm!(
        ".wrap_target",
        "set pins, 1 [31]",
        "set pins, 0 [31]",
        ".wrap"
    );

    // Initialize and start PIO
    let (mut pio, sm0, _, _, _) = pac.PIO0.split(&mut pac.RESETS);
    let installed = pio.install(&program.program).unwrap();
    let div = 0f32; // as slow as possible (0 is interpreted as 65536)
    let (mut sm, _, _) = rp2040_hal::pio::PIOBuilder::from_program(installed)
        .set_pins(led_pin_id, 1)
        .clock_divisor(div)
        .build(sm0);
    // The GPIO pin needs to be configured as an output.
    sm.set_pindirs([(led_pin_id, hal::pio::PinDir::Output)]);
    sm.start();

    // PIO runs in background, independently from CPU
    #[allow(clippy::empty_loop)]
    loop {}
}
