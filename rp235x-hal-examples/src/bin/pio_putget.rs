//! This example tests RXFIFO reads/writes using PUTGET mode. GPIO25 is set high if the
//! test is successful and low if the test fails.
//!
//! If a LED is connected to that pin, like on a Pico board, the LED should turn on.
#![no_std]
#![no_main]

use rp235x_hal as hal;

use hal::gpio::{FunctionPio0, Pin};
use hal::pio::Buffers;
use hal::pio::PIOExt;
use hal::Sio;

// Ensure we halt the program on panic (if we don't mention this crate it won't
// be linked)
use panic_halt as _;

/// Tell the Boot ROM about our application
#[link_section = ".start_block"]
#[used]
pub static IMAGE_DEF: hal::block::ImageDef = hal::block::ImageDef::secure_exe();

/// Entry point to our bare-metal application.
///
/// The `#[hal::entry]` macro ensures the Cortex-M start-up code calls this function
/// as soon as all global variables and the spinlock are initialised.
#[hal::entry]
fn main() -> ! {
    let mut pac = hal::pac::Peripherals::take().unwrap();

    let sio = Sio::new(pac.SIO);
    let pins = hal::gpio::Pins::new(
        pac.IO_BANK0,
        pac.PADS_BANK0,
        sio.gpio_bank0,
        &mut pac.RESETS,
    );

    // configure LED pin for Pio0.
    let led: Pin<_, FunctionPio0, _> = pins.gpio25.into_function();
    // PIN id for use inside of PIO
    let led_pin_id = led.id().num;

    // Define a program to test RXFIFO reads/writes
    let program = pio::pio_asm!(
        "
            ; Set rxfifo[y] to the value of y
            set y, 3
        write_another_word:
            mov isr, y
            mov rxfifo[y], isr
            jmp y-- write_another_word

            ; Read and verify each rxfifo word
            set y, 3
        read_another_word:
            mov osr, rxfifo[y]
            mov x, osr
            jmp x != y, failure
            jmp y-- read_another_word

        success:
            set pins, 1
            jmp success

        failure:
            set pins, 0
            jmp failure
        "
    );

    // Initialize and start PIO
    let (mut pio, sm0, _, _, _) = pac.PIO0.split(&mut pac.RESETS);
    let installed = pio.install(&program.program).unwrap();
    let (mut sm, _, _) = hal::pio::PIOBuilder::from_installed_program(installed)
        .set_pins(led_pin_id, 1)
        .buffers(Buffers::RxPutGet)
        .build(sm0);
    sm.set_pindirs([(led_pin_id, hal::pio::PinDir::Output)]);
    sm.start();

    // PIO runs in background, independently from CPU
    loop {
        hal::arch::wfi();
    }
}

/// Program metadata for `picotool info`
#[link_section = ".bi_entries"]
#[used]
pub static PICOTOOL_ENTRIES: [hal::binary_info::EntryAddr; 5] = [
    hal::binary_info::rp_cargo_bin_name!(),
    hal::binary_info::rp_cargo_version!(),
    hal::binary_info::rp_program_description!(c"PIO PutGet Example"),
    hal::binary_info::rp_cargo_homepage_url!(),
    hal::binary_info::rp_program_build_attribute!(),
];

// End of file
