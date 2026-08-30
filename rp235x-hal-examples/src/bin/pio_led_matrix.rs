//! This example drives a LED matrix consisting of 8x8 LEDs with the following
//! configuration:
//!
//! - LED anodes are wired to pins 4..=11, with appropriate resistors.
//! - LED cathodes are wired to the outputs of a 74HC164 shift register,
//!   gated through a 2N7000 NFET transistor (this is important: if you do
//!   not use a transistor, you'll have to invert the sideset data bits
//!   in the PIO program (and you may fry your 74HC164)).
//! - Pin 0 is the clock to the 74HC164, and Pin 1 is the value to be
//!   shifted into it.
//!
//! The display shown by the LEDs is controlled by writing bits into the RX
//! buffer, which is configured as random-access memory instead of as a FIFO.
#![no_std]
#![no_main]

use rp235x_hal as hal;

use hal::gpio::FunctionPio0;
use hal::pio::PIOExt;
use hal::Sio;

use embedded_hal::delay::DelayNs;

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

/// Entry point to our bare-metal application.
///
/// The `#[hal::entry]` macro ensures the Cortex-M start-up code calls this function
/// as soon as all global variables and the spinlock are initialised.
#[hal::entry]
fn main() -> ! {
    let mut pac = hal::pac::Peripherals::take().unwrap();

    // Set up the watchdog driver - needed by the clock setup code
    let mut watchdog = hal::Watchdog::new(pac.WATCHDOG);

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

    let sio = Sio::new(pac.SIO);
    let pins = hal::gpio::Pins::new(
        pac.IO_BANK0,
        pac.PADS_BANK0,
        sio.gpio_bank0,
        &mut pac.RESETS,
    );

    let mut timer = hal::Timer::new_timer0(pac.TIMER0, &mut pac.RESETS, &clocks);

    let shift_clk = pins.gpio0.into_function::<FunctionPio0>();
    let shift_value = pins.gpio1.into_function::<FunctionPio0>();

    let led0 = pins.gpio4.into_function::<FunctionPio0>();
    let led1 = pins.gpio5.into_function::<FunctionPio0>();
    let led2 = pins.gpio6.into_function::<FunctionPio0>();
    let led3 = pins.gpio7.into_function::<FunctionPio0>();
    let led4 = pins.gpio8.into_function::<FunctionPio0>();
    let led5 = pins.gpio9.into_function::<FunctionPio0>();
    let led6 = pins.gpio10.into_function::<FunctionPio0>();
    let led7 = pins.gpio11.into_function::<FunctionPio0>();

    let program = pio::pio_asm!(
        ".side_set 2",
        "    mov osr, rxfifo[0]      side 0b10",
        "    out pins, 8             side 0b11",
        "    nop                     side 0b00",
        "    out pins, 8             side 0b01",
        "    nop                     side 0b00",
        "    out pins, 8             side 0b01",
        "    nop                     side 0b00",
        "    out pins, 8             side 0b01",
        "    mov osr, rxfifo[1]      side 0b00",
        "    out pins, 8             side 0b01",
        "    nop                     side 0b00",
        "    out pins, 8             side 0b01",
        "    nop                     side 0b00",
        "    out pins, 8             side 0b01",
        "    nop                     side 0b00",
        "    out pins, 8             side 0b01",
    );

    let (mut pio, sm0, _, _, _) = pac.PIO0.split(&mut pac.RESETS);
    let installed = pio.install(&program.program).unwrap();
    let (mut sm, mut buffer, _) = hal::pio::PIOBuilder::from_installed_program(installed)
        .clock_divisor_fixed_point(4096, 0)
        .out_sticky(true)
        .out_shift_direction(hal::pio::ShiftDirection::Right)
        .with_rx_get()
        .side_set_pin_base(shift_clk.id().num)
        .out_pins(led0.id().num, 8)
        .build(sm0);
    sm.set_pindirs([
        (shift_clk.id().num, hal::pio::PinDir::Output),
        (shift_value.id().num, hal::pio::PinDir::Output),
        (led0.id().num, hal::pio::PinDir::Output),
        (led1.id().num, hal::pio::PinDir::Output),
        (led2.id().num, hal::pio::PinDir::Output),
        (led3.id().num, hal::pio::PinDir::Output),
        (led4.id().num, hal::pio::PinDir::Output),
        (led5.id().num, hal::pio::PinDir::Output),
        (led6.id().num, hal::pio::PinDir::Output),
        (led7.id().num, hal::pio::PinDir::Output),
    ]);
    sm.start();

    // PIO runs in background, independently from CPU
    loop {
        buffer.write_at(0, 0xaaaaaaaa);
        buffer.write_at(1, 0xaaaaaaaa);
        timer.delay_ms(500);
        buffer.write_at(0, 0x55555555);
        buffer.write_at(1, 0x55555555);
        timer.delay_ms(500);
    }
}

/// Program metadata for `picotool info`
#[link_section = ".bi_entries"]
#[used]
pub static PICOTOOL_ENTRIES: [hal::binary_info::EntryAddr; 5] = [
    hal::binary_info::rp_cargo_bin_name!(),
    hal::binary_info::rp_cargo_version!(),
    hal::binary_info::rp_program_description!(c"PIO LED Matrix Example"),
    hal::binary_info::rp_cargo_homepage_url!(),
    hal::binary_info::rp_program_build_attribute!(),
];

// End of file
