//! # 'ROM Functions' Example
//!
//! This application demonstrates how to call functions in the RP2040's boot ROM.
//!
//! It may need to be adapted to your particular board layout and/or pin assignment.
//!
//! See the `Cargo.toml` file for Copyright and licence details.

#![no_std]
#![no_main]

// The macro for our start-up function
use cortex_m_rt::entry;

// Ensure we halt the program on panic (if we don't mention this crate it won't
// be linked)
use panic_halt as _;

// Alias for our HAL crate
use rp2040_hal as hal;

// A shorter alias for the Peripheral Access Crate, which provides low-level
// register access
use hal::pac;

// Some traits we need
use core::fmt::Write;

/// The linker will place this boot block at the start of our program image. We
/// need this to help the ROM bootloader get our code up and running.
#[link_section = ".boot2"]
#[used]
pub static BOOT2: [u8; 256] = rp2040_boot2::BOOT_LOADER_W25Q080;

/// External high-speed crystal on the Raspberry Pi Pico board is 12 MHz. Adjust
/// if your board has a different frequency
const XTAL_FREQ_HZ: u32 = 12_000_000u32;

/// Our Cortex-M systick goes from this value down to zero. For our timer maths
/// to work, this value must be of the form `2**N - 1`.
const SYSTICK_RELOAD: u32 = 0x00FF_FFFF;

/// Entry point to our bare-metal application.
///
/// The `#[entry]` macro ensures the Cortex-M start-up code calls this function
/// as soon as all global variables are initialised.
///
/// The function configures the RP2040 peripherals, then writes to the UART in
/// an infinite loop.
#[entry]
fn main() -> ! {
    // Grab our singleton objects
    let mut pac = pac::Peripherals::take().unwrap();
    let mut core = pac::CorePeripherals::take().unwrap();

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

    let uart_pins = (
        // UART TX (characters sent from RP2040) on pin 1 (GPIO0)
        pins.gpio0.into_mode::<hal::gpio::FunctionUart>(),
        // UART RX (characters received by RP2040) on pin 2 (GPIO1)
        pins.gpio1.into_mode::<hal::gpio::FunctionUart>(),
    );
    let mut uart = hal::uart::UartPeripheral::new(pac.UART0, uart_pins, &mut pac.RESETS)
        .enable(
            hal::uart::common_configs::_9600_8_N_1,
            clocks.peripheral_clock.into(),
        )
        .unwrap();

    writeln!(uart, "ROM Copyright: {}", hal::rom_data::copyright_string()).unwrap();
    writeln!(
        uart,
        "ROM Git Revision: 0x{:x}",
        hal::rom_data::git_revision()
    )
    .unwrap();

    // Some ROM functions are exported directly, so we can just call them
    writeln!(
        uart,
        "popcount32(0xF000_0001) = {}",
        hal::rom_data::popcount32(0xF000_0001)
    )
    .unwrap();

    // Try to hide the numbers from the compiler so it is forced to do the maths
    let x = hal::rom_data::popcount32(0xFF) as f32; // 8
    let y = hal::rom_data::popcount32(0xFFF) as f32; // 12

    // Use systick as a count-down timer
    core.SYST.set_reload(SYSTICK_RELOAD);
    core.SYST.clear_current();
    core.SYST.enable_counter();

    // Do some simple sums
    let start_soft = cortex_m::peripheral::SYST::get_current();
    core::sync::atomic::compiler_fence(core::sync::atomic::Ordering::SeqCst);
    let soft_result = x * y;
    core::sync::atomic::compiler_fence(core::sync::atomic::Ordering::SeqCst);
    let end_soft = cortex_m::peripheral::SYST::get_current();

    writeln!(
        uart,
        "{} x {} = {} in {} systicks (doing soft-float maths)",
        x,
        y,
        soft_result,
        calc_delta(start_soft, end_soft)
    )
    .unwrap();

    // Some functions require a look-up in a table. First we do the lookup and
    // find the function pointer in ROM (you only want to do this once per
    // function).
    let fmul = hal::rom_data::float_funcs::fmul::ptr();

    // Then we can call the function whenever we want
    let start_rom = cortex_m::peripheral::SYST::get_current();
    let rom_result = fmul(x, y);
    let end_rom = cortex_m::peripheral::SYST::get_current();

    writeln!(
        uart,
        "{} x {} = {} in {} systicks (using the ROM)",
        x,
        y,
        rom_result,
        calc_delta(start_rom, end_rom)
    )
    .unwrap();

    // Now just spin (whilst the UART does its thing)
    for _ in 0..1_000_000 {
        cortex_m::asm::nop();
    }

    // Reboot back into USB mode (no activity, both interfaces enabled)
    rp2040_hal::rom_data::reset_to_usb_boot(0, 0);

    // In case the reboot fails
    loop {
        cortex_m::asm::nop();
    }
}

/// Calculate the number of systicks elapsed between two counter readings.
///
/// Note: SYSTICK starts at `SYSTICK_RELOAD` and counts down towards zero, so
/// these comparisons might appear to be backwards.
///
/// ```
/// assert_eq!(1, calc_delta(SYSTICK_RELOAD, SYSTICK_RELOAD - 1));
/// assert_eq!(2, calc_delta(0, SYSTICK_RELOAD - 1));
/// ```
fn calc_delta(start: u32, end: u32) -> u32 {
    if start < end {
        (start.wrapping_sub(end)) & SYSTICK_RELOAD
    } else {
        start - end
    }
}

// End of file
