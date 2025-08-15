//! # GPIO 'Blinky' Example, with Binary Info
//!
//! This application demonstrates how to control a GPIO pin on the RP2040, and
//! includes some picotool-compatible metadata.
//!
//! It may need to be adapted to your particular board layout and/or pin assignment.
//!
//! See the top-level `README.md` file for Copyright and license details.

#![no_std]
#![no_main]

// Ensure we halt the program on panic (if we don't mention this crate it won't
// be linked)
use panic_halt as _;

/// The linker will place this boot block at the start of our program image. We
/// need this to help the ROM bootloader get our code up and running.
/// Note: This boot block is not necessary when using a rp-hal based BSP
/// as the BSPs already perform this step.
#[link_section = ".boot2"]
#[used]
#[cfg(feature = "rp2040")]
pub static BOOT2: [u8; 256] = rp2040_boot2::BOOT_LOADER_GENERIC_03H;

/// External high-speed crystal on the Raspberry Pi Pico board is 12 MHz. Adjust
/// if your board has a different frequency
const XTAL_FREQ_HZ: u32 = 12_000_000u32;

/// Entry point to our bare-metal application.
///
/// The `#[entry]` macro ensures the Cortex-M start-up code calls this function
/// as soon as all global variables are initialised.
#[cortex_m_rt::entry]
fn main() -> ! {
    // Grab our singleton objects
    let mut pac = rp_hal::Peripherals::take().unwrap();
    loop {
    }
}

/// This is a list of references to our table entries
///
/// They must be in the `.bi_entries` section as we tell picotool the start and
/// end addresses of that section.
#[link_section = ".bi_entries"]
#[used]
pub static PICOTOOL_ENTRIES: [rp_binary_info::EntryAddr; 8] = [
    rp_binary_info::rp_program_name!(c"rp-hal Binary Info Example"),
    rp_binary_info::rp_cargo_version!(),
    rp_binary_info::rp_program_description!(c"A GPIO blinky with extra metadata."),
    rp_binary_info::rp_program_url!(c"https://github.com/rp-rs/rp-hal"),
    rp_binary_info::rp_program_build_attribute!(),
    rp_binary_info::rp_pico_board!(c"pico"),
    rp_binary_info::rp_binary_end!(__flash_binary_end),
    // An example with a non-Raspberry-Pi tag
    rp_binary_info::int!(rp_binary_info::make_tag(b"JP"), 0x0000_0001, 0x12345678),
];

extern "C" {
    static __flash_binary_end: u32;
}

// End of file
