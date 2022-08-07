//! # 'ROM Functions' Example
//!
//! This application demonstrates how to call functions in the RP2040's boot ROM.
//!
//! It may need to be adapted to your particular board layout and/or pin assignment,
//! but only outputs via USB Serial port.
//!
//! Send characters to get results or send `!` to put the rp2040 in usb boot
//!
//! See the `Cargo.toml` file for Copyright and license details.

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

// USB Device support
use usb_device::{class_prelude::*, prelude::*};

// USB Communications Class Device support
use usbd_serial::SerialPort;

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

struct Buffer<'a> {
    buf: &'a mut [u8],
    index: usize,
}

// write_str from https://stackoverflow.com/a/39491059
impl<'a> core::fmt::Write for Buffer<'a> {
    fn write_str(&mut self, s: &str) -> core::fmt::Result {
        let b = s.as_bytes();
        let remaining_buf = &mut self.buf[self.index..];
        self.index += b.len();
        if remaining_buf.len() < b.len() {
            return Err(core::fmt::Error);
        }
        let remaining_buf = &mut remaining_buf[..b.len()];
        remaining_buf.copy_from_slice(b);
        Ok(())
    }
}

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
    let _pins = hal::gpio::Pins::new(
        pac.IO_BANK0,
        pac.PADS_BANK0,
        sio.gpio_bank0,
        &mut pac.RESETS,
    );

    let mut results_buf = Buffer {
        buf: &mut [0x0; 65536],
        index: 0,
    };

    writeln!(
        results_buf,
        "ROM Copyright: {}",
        hal::rom_data::copyright_string()
    )
    .unwrap();

    writeln!(
        results_buf,
        "ROM Version: {}",
        hal::rom_data::rom_version_number()
    )
    .unwrap();

    writeln!(
        results_buf,
        "ROM Git Revision: 0x{:x}",
        hal::rom_data::git_revision()
    )
    .unwrap();

    // Some ROM functions are exported directly, so we can just call them
    writeln!(
        results_buf,
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
        results_buf,
        "{} x {} = {} in {} systicks (doing f32 soft-float maths)",
        x,
        y,
        soft_result,
        calc_delta(start_soft, end_soft)
    )
    .unwrap();

    // Some functions require a look-up in a table. First we do the lookup and
    // find the function pointer in ROM (you only want to do this once per
    // function).
    let fnp = hal::rom_data::float_funcs::fmul::ptr();
    let start_fn = cortex_m::peripheral::SYST::get_current();
    // Then we can call the function whenever we want
    let result = fnp(x, y);
    let end_fn = cortex_m::peripheral::SYST::get_current();
    writeln!(
        results_buf,
        "fmul({}, {}) = {} in {} systicks (using the ROM)",
        x,
        y,
        result,
        calc_delta(start_fn, end_fn)
    )
    .unwrap();
    // the first rom version does not implement f64 rom funcs
    if hal::rom_data::rom_version_number() > 1 {
        // Try to hide the numbers from the compiler so it is forced to do the maths
        let x = hal::rom_data::double_funcs::dsin(8f64) as f64;
        let y = hal::rom_data::double_funcs::dsin(12f64) as f64;

        let start_soft = cortex_m::peripheral::SYST::get_current();
        core::sync::atomic::compiler_fence(core::sync::atomic::Ordering::SeqCst);
        let soft_result = x * y;
        core::sync::atomic::compiler_fence(core::sync::atomic::Ordering::SeqCst);
        let end_soft = cortex_m::peripheral::SYST::get_current();

        writeln!(
            results_buf,
            "{:.3} x {:.3} = {:.3} in {} systicks (doing f64 soft-float maths)",
            x,
            y,
            soft_result,
            calc_delta(start_soft, end_soft)
        )
        .unwrap();
        let fnp = hal::rom_data::double_funcs::dmul::ptr();
        let start_fn = cortex_m::peripheral::SYST::get_current();
        let result = fnp(x, y);
        let end_fn = cortex_m::peripheral::SYST::get_current();
        writeln!(
            results_buf,
            "dmul({:.3}, {:.3}) = {:.3} in {} systicks (using the ROM)",
            x,
            y,
            result,
            calc_delta(start_fn, end_fn)
        )
        .unwrap();
    }

    // Set up the USB driver
    let usb_bus = UsbBusAllocator::new(hal::usb::UsbBus::new(
        pac.USBCTRL_REGS,
        pac.USBCTRL_DPRAM,
        clocks.usb_clock,
        true,
        &mut pac.RESETS,
    ));

    // Set up the USB Communications Class Device driver
    let mut serial = SerialPort::new(&usb_bus);

    // Create a USB device with a fake VID and PID
    let mut usb_dev = UsbDeviceBuilder::new(&usb_bus, UsbVidPid(0x16c0, 0x27dd))
        .manufacturer("Fake company")
        .product("Serial port")
        .serial_number("TEST")
        .device_class(2) // from: https://www.usb.org/defined-class-codes
        .build();

    let mut send_results = false;
    let results_buf_len: usize = results_buf.buf.len();
    let mut results_buf_count: usize = 0;

    let mut wr_ptr = &results_buf.buf[..results_buf_len];
    loop {
        // Check for new data
        if usb_dev.poll(&mut [&mut serial]) {
            let mut buf = [0u8; 64];
            match serial.read(&mut buf) {
                Err(_e) => {
                    // Do nothing
                }
                Ok(0) => {
                    // Do nothing
                }
                Ok(_c) => {
                    send_results = true;
                    // Reboot back into USB mode if ! is received
                    if buf.contains(&"!".as_bytes()[0]) {
                        hal::rom_data::reset_to_usb_boot(0, 0);
                    }
                }
            }
        }
        // If data was recieved on the usb poll, write out the results of all
        // the rom functions tested
        if send_results {
            match serial.write(wr_ptr) {
                Ok(len) => {
                    wr_ptr = &wr_ptr[len..];
                    results_buf_count += len;
                    if results_buf_count == results_buf_len {
                        send_results = false;
                        results_buf_count = 0;
                        wr_ptr = &results_buf.buf[..results_buf_len];
                    }
                }
                // The write buffer will fill up, but we will just try again
                // in the next loop untill all bytes are sent
                Err(_) => {}
            };
        }
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
