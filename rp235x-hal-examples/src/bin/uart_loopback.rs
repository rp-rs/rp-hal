//! # UART Loopback Example
//!
//! This application tests handling UART errors.
//!
//! It may need to be adapted to your particular board layout and/or pin
//! assignment. We assume you have connected GP0 to a TTL UART on your host
//! computer at 115200 baud. We assume that GP1 is connected to GP4, which is
//! our UART loopback connection.
//!
//! See the `Cargo.toml` file for Copyright and license details.

#![no_std]
#![no_main]

// Alias for our HAL crate
use rp235x_hal as hal;

// Some things we need
use core::fmt::Write;
use embedded_hal::delay::DelayNs;
use hal::clocks::Clock;
use hal::fugit::RateExtU32;

// UART related types
use hal::uart::{DataBits, Parity, StopBits, UartConfig};

/// Tell the Boot ROM about our application
#[link_section = ".start_block"]
#[used]
pub static IMAGE_DEF: hal::block::ImageDef = hal::block::ImageDef::secure_exe();

/// External high-speed crystal on the Raspberry Pi Pico 2 board is 12 MHz.
/// Adjust if your board has a different frequency
const XTAL_FREQ_HZ: u32 = 12_000_000u32;

/// Some sample text we can send.
///
/// It's the same length as the UART FIFO, deliberately.
static SAMPLE32: [u8; 32] = *b"abcdefghijklmnopqrstuvwxyz012345";

/// Entry point to our bare-metal application.
///
/// The `#[hal::entry]` macro ensures the Cortex-M start-up code calls this function
/// as soon as all global variables and the spinlock are initialised.
///
/// The function configures the rp235x peripherals, then writes to the UART in
/// an infinite loop.
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

    let mut delay = hal::Timer::new_timer0(pac.TIMER0, &mut pac.RESETS, &clocks);

    // The single-cycle I/O block controls our GPIO pins
    let sio = hal::Sio::new(pac.SIO);

    // Set the pins to their default state
    let pins = hal::gpio::Pins::new(
        pac.IO_BANK0,
        pac.PADS_BANK0,
        sio.gpio_bank0,
        &mut pac.RESETS,
    );

    let uart0_pins = (
        // UART TX (characters sent from RP2350)
        pins.gpio0.into_function(),
        // UART RX (characters received by RP2350)
        pins.gpio1.into_pull_up_input().into_function(),
    );
    let mut uart0 = hal::uart::UartPeripheral::new(pac.UART0, uart0_pins, &mut pac.RESETS)
        .enable(
            UartConfig::new(
                115200.Hz(),
                DataBits::Eight,
                Some(Parity::Even),
                StopBits::One,
            ),
            clocks.peripheral_clock.freq(),
        )
        .unwrap();
    uart0.set_fifos(true);

    let uart1_pins = (
        // UART TX (characters sent from RP2350)
        pins.gpio4.into_function(),
        // UART RX (characters received by RP2350)
        pins.gpio5.into_pull_up_input().into_function(),
    );
    let mut uart1 = hal::uart::UartPeripheral::new(pac.UART1, uart1_pins, &mut pac.RESETS)
        .enable(
            UartConfig::new(
                115200.Hz(),
                DataBits::Eight,
                Some(Parity::Even),
                StopBits::One,
            ),
            clocks.peripheral_clock.freq(),
        )
        .unwrap();
    uart1.set_fifos(true);

    let mut buffer = [0u8; 128];

    // ======================================================================
    // Single byte read/write
    // ======================================================================

    let sample = &SAMPLE32[0..1];
    _ = writeln!(uart0, "** Testing single byte write/read");
    uart1.write_full_blocking(sample);
    delay.delay_ms(100);
    uart0
        .read_full_blocking(&mut buffer[0..sample.len()])
        .unwrap();
    if &buffer[0..sample.len()] != sample {
        _ = writeln!(
            uart0,
            "Failed:\n{:02x?} !=\n{:02x?}",
            &buffer[0..sample.len()],
            sample
        );
        panic!("Test failed");
    }
    _ = writeln!(uart0, "Tested OK");

    // ======================================================================
    // FIFO backed read/write
    // ======================================================================

    let sample = &SAMPLE32[..];
    _ = writeln!(uart0, "** Testing FIFO write/read");
    uart1.write_full_blocking(sample);
    delay.delay_ms(100);
    uart0
        .read_full_blocking(&mut buffer[0..sample.len()])
        .unwrap();
    if &buffer[0..sample.len()] != sample {
        _ = writeln!(
            uart0,
            "Failed:\n{:02x?} !=\n{:02x?}",
            &buffer[0..sample.len()],
            sample
        );
        panic!("Test failed");
    }
    _ = writeln!(uart0, "Tested OK");

    // ======================================================================
    // FIFO overflow read/write
    //
    // Note: The Arm Primecell PL022 UART that Raspberry Pi uses has a 32-byte
    // FIFO. We're about to overflow that FIFO.
    // ======================================================================

    let sample = &SAMPLE32[..];
    _ = writeln!(uart0, "** Testing FIFO overflow write/read");
    _ = writeln!(uart0, "Sending {} bytes...", sample.len() + 1);
    // send 32 bytes to the receiving FIFO
    uart1.write_full_blocking(sample);
    // Now send one more byte to overflow the receiving FIFO. This byte is lost
    // to the wind.
    //
    // NB: It fits into the TX FIFO because this is a 'blocking' call that
    // waited for FIFO space.
    uart1.write_full_blocking(&[0x00]);
    // Let the TX FIFO drain.
    delay.delay_ms(100);
    // the first 32 bytes should read fine
    uart0
        .read_full_blocking(&mut buffer[0..sample.len()])
        .unwrap();
    if &buffer[0..sample.len()] != sample {
        _ = writeln!(
            uart0,
            "Failed:\n{:02x?} !=\n{:02x?}",
            &buffer[0..sample.len()],
            sample
        );
        panic!("Test failed");
    }
    _ = writeln!(uart0, "Got first 32 bytes OK...");

    _ = writeln!(
        uart0,
        "I now want to see Overrun([]), WouldBlock, WouldBlock"
    );
    _ = writeln!(uart0, "RX: {:?}", uart0.read_raw(&mut buffer[..]));
    _ = writeln!(uart0, "RX: {:?}", uart0.read_raw(&mut buffer[..]));
    _ = writeln!(uart0, "RX: {:?}", uart0.read_raw(&mut buffer[..]));
    // Now send two more bytes - the first will also be flagged with an overrun error.
    _ = writeln!(uart0, "Sending two more bytes...");
    uart1.write_full_blocking(&[0x01, 0x02]);
    // let them transfer over
    delay.delay_ms(100);
    // annoyingly we see the overrun error again, then our data
    _ = writeln!(uart0, "I want to see Overrun([1]), Ok(1), WouldBlock");
    _ = writeln!(uart0, "RX: {:?}", uart0.read_raw(&mut buffer[..]));
    _ = writeln!(uart0, "RX: {:?}", uart0.read_raw(&mut buffer[..]));
    _ = writeln!(uart0, "RX: {:?}", uart0.read_raw(&mut buffer[..]));

    // ======================================================================
    // FIFO read/write with parity error
    // ======================================================================

    _ = writeln!(uart0, "** Testing FIFO read with parity errors");
    // Send three bytes with correct parity
    uart1.write_full_blocking(&[0x00, 0x01, 0x02]);
    delay.delay_ms(100);
    // send one with bad settings
    uart1 = uart1
        .disable()
        .enable(
            UartConfig::new(
                115200.Hz(),
                DataBits::Eight,
                Some(Parity::Odd),
                StopBits::One,
            ),
            clocks.peripheral_clock.freq(),
        )
        .unwrap();
    uart1.write_full_blocking(&[0x03]);
    delay.delay_ms(100);
    // send three more with good parity
    uart1 = uart1
        .disable()
        .enable(
            UartConfig::new(
                115200.Hz(),
                DataBits::Eight,
                Some(Parity::Even),
                StopBits::One,
            ),
            clocks.peripheral_clock.freq(),
        )
        .unwrap();
    uart1.write_full_blocking(&[0x04, 0x05, 0x06]);
    delay.delay_ms(100);

    _ = writeln!(uart0, "I want to see Parity error [0, 1, 2]");
    match uart0.read_raw(&mut buffer[..]) {
        Ok(n) => {
            _ = writeln!(uart0, "RX: {:?}", &buffer[0..n]);
        }
        Err(e) => {
            _ = writeln!(uart0, "RXE: {:?}", e);
        }
    }
    _ = writeln!(uart0, "I want to see RX: [4, 5, 6]");
    match uart0.read_raw(&mut buffer[..]) {
        Ok(n) => {
            _ = writeln!(uart0, "RX: {:?}", &buffer[0..n]);
        }
        Err(e) => {
            _ = writeln!(uart0, "RXE: {:?}", e);
        }
    }
    _ = writeln!(uart0, "I want to see WouldBlock");
    match uart0.read_raw(&mut buffer[..]) {
        Ok(n) => {
            _ = writeln!(uart0, "RX: {:?}", &buffer[0..n]);
        }
        Err(e) => {
            _ = writeln!(uart0, "RXE: {:?}", e);
        }
    }

    // ======================================================================
    // FIFO backed read/write with embedded_io traits.
    // ======================================================================

    let sample = &SAMPLE32[..];
    _ = writeln!(uart0, "** Testing FIFO write/read with embedded-io");

    embedded_io::Write::write_all(&mut uart1, sample).unwrap();
    delay.delay_ms(100);

    embedded_io::Read::read_exact(&mut uart0, &mut buffer[0..sample.len()]).unwrap();
    if &buffer[0..sample.len()] != sample {
        _ = writeln!(
            uart0,
            "Failed:\n{:02x?} !=\n{:02x?}",
            &buffer[0..sample.len()],
            sample
        );
        panic!("Test failed");
    }
    _ = writeln!(uart0, "Tested OK");

    // ======================================================================
    // FIFO read/write with parity error using embedded-io
    // ======================================================================

    _ = writeln!(uart0, "** Testing FIFO read with parity errors");
    // Send three bytes with correct parity
    uart1.write_full_blocking(&[0x00, 0x01, 0x02]);
    delay.delay_ms(100);
    // send one with bad settings
    uart1 = uart1
        .disable()
        .enable(
            UartConfig::new(
                115200.Hz(),
                DataBits::Eight,
                Some(Parity::Odd),
                StopBits::One,
            ),
            clocks.peripheral_clock.freq(),
        )
        .unwrap();
    uart1.write_full_blocking(&[0x03]);
    delay.delay_ms(100);
    // send three more with good parity
    uart1 = uart1
        .disable()
        .enable(
            UartConfig::new(
                115200.Hz(),
                DataBits::Eight,
                Some(Parity::Even),
                StopBits::One,
            ),
            clocks.peripheral_clock.freq(),
        )
        .unwrap();
    uart1.write_full_blocking(&[0x04, 0x05, 0x06]);
    delay.delay_ms(100);

    _ = writeln!(uart0, "I want to see RX: [0, 1, 2]");
    match embedded_io::Read::read(&mut uart0, &mut buffer[..]) {
        Ok(n) => {
            _ = writeln!(uart0, "RX: {:?}", &buffer[0..n]);
        }
        Err(e) => {
            _ = writeln!(uart0, "RXE: {:?}", e);
        }
    }
    _ = writeln!(uart0, "I want to see ParityError");
    match embedded_io::Read::read(&mut uart0, &mut buffer[..]) {
        Ok(n) => {
            _ = writeln!(uart0, "RX: {:?}", &buffer[0..n]);
        }
        Err(e) => {
            _ = writeln!(uart0, "RXE: {:?}", e);
        }
    }

    _ = writeln!(uart0, "I want to see RX: [4, 5, 6]");
    match embedded_io::Read::read(&mut uart0, &mut buffer[..]) {
        Ok(n) => {
            _ = writeln!(uart0, "RX: {:?}", &buffer[0..n]);
        }
        Err(e) => {
            _ = writeln!(uart0, "RXE: {:?}", e);
        }
    }

    _ = writeln!(uart0, "I want to see RX ready: false");
    match embedded_io::ReadReady::read_ready(&mut uart0) {
        Ok(ready) => {
            _ = writeln!(uart0, "RX ready: {}", ready);
        }
        Err(e) => {
            _ = writeln!(uart0, "RXE: {:?}", e);
        }
    }

    // ======================================================================
    // Tests complete
    // ======================================================================

    _ = writeln!(uart0, "Tests complete. Review output for correctness.");

    // Do a reset into the bootloader.
    hal::reboot::reboot(
        hal::reboot::RebootKind::BootSel {
            msd_disabled: false,
            picoboot_disabled: false,
        },
        hal::reboot::RebootArch::Normal,
    );
}

/// Program metadata for `picotool info`
#[link_section = ".bi_entries"]
#[used]
pub static PICOTOOL_ENTRIES: [hal::binary_info::EntryAddr; 5] = [
    hal::binary_info::rp_cargo_bin_name!(),
    hal::binary_info::rp_cargo_version!(),
    hal::binary_info::rp_program_description!(c"UART Loopback Example"),
    hal::binary_info::rp_cargo_homepage_url!(),
    hal::binary_info::rp_program_build_attribute!(),
];

#[panic_handler]
fn panic(_: &core::panic::PanicInfo) -> ! {
    // wait about 1s for UART FIFOs to flush
    for _ in 0..75_000_000 {
        hal::arch::nop();
    }
    // Do a reset into the bootloader.
    hal::reboot::reboot(
        hal::reboot::RebootKind::BootSel {
            msd_disabled: false,
            picoboot_disabled: false,
        },
        hal::reboot::RebootArch::Normal,
    );
}

// End of file
