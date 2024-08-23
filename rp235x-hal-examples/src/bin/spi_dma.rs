//! # SPI DMA Example
//!
//! This application demonstrates how to use DMA for SPI transfers.
//!
//! The application expects the MISO and MOSI pins to be wired together so that it is able to check
//! whether the data was sent and received correctly.
//!
//! See the `Cargo.toml` file for Copyright and licence details.
#![no_std]
#![no_main]

use rp235x_hal as hal;

use hal::singleton;

use embedded_hal::delay::DelayNs;
use embedded_hal::digital::OutputPin;
use hal::clocks::Clock;
use hal::dma::{bidirectional, DMAExt};
use hal::fugit::RateExtU32;

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

#[hal::entry]
fn main() -> ! {
    let mut pac = hal::pac::Peripherals::take().unwrap();

    // Setup clocks and the watchdog.
    let mut watchdog = hal::watchdog::Watchdog::new(pac.WATCHDOG);
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

    // Setup the pins.
    let sio = hal::sio::Sio::new(pac.SIO);
    let pins = hal::gpio::Pins::new(
        pac.IO_BANK0,
        pac.PADS_BANK0,
        sio.gpio_bank0,
        &mut pac.RESETS,
    );

    // Set up our SPI pins into the correct mode
    let spi_mosi = pins.gpio7.into_function::<hal::gpio::FunctionSpi>();
    let spi_miso = pins.gpio4.into_function::<hal::gpio::FunctionSpi>();
    let spi_sclk = pins.gpio6.into_function::<hal::gpio::FunctionSpi>();
    let spi = hal::spi::Spi::<_, _, _, 8>::new(pac.SPI0, (spi_mosi, spi_miso, spi_sclk));

    // Exchange the uninitialised SPI driver for an initialised one
    let spi = spi.init(
        &mut pac.RESETS,
        clocks.peripheral_clock.freq(),
        16_000_000u32.Hz(),
        embedded_hal::spi::MODE_0,
    );

    // Initialize DMA.
    let dma = pac.DMA.split(&mut pac.RESETS);
    // Configure GPIO25 as an output
    let mut led_pin = pins.gpio25.into_push_pull_output();
    let mut delay = hal::Timer::new_timer0(pac.TIMER0, &mut pac.RESETS, &clocks);

    // Use DMA to transfer some bytes (single buffering).
    let tx_buf = singleton!(: [u8; 16] = [0x42; 16]).unwrap();
    let rx_buf = singleton!(: [u8; 16] = [0; 16]).unwrap();

    // Use BidirectionalConfig to simultaneously write to spi from tx_buf and read into rx_buf
    let transfer = bidirectional::Config::new((dma.ch0, dma.ch1), tx_buf, spi, rx_buf).start();
    // Wait for both DMA channels to finish
    let ((_ch0, _ch1), tx_buf, _spi, rx_buf) = transfer.wait();

    // Compare buffers to see if the data was transferred correctly
    for i in 0..rx_buf.len() {
        if rx_buf[i] != tx_buf[i] {
            // Fast blink on error
            loop {
                led_pin.set_high().unwrap();
                delay.delay_ms(100);
                led_pin.set_low().unwrap();
                delay.delay_ms(100);
            }
        }
    }

    // Slow blink on success
    loop {
        led_pin.set_high().unwrap();
        delay.delay_ms(500);
        led_pin.set_low().unwrap();
        delay.delay_ms(500);
    }
}

/// Program metadata for `picotool info`
#[link_section = ".bi_entries"]
#[used]
pub static PICOTOOL_ENTRIES: [hal::binary_info::EntryAddr; 5] = [
    hal::binary_info::rp_cargo_bin_name!(),
    hal::binary_info::rp_cargo_version!(),
    hal::binary_info::rp_program_description!(c"SPI DMA Example"),
    hal::binary_info::rp_cargo_homepage_url!(),
    hal::binary_info::rp_program_build_attribute!(),
];

// End of file
