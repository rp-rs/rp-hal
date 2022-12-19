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

use cortex_m::singleton;
use cortex_m_rt::entry;
use embedded_hal::digital::v2::OutputPin;
use fugit::RateExtU32;
use hal::dma::{bidirectional, DMAExt};
use hal::pac;
use panic_halt as _;
use rp2040_hal as hal;
use rp2040_hal::clocks::Clock;

/// The linker will place this boot block at the start of our program image. We
/// need this to help the ROM bootloader get our code up and running.
#[link_section = ".boot2"]
#[used]
pub static BOOT2: [u8; 256] = rp2040_boot2::BOOT_LOADER_GENERIC_03H;

/// External high-speed crystal on the Raspberry Pi Pico board is 12 MHz. Adjust
/// if your board has a different frequency
const XTAL_FREQ_HZ: u32 = 12_000_000u32;

#[entry]
fn main() -> ! {
    let mut pac = pac::Peripherals::take().unwrap();
    let core = pac::CorePeripherals::take().unwrap();

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
    .ok()
    .unwrap();

    // Setup the pins.
    let sio = hal::sio::Sio::new(pac.SIO);
    let pins = hal::gpio::Pins::new(
        pac.IO_BANK0,
        pac.PADS_BANK0,
        sio.gpio_bank0,
        &mut pac.RESETS,
    );

    // These are implicitly used by the spi driver if they are in the correct mode
    let _spi_sclk = pins.gpio6.into_mode::<hal::gpio::FunctionSpi>();
    let _spi_mosi = pins.gpio7.into_mode::<hal::gpio::FunctionSpi>();
    let _spi_miso = pins.gpio4.into_mode::<hal::gpio::FunctionSpi>();
    let spi = hal::spi::Spi::<_, _, 8>::new(pac.SPI0);

    // Exchange the uninitialised SPI driver for an initialised one
    let spi = spi.init(
        &mut pac.RESETS,
        clocks.peripheral_clock.freq(),
        16_000_000u32.Hz(),
        &embedded_hal::spi::MODE_0,
    );

    // Initialize DMA.
    let dma = pac.DMA.split(&mut pac.RESETS);
    // Configure GPIO25 as an output
    let mut led_pin = pins.gpio25.into_push_pull_output();
    let mut delay = cortex_m::delay::Delay::new(core.SYST, clocks.system_clock.freq().to_Hz());

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
