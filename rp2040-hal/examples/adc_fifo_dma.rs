//! # ADC DMA Example
//!
//! This application demonstrates how to configure the ADC into FIFO mode sampling all 4 ADC pins
//! with round-robin sampling. It uses DMA to ensure we don't miss any samples.
//!
//! ADC readings are output to the UART on pins 1 and 2 at 9600 baud.
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

// Some traits we need
use core::fmt::Write;
use fugit::RateExtU32;
use pac::interrupt;
use rp2040_hal::dma::DMAExt;
use rp2040_hal::uart::UartConfig;
use rp2040_hal::Clock;
// A shorter alias for the Peripheral Access Crate, which provides low-level
// register access
use hal::{
    pac,
    uart::{DataBits, StopBits},
};

/// The linker will place this boot block at the start of our program image. We
/// need this to help the ROM bootloader get our code up and running.
#[link_section = ".boot2"]
#[used]
pub static BOOT2: [u8; 256] = rp2040_boot2::BOOT_LOADER_W25Q080;

/// External high-speed crystal on the Raspberry Pi Pico board is 12 MHz. Adjust
/// if your board has a different frequency
const XTAL_FREQ_HZ: u32 = 12_000_000u32;

use core::cell::RefCell;
use cortex_m::interrupt::Mutex;
use rp2040_hal::Adc as RpAdc;
static ADC_OBJ: Mutex<RefCell<Option<RpAdc>>> = Mutex::new(RefCell::new(None));
static mut ADC_READING: u16 = 0;
static mut ADC_READING_GOOD: bool = false;

/// Entry point to our bare-metal application.
///
/// The `#[entry]` macro ensures the Cortex-M start-up code calls this function
/// as soon as all global variables are initialised.
///
/// The function configures the RP2040 peripherals, then prints the temperature
/// in an infinite loop.
#[entry]
fn main() -> ! {
    // Grab our singleton objects
    let mut pac = pac::Peripherals::take().unwrap();
    let core = pac::CorePeripherals::take().unwrap();

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

    // The delay object lets us wait for specified amounts of time (in
    // milliseconds)
    let mut delay = cortex_m::delay::Delay::new(core.SYST, clocks.system_clock.freq().to_Hz());

    // The single-cycle I/O block controls our GPIO pins
    let sio = hal::Sio::new(pac.SIO);

    // Set the pins to their default state
    let pins = hal::gpio::Pins::new(
        pac.IO_BANK0,
        pac.PADS_BANK0,
        sio.gpio_bank0,
        &mut pac.RESETS,
    );

    // UART TX (characters sent from pico) on pin 1 (GPIO0) and RX (on pin 2 (GPIO1)
    let uart_pins = (
        pins.gpio0.into_mode::<hal::gpio::FunctionUart>(),
        pins.gpio1.into_mode::<hal::gpio::FunctionUart>(),
    );

    // Create a UART driver
    let mut uart = hal::uart::UartPeripheral::new(pac.UART0, uart_pins, &mut pac.RESETS)
        .enable(
            UartConfig::new(9600.Hz(), DataBits::Eight, None, StopBits::One),
            clocks.peripheral_clock.freq(),
        )
        .unwrap();

    let dma = pac.DMA.split(&mut pac.RESETS);
    // These are implicitly used by the spi driver if they are in the correct mode
    let _spi_sclk = pins.gpio6.into_mode::<hal::gpio::FunctionSpi>();
    let _spi_mosi = pins.gpio7.into_mode::<hal::gpio::FunctionSpi>();
    let _spi_miso = pins.gpio4.into_mode::<hal::gpio::FunctionSpi>();
    let spi = hal::spi::Spi::<_, _, 8>::new(pac.SPI0);

    // Write to the UART
    uart.write_full_blocking(b"ADC FIFO DMA example\r\n");

    // Enable ADC
    let adc = hal::Adc::new(pac.ADC, &mut pac.RESETS);

    // Configure GPIO26 to GPIO29 as an ADC inputs
    // NOTE: GPIO29 is not broken out on Pico, but you can use this with solderparty rp2040 stamp
    let _adc_pin_0 = pins.gpio26.into_floating_input();
    let _adc_pin_1 = pins.gpio27.into_floating_input();
    let _adc_pin_2 = pins.gpio28.into_floating_input();
    let _adc_pin_3 = pins.gpio29.into_floating_input();

    // We're going to use both hardware and software double-buffering here.
    // So the hardware has one spare buffer at a time, and the software task also has a spare buffer.
    // This means that we can hold onto the buffer a bit easier
    let buf0 = cortex_m::singleton!(: [u8; 4] = [0; 4]).unwrap();
    let buf1 = cortex_m::singleton!(: [u8; 4] = [0; 4]).unwrap();
    let buf2 = cortex_m::singleton!(: [u8; 4] = [0; 4]).unwrap();
    // Move our spare software buffer into buf_spare
    let mut buf_spare = Some(buf2);

    // Start our ADC in round-robin mode, sampling our set of channels 1000 times per second
    adc.start_many_round_robin(0b1111, 1000);
    // There must be no delay between starting the ADC and setting up the DMA transfer, or we might lose samples
    let mut adc_transfer =
        hal::dma::double_buffer::Config::new((dma.ch0, dma.ch1), adc, buf0).start();
    // Give our second hardware buffer to the DMA driver
    let mut adc_transfer = adc_transfer.write_next(buf1);

    let mut counter = 0;
    loop {
        // let done = adc_transfer.is_done();
        if adc_transfer.is_done() {
            let (adc_buf, AdcXfer) = adc_transfer.wait();
            if let Some(buf) = buf_spare {
                adc_transfer = AdcXfer.write_next(buf);
            } else {
                panic!("Didn't have a spare buffer");
            }

            // We can't print ADC samples at 1000hz, and it's impossible to read anyway.
            // Take a subsample of the ADC samples instead
            counter += 1;
            if counter > 100 {
                // Creating new bindings here to make the writeln! call a bit cleaner
                let (adc1, adc2, adc3, adc4) = (adc_buf[0], adc_buf[1], adc_buf[2], adc_buf[3]);
                let _ = writeln!(uart, "{adc1} {adc2} {adc3} {adc4}\r\n");
                counter = 0;
            }

            // We've finished with the ADC buffer, put it back in the software spare slot
            buf_spare = Some(adc_buf);
        }
    }
}

// End of file
