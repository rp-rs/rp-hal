//! # ADC Example
//!
//! This application demonstrates how to read ADC samples from the temperature
//! sensor and pin and output them to the UART on pins 1 and 2 at 9600 baud.
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

// Some traits we need
use core::fmt::Write;
use fugit::RateExtU32;
use pac::interrupt;
// A shorter alias for the Peripheral Access Crate, which provides low-level
// register access
use hal::{
    pac,
    uart::{DataBits, StopBits, UartConfig},
    Adc, Clock,
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
static ADC_OBJ: Mutex<RefCell<Option<Adc>>> = Mutex::new(RefCell::new(None));
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

    // Write to the UART
    uart.write_full_blocking(b"ADC FIFO example\r\n");

    // Enable ADC
    let adc = hal::Adc::new(pac.ADC, &mut pac.RESETS);

    // Configure GPIO26 as an ADC input
    let _adc_pin_0 = pins.gpio26.into_floating_input();
    cortex_m::interrupt::free(|cs| {
        // Start our ADC in round-robin mode, sampling our set of channels 1000 times per second
        adc.start_many_round_robin(0b1, 1000);
        ADC_OBJ.borrow(cs).replace(Some(adc));
        unsafe {
            pac::NVIC::unmask(pac::Interrupt::ADC_IRQ_FIFO);
        }
    });

    loop {
        unsafe {
            if ADC_READING_GOOD {
                // Print our ADC value, discarding any error from writeln
                let _ = writeln!(uart, "adc value {:?}\r\n", ADC_READING);
                ADC_READING_GOOD = false;
            }
        }
        delay.delay_ms(1000);
    }
}

#[interrupt]
fn ADC_IRQ_FIFO() {
    cortex_m::interrupt::free(|cs| {
        let adc = ADC_OBJ.borrow(cs).take();
        if let Some(mut adc) = adc {
            // If we got less samples than we expected
            if adc.fifo_len() < 1 {
                // Flush the fifo so we're ready for next time
                while adc.fifo_len() > 0 {
                    let _ = adc.read_fifo();
                }
                // Set the flag so we can print that the value is bad
                unsafe {
                    ADC_READING_GOOD = false;
                }
                return;
            }
            // Grab our value from the fifo
            if let Some(a) = adc.read_fifo() {
                unsafe {
                    // Store it in our static variable
                    ADC_READING = a;
                    // Let the main loop know the value is good
                    ADC_READING_GOOD = true;
                }
            }
            ADC_OBJ.borrow(cs).replace(Some(adc));
        } else {
            panic!("Interrupt fired while refcell didn't contain an Adc instance");
        }
    });
}

// End of file
