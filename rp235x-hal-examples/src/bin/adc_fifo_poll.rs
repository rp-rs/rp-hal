//! # ADC FIFO Example
//!
//! This application demonstrates how to read ADC samples in free-running mode,
//! and reading them from the FIFO by polling the fifo's `len()`.
//!
//! It may need to be adapted to your particular board layout and/or pin assignment.
//!
//! See the `Cargo.toml` file for Copyright and license details.

#![no_std]
#![no_main]

// Ensure we halt the program on panic (if we don't mention this crate it won't
// be linked)
use panic_halt as _;

// Alias for our HAL crate
use rp235x_hal as hal;

// Some things we need
use core::fmt::Write;
use embedded_hal::delay::DelayNs;
use fugit::RateExtU32;
use hal::Clock;

// UART related types
use hal::uart::{DataBits, StopBits, UartConfig};

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
///
/// The function configures the rp235x peripherals, then prints the temperature
/// in an infinite loop.
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

    // The delay object lets us wait for specified amounts of time (in
    // milliseconds)
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

    // UART TX (characters sent from pico) on pin 1 (GPIO0) and RX (on pin 2 (GPIO1)
    let uart_pins = (
        pins.gpio0.into_function::<hal::gpio::FunctionUart>(),
        pins.gpio1.into_function::<hal::gpio::FunctionUart>(),
    );

    // Create a UART driver
    let mut uart = hal::uart::UartPeripheral::new(pac.UART0, uart_pins, &mut pac.RESETS)
        .enable(
            UartConfig::new(115200.Hz(), DataBits::Eight, None, StopBits::One),
            clocks.peripheral_clock.freq(),
        )
        .unwrap();

    // Write to the UART
    uart.write_full_blocking(b"ADC FIFO poll example\r\n");

    // Enable ADC
    let mut adc = hal::Adc::new(pac.ADC, &mut pac.RESETS);

    // Enable the temperature sense channel
    let mut temperature_sensor = adc.take_temp_sensor().unwrap();

    // Configure GPIO26 as an ADC input
    let adc_pin_0 = hal::adc::AdcPin::new(pins.gpio26.into_floating_input()).unwrap();

    // Configure free-running mode:
    let mut adc_fifo = adc
        .build_fifo()
        // Set clock divider to target a sample rate of 1000 samples per second (1ksps).
        // The value was calculated by `(48MHz / 1ksps) - 1 = 47999.0`.
        // Please check the `clock_divider` method documentation for details.
        .clock_divider(47999, 0)
        // sample the temperature sensor first
        .set_channel(&mut temperature_sensor)
        // then alternate between GPIO26 and the temperature sensor
        .round_robin((&adc_pin_0, &temperature_sensor))
        // Uncomment this line to produce 8-bit samples, instead of 12 bit (lower bits are discarded)
        //.shift_8bit()
        // start sampling
        .start();

    // we'll capture 1000 samples in total (500 per channel)
    let mut temp_samples = [0; 500];
    let mut pin_samples = [0; 500];
    let mut i = 0;

    // initialize a timer, to measure the total sampling time (printed below)
    let timer = hal::Timer::new_timer1(pac.TIMER1, &mut pac.RESETS, &clocks);

    loop {
        // busy-wait until the FIFO contains at least two samples:
        while adc_fifo.len() < 2 {}

        // fetch two values from the fifo
        let temp_result = adc_fifo.read();
        let pin_result = adc_fifo.read();

        // uncomment this line, to trigger an "underrun" condition
        //let _extra_sample = adc_fifo.read();

        if adc_fifo.is_over() {
            // samples were pushed into the fifo faster they were read
            uart.write_full_blocking(b"FIFO overrun!\r\n");
        }
        if adc_fifo.is_under() {
            // we tried to read samples more quickly than they were pushed into the fifo
            uart.write_full_blocking(b"FIFO underrun!\r\n");
        }

        temp_samples[i] = temp_result;
        pin_samples[i] = pin_result;

        i += 1;

        // uncomment this line to trigger an "overrun" condition
        //delay.delay_ms(1000);

        if i == 500 {
            break;
        }
    }

    let time_taken = timer.get_counter();

    uart.write_full_blocking(b"Done sampling, printing results:\r\n");

    // Stop free-running mode (the returned `adc` can be reused for future captures)
    let _adc = adc_fifo.stop();

    // Print the measured values
    for i in 0..500 {
        writeln!(
            uart,
            "Temp:\t{}\tPin\t{}\r",
            temp_samples[i], pin_samples[i]
        )
        .unwrap();
    }

    writeln!(uart, "Sampling took: {}\r", time_taken).unwrap();

    loop {
        delay.delay_ms(1000);
    }
}

/// Program metadata for `picotool info`
#[link_section = ".bi_entries"]
#[used]
pub static PICOTOOL_ENTRIES: [hal::binary_info::EntryAddr; 5] = [
    hal::binary_info::rp_cargo_bin_name!(),
    hal::binary_info::rp_cargo_version!(),
    hal::binary_info::rp_program_description!(c"ADC FIFO Poll Example"),
    hal::binary_info::rp_cargo_homepage_url!(),
    hal::binary_info::rp_program_build_attribute!(),
];

// End of file
