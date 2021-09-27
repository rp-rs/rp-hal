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

// An ADC trait we need
use embedded_hal::adc::OneShot;

// A debug/string formatting trait we need
use core::fmt::Write;

// A shorter alias for the Peripheral Access Crate, which provides low-level
// register access
use hal::pac;

// The linker will place this boot block at the start of our program image. We
// need this to help the ROM bootloader get our code up and running.
#[link_section = ".boot2"]
#[used]
pub static BOOT2: [u8; 256] = rp2040_boot2::BOOT_LOADER;

/// External high-speed crystal on the Raspberry Pi Pico board is 12 MHz. Adjust
/// if your board has a different frequency
const XTAL_FREQ_HZ: u32 = 12_000_000u32;

/// Run RP2040 at 125 MHz
const SYS_FREQ_HZ: u32 = hal::pll::common_configs::PLL_SYS_125MHZ.vco_freq.0;

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
    let mut watchdog = hal::watchdog::Watchdog::new(pac.WATCHDOG);

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
    let mut delay = cortex_m::delay::Delay::new(core.SYST, SYS_FREQ_HZ);

    // The single-cycle I/O block controls our GPIO pins
    let sio = hal::sio::Sio::new(pac.SIO);

    // Set the pins to their default state
    let pins = hal::gpio::Pins::new(
        pac.IO_BANK0,
        pac.PADS_BANK0,
        sio.gpio_bank0,
        &mut pac.RESETS,
    );

    // Create a UART driver
    let mut uart = hal::uart::UartPeripheral::<_, _>::enable(
        pac.UART0,
        &mut pac.RESETS,
        hal::uart::common_configs::_9600_8_N_1,
        clocks.peripheral_clock.into(),
    )
    .unwrap();

    // UART TX (characters sent from pico) on pin 1 (GPIO0) and RX (on pin 2 (GPIO1)
    let _tx_pin = pins.gpio0.into_mode::<hal::gpio::FunctionUart>();
    let _rx_pin = pins.gpio1.into_mode::<hal::gpio::FunctionUart>();

    // Write to the UART
    uart.write_full_blocking(b"ADC example\r\n");

    // Enable ADC
    let mut adc = hal::adc::Adc::new(pac.ADC, &mut pac.RESETS);

    // Enable the temperature sense channel
    let mut temperature_sensor = adc.enable_temp_sensor();

    // Configure GPIO26 as an ADC input
    let mut adc_pin_0 = pins.gpio26.into_floating_input();
    loop {
        // Read the raw ADC counts from the temperature sensor channel.
        let temp_sens_adc_counts: u16 = adc.read(&mut temperature_sensor).unwrap();
        let pin_adc_counts: u16 = adc.read(&mut adc_pin_0).unwrap();
        writeln!(
            uart,
            "ADC readings: Temperature: {:02} Pin: {:02}\r\n",
            temp_sens_adc_counts, pin_adc_counts
        )
        .unwrap();
        delay.delay_ms(1000);
    }
}

// End of file
