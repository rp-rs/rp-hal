//! # UART IRQ Echo Example
//!
//! This application demonstrates how to use the UART Driver to talk to a serial
//! connection. In this example, the IRQ owns the UART and you cannot do any UART
//! access from the main thread.
//!
//! The pinouts are:
//!
//! * GPIO 0 - UART TX (out of the RP2040)
//! * GPIO 1 - UART RX (in to the RP2040)
//! * GPIO 25 - An LED we can blink (active high)
//!
//! See the `Cargo.toml` file for Copyright and license details.

#![no_std]
#![no_main]

// These are the traits we need from Embedded HAL to treat our hardware
// objects as generic embedded devices.
use embedded_hal::{
    digital::v2::OutputPin,
    serial::{Read, Write},
};

// We need this for the 'Delay' object to work.
use embedded_time::fixed_point::FixedPoint;

// We also need this for the 'Delay' object to work.
use rp2040_hal::Clock;

// The macro for our start-up function
use rp_pico::entry;

// Ensure we halt the program on panic (if we don't mention this crate it won't
// be linked)
use panic_halt as _;

// Alias for our HAL crate
use rp2040_hal as hal;

// A shorter alias for the Peripheral Access Crate, which provides low-level
// register access
use hal::pac;

// Our interrupt macro
use hal::pac::interrupt;

// Some short-cuts to useful types
use core::cell::RefCell;
use cortex_m::interrupt::Mutex;

/// Import the GPIO pins we use
use hal::gpio::pin::bank0::{Gpio0, Gpio1};

/// Alias the type for our UART pins to make things clearer.
type UartPins = (
    hal::gpio::Pin<Gpio0, hal::gpio::Function<hal::gpio::Uart>>,
    hal::gpio::Pin<Gpio1, hal::gpio::Function<hal::gpio::Uart>>,
);

/// Alias the type for our UART to make things clearer.
type Uart = hal::uart::UartPeripheral<hal::uart::Enabled, pac::UART0, UartPins>;

/// External high-speed crystal on the Raspberry Pi Pico board is 12 MHz. Adjust
/// if your board has a different frequency
const XTAL_FREQ_HZ: u32 = 12_000_000u32;

/// This how we transfer the UART into the Interrupt Handler
static GLOBAL_UART: Mutex<RefCell<Option<Uart>>> = Mutex::new(RefCell::new(None));

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

    // Lets us wait for fixed periods of time
    let mut delay = cortex_m::delay::Delay::new(core.SYST, clocks.system_clock.freq().integer());

    // The single-cycle I/O block controls our GPIO pins
    let sio = hal::Sio::new(pac.SIO);

    // Set the pins to their default state
    let pins = rp_pico::Pins::new(
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

    // Make a UART on the given pins
    let mut uart = hal::uart::UartPeripheral::new(pac.UART0, uart_pins, &mut pac.RESETS)
        .enable(
            hal::uart::common_configs::_9600_8_N_1,
            clocks.peripheral_clock.into(),
        )
        .unwrap();

    unsafe {
        // Enable the UART interrupt in the *Nested Vectored Interrupt
        // Controller*, which is part of the Cortex-M0+ core.
        pac::NVIC::unmask(hal::pac::Interrupt::UART0_IRQ);
    }

    // Tell the UART to raise its interrupt line on the NVIC when the RX FIFO
    // has data in it.
    uart.enable_rx_interrupt();

    // Write something to the UART on start-up so we can check the output pin
    // is wired correctly.
    uart.write_full_blocking(b"uart_interrupt example started...\n");

    // Now we give away the entire UART peripheral, via the variable
    // `GLOBAL_UART`. We can no longer access the UART from this main thread.
    cortex_m::interrupt::free(|cs| {
        GLOBAL_UART.borrow(cs).replace(Some(uart));
    });

    // But we can blink an LED.
    let mut led_pin = pins.led.into_push_pull_output();

    loop {
        // The normal *Wait For Interrupts* (WFI) has a race-hazard - the
        // interrupt could occur between the CPU checking for interrupts and
        // the CPU going to sleep. We wait for events (and interrupts), and
        // then we set an event in every interrupt handler. This ensures we
        // always wake up correctly.
        cortex_m::asm::wfe();
        // Light the LED to indicate we saw an interrupt.
        led_pin.set_high().unwrap();
        delay.delay_ms(100);
        led_pin.set_low().unwrap();
    }
}

#[interrupt]
fn UART0_IRQ() {
    // This variable is special. It gets mangled by the `#[interrupt]` macro
    // into something that we can access without the `unsafe` keyword. It can
    // do this because this function cannot be called re-entrantly. We know
    // this because the function's 'real' name is unknown, and hence it cannot
    // be called from the main thread. We also know that the NVIC will not
    // re-entrantly call an interrupt.
    static mut UART: Option<hal::uart::UartPeripheral<hal::uart::Enabled, pac::UART0, UartPins>> =
        None;

    // This is one-time lazy initialisation. We steal the variable given to us
    // via `GLOBAL_UART`.
    if UART.is_none() {
        cortex_m::interrupt::free(|cs| {
            *UART = GLOBAL_UART.borrow(cs).take();
        });
    }

    // Check if we have a UART to work with
    if let Some(uart) = UART {
        // Echo the input back to the output until the FIFO is empty. Reading
        // from the UART should also clear the UART interrupt flag.
        while let Ok(byte) = uart.read() {
            let _ = uart.write(byte);
        }
    }

    // Set an event to ensure the main thread always wakes up, even if it's in
    // the process of going to sleep.
    cortex_m::asm::sev();
}

// End of file
