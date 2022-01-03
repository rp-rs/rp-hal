//! # UART IRQ TX Buffer Example
//!
//! This application demonstrates how to use the UART Driver to talk to a
//! serial connection. In this example, the IRQ owns the UART and you cannot
//! do any UART access from the main thread. You can, however, write to a
//! static queue, and have the queue contents transferred to the UART under
//! interrupt.
//!
//! The pinouts are:
//!
//! * GPIO 0 - UART TX (out of the RP2040)
//! * GPIO 1 - UART RX (in to the RP2040)
//! * GPIO 25 - An LED we can blink (active high)
//!
//! See the `Cargo.toml` file for Copyright and licence details.

#![no_std]
#![no_main]

// These are the traits we need from Embedded HAL to treat our hardware
// objects as generic embedded devices.
use embedded_hal::{digital::v2::OutputPin, serial::Write as UartWrite};

// We need this for the 'Delay' object to work.
use embedded_time::fixed_point::FixedPoint;

// The writeln! trait.
use core::fmt::Write;

// We also need this for the 'Delay' object to work.
use rp2040_hal::Clock;

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

// Our interrupt macro
use pac::interrupt;

// Some short-cuts to useful types
use core::cell::RefCell;
use cortex_m::interrupt::Mutex;
use heapless::spsc::Queue;

/// Import the GPIO pins we use
use hal::gpio::pin::bank0::{Gpio0, Gpio1};

/// Alias the type for our UART pins to make things clearer.
type UartPins = (
    hal::gpio::Pin<Gpio0, hal::gpio::Function<hal::gpio::Uart>>,
    hal::gpio::Pin<Gpio1, hal::gpio::Function<hal::gpio::Uart>>,
);

/// Alias the type for our UART to make things clearer.
type Uart = hal::uart::UartPeripheral<hal::uart::Enabled, pac::UART0, UartPins>;

/// This describes the queue we use for outbound UART data
struct UartQueue {
    mutex_cell_queue: Mutex<RefCell<Queue<u8, 64>>>,
    interrupt: pac::Interrupt,
}

/// External high-speed crystal on the Raspberry Pi Pico board is 12 MHz. Adjust
/// if your board has a different frequency
const XTAL_FREQ_HZ: u32 = 12_000_000u32;

/// This how we transfer the UART into the Interrupt Handler
static GLOBAL_UART: Mutex<RefCell<Option<Uart>>> = Mutex::new(RefCell::new(None));

/// This is our outbound UART queue. We write to it from the main thread, and
/// read from it in the UART IRQ.
static UART_TX_QUEUE: UartQueue = UartQueue {
    mutex_cell_queue: Mutex::new(RefCell::new(Queue::new())),
    interrupt: hal::pac::Interrupt::UART0_IRQ,
};

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

    // Tell the UART to raise its interrupt line on the NVIC when the TX FIFO
    // has space in it.
    uart.enable_tx_interrupt();

    // Now we give away the entire UART peripheral, via the variable
    // `GLOBAL_UART`. We can no longer access the UART from this main thread.
    cortex_m::interrupt::free(|cs| {
        GLOBAL_UART.borrow(cs).replace(Some(uart));
    });

    // But we can blink an LED.
    let mut led_pin = pins.led.into_push_pull_output();

    loop {
        // Light the LED whilst the main thread is in the transmit routine. It
        // shouldn't be on very long, but it will be on until we get enough
        // data /out/ of the queue and over the UART for this remainder of
        // this string to fit.
        led_pin.set_high().unwrap();
        // Note we can only write to &UART_TX_QUEUE, because it's not mutable and
        // `core::fmt::Write` takes mutable references.
        writeln!(
            &UART_TX_QUEUE,
            "Hello, this was sent under interrupt! It's quite a \
            long message, designed not to fit in either the \
            hardware FIFO or the software queue."
        )
        .unwrap();
        led_pin.set_low().unwrap();
        // Wait for a second - the UART TX IRQ will transmit the remainder of
        // our queue contents in the background.
        delay.delay_ms(1000);
    }
}

impl UartQueue {
    /// Try and get some data out of the UART Queue. Returns None if queue empty.
    fn read_byte(&self) -> Option<u8> {
        cortex_m::interrupt::free(|cs| {
            let cell_queue = self.mutex_cell_queue.borrow(cs);
            let mut queue = cell_queue.borrow_mut();
            queue.dequeue()
        })
    }

    /// Peek at the next byte in the queue without removing it.
    fn peek_byte(&self) -> Option<u8> {
        cortex_m::interrupt::free(|cs| {
            let cell_queue = self.mutex_cell_queue.borrow(cs);
            let queue = cell_queue.borrow_mut();
            queue.peek().cloned()
        })
    }

    /// Write some data to the queue, spinning until it all fits.
    fn write_bytes_blocking(&self, data: &[u8]) {
        // Go through all the bytes we need to write.
        for byte in data.iter() {
            // Keep trying until there is space in the queue. But release the
            // mutex between each attempt, otherwise the IRQ will never run
            // and we will never have space!
            let mut written = false;
            while !written {
                // Grab the mutex, by turning interrupts off. NOTE: This
                // doesn't work if you are using Core 1 as we only turn
                // interrupts off on one core.
                cortex_m::interrupt::free(|cs| {
                    // Grab the mutex contents.
                    let cell_queue = self.mutex_cell_queue.borrow(cs);
                    // Grab mutable access to the queue. This can't fail
                    // because there are no interrupts running.
                    let mut queue = cell_queue.borrow_mut();
                    // Try and put the byte in the queue.
                    if queue.enqueue(*byte).is_ok() {
                        // It worked! We must have had space.
                        if !pac::NVIC::is_enabled(self.interrupt) {
                            unsafe {
                                // Now enable the UART interrupt in the *Nested
                                // Vectored Interrupt Controller*, which is part
                                // of the Cortex-M0+ core. If the FIFO has space,
                                // the interrupt will run as soon as we're out of
                                // the closure.
                                pac::NVIC::unmask(self.interrupt);
                                // We also have to kick the IRQ in case the FIFO
                                // was already below the threshold level.
                                pac::NVIC::pend(self.interrupt);
                            }
                        }
                        written = true;
                    }
                });
            }
        }
    }
}

impl core::fmt::Write for &UartQueue {
    /// This function allows us to `writeln!` on our global static UART queue.
    /// Note we have an impl for &UartQueue, because our global static queue
    /// is not mutable and `core::fmt::Write` takes mutable references.
    fn write_str(&mut self, data: &str) -> core::fmt::Result {
        self.write_bytes_blocking(data.as_bytes());
        Ok(())
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
        // Check if we have data to transmit
        while let Some(byte) = UART_TX_QUEUE.peek_byte() {
            if uart.write(byte).is_ok() {
                // The UART took it, so pop it off the queue.
                let _ = UART_TX_QUEUE.read_byte();
            } else {
                break;
            }
        }

        if UART_TX_QUEUE.peek_byte().is_none() {
            pac::NVIC::mask(hal::pac::Interrupt::UART0_IRQ);
        }
    }

    // Set an event to ensure the main thread always wakes up, even if it's in
    // the process of going to sleep.
    cortex_m::asm::sev();
}

// End of file
