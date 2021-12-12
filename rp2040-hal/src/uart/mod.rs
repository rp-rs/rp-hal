//! Universal Asynchronous Receiver Transmitter (UART)
//!
//! See [Chapter 4 Section 2](https://datasheets.raspberrypi.org/rp2040/rp2040_datasheet.pdf) of the datasheet for more details
//!
//! ## Usage
//!
//! See [examples/uart.rs](https://github.com/rp-rs/rp-hal/tree/main/rp2040-hal/examples/uart.rs) for a more complete example
//! ```no_run
//! use rp2040_hal::{clocks::init_clocks_and_plls, gpio::{Pins, FunctionUart}, pac, sio::Sio, uart::{self, UartPeripheral}, watchdog::Watchdog};
//!
//! const XOSC_CRYSTAL_FREQ: u32 = 12_000_000; // Typically found in BSP crates
//!
//! let mut peripherals = pac::Peripherals::take().unwrap();
//! let sio = Sio::new(peripherals.SIO);
//! let pins = Pins::new(peripherals.IO_BANK0, peripherals.PADS_BANK0, sio.gpio_bank0, &mut peripherals.RESETS);
//! let mut watchdog = Watchdog::new(peripherals.WATCHDOG);
//! let mut clocks = init_clocks_and_plls(XOSC_CRYSTAL_FREQ, peripherals.XOSC, peripherals.CLOCKS, peripherals.PLL_SYS, peripherals.PLL_USB, &mut peripherals.RESETS, &mut watchdog).ok().unwrap();
//!
//! // Set up UART on GP0 and GP1 (Pico pins 1 and 2)
//! let pins = (
//!     pins.gpio0.into_mode::<FunctionUart>(),
//!     pins.gpio1.into_mode::<FunctionUart>(),
//! );
//! // Need to perform clock init before using UART or it will freeze.
//! let uart = UartPeripheral::new(peripherals.UART0, pins, &mut peripherals.RESETS)
//!     .enable(
//!         uart::common_configs::_9600_8_N_1,
//!         clocks.peripheral_clock.into(),
//!     ).unwrap();
//!
//! uart.write_full_blocking(b"Hello World!\r\n");
//! ```

mod peripheral;
mod pins;
mod reader;
mod utils;
mod writer;

pub use self::peripheral::UartPeripheral;
pub use self::pins::*;
pub use self::reader::{ReadError, ReadErrorType, Reader};
pub use self::utils::*;
pub use self::writer::Writer;

/// Common configurations for UART.
pub mod common_configs;
