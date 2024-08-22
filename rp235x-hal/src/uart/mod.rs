//! Universal Asynchronous Receiver Transmitter (UART)
//!
//! See [Section 12.1](https://rptl.io/rp2350-datasheet#section_uart) of the datasheet for more details.
//!
//! ## Usage
//!
//! See [examples/uart.rs](https://github.com/rp-rs/rp-hal/tree/main/rp235x-hal-examples/src/bin/uart.rs) for a more complete example.
//!
//! ```no_run
//! use fugit::RateExtU32;
//! use rp235x_hal::{
//!     self as hal,
//!     clocks::init_clocks_and_plls,
//!     gpio::{FunctionUart, Pins},
//!     pac,
//!     sio::Sio,
//!     uart::{self, DataBits, StopBits, UartConfig, UartPeripheral},
//!     watchdog::Watchdog,
//!     Clock,
//! };
//!
//! const XOSC_CRYSTAL_FREQ: u32 = 12_000_000; // Typically found in BSP crates
//!
//! let mut peripherals = hal::pac::Peripherals::take().unwrap();
//! let sio = Sio::new(peripherals.SIO);
//! let pins = Pins::new(
//!     peripherals.IO_BANK0,
//!     peripherals.PADS_BANK0,
//!     sio.gpio_bank0,
//!     &mut peripherals.RESETS,
//! );
//! let mut watchdog = Watchdog::new(peripherals.WATCHDOG);
//! let mut clocks = init_clocks_and_plls(
//!     XOSC_CRYSTAL_FREQ,
//!     peripherals.XOSC,
//!     peripherals.CLOCKS,
//!     peripherals.PLL_SYS,
//!     peripherals.PLL_USB,
//!     &mut peripherals.RESETS,
//!     &mut watchdog,
//! )
//! .ok()
//! .unwrap();
//!
//! // Set up UART on GP0 and GP1 (Pico pins 1 and 2)
//! let pins = (pins.gpio0.into_function(), pins.gpio1.into_function());
//! // Need to perform clock init before using UART or it will freeze.
//! let uart = UartPeripheral::new(peripherals.UART0, pins, &mut peripherals.RESETS)
//!     .enable(
//!         UartConfig::new(9600.Hz(), DataBits::Eight, None, StopBits::One),
//!         clocks.peripheral_clock.freq(),
//!     )
//!     .unwrap();
//!
//! uart.write_full_blocking(b"Hello World!\r\n");
//! ```

mod peripheral;
mod pins;
mod reader;
mod utils;
mod writer;

pub use peripheral::UartPeripheral;
pub use pins::*;
pub use reader::{ReadError, ReadErrorType, Reader};
pub use utils::*;
pub use writer::Writer;

/// Common configurations for UART.
#[deprecated(note = "Use UartConfig::new(...) instead.")]
pub mod common_configs;
