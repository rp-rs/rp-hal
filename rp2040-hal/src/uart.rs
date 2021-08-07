//! Universal Asynchronous Receiver Transmitter (UART)
//!
//! See [Chapter 4 Section 2](https://datasheets.raspberrypi.org/rp2040/rp2040_datasheet.pdf) of the datasheet for more details
//!
//! ## Usage
//!
//! See [examples/uart.rs](https://github.com/rp-rs/rp-hal/tree/main/rp2040-hal/examples/uart.rs) for a more complete example
//! ```rust
//! // Need to perform clock init before using UART or it will freeze.
//! // Skipping it here for brevity
//! let uart = UartPeripheral::<_, _>::enable(
//!         pac.UART0,
//!         &mut pac.RESETS,
//!         hal::uart::common_configs::_9600_8_N_1,
//!         clocks.peripheral_clock.into(),
//!     ).unwrap();
//!
//! // Set up UART on GP0 and GP1 (Pico pins 1 and 2)
//! let _tx_pin = pins.gpio0.into_mode::<gpio::FunctionUart>();
//! let _rx_pin = pins.gpio1.into_mode::<gpio::FunctionUart>();
//! uart.write_full_blocking(b"Hello World!\r\n");
//! ```

use core::convert::Infallible;
use core::ops::Deref;
use embedded_time::fixed_point::FixedPoint;
use embedded_time::rate::Baud;
use embedded_time::rate::Hertz;

use embedded_hal::serial::{Read, Write};

use nb::Error::{Other, WouldBlock};

use crate::pac::{
    uart0::{uartlcr_h::W as UART_LCR_H_Writer, RegisterBlock},
    UART0, UART1,
};

use crate::resets::SubsystemReset;

/// Error type for UART operations.
#[derive(Debug)]
pub enum Error {
    /// Bad argument : when things overflow, ...
    BadArgument,
}

/// When there's a read error.
pub struct ReadError<'err> {
    /// The type of error
    pub err_type: ReadErrorType,

    /// Reference to the data that was read but eventually discared because of the error.
    pub discared: &'err [u8],
}

/// Possible types of read errors. See Chapter 4, Section 2 §8 - Table 436: "UARTDR Register"
pub enum ReadErrorType {
    /// Triggered when the FIFO (or shift-register) is overflowed.
    Overrun,

    /// Triggered when a break is received
    Break,

    /// Triggered when there is a parity mismatch between what's received and our settings.
    Parity,

    /// Triggered when the received character didn't have a valid stop bit.
    Framing,
}

/// State of the UART Peripheral.
pub trait State {}

/// Trait to handle both underlying devices (UART0 & UART1)
pub trait UartDevice: Deref<Target = RegisterBlock> + SubsystemReset {}

impl UartDevice for UART0 {}
impl UartDevice for UART1 {}

/// UART is enabled.
pub struct Enabled;

/// UART is disabled.
pub struct Disabled;

impl State for Enabled {}
impl State for Disabled {}

/// Data bits
pub enum DataBits {
    /// 5 bits
    Five,
    /// 6 bits
    Six,
    /// 7 bits
    Seven,
    /// 8 bits
    Eight,
}

/// Stop bits
pub enum StopBits {
    /// 1 bit
    One,

    /// 2 bits
    Two,
}

/// Parity
/// The "none" state of parity is represented with the Option type (None).
pub enum Parity {
    /// Odd parity
    Odd,

    /// Even parity
    Even,
}

/// A struct holding the configuration for an UART device.
pub struct UartConfig {
    baudrate: Baud,
    data_bits: DataBits,
    stop_bits: StopBits,
    parity: Option<Parity>,
}

/// Common configurations for UART.
pub mod common_configs {
    use super::{DataBits, StopBits, UartConfig};
    use embedded_time::rate::Baud;

    /// 9600 baud, 8 data bits, no parity, 1 stop bit
    pub const _9600_8_N_1: UartConfig = UartConfig {
        baudrate: Baud(9600),
        data_bits: DataBits::Eight,
        stop_bits: StopBits::One,
        parity: None,
    };

    /// 19200 baud, 8 data bits, no parity, 1 stop bit
    pub const _19200_8_N_1: UartConfig = UartConfig {
        baudrate: Baud(19200),
        data_bits: DataBits::Eight,
        stop_bits: StopBits::One,
        parity: None,
    };

    /// 38400 baud, 8 data bits, no parity, 1 stop bit
    pub const _38400_8_N_1: UartConfig = UartConfig {
        baudrate: Baud(38400),
        data_bits: DataBits::Eight,
        stop_bits: StopBits::One,
        parity: None,
    };

    /// 57600 baud, 8 data bits, no parity, 1 stop bit
    pub const _57600_8_N_1: UartConfig = UartConfig {
        baudrate: Baud(57600),
        data_bits: DataBits::Eight,
        stop_bits: StopBits::One,
        parity: None,
    };

    /// 115200 baud, 8 data bits, no parity, 1 stop bit
    pub const _115200_8_N_1: UartConfig = UartConfig {
        baudrate: Baud(115200),
        data_bits: DataBits::Eight,
        stop_bits: StopBits::One,
        parity: None,
    };
}

/// An UART Peripheral based on an underlying UART device.
pub struct UartPeripheral<S: State, D: UartDevice> {
    device: D,
    _state: S,
    config: UartConfig,
    effective_baudrate: Baud,
}

impl<S: State, D: UartDevice> UartPeripheral<S, D> {
    fn transition<To: State>(self, state: To) -> UartPeripheral<To, D> {
        UartPeripheral {
            device: self.device,
            config: self.config,
            effective_baudrate: self.effective_baudrate,
            _state: state,
        }
    }

    /// Releases the underlying device.
    pub fn free(self) -> D {
        self.device
    }
}

impl<D: UartDevice> UartPeripheral<Disabled, D> {
    /// Enables the provided UART device with the given configuration.
    pub fn enable(
        mut device: D,
        resets: &mut pac::RESETS,
        config: UartConfig,
        frequency: Hertz,
    ) -> Result<UartPeripheral<Enabled, D>, Error> {
        device.reset_bring_up(resets);

        let effective_baudrate = configure_baudrate(&mut device, &config.baudrate, &frequency)?;

        // Enable the UART, both TX and RX
        device.uartcr.write(|w| {
            w.uarten().set_bit();
            w.txe().set_bit();
            w.rxe().set_bit();
            w
        });

        device.uartlcr_h.write(|w| {
            w.fen().set_bit();

            set_format(w, &config.data_bits, &config.stop_bits, &config.parity);
            w
        });

        device.uartdmacr.write(|w| {
            w.txdmae().set_bit();
            w.rxdmae().set_bit();
            w
        });

        Ok(UartPeripheral {
            device,
            config,
            effective_baudrate,
            _state: Enabled,
        })
    }
}

impl<D: UartDevice> UartPeripheral<Enabled, D> {
    /// Disable this UART Peripheral, falling back to the Disabled state.
    pub fn disable(self) -> UartPeripheral<Disabled, D> {
        // Disable the UART, both TX and RX
        self.device.uartcr.write(|w| {
            w.uarten().clear_bit();
            w.txe().clear_bit();
            w.rxe().clear_bit();
            w
        });

        self.transition(Disabled)
    }

    pub(crate) fn transmit_flushed(&self) -> nb::Result<(), Infallible> {
        if self.device.uartfr.read().txfe().bit_is_set() {
            Ok(())
        } else {
            Err(WouldBlock)
        }
    }

    fn uart_is_writable(&self) -> bool {
        self.device.uartfr.read().txff().bit_is_clear()
    }

    fn uart_is_readable(&self) -> bool {
        self.device.uartfr.read().rxfe().bit_is_clear()
    }

    /// Writes bytes to the UART.
    /// This function writes as long as it can. As soon that the FIFO is full, if :
    /// - 0 bytes were written, a WouldBlock Error is returned
    /// - some bytes were written, it is deemed to be a success
    /// Upon success, the remaining slice is returned.
    pub fn write_raw<'d>(&self, data: &'d [u8]) -> nb::Result<&'d [u8], Infallible> {
        let mut bytes_written = 0;

        for c in data {
            if !self.uart_is_writable() {
                if bytes_written == 0 {
                    return Err(WouldBlock);
                } else {
                    return Ok(&data[bytes_written..]);
                }
            }

            self.device.uartdr.write(|w| unsafe {
                w.data().bits(*c);
                w
            });

            bytes_written += 1;
        }
        Ok(&data[bytes_written..])
    }

    /// Reads bytes from the UART.
    /// This function reads as long as it can. As soon that the FIFO is empty, if :
    /// - 0 bytes were read, a WouldBlock Error is returned
    /// - some bytes were read, it is deemed to be a success
    /// Upon success, the remaining slice is returned.
    pub fn read_raw<'b>(&self, buffer: &'b mut [u8]) -> nb::Result<&'b mut [u8], ReadError<'b>> {
        let mut bytes_read = 0;

        Ok(loop {
            if !self.uart_is_readable() {
                if bytes_read == 0 {
                    return Err(WouldBlock);
                } else {
                    break &mut buffer[bytes_read..];
                }
            }

            if bytes_read < buffer.len() {
                let mut error: Option<ReadErrorType> = None;

                if self.device.uartdr.read().oe().bit_is_set() {
                    error = Some(ReadErrorType::Overrun);
                }

                if self.device.uartdr.read().be().bit_is_set() {
                    error = Some(ReadErrorType::Break);
                }

                if self.device.uartdr.read().pe().bit_is_set() {
                    error = Some(ReadErrorType::Parity);
                }

                if self.device.uartdr.read().fe().bit_is_set() {
                    error = Some(ReadErrorType::Framing);
                }

                if let Some(err_type) = error {
                    return Err(Other(ReadError {
                        err_type,
                        discared: buffer,
                    }));
                }

                buffer[bytes_read] = self.device.uartdr.read().data().bits();
                bytes_read += 1;
            } else {
                break &mut buffer[bytes_read..];
            }
        })
    }

    /// Writes bytes to the UART.
    /// This function blocks until the full buffer has been sent.
    pub fn write_full_blocking(&self, data: &[u8]) {
        let mut temp = data;

        while !temp.is_empty() {
            temp = match self.write_raw(temp) {
                Ok(remaining) => remaining,
                Err(WouldBlock) => continue,
                Err(_) => unreachable!(),
            }
        }
    }

    /// Reads bytes from the UART.
    /// This function blocks until the full buffer has been received.
    pub fn read_full_blocking(&self, buffer: &mut [u8]) -> Result<(), ReadErrorType> {
        let mut offset = 0;

        while offset != buffer.len() {
            offset += match self.read_raw(&mut buffer[offset..]) {
                Ok(remaining) => remaining.len(),
                Err(e) => match e {
                    Other(inner) => return Err(inner.err_type),
                    WouldBlock => continue,
                },
            }
        }

        Ok(())
    }
}

/// The PL011 (PrimeCell UART) supports a fractional baud rate divider
/// From the wanted baudrate, we calculate the divider's two parts: integer and fractional parts.
/// Code inspired from the C SDK.
fn calculate_baudrate_dividers(
    wanted_baudrate: &Baud,
    frequency: &Hertz,
) -> Result<(u16, u16), Error> {
    // See Chapter 4, Section 2 §7.1 from the datasheet for an explanation of how baudrate is
    // calculated
    let baudrate_div = frequency
        .integer()
        .checked_mul(8)
        .and_then(|r| r.checked_div(wanted_baudrate.integer()))
        .ok_or(Error::BadArgument)?;

    Ok(match (baudrate_div >> 7, ((baudrate_div & 0x7F) + 1) / 2) {
        (0, _) => (1, 0),

        (int_part, _) if int_part >= 65535 => (65535, 0),

        (int_part, frac_part) => (int_part as u16, frac_part as u16),
    })
}

/// Baudrate configuration. Code loosely inspired from the C SDK.
fn configure_baudrate(
    device: &mut dyn UartDevice,
    wanted_baudrate: &Baud,
    frequency: &Hertz,
) -> Result<Baud, Error> {
    let (baud_div_int, baud_div_frac) = calculate_baudrate_dividers(wanted_baudrate, frequency)?;

    // First we load the integer part of the divider.
    device.uartibrd.write(|w| unsafe {
        w.baud_divint().bits(baud_div_int as u16);
        w
    });

    // Then we load the fractional part of the divider.
    device.uartfbrd.write(|w| unsafe {
        w.baud_divfrac().bits(baud_div_frac as u8);
        w
    });

    // PL011 needs a (dummy) line control register write to latch in the
    // divisors. We don't want to actually change LCR contents here.
    device.uartlcr_h.modify(|_, w| w);

    Ok(Baud(
        (4 * frequency.integer()) / (64 * baud_div_int + baud_div_frac) as u32,
    ))
}

/// Format configuration. Code loosely inspired from the C SDK.
fn set_format<'w>(
    w: &'w mut UART_LCR_H_Writer,
    data_bits: &DataBits,
    stop_bits: &StopBits,
    parity: &Option<Parity>,
) -> &'w mut UART_LCR_H_Writer {
    match parity {
        Some(p) => {
            w.pen().set_bit();
            match p {
                Parity::Odd => w.eps().clear_bit(),
                Parity::Even => w.eps().set_bit(),
            };
        }
        None => {
            w.pen().bit(false);
        }
    };

    unsafe {
        w.wlen().bits(match data_bits {
            DataBits::Five => 0b00,
            DataBits::Six => 0b01,
            DataBits::Seven => 0b10,
            DataBits::Eight => 0b11,
        })
    };

    match stop_bits {
        StopBits::One => w.stp2().clear_bit(),
        StopBits::Two => w.stp2().set_bit(),
    };

    w
}

impl<D: UartDevice> Read<u8> for UartPeripheral<Enabled, D> {
    type Error = ReadErrorType;

    fn read(&mut self) -> nb::Result<u8, Self::Error> {
        let byte: &mut [u8] = &mut [0; 1];

        match self.read_raw(byte) {
            Ok(_) => Ok(byte[0]),
            Err(e) => match e {
                Other(inner) => Err(Other(inner.err_type)),
                WouldBlock => Err(WouldBlock),
            },
        }
    }
}

impl<D: UartDevice> Write<u8> for UartPeripheral<Enabled, D> {
    type Error = Infallible;

    fn write(&mut self, word: u8) -> nb::Result<(), Self::Error> {
        if self.write_raw(&[word]).is_err() {
            Err(WouldBlock)
        } else {
            Ok(())
        }
    }

    fn flush(&mut self) -> nb::Result<(), Self::Error> {
        self.transmit_flushed()
    }
}
