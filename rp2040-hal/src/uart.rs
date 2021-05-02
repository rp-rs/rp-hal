//! Universal Asynchronous Receiver Transmitter (UART)
// See [Chapter 4 Section 2](https://datasheets.raspberrypi.org/rp2040/rp2040_datasheet.pdf) for more details

use core::convert::Infallible;
use core::ops::Deref;
use embedded_time::rate::Baud;
use embedded_time::rate::Hertz;
use embedded_time::fixed_point::FixedPoint;

use embedded_hal::serial::{
    Read,
    Write
};

use nb::Error::{
    WouldBlock,
    Other
};

use crate::pac::{
    uart0::{
        uartlcr_h::W as UART_LCR_H_Writer,
        RegisterBlock
    },
    UART0,
    UART1

};

/// Error type for UART operations.
#[derive(Debug)]
pub enum Error {
    /// Bad argument : when things overflow, ...
    BadArgument
}


/// When there's a read error.
pub struct ReadError<'err> {
    /// The type of error
    pub err_type: ReadErrorType,

    /// Reference to the data that was read but eventually discared because of the error.
    pub discared: &'err [u8]
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
    Framing
}

/// State of the UART Peripheral.
pub trait State {}

/// Trait to handle both underlying devices (UART0 & UART1)
pub trait UARTDevice: Deref<Target = RegisterBlock> {}

impl UARTDevice for UART0 {}
impl UARTDevice for UART1 {}

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
    Eight
}

/// Stop bits
pub enum StopBits {
    /// 1 bit
    One,

    /// 2 bits
    Two
}

/// Parity
/// The "none" state of parity is represented with the Option type (None).
pub enum Parity {
    /// Odd parity
    Odd,

    /// Even parity
    Even
}


/// A struct holding the configuration for an UART device.
pub struct UARTConfig {
    baudrate: Baud,
    data_bits: DataBits,
    stop_bits: StopBits,
    parity: Option<Parity>
}

/// Common configurations for UART.
pub mod common_configs {
    use super::{ UARTConfig, DataBits, StopBits };
    use embedded_time::rate::Baud;

    /// 9600 baud, 8 data bits, no parity, 1 stop bit
    pub const _9600_8_N_1: UARTConfig = UARTConfig {
        baudrate: Baud(9600),
        data_bits: DataBits::Eight,
        stop_bits: StopBits::One,
        parity: None
    };

    /// 19200 baud, 8 data bits, no parity, 1 stop bit
    pub const _19200_8_N_1: UARTConfig = UARTConfig {
        baudrate: Baud(19200),
        data_bits: DataBits::Eight,
        stop_bits: StopBits::One,
        parity: None
    };

    /// 38400 baud, 8 data bits, no parity, 1 stop bit
    pub const _38400_8_N_1: UARTConfig = UARTConfig {
        baudrate: Baud(38400),
        data_bits: DataBits::Eight,
        stop_bits: StopBits::One,
        parity: None
    };

    /// 57600 baud, 8 data bits, no parity, 1 stop bit
    pub const _57600_8_N_1: UARTConfig = UARTConfig {
        baudrate: Baud(57600),
        data_bits: DataBits::Eight,
        stop_bits: StopBits::One,
        parity: None
    };

    /// 115200 baud, 8 data bits, no parity, 1 stop bit
    pub const _115200_8_N_1: UARTConfig = UARTConfig {
        baudrate: Baud(115200),
        data_bits: DataBits::Eight,
        stop_bits: StopBits::One,
        parity: None
    };
}

/// An UART Peripheral based on an underlying UART device.
pub struct UARTPeripheral<S: State, D: UARTDevice> {
    device: D,
    _state: S,
    config: UARTConfig,
    effective_baudrate: Baud
}

impl<S: State, D: UARTDevice> UARTPeripheral<S, D> {
    fn transition<To: State>(self, state: To) -> UARTPeripheral<To, D> {
        UARTPeripheral {
            device: self.device,
            config: self.config,
            effective_baudrate: self.effective_baudrate,
            _state: state
        }
    }

    /// Releases the underlying device.
    pub fn free(self) -> D{
        self.device
    }
}

impl<D: UARTDevice> UARTPeripheral<Disabled, D> {

    /// Enables the provided UART device with the given configuration.
    pub fn enable(mut device: D, config: UARTConfig, frequency: Hertz) -> Result<UARTPeripheral<Enabled, D>, Error> {

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

        Ok(UARTPeripheral {
            device, config, effective_baudrate, _state: Enabled
        })
    }
}

impl<D: UARTDevice> UARTPeripheral<Enabled, D> {

    /// Disable this UART Peripheral, falling back to the Disabled state.
    pub fn disable(self) -> UARTPeripheral<Disabled, D> {

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
        }
        else { Err(WouldBlock) }
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
    /// Upon success, the number of written bytes is returned.
    pub fn write_raw <'d>(&self, data: &'d [u8]) -> nb::Result<&'d [u8], Infallible> {

        let mut bytes_written = 0;

        for c in data {

            if !self.uart_is_writable() {
                if bytes_written == 0 {
                    return Err(WouldBlock)
                }
                else {
                    return Ok(&data[bytes_written..])
                }
            }

            self.device.uartdr.write(|w| unsafe {
                w.data().bits(*c);
                w
            });

            bytes_written += 1;
        }
        return Ok(&data[bytes_written..])
    }

    /// Reads bytes from the UART.
    /// This function reads as long as it can. As soon that the FIFO is empty, if :
    /// - 0 bytes were read, a WouldBlock Error is returned
    /// - some bytes were read, it is deemed to be a success
    /// Upon success, the number of read bytes is returned.
    pub fn read_raw<'b>(&self, buffer: &'b mut [u8]) -> nb::Result<&'b mut [u8], ReadError<'b>> {

        let mut bytes_read = 0;

        Ok(loop {
            if !self.uart_is_readable() {
                if bytes_read == 0 {
                    return Err(WouldBlock)
                }
                else {
                    break &mut buffer[bytes_read..]
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
                        err_type, discared: buffer
                    }));
                }

                buffer[bytes_read] = self.device.uartdr.read().data().bits();
                bytes_read += 1;
            }
            else {
                break &mut buffer[bytes_read..]
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
                Err(_) => unreachable!()
            }
        }
    }

    /// Reads bytes from the UART.
    /// This function blocks until the full buffer has been received.
    pub fn read_full_blocking(&self, buffer: &mut [u8]) {
        let mut offset = 0;

        while offset != buffer.len() {
            offset += match self.read_raw(&mut buffer[offset..]) {
                Ok(remaining) => { remaining.len() },
                Err(WouldBlock) => continue,
                Err(_) => unreachable!()
            }
        }
    }
}

/// Baudrate dividers calculation. Code inspired from the C SDK.
fn calculate_baudrate_dividers(wanted_baudrate: &Baud, frequency: &Hertz) -> Result<(u16, u16), Error> {

    // baudrate_div = frequency * 8 / wanted_baudrate
    let baudrate_div = frequency.checked_mul(&8).
        and_then(|r| r.checked_div(wanted_baudrate.integer())).
        ok_or(Error::BadArgument)?;

    let baudrate_div: u32 = *baudrate_div.integer();

    Ok(match (baudrate_div >> 7, ((baudrate_div & 0x7F) + 1) / 2) {

        (0, _) => (1, 0),

        (ibrd, _) if ibrd >= 65535 => (65535, 0),

        (ibrd, fbrd) => (ibrd as u16, fbrd as u16)
    })
}

/// Baudrate configuration. Code loosely inspired from the C SDK.
fn configure_baudrate(device: &mut dyn UARTDevice, wanted_baudrate: &Baud, frequency: &Hertz) -> Result<Baud, Error> {

    let (baud_ibrd, baud_fbrd) = calculate_baudrate_dividers(wanted_baudrate, frequency)?;

    // Load PL011's baud divisor registers
    device.uartibrd.write(|w| unsafe {
        w.baud_divint().bits(baud_ibrd as u16);
        w
    });
    device.uartfbrd.write(|w| unsafe {
        w.baud_divfrac().bits(baud_fbrd as u8);
        w
    });

    // PL011 needs a (dummy) line control register write to latch in the
    // divisors. We don't want to actually change LCR contents here.
    device.uartlcr_h.modify(|_,w| { w });

    Ok(Baud((4 * *frequency.integer()) / (64 * baud_ibrd + baud_fbrd) as u32))
}


/// Format configuration. Code loosely inspired from the C SDK.
fn set_format<'w>(w: &'w mut UART_LCR_H_Writer, data_bits: &DataBits, stop_bits: &StopBits, parity: &Option<Parity>) -> &'w mut UART_LCR_H_Writer {

    match parity {
        Some(p) => {
            w.pen().set_bit();
            match p {
                Parity::Odd => w.eps().bit(false),
                Parity::Even => w.eps().set_bit()
            };
        },
        None => { w.pen().bit(false); }
    };

    unsafe { w.wlen().bits(
        match data_bits {
                DataBits::Five => { 0b00 }
                DataBits::Six => { 0b01 }
                DataBits::Seven => { 0b10 }
                DataBits::Eight => { 0b11 }
            }
        )
    };

    match stop_bits {
        StopBits::One => w.stp2().bit(false),
        StopBits::Two => w.stp2().set_bit()
    };

    w
}

impl<D: UARTDevice> Read<u8> for UARTPeripheral<Enabled, D> {
    type Error = Infallible;

    fn read(&mut self) -> nb::Result<u8, Self::Error> {

        let byte: &mut [u8] = &mut [0; 1];

        if let Err(_) = self.read_raw(byte) {
            Err(WouldBlock)
        }
        else {
            Ok(byte[0])
        }
    }
}

impl<D: UARTDevice> Write<u8> for UARTPeripheral<Enabled, D> {
    type Error = Infallible;

    fn write(&mut self, word: u8) -> nb::Result<(), Self::Error> {
        if let Err(_) = self.write_raw(&[word]) {
            Err(WouldBlock)
        }
        else {
            Ok(())
        }
    }

    fn flush(&mut self) -> nb::Result<(), Self::Error> {
        self.transmit_flushed()
    }
}
