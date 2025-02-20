//! Universal Asynchronous Receiver Transmitter - Bi-directional Peripheral Code
//!
//! This module brings together `uart::reader` and `uart::writer` to give a
//! UartPeripheral object that can both read and write.

use core::{convert::Infallible, fmt};
use embedded_hal_0_2::serial as eh0;
use fugit::HertzU32;
use nb::Error::{Other, WouldBlock};

use crate::{
    pac::{self, uart0::uartlcr_h::W as UART_LCR_H_Writer, Peripherals, UART0, UART1},
    typelevel::OptionT,
    uart::*,
};

use embedded_hal_nb::serial::{ErrorType, Read, Write};

/// An UART Peripheral based on an underlying UART device.
pub struct UartPeripheral<S: State, D: UartDevice, P: ValidUartPinout<D>> {
    device: D,
    _state: S,
    pins: P,
    read_error: Option<ReadErrorType>,
}

impl<S: State, D: UartDevice, P: ValidUartPinout<D>> UartPeripheral<S, D, P> {
    fn transition<To: State>(self, state: To) -> UartPeripheral<To, D, P> {
        UartPeripheral {
            device: self.device,
            pins: self.pins,
            _state: state,
            read_error: None,
        }
    }

    /// Releases the underlying device and pins.
    pub fn free(self) -> (D, P) {
        (self.device, self.pins)
    }
}

impl<D: UartDevice, P: ValidUartPinout<D>> UartPeripheral<Disabled, D, P> {
    /// Creates an UartPeripheral in Disabled state.
    pub fn new(device: D, pins: P, resets: &mut pac::RESETS) -> UartPeripheral<Disabled, D, P> {
        device.reset_bring_down(resets);
        device.reset_bring_up(resets);

        UartPeripheral {
            device,
            _state: Disabled,
            pins,
            read_error: None,
        }
    }

    /// Enables the provided UART device with the given configuration.
    pub fn enable(
        self,
        config: UartConfig,
        frequency: HertzU32,
    ) -> Result<UartPeripheral<Enabled, D, P>, Error> {
        let (mut device, pins) = self.free();
        configure_baudrate(&mut device, config.baudrate, frequency)?;

        device.uartlcr_h().write(|w| {
            // FIFOs are enabled
            w.fen().set_bit(); // Leaved here for backward compatibility
            set_format(w, &config.data_bits, &config.stop_bits, &config.parity);
            w
        });

        // Enable the UART, and the TX,RC,CTS and RTS based on the pins
        device.uartcr().write(|w| {
            w.uarten().set_bit();
            w.txe().bit(P::Tx::IS_SOME);
            w.rxe().bit(P::Rx::IS_SOME);
            w.ctsen().bit(P::Cts::IS_SOME);
            w.rtsen().bit(P::Rts::IS_SOME);

            w
        });

        device.uartdmacr().write(|w| {
            w.txdmae().set_bit();
            w.rxdmae().set_bit();
            w
        });

        Ok(UartPeripheral {
            device,
            pins,
            _state: Enabled,
            read_error: None,
        })
    }
}

impl<D: UartDevice, P: ValidUartPinout<D>> UartPeripheral<Enabled, D, P> {
    /// Disable this UART Peripheral, falling back to the Disabled state.
    pub fn disable(self) -> UartPeripheral<Disabled, D, P> {
        // Disable the UART, both TX and RX
        self.device.uartcr().write(|w| {
            w.uarten().clear_bit();
            w.txe().clear_bit();
            w.rxe().clear_bit();
            w.ctsen().clear_bit();
            w.rtsen().clear_bit();
            w
        });

        self.transition(Disabled)
    }

    /// Enable/disable the rx/tx FIFO
    ///
    /// Unfortunately, it's not possible to enable/disable rx/tx
    /// independently on this chip
    /// Default is false
    pub fn set_fifos(&mut self, enable: bool) {
        super::reader::set_fifos(&self.device, enable)
    }

    /// Set rx FIFO watermark
    ///
    /// See DS: Table 423
    pub fn set_rx_watermark(&mut self, watermark: FifoWatermark) {
        super::reader::set_rx_watermark(&self.device, watermark)
    }

    /// Set tx FIFO watermark
    ///
    /// See DS: Table 423
    pub fn set_tx_watermark(&mut self, watermark: FifoWatermark) {
        super::writer::set_tx_watermark(&self.device, watermark)
    }

    /// Enables the Receive Interrupt.
    ///
    /// The relevant UARTx IRQ will fire when there is data in the receive register.
    pub fn enable_rx_interrupt(&mut self) {
        super::reader::enable_rx_interrupt(&self.device)
    }

    /// Enables the Transmit Interrupt.
    ///
    /// The relevant UARTx IRQ will fire when there is space in the transmit FIFO.
    pub fn enable_tx_interrupt(&mut self) {
        super::writer::enable_tx_interrupt(&self.device)
    }

    /// Disables the Receive Interrupt.
    pub fn disable_rx_interrupt(&mut self) {
        super::reader::disable_rx_interrupt(&self.device)
    }

    /// Disables the Transmit Interrupt.
    pub fn disable_tx_interrupt(&mut self) {
        super::writer::disable_tx_interrupt(&self.device)
    }

    /// Is there space in the UART TX FIFO for new data to be written?
    pub fn uart_is_writable(&self) -> bool {
        super::writer::uart_is_writable(&self.device)
    }

    /// Is the UART still busy transmitting data?
    pub fn uart_is_busy(&self) -> bool {
        super::writer::uart_is_busy(&self.device)
    }

    /// Is there data in the UART RX FIFO ready to be read?
    pub fn uart_is_readable(&self) -> bool {
        super::reader::is_readable(&self.device)
    }

    /// Writes bytes to the UART.
    /// This function writes as long as it can. As soon that the FIFO is full, if :
    /// - 0 bytes were written, a WouldBlock Error is returned
    /// - some bytes were written, it is deemed to be a success
    ///
    /// Upon success, the remaining slice is returned.
    pub fn write_raw<'d>(&self, data: &'d [u8]) -> nb::Result<&'d [u8], Infallible> {
        super::writer::write_raw(&self.device, data)
    }

    /// Reads bytes from the UART.
    /// This function reads as long as it can. As soon that the FIFO is empty, if :
    /// - 0 bytes were read, a WouldBlock Error is returned
    /// - some bytes were read, it is deemed to be a success
    ///
    /// Upon success, it will return how many bytes were read.
    pub fn read_raw<'b>(&self, buffer: &'b mut [u8]) -> nb::Result<usize, ReadError<'b>> {
        super::reader::read_raw(&self.device, buffer)
    }

    /// Writes bytes to the UART.
    ///
    /// This function blocks until the full buffer has been sent.
    pub fn write_full_blocking(&self, data: &[u8]) {
        super::writer::write_full_blocking(&self.device, data);
    }

    /// Reads bytes from the UART.
    ///
    /// This function blocks until the full buffer has been received.
    pub fn read_full_blocking(&self, buffer: &mut [u8]) -> Result<(), ReadErrorType> {
        super::reader::read_full_blocking(&self.device, buffer)
    }

    /// Initiates a break
    ///
    /// If transmitting, this takes effect immediately after the current byte has completed.  
    /// For proper execution of the break command, this must be held for at least 2 complete frames
    /// worth of time.
    ///
    /// <div class="warning">The device won’t be able to send anything while breaking.</div>
    ///
    /// # Example
    ///
    /// ```no_run
    /// # use rp235x_hal::uart::{Pins, ValidUartPinout, Enabled, UartPeripheral};
    /// # use rp235x_hal::pac::UART0;
    /// # use rp235x_hal::timer::{Timer, CopyableTimer0};
    /// # use rp235x_hal::typelevel::OptionTNone;
    /// # use embedded_hal_0_2::blocking::delay::DelayUs;
    /// # type PINS = Pins<OptionTNone, OptionTNone, OptionTNone, OptionTNone>;
    /// # fn example(mut serial: UartPeripheral<Enabled, rp235x_hal::pac::UART0, PINS>, mut timer: Timer<CopyableTimer0>) {
    /// serial.lowlevel_break_start();
    /// // at 115_200Bps on 8N1 configuration, 20bits takes (20*10⁶)/115200 = 173.611…μs.
    /// timer.delay_us(175);
    /// serial.lowlevel_break_stop();
    /// }
    /// ```
    pub fn lowlevel_break_start(&mut self) {
        self.device.uartlcr_h().modify(|_, w| w.brk().set_bit());
    }

    /// Terminates a break condition.
    ///
    /// See `lowlevel_break_start` for more details.
    pub fn lowlevel_break_stop(&mut self) {
        self.device.uartlcr_h().modify(|_, w| w.brk().clear_bit());
    }

    /// Join the reader and writer halves together back into the original Uart peripheral.
    ///
    /// A reader/writer pair can be obtained by calling [`split`].
    ///
    /// [`split`]: #method.split
    pub fn join(reader: Reader<D, P>, writer: Writer<D, P>) -> Self {
        let _ = writer;
        Self {
            device: reader.device,
            _state: Enabled,
            pins: reader.pins,
            read_error: reader.read_error,
        }
    }
}

impl<P: ValidUartPinout<UART0>> UartPeripheral<Enabled, UART0, P> {
    /// Split this peripheral into a separate reader and writer.
    pub fn split(self) -> (Reader<UART0, P>, Writer<UART0, P>) {
        let reader = Reader {
            device: self.device,
            pins: self.pins,
            read_error: self.read_error,
        };
        // Safety: reader and writer will never write to the same address
        let device_copy = unsafe { Peripherals::steal().UART0 };
        let writer = Writer {
            device: device_copy,
            device_marker: core::marker::PhantomData,
            pins: core::marker::PhantomData,
        };
        (reader, writer)
    }
}

impl<P: ValidUartPinout<UART1>> UartPeripheral<Enabled, UART1, P> {
    /// Split this peripheral into a separate reader and writer.
    pub fn split(self) -> (Reader<UART1, P>, Writer<UART1, P>) {
        let reader = Reader {
            device: self.device,
            pins: self.pins,
            read_error: self.read_error,
        };
        // Safety: reader and writer will never write to the same address
        let device_copy = unsafe { Peripherals::steal().UART1 };
        let writer = Writer {
            device: device_copy,
            device_marker: core::marker::PhantomData,
            pins: core::marker::PhantomData,
        };
        (reader, writer)
    }
}

/// The PL011 (PrimeCell UART) supports a fractional baud rate divider
/// From the wanted baudrate, we calculate the divider's two parts: integer and fractional parts.
/// Code inspired from the C SDK.
fn calculate_baudrate_dividers(
    wanted_baudrate: HertzU32,
    frequency: HertzU32,
) -> Result<(u16, u16), Error> {
    // See [Section 12.1.7.1](https://rptl.io/rp2350-datasheet#section_uart)
    // of the RP2350 datasheet for an explanation of how baudrate is calculated
    let baudrate_div = frequency
        .to_Hz()
        .checked_mul(8)
        .and_then(|r| r.checked_div(wanted_baudrate.to_Hz()))
        .ok_or(Error::BadArgument)?;

    Ok(
        match (baudrate_div >> 7, (baudrate_div & 0x7F).div_ceil(2)) {
            (0, _) => (1, 0),

            (int_part, _) if int_part >= 65535 => (65535, 0),

            (int_part, frac_part) => (int_part as u16, frac_part as u16),
        },
    )
}

/// Baudrate configuration. Code loosely inspired from the C SDK.
#[allow(unknown_lints)]
#[allow(clippy::needless_pass_by_ref_mut)]
fn configure_baudrate<U: UartDevice>(
    device: &mut U,
    wanted_baudrate: HertzU32,
    frequency: HertzU32,
) -> Result<HertzU32, Error> {
    let (baud_div_int, baud_div_frac) = calculate_baudrate_dividers(wanted_baudrate, frequency)?;

    // First we load the integer part of the divider.
    device.uartibrd().write(|w| unsafe {
        w.baud_divint().bits(baud_div_int);
        w
    });

    // Then we load the fractional part of the divider.
    device.uartfbrd().write(|w| unsafe {
        w.baud_divfrac().bits(baud_div_frac as u8);
        w
    });

    // PL011 needs a (dummy) line control register write to latch in the
    // divisors. We don't want to actually change LCR contents here.
    device.uartlcr_h().modify(|_, w| w);

    Ok(HertzU32::from_raw(
        (4 * frequency.to_Hz()) / (64 * baud_div_int as u32 + baud_div_frac as u32),
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

impl<D: UartDevice, P: ValidUartPinout<D>> eh0::Read<u8> for UartPeripheral<Enabled, D, P> {
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

impl<D: UartDevice, P: ValidUartPinout<D>> ErrorType for UartPeripheral<Enabled, D, P> {
    type Error = ReadErrorType;
}

impl<D: UartDevice, P: ValidUartPinout<D>> Read<u8> for UartPeripheral<Enabled, D, P> {
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

impl<D: UartDevice, P: ValidUartPinout<D>> eh0::Write<u8> for UartPeripheral<Enabled, D, P> {
    type Error = Infallible;

    fn write(&mut self, word: u8) -> nb::Result<(), Self::Error> {
        if self.write_raw(&[word]).is_err() {
            Err(WouldBlock)
        } else {
            Ok(())
        }
    }

    fn flush(&mut self) -> nb::Result<(), Self::Error> {
        super::writer::transmit_flushed(&self.device)
    }
}

impl<D: UartDevice, P: ValidUartPinout<D>> Write<u8> for UartPeripheral<Enabled, D, P> {
    fn write(&mut self, word: u8) -> nb::Result<(), Self::Error> {
        if self.write_raw(&[word]).is_err() {
            Err(WouldBlock)
        } else {
            Ok(())
        }
    }

    fn flush(&mut self) -> nb::Result<(), Self::Error> {
        super::writer::transmit_flushed(&self.device).map_err(|e| match e {
            WouldBlock => WouldBlock,
            Other(v) => match v {},
        })
    }
}

impl<D: UartDevice, P: ValidUartPinout<D>> fmt::Write for UartPeripheral<Enabled, D, P> {
    fn write_str(&mut self, s: &str) -> fmt::Result {
        s.bytes()
            .try_for_each(|c| nb::block!(self.write(c)))
            .map_err(|_| fmt::Error)
    }
}

impl embedded_io::Error for ReadErrorType {
    fn kind(&self) -> embedded_io::ErrorKind {
        embedded_io::ErrorKind::Other
    }
}
impl<D: UartDevice, P: ValidUartPinout<D>> embedded_io::ErrorType
    for UartPeripheral<Enabled, D, P>
{
    type Error = ReadErrorType;
}
impl<D: UartDevice, P: ValidUartPinout<D>> embedded_io::Read for UartPeripheral<Enabled, D, P> {
    fn read(&mut self, buf: &mut [u8]) -> Result<usize, Self::Error> {
        // If the last read stored an error, report it now
        if let Some(err) = self.read_error.take() {
            return Err(err);
        }
        match nb::block!(self.read_raw(buf)) {
            Ok(bytes_read) => Ok(bytes_read),
            Err(err) if !err.discarded.is_empty() => {
                // If an error was reported but some bytes were already read,
                // return the data now and store the error for the next
                // invocation.
                self.read_error = Some(err.err_type);
                Ok(err.discarded.len())
            }
            Err(err) => Err(err.err_type),
        }
    }
}

impl<D: UartDevice, P: ValidUartPinout<D>> embedded_io::ReadReady
    for UartPeripheral<Enabled, D, P>
{
    fn read_ready(&mut self) -> Result<bool, Self::Error> {
        Ok(self.uart_is_readable() || self.read_error.is_some())
    }
}

impl<D: UartDevice, P: ValidUartPinout<D>> embedded_io::Write for UartPeripheral<Enabled, D, P> {
    fn write(&mut self, buf: &[u8]) -> Result<usize, Self::Error> {
        // Blocks if and only if no bytes can be written.
        let remaining = nb::block!(super::writer::write_raw(&self.device, buf)).unwrap(); // Infallible
        Ok(buf.len() - remaining.len())
    }
    fn flush(&mut self) -> Result<(), Self::Error> {
        nb::block!(super::writer::transmit_flushed(&self.device)).unwrap(); // Infallible
        Ok(())
    }
}

impl<D: UartDevice, P: ValidUartPinout<D>> embedded_io::WriteReady
    for UartPeripheral<Enabled, D, P>
{
    fn write_ready(&mut self) -> Result<bool, Self::Error> {
        Ok(self.uart_is_writable())
    }
}
