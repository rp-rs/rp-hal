//! Universal Asynchronous Receiver Transmitter - Transmitter Code
//!
//! This module is for transmitting data with a UART.

use super::{UartDevice, ValidUartPinout};
use core::fmt;
use core::{convert::Infallible, marker::PhantomData};
use embedded_hal::serial::Write;
use nb::Error::*;
use rp2040_pac::uart0::RegisterBlock;

#[cfg(feature = "eh1_0_alpha")]
use eh1_0_alpha::serial as eh1;

/// Returns `Err(WouldBlock)` if the UART TX FIFO still has data in it or
/// `Ok(())` if the FIFO is empty.
pub(crate) fn transmit_flushed(rb: &RegisterBlock) -> nb::Result<(), Infallible> {
    if rb.uartfr.read().txfe().bit_is_set() {
        Ok(())
    } else {
        Err(WouldBlock)
    }
}

/// Returns `true` if the TX FIFO has space, or false if it is full
pub(crate) fn uart_is_writable(rb: &RegisterBlock) -> bool {
    rb.uartfr.read().txff().bit_is_clear()
}

/// Writes bytes to the UART.
///
/// This function writes as long as it can. As soon that the FIFO is full,
/// if:
/// - 0 bytes were written, a WouldBlock Error is returned
/// - some bytes were written, it is deemed to be a success
///
/// Upon success, the remaining (unwritten) slice is returned.
pub(crate) fn write_raw<'d>(
    rb: &RegisterBlock,
    data: &'d [u8],
) -> nb::Result<&'d [u8], Infallible> {
    let mut bytes_written = 0;

    for c in data {
        if !uart_is_writable(rb) {
            if bytes_written == 0 {
                return Err(WouldBlock);
            } else {
                return Ok(&data[bytes_written..]);
            }
        }

        rb.uartdr.write(|w| unsafe {
            w.data().bits(*c);
            w
        });

        bytes_written += 1;
    }
    Ok(&data[bytes_written..])
}

/// Writes bytes to the UART.
///
/// This function blocks until the full buffer has been sent.
pub(crate) fn write_full_blocking(rb: &RegisterBlock, data: &[u8]) {
    let mut temp = data;

    while !temp.is_empty() {
        temp = match write_raw(rb, temp) {
            Ok(remaining) => remaining,
            Err(WouldBlock) => continue,
            Err(_) => unreachable!(),
        }
    }
}

/// Enables the Transmit Interrupt.
///
/// The relevant UARTx IRQ will fire when there is space in the transmit FIFO.
pub(crate) fn enable_tx_interrupt(rb: &RegisterBlock) {
    // Access the UART FIFO Level Select. We set the TX FIFO trip level
    // to be when it's half-empty..

    // 2 means '<= 1/2 full'.
    rb.uartifls.modify(|_r, w| unsafe { w.txiflsel().bits(2) });

    // Access the UART Interrupt Mask Set/Clear register. Setting a bit
    // high enables the interrupt.

    // We set the TX interrupt. This means we will get an interrupt when
    // the TX FIFO level is triggered. This means we don't have to
    // interrupt on every single byte, but can make use of the hardware
    // FIFO.
    rb.uartimsc.modify(|_r, w| {
        w.txim().set_bit();
        w
    });
}

/// Disables the Transmit Interrupt.
pub(crate) fn disable_tx_interrupt(rb: &RegisterBlock) {
    // Access the UART Interrupt Mask Set/Clear register. Setting a bit
    // low disables the interrupt.

    rb.uartimsc.modify(|_r, w| {
        w.txim().clear_bit();
        w
    });
}

/// Half of an [`UartPeripheral`] that is only capable of writing. Obtained by calling [`UartPeripheral::split()`]
///
/// [`UartPeripheral`]: struct.UartPeripheral.html
/// [`UartPeripheral::split()`]: struct.UartPeripheral.html#method.split
pub struct Writer<D: UartDevice, P: ValidUartPinout<D>> {
    pub(super) device: D,
    pub(super) device_marker: PhantomData<D>,
    pub(super) pins: PhantomData<P>,
}

impl<D: UartDevice, P: ValidUartPinout<D>> Writer<D, P> {
    /// Writes bytes to the UART.
    ///
    /// This function writes as long as it can. As soon that the FIFO is full,
    /// if:
    /// - 0 bytes were written, a WouldBlock Error is returned
    /// - some bytes were written, it is deemed to be a success
    ///
    /// Upon success, the remaining (unwritten) slice is returned.
    pub fn write_raw<'d>(&self, data: &'d [u8]) -> nb::Result<&'d [u8], Infallible> {
        write_raw(&self.device, data)
    }

    /// Writes bytes to the UART.
    ///
    /// This function blocks until the full buffer has been sent.
    pub fn write_full_blocking(&self, data: &[u8]) {
        write_full_blocking(&self.device, data);
    }

    /// Enables the Transmit Interrupt.
    ///
    /// The relevant UARTx IRQ will fire when there is space in the transmit FIFO.
    pub fn enable_tx_interrupt(&mut self) {
        enable_tx_interrupt(&self.device)
    }

    /// Disables the Transmit Interrupt.
    pub fn disable_tx_interrupt(&mut self) {
        disable_tx_interrupt(&self.device)
    }
}

impl<D: UartDevice, P: ValidUartPinout<D>> Write<u8> for Writer<D, P> {
    type Error = Infallible;

    fn write(&mut self, word: u8) -> nb::Result<(), Self::Error> {
        if self.write_raw(&[word]).is_err() {
            Err(WouldBlock)
        } else {
            Ok(())
        }
    }

    fn flush(&mut self) -> nb::Result<(), Self::Error> {
        transmit_flushed(&self.device)
    }
}

#[cfg(feature = "eh1_0_alpha")]
impl<D: UartDevice, P: ValidUartPinout<D>> eh1::ErrorType for Writer<D, P> {
    type Error = super::utils::SerialInfallible;
}

#[cfg(feature = "eh1_0_alpha")]
impl<D: UartDevice, P: ValidUartPinout<D>> eh1::nb::Write<u8> for Writer<D, P> {
    fn write(&mut self, word: u8) -> nb::Result<(), Self::Error> {
        if self.write_raw(&[word]).is_err() {
            Err(WouldBlock)
        } else {
            Ok(())
        }
    }

    fn flush(&mut self) -> nb::Result<(), Self::Error> {
        transmit_flushed(&self.device).map_err(|e| match e {
            WouldBlock => WouldBlock,
            Other(v) => match v {},
        })
    }
}

impl<D: UartDevice, P: ValidUartPinout<D>> fmt::Write for Writer<D, P> {
    fn write_str(&mut self, s: &str) -> fmt::Result {
        s.bytes()
            .try_for_each(|c| nb::block!(self.write(c)))
            .map_err(|_| fmt::Error)
    }
}
