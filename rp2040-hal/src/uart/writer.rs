//! Universal Asynchronous Receiver Transmitter - Transmitter Code
//!
//! This module is for transmitting data with a UART.

use super::{FifoWatermark, UartDevice, ValidUartPinout};
use crate::dma::{EndlessWriteTarget, WriteTarget};
use crate::pac::uart0::RegisterBlock;
use core::fmt;
use core::{convert::Infallible, marker::PhantomData};
use embedded_hal_0_2::serial::Write as Write02;
use embedded_hal_nb::serial::{ErrorType, Write};
use nb::Error::*;

/// Set tx FIFO watermark
///
/// See DS: Table 423
pub fn set_tx_watermark(rb: &RegisterBlock, watermark: FifoWatermark) {
    let wm = match watermark {
        FifoWatermark::Bytes4 => 4,
        FifoWatermark::Bytes8 => 3,
        FifoWatermark::Bytes16 => 2,
        FifoWatermark::Bytes24 => 1,
        FifoWatermark::Bytes28 => 0,
    };
    rb.uartifls()
        .modify(|_r, w| unsafe { w.txiflsel().bits(wm) });
}

/// Returns `Err(WouldBlock)` if the UART is still busy transmitting data.
/// It returns Ok(()) when the TX fifo and the transmit shift register are empty
/// and the last stop bit is sent.
pub(crate) fn transmit_flushed(rb: &RegisterBlock) -> nb::Result<(), Infallible> {
    if rb.uartfr().read().busy().bit_is_set() {
        Err(WouldBlock)
    } else {
        Ok(())
    }
}

/// Returns `true` if the TX FIFO has space, or false if it is full
pub(crate) fn uart_is_writable(rb: &RegisterBlock) -> bool {
    rb.uartfr().read().txff().bit_is_clear()
}

/// Returns `true` if the UART is busy transmitting data, `false` after all
/// bits (including stop bits) have been transmitted.
pub(crate) fn uart_is_busy(rb: &RegisterBlock) -> bool {
    rb.uartfr().read().busy().bit_is_set()
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

        rb.uartdr().write(|w| unsafe {
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
    rb.uartifls()
        .modify(|_r, w| unsafe { w.txiflsel().bits(2) });

    // Access the UART Interrupt Mask Set/Clear register. Setting a bit
    // high enables the interrupt.

    // We set the TX interrupt. This means we will get an interrupt when
    // the TX FIFO level is triggered. This means we don't have to
    // interrupt on every single byte, but can make use of the hardware
    // FIFO.
    rb.uartimsc().modify(|_r, w| {
        w.txim().set_bit();
        w
    });
}

/// Disables the Transmit Interrupt.
pub(crate) fn disable_tx_interrupt(rb: &RegisterBlock) {
    // Access the UART Interrupt Mask Set/Clear register. Setting a bit
    // low disables the interrupt.

    rb.uartimsc().modify(|_r, w| {
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
    /// # use rp2040_hal::uart::{Pins, ValidUartPinout, Enabled, UartPeripheral};
    /// # use rp2040_hal::pac::UART0;
    /// # use rp2040_hal::typelevel::OptionTNone;
    /// # use embedded_hal_0_2::blocking::delay::DelayUs;
    /// # type PINS = Pins<OptionTNone, OptionTNone, OptionTNone, OptionTNone>;
    /// # let mut serial: UartPeripheral<Enabled, UART0, PINS> = unsafe { core::mem::zeroed() };
    /// # let mut timer: rp2040_hal::Timer = unsafe { core::mem::zeroed() };
    /// serial.lowlevel_break_start();
    /// // at 115_200Bps on 8N1 configuration, 20bits takes (20*10⁶)/115200 = 173.611…μs.
    /// timer.delay_us(175);
    /// serial.lowlevel_break_stop();
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
}

impl<D: UartDevice, P: ValidUartPinout<D>> embedded_io::ErrorType for Writer<D, P> {
    type Error = Infallible;
}

impl<D: UartDevice, P: ValidUartPinout<D>> embedded_io::Write for Writer<D, P> {
    fn write(&mut self, buf: &[u8]) -> Result<usize, Self::Error> {
        // Blocks if and only if no bytes can be written.
        let remaining = nb::block!(write_raw(&self.device, buf)).unwrap(); // Infallible
        Ok(buf.len() - remaining.len())
    }
    fn flush(&mut self) -> Result<(), Self::Error> {
        nb::block!(transmit_flushed(&self.device)).unwrap(); // Infallible
        Ok(())
    }
}

impl<D: UartDevice, P: ValidUartPinout<D>> embedded_io::WriteReady for Writer<D, P> {
    fn write_ready(&mut self) -> Result<bool, Self::Error> {
        Ok(uart_is_writable(&self.device))
    }
}

impl<D: UartDevice, P: ValidUartPinout<D>> Write02<u8> for Writer<D, P> {
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

// Safety: This only writes to the TX fifo, so it doesn't
// interact with rust-managed memory.
unsafe impl<D: UartDevice, P: ValidUartPinout<D>> WriteTarget for Writer<D, P> {
    type TransmittedWord = u8;

    fn tx_treq() -> Option<u8> {
        Some(D::tx_dreq())
    }

    fn tx_address_count(&mut self) -> (u32, u32) {
        (self.device.uartdr().as_ptr() as u32, u32::MAX)
    }

    fn tx_increment(&self) -> bool {
        false
    }
}

impl<D: UartDevice, P: ValidUartPinout<D>> EndlessWriteTarget for Writer<D, P> {}

impl<D: UartDevice, P: ValidUartPinout<D>> ErrorType for Writer<D, P> {
    type Error = Infallible;
}

impl<D: UartDevice, P: ValidUartPinout<D>> Write<u8> for Writer<D, P> {
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
            .try_for_each(|c| nb::block!(Write::write(self, c)))
            .map_err(|_| fmt::Error)
    }
}
