use super::{UartDevice, ValidUartPinout};
use core::fmt;
use core::{convert::Infallible, marker::PhantomData};
use embedded_hal::serial::Write;
use nb::Error::*;
use rp2040_pac::uart0::RegisterBlock;

#[cfg(feature = "eh1_0_alpha")]
use eh1_0_alpha::serial::nb as eh1;

pub(crate) fn transmit_flushed(rb: &RegisterBlock) -> nb::Result<(), Infallible> {
    if rb.uartfr.read().txfe().bit_is_set() {
        Ok(())
    } else {
        Err(WouldBlock)
    }
}

fn uart_is_writable(rb: &RegisterBlock) -> bool {
    rb.uartfr.read().txff().bit_is_clear()
}

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

/// Half of an [`UartPeripheral`] that is only capable of writing. Obtained by calling [`UartPeripheral::split()`]
pub struct Writer<D: UartDevice, P: ValidUartPinout<D>> {
    pub(super) device: &'static RegisterBlock,
    pub(super) device_marker: PhantomData<D>,
    pub(super) pins: PhantomData<P>,
}

impl<D: UartDevice, P: ValidUartPinout<D>> Writer<D, P> {
    /// Writes bytes to the UART.
    /// This function writes as long as it can. As soon that the FIFO is full, if :
    /// - 0 bytes were written, a WouldBlock Error is returned
    /// - some bytes were written, it is deemed to be a success
    /// Upon success, the remaining slice is returned.
    pub fn write_raw<'d>(&self, data: &'d [u8]) -> nb::Result<&'d [u8], Infallible> {
        write_raw(self.device, data)
    }

    /// Writes bytes to the UART.
    /// This function blocks until the full buffer has been sent.
    pub fn write_full_blocking(&self, data: &[u8]) {
        super::writer::write_full_blocking(self.device, data);
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
        super::writer::transmit_flushed(self.device)
    }
}

#[cfg(feature = "eh1_0_alpha")]
impl<D: UartDevice, P: ValidUartPinout<D>> eh1::Write<u8> for Writer<D, P> {
    type Error = super::utils::SerialInfallible;

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
