use super::{UartConfig, UartDevice, ValidUartPinout};
use embedded_hal::serial::Read;
use embedded_time::rate::Baud;
use nb::Error::*;

#[cfg(feature = "eh1_0_alpha")]
use eh1_0_alpha::serial::nb as eh1;

/// When there's a read error.
pub struct ReadError<'err> {
    /// The type of error
    pub err_type: ReadErrorType,

    /// Reference to the data that was read but eventually discared because of the error.
    pub discared: &'err [u8],
}

/// Possible types of read errors. See Chapter 4, Section 2 §8 - Table 436: "UARTDR Register"
#[cfg_attr(feature = "eh1_0_alpha", derive(Debug))]
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

#[cfg(feature = "eh1_0_alpha")]
impl eh1_0_alpha::serial::Error for ReadErrorType {
    fn kind(&self) -> eh1_0_alpha::serial::ErrorKind {
        match self {
            ReadErrorType::Overrun => eh1_0_alpha::serial::ErrorKind::Overrun,
            ReadErrorType::Break => eh1_0_alpha::serial::ErrorKind::Other,
            ReadErrorType::Parity => eh1_0_alpha::serial::ErrorKind::Parity,
            ReadErrorType::Framing => eh1_0_alpha::serial::ErrorKind::FrameFormat,
        }
    }
}

pub(crate) fn is_readable<D: UartDevice>(device: &D) -> bool {
    device.uartfr.read().rxfe().bit_is_clear()
}

pub(crate) fn read_raw<'b, D: UartDevice>(
    device: &D,
    buffer: &'b mut [u8],
) -> nb::Result<usize, ReadError<'b>> {
    let mut bytes_read = 0;

    Ok(loop {
        if !is_readable(device) {
            if bytes_read == 0 {
                return Err(WouldBlock);
            } else {
                break bytes_read;
            }
        }

        if bytes_read < buffer.len() {
            let mut error: Option<ReadErrorType> = None;

            let read = device.uartdr.read();

            if read.oe().bit_is_set() {
                error = Some(ReadErrorType::Overrun);
            }

            if read.be().bit_is_set() {
                error = Some(ReadErrorType::Break);
            }

            if read.pe().bit_is_set() {
                error = Some(ReadErrorType::Parity);
            }

            if read.fe().bit_is_set() {
                error = Some(ReadErrorType::Framing);
            }

            if let Some(err_type) = error {
                return Err(Other(ReadError {
                    err_type,
                    discared: buffer,
                }));
            }

            buffer[bytes_read] = read.data().bits();
            bytes_read += 1;
        } else {
            break bytes_read;
        }
    })
}

pub(crate) fn read_full_blocking<D: UartDevice>(
    device: &D,
    buffer: &mut [u8],
) -> Result<(), ReadErrorType> {
    let mut offset = 0;

    while offset != buffer.len() {
        offset += match read_raw(device, &mut buffer[offset..]) {
            Ok(bytes_read) => bytes_read,
            Err(e) => match e {
                Other(inner) => return Err(inner.err_type),
                WouldBlock => continue,
            },
        }
    }

    Ok(())
}

/// Half of an [`UartPeripheral`] that is only capable of reading. Obtained by calling [`UartPeripheral::split()`]
pub struct Reader<D: UartDevice, P: ValidUartPinout<D>> {
    pub(super) device: D,
    pub(super) pins: P,
    pub(super) config: UartConfig,
    pub(super) effective_baudrate: Baud,
}

impl<D: UartDevice, P: ValidUartPinout<D>> Reader<D, P> {
    /// Reads bytes from the UART.
    /// This function reads as long as it can. As soon that the FIFO is empty, if :
    /// - 0 bytes were read, a WouldBlock Error is returned
    /// - some bytes were read, it is deemed to be a success
    /// Upon success, it will return how many bytes were read.
    pub fn read_raw<'b>(&self, buffer: &'b mut [u8]) -> nb::Result<usize, ReadError<'b>> {
        read_raw(&self.device, buffer)
    }

    /// Reads bytes from the UART.
    /// This function blocks until the full buffer has been received.
    pub fn read_full_blocking(&self, buffer: &mut [u8]) -> Result<(), ReadErrorType> {
        read_full_blocking(&self.device, buffer)
    }
}

impl<D: UartDevice, P: ValidUartPinout<D>> Read<u8> for Reader<D, P> {
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

#[cfg(feature = "eh1_0_alpha")]
impl<D: UartDevice, P: ValidUartPinout<D>> eh1::Read<u8> for Reader<D, P> {
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
