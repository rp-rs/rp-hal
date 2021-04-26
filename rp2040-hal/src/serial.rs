//! Implementation for the embedded_hal::serial traits for the UART.
// See [embedded-hal](https://docs.rs/embedded-hal/0.2.4/embedded_hal/serial/index.html) for more details

use core::convert::Infallible;

use crate::uart::{
    UARTPeripheral,
    UARTDevice,
    Enabled
};

use embedded_hal::serial::{
    Read,
    Write
};

use nb::Error::WouldBlock;

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
