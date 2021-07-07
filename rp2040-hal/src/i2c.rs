//! Inter-Integrated Circuit (I2C) bus
// Based on: https://github.com/raspberrypi/pico-sdk/blob/master/src/rp2_common/hardware_i2c/i2c.c
// Structure from: https://github.com/japaric/stm32f30x-hal/blob/master/src/i2c.rs
// See [Chapter 4 Section 3](https://datasheets.raspberrypi.org/rp2040/rp2040_datasheet.pdf) for more details

use crate::gpio::pin::bank0::{
    Gpio0, Gpio1, Gpio10, Gpio11, Gpio12, Gpio13, Gpio14, Gpio15, Gpio16, Gpio17, Gpio18, Gpio19,
    Gpio2, Gpio20, Gpio21, Gpio26, Gpio27, Gpio3, Gpio4, Gpio5, Gpio6, Gpio7, Gpio8, Gpio9,
};
use embedded_time::rate::Hertz;
use hal::blocking::i2c::{Write, WriteRead};
use rp2040_pac::{I2C0, I2C1, RESETS};

/// I2C error
#[non_exhaustive]
#[derive(Debug)]
pub enum Error {
    /// Bus error
    Bus,
    /// Arbitration loss
    Arbitration,
    // Overrun, // slave mode only
    // Pec, // SMBUS mode only
    // Timeout, // SMBUS mode only
    // Alert, // SMBUS mode only
}

#[doc(hidden)]
mod sealed {

    use crate::gpio::pin::bank0::{
        Gpio0, Gpio1, Gpio10, Gpio11, Gpio12, Gpio13, Gpio14, Gpio15, Gpio16, Gpio17, Gpio18,
        Gpio19, Gpio2, Gpio20, Gpio21, Gpio26, Gpio27, Gpio3, Gpio4, Gpio5, Gpio6, Gpio7, Gpio8,
        Gpio9,
    };

    pub trait Sealed {}
    impl Sealed for Gpio0 {}
    impl Sealed for Gpio1 {}
    impl Sealed for Gpio2 {}
    impl Sealed for Gpio3 {}
    impl Sealed for Gpio4 {}
    impl Sealed for Gpio5 {}
    impl Sealed for Gpio6 {}
    impl Sealed for Gpio7 {}
    impl Sealed for Gpio8 {}
    impl Sealed for Gpio9 {}
    impl Sealed for Gpio10 {}
    impl Sealed for Gpio11 {}
    impl Sealed for Gpio12 {}
    impl Sealed for Gpio13 {}
    impl Sealed for Gpio14 {}
    impl Sealed for Gpio15 {}
    impl Sealed for Gpio16 {}
    impl Sealed for Gpio17 {}
    impl Sealed for Gpio18 {}
    impl Sealed for Gpio19 {}
    impl Sealed for Gpio20 {}
    impl Sealed for Gpio21 {}
    impl Sealed for Gpio26 {}
    impl Sealed for Gpio27 {}
}

/// SCL pin
pub trait SclPin<I2C>: sealed::Sealed {}

/// SDA pin
pub trait SdaPin<I2C>: sealed::Sealed {}

impl SdaPin<I2C0> for Gpio0 {}
impl SclPin<I2C0> for Gpio1 {}

impl SdaPin<I2C1> for Gpio2 {}
impl SclPin<I2C1> for Gpio3 {}

impl SdaPin<I2C0> for Gpio4 {}
impl SclPin<I2C0> for Gpio5 {}

impl SdaPin<I2C1> for Gpio6 {}
impl SclPin<I2C1> for Gpio7 {}

impl SdaPin<I2C0> for Gpio8 {}
impl SclPin<I2C0> for Gpio9 {}

impl SdaPin<I2C1> for Gpio10 {}
impl SclPin<I2C1> for Gpio11 {}

impl SdaPin<I2C0> for Gpio12 {}
impl SclPin<I2C0> for Gpio13 {}

impl SdaPin<I2C1> for Gpio14 {}
impl SclPin<I2C1> for Gpio15 {}

impl SdaPin<I2C0> for Gpio16 {}
impl SclPin<I2C0> for Gpio17 {}

impl SdaPin<I2C1> for Gpio18 {}
impl SclPin<I2C1> for Gpio19 {}

impl SdaPin<I2C0> for Gpio20 {}
impl SclPin<I2C0> for Gpio21 {}

impl SdaPin<I2C1> for Gpio26 {}
impl SclPin<I2C1> for Gpio27 {}

/// I2C peripheral operating in master mode
pub struct I2c<I2C, PINS> {
    i2c: I2C,
    pins: PINS,
}

/*macro_rules! busy_wait {
    ($i2c:expr, $flag:ident) => {
        loop {
            let isr = $i2c.isr.read();

            if isr.berr().bit_is_set() {
                return Err(Error::Bus);
            } else if isr.arlo().bit_is_set() {
                return Err(Error::Arbitration);
            } else if isr.$flag().bit_is_set() {
                break;
            } else {
                // try again
            }
        }
    };
}*/

macro_rules! hal {
    ($($I2CX:ident: ($i2cX:ident),)+) => {
        $(
            impl<SCL, SDA> I2c<$I2CX, (SCL, SDA)> {
                /// Configures the I2C peripheral to work in master mode
                pub fn $i2cX<F>(
                    i2c: $I2CX,
                    pins: (SCL, SDA),
                    freq: F,
                ) -> Self where
                    F: Into<Hertz>,
                    SCL: SclPin<$I2CX>,
                    SDA: SdaPin<$I2CX>,
                {
                    let freq = freq.into().0;
                    assert!(freq <= 1_000_000);
                    assert!(freq > 0);

                    unsafe {
                        // Reset i2c hardware
                        let resets =  &*RESETS::ptr();
                        resets.reset.write(|w| w.$i2cX().set_bit());
                        resets.reset.write(|w| w.$i2cX().clear_bit());
                        while resets.reset_done.read().$i2cX().bit_is_clear() {}
                    }

                    i2c.ic_enable.write(|w| w.enable().clear_bit());

                    i2c.ic_con.write(|w| {
                        w.speed().fast();
                        w.master_mode().enabled();
                        w.ic_slave_disable().slave_disabled();
                        w.ic_restart_en().enabled()
                    });

                    i2c.ic_tx_tl.write(|w| unsafe { w.tx_tl().bits(0) });
                    i2c.ic_rx_tl.write(|w| unsafe { w.rx_tl().bits(0) });

                    i2c.ic_dma_cr.write(|w| {
                        w.tdmae().enabled();
                        w.rdmae().enabled()
                    });

                    // TODO: Get value from clocks
                    let freq_in = 125_000_000;

                    // TODO there are some subtleties to I2C timing which we are completely ignoring here
                    let period = (freq_in + freq / 2) / freq;
                    let hcnt = period * 3 / 5; // oof this one hurts
                    let lcnt = period - hcnt;

                    // Check for out-of-range divisors:
                    assert!(hcnt < 0xffff);
                    assert!(lcnt < 0xffff);
                    assert!(hcnt > 8);
                    assert!(lcnt > 8);

                    unsafe {
                        i2c.ic_fs_scl_hcnt.write(|w| w.ic_fs_scl_hcnt().bits(hcnt as u16));
                        i2c.ic_fs_scl_lcnt.write(|w| w.ic_fs_scl_lcnt().bits(lcnt as u16));
                        i2c.ic_fs_spklen.write(|w| w.ic_fs_spklen().bits(if lcnt < 16 { 1 } else { (lcnt / 16) as u8 }));
                    }

                    i2c.ic_enable.write(|w| w.enable().set_bit());

                    I2c { i2c, pins }
                }

                /// Releases the I2C peripheral and associated pins
                pub fn free(self) -> ($I2CX, (SCL, SDA)) {
                    (self.i2c, self.pins)
                }
            }

            impl<PINS> Write for I2c<$I2CX, PINS> {
                type Error = Error;

                fn write(&mut self, _addr: u8, bytes: &[u8]) -> Result<(), Error> {
                    // TODO support transfers of more than 255 bytes
                    assert!(bytes.len() < 256 && bytes.len() > 0);

                    Ok(())
                }
            }

            impl<PINS> WriteRead for I2c<$I2CX, PINS> {
                type Error = Error;

                fn write_read(
                    &mut self,
                    _addr: u8,
                    bytes: &[u8],
                    buffer: &mut [u8],
                ) -> Result<(), Error> {
                    // TODO support transfers of more than 255 bytes
                    assert!(bytes.len() < 256 && bytes.len() > 0);
                    assert!(buffer.len() < 256 && buffer.len() > 0);


                    Ok(())
                }
            }
        )+
    }
}

hal! {
    I2C0: (i2c0),
    I2C1: (i2c1),
}
