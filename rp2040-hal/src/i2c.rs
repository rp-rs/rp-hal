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
    /// I2c abort with error
    Abort(u32),
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
pub struct I2C<I2C, PINS> {
    i2c: I2C,
    pins: PINS,
}

fn i2c_reserved_addr(addr: u8) -> bool {
    (addr & 0x78) == 0 || (addr & 0x78) == 0x78
}

macro_rules! hal {
    ($($I2CX:ident: ($i2cX:ident),)+) => {
        $(
            impl<SCL, SDA> I2C<$I2CX, (SCL, SDA)> {
                /// Configures the I2C peripheral to work in master mode
                pub fn $i2cX<F>(i2c: $I2CX, pins: (SCL, SDA), freq: F) -> Self
                where
                    F: Into<Hertz>,
                    SCL: SclPin<$I2CX>,
                    SDA: SdaPin<$I2CX>,
                {
                    let freq = freq.into().0;
                    assert!(freq <= 1_000_000);
                    assert!(freq > 0);

                    unsafe {
                        // Reset i2c hardware
                        let resets = &*RESETS::ptr();
                        resets.reset.write(|w| w.$i2cX().set_bit());
                        resets.reset.write(|w| w.$i2cX().clear_bit());
                        while resets.reset_done.read().$i2cX().bit_is_clear() {}
                    }

                    i2c.ic_enable.write(|w| w.enable().disabled());

                    i2c.ic_con.write(|w| {
                        w.speed().fast();
                        w.master_mode().enabled();
                        w.ic_slave_disable().slave_disabled();
                        w.ic_restart_en().enabled();
                        w.tx_empty_ctrl().enabled()
                    });

                    i2c.ic_tx_tl.write(|w| unsafe { w.tx_tl().bits(0) });
                    i2c.ic_rx_tl.write(|w| unsafe { w.rx_tl().bits(0) });

                    i2c.ic_dma_cr.write(|w| {
                        w.tdmae().enabled();
                        w.rdmae().enabled()
                    });

                    // TODO: Get value from clocks
                    let freq_in = 125_000_000;

                    // There are some subtleties to I2C timing which we are completely ignoring here
                    // See: https://github.com/raspberrypi/pico-sdk/blob/bfcbefafc5d2a210551a4d9d80b4303d4ae0adf7/src/rp2_common/hardware_i2c/i2c.c#L69
                    let period = (freq_in + freq / 2) / freq;
                    let hcnt = period * 3 / 5; // oof this one hurts
                    let lcnt = period - hcnt;

                    // Check for out-of-range divisors:
                    assert!(hcnt < 0xffff);
                    assert!(lcnt < 0xffff);
                    assert!(hcnt > 8);
                    assert!(lcnt > 8);

                    // Per I2C-bus specification a device in standard or fast mode must
                    // internally provide a hold time of at least 300ns for the SDA signal to
                    // bridge the undefined region of the falling edge of SCL. A smaller hold
                    // time of 120ns is used for fast mode plus.
                    let sda_tx_hold_count = if freq < 1000000 {
                        // sda_tx_hold_count = freq_in [cycles/s] * 300ns * (1s / 1e9ns)
                        // Reduce 300/1e9 to 3/1e7 to avoid numbers that don't fit in uint.
                        // Add 1 to avoid division truncation.
                        ((freq_in * 3) / 10000000) + 1
                    } else {
                        // sda_tx_hold_count = freq_in [cycles/s] * 120ns * (1s / 1e9ns)
                        // Reduce 120/1e9 to 3/25e6 to avoid numbers that don't fit in uint.
                        // Add 1 to avoid division truncation.
                        ((freq_in * 3) / 25000000) + 1
                    };
                    assert!(sda_tx_hold_count <= lcnt - 2);

                    unsafe {
                        i2c.ic_fs_scl_hcnt
                            .write(|w| w.ic_fs_scl_hcnt().bits(hcnt as u16));
                        i2c.ic_fs_scl_lcnt
                            .write(|w| w.ic_fs_scl_lcnt().bits(lcnt as u16));
                        i2c.ic_fs_spklen.write(|w| {
                            w.ic_fs_spklen()
                                .bits(if lcnt < 16 { 1 } else { (lcnt / 16) as u8 })
                        });
                        i2c.ic_sda_hold
                            .write(|w| w.ic_sda_tx_hold().bits(sda_tx_hold_count as u16));
                    }

                    i2c.ic_enable.write(|w| w.enable().enabled());

                    I2C { i2c, pins }
                }

                /// Releases the I2C peripheral and associated pins
                pub fn free(self) -> ($I2CX, (SCL, SDA)) {
                    (self.i2c, self.pins)
                }
            }

            impl<PINS> Write for I2C<$I2CX, PINS> {
                type Error = Error;

                fn write(&mut self, addr: u8, bytes: &[u8]) -> Result<(), Error> {
                    // TODO support transfers of more than 255 bytes
                    assert!(bytes.len() < 256 && bytes.len() > 0);

                    assert!(addr < 0x80);
                    assert!(!i2c_reserved_addr(addr));

                    self.i2c.ic_enable.write(|w| w.enable().disabled());
                    self.i2c
                        .ic_tar
                        .write(|w| unsafe { w.ic_tar().bits(addr as u16) });
                    self.i2c.ic_enable.write(|w| w.enable().enabled());

                    let mut abort = false;
                    let mut abort_reason = 0;

                    for (i, byte) in bytes.iter().enumerate() {
                        let last = i == bytes.len() - 1;

                        self.i2c.ic_data_cmd.write(|w| {
                            if last {
                                w.stop().enable();
                            } else {
                                w.stop().disable();
                            }
                            unsafe { w.dat().bits(*byte) }
                        });

                        // Wait until the transmission of the address/data from the internal
                        // shift register has completed. For this to function correctly, the
                        // TX_EMPTY_CTRL flag in IC_CON must be set. The TX_EMPTY_CTRL flag
                        // was set in i2c_init.
                        while self.i2c.ic_raw_intr_stat.read().tx_empty().is_inactive() {}

                        abort_reason = self.i2c.ic_tx_abrt_source.read().bits();
                        if abort_reason != 0 {
                            // Note clearing the abort flag also clears the reason, and
                            // this instance of flag is clear-on-read! Note also the
                            // IC_CLR_TX_ABRT register always reads as 0.
                            self.i2c.ic_clr_tx_abrt.read().clr_tx_abrt();
                            abort = true;
                        }

                        if abort || last {
                            // If the transaction was aborted or if it completed
                            // successfully wait until the STOP condition has occured.

                            while self.i2c.ic_raw_intr_stat.read().stop_det().is_inactive() {}

                            self.i2c.ic_clr_stop_det.read().clr_stop_det();
                        }

                        // Note the hardware issues a STOP automatically on an abort condition.
                        // Note also the hardware clears RX FIFO as well as TX on abort,
                        // ecause we set hwparam IC_AVOID_RX_FIFO_FLUSH_ON_TX_ABRT to 0.
                        if abort {
                            break;
                        }
                    }

                    if abort {
                        Err(Error::Abort(abort_reason))
                    } else {
                        Ok(())
                    }
                }
            }

            impl<PINS> WriteRead for I2C<$I2CX, PINS> {
                type Error = Error;

                fn write_read(&mut self, addr: u8, bytes: &[u8], buffer: &mut [u8]) -> Result<(), Error> {
                    // TODO support transfers of more than 255 bytes
                    assert!(bytes.len() < 256 && bytes.len() > 0);
                    assert!(buffer.len() < 256 && buffer.len() > 0);

                    assert!(addr < 0x80);
                    assert!(!i2c_reserved_addr(addr));

                    self.i2c.ic_enable.write(|w| w.enable().disabled());
                    self.i2c
                        .ic_tar
                        .write(|w| unsafe { w.ic_tar().bits(addr as u16) });
                    self.i2c.ic_enable.write(|w| w.enable().enabled());

                    let mut abort = false;
                    let mut abort_reason = 0;

                    for byte in bytes {
                        self.i2c.ic_data_cmd.write(|w| {
                            w.stop().disable();
                            unsafe { w.dat().bits(*byte) }
                        });

                        // Wait until the transmission of the address/data from the internal
                        // shift register has completed. For this to function correctly, the
                        // TX_EMPTY_CTRL flag in IC_CON must be set. The TX_EMPTY_CTRL flag
                        // was set in i2c_init.
                        while self.i2c.ic_raw_intr_stat.read().tx_empty().is_inactive() {}

                        abort_reason = self.i2c.ic_tx_abrt_source.read().bits();
                        if abort_reason != 0 {
                            // Note clearing the abort flag also clears the reason, and
                            // this instance of flag is clear-on-read! Note also the
                            // IC_CLR_TX_ABRT register always reads as 0.
                            self.i2c.ic_clr_tx_abrt.read().clr_tx_abrt();
                            abort = true;
                        }

                        if abort {
                            // If the transaction was aborted or if it completed
                            // successfully wait until the STOP condition has occured.

                            while self.i2c.ic_raw_intr_stat.read().stop_det().is_inactive() {}

                            self.i2c.ic_clr_stop_det.read().clr_stop_det();
                        }

                        // Note the hardware issues a STOP automatically on an abort condition.
                        // Note also the hardware clears RX FIFO as well as TX on abort,
                        // ecause we set hwparam IC_AVOID_RX_FIFO_FLUSH_ON_TX_ABRT to 0.
                        if abort {
                            break;
                        }
                    }

                    for (i, byte) in buffer.iter_mut().enumerate() {
                        let first = i == 0;
                        let last = i == bytes.len() - 1;

                        while 16 - self.i2c.ic_txflr.read().txflr().bits() > 0 {}

                        self.i2c.ic_data_cmd.write(|w| {
                            if first {
                                w.restart().enable();
                            } else {
                                w.restart().disable();
                            }

                            if last {
                                w.stop().enable();
                            } else {
                                w.stop().disable();
                            }

                            w.cmd().read()
                        });

                        while !abort && self.i2c.ic_rxflr.read().bits() == 0 {
                            abort_reason = self.i2c.ic_tx_abrt_source.read().bits();
                            abort = self.i2c.ic_clr_tx_abrt.read().bits() > 0;
                        }

                        if abort {
                            break;
                        }

                        *byte = self.i2c.ic_data_cmd.read().dat().bits();
                    }

                    if abort {
                        Err(Error::Abort(abort_reason))
                    } else {
                        Ok(())
                    }
                }
            }
         )+
    }
}

hal! {
    I2C0: (i2c0),
    I2C1: (i2c1),
}
