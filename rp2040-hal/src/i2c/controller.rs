use core::{marker::PhantomData, ops::Deref};

use crate::{
    gpio::pin::bank0::BankPinId,
    gpio::pin::{FunctionI2C, Pin, PinId},
    resets::SubsystemReset,
};
use embedded_time::rate::Hertz;
use hal::blocking::i2c::{Read, Write, WriteRead};
use pac::{i2c0::RegisterBlock as Block, RESETS};

#[cfg(feature = "eh1_0_alpha")]
use eh1_0_alpha::i2c as eh1;

use super::{i2c_reserved_addr, Controller, Error, SclPin, SdaPin, I2C};

#[cfg(feature = "embassy-traits")]
mod embassy_support;

impl<T: SubsystemReset + Deref<Target = Block>, Sda: PinId + BankPinId, Scl: PinId + BankPinId>
    I2C<T, (Pin<Sda, FunctionI2C>, Pin<Scl, FunctionI2C>), Controller>
{
    /// Configures the I2C peripheral to work in controller mode
    pub fn new_controller<F, SystemF>(
        i2c: T,
        sda_pin: Pin<Sda, FunctionI2C>,
        scl_pin: Pin<Scl, FunctionI2C>,
        freq: F,
        resets: &mut RESETS,
        system_clock: SystemF,
    ) -> Self
    where
        F: Into<Hertz<u64>>,
        Sda: SdaPin<T>,
        Scl: SclPin<T>,
        SystemF: Into<Hertz<u32>>,
    {
        let freq = freq.into().0;
        assert!(freq <= 1_000_000);
        assert!(freq > 0);
        let freq = freq as u32;

        i2c.reset_bring_down(resets);
        i2c.reset_bring_up(resets);

        i2c.ic_enable.write(|w| w.enable().disabled());

        // select controller mode & speed
        i2c.ic_con.modify(|_, w| {
            w.speed().fast();
            w.master_mode().enabled();
            w.ic_slave_disable().slave_disabled();
            w.ic_restart_en().enabled();
            w.tx_empty_ctrl().enabled()
        });

        // Clear FIFO threshold
        i2c.ic_tx_tl.write(|w| unsafe { w.tx_tl().bits(0) });
        i2c.ic_rx_tl.write(|w| unsafe { w.rx_tl().bits(0) });

        let freq_in = system_clock.into().0;

        // There are some subtleties to I2C timing which we are completely ignoring here
        // See: https://github.com/raspberrypi/pico-sdk/blob/bfcbefafc5d2a210551a4d9d80b4303d4ae0adf7/src/rp2_common/hardware_i2c/i2c.c#L69
        let period = (freq_in + freq / 2) / freq;
        let lcnt = period * 3 / 5; // spend 3/5 (60%) of the period low
        let hcnt = period - lcnt; // and 2/5 (40%) of the period high

        // Check for out-of-range divisors:
        assert!(hcnt <= 0xffff);
        assert!(lcnt <= 0xffff);
        assert!(hcnt >= 8);
        assert!(lcnt >= 8);

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
            // fast mode plus requires a clk_in > 32MHz
            assert!(freq_in >= 32_000_000);

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
                .modify(|_r, w| w.ic_sda_tx_hold().bits(sda_tx_hold_count as u16));
        }

        // Enable I2C block
        i2c.ic_enable.write(|w| w.enable().enabled());

        Self {
            i2c,
            pins: (sda_pin, scl_pin),
            mode: PhantomData,
        }
    }
}
impl<T: Deref<Target = Block>, PINS> I2C<T, PINS, Controller> {
    fn validate(
        addr: u16,
        opt_tx_empty: Option<bool>,
        opt_rx_empty: Option<bool>,
    ) -> Result<(), Error> {
        // validate tx parameters if present
        if opt_tx_empty.unwrap_or(false) {
            return Err(Error::InvalidWriteBufferLength);
        }

        // validate rx parameters if present
        if opt_rx_empty.unwrap_or(false) {
            return Err(Error::InvalidReadBufferLength);
        }

        // validate address
        if addr >= 0x80 {
            Err(Error::AddressOutOfRange(addr))
        } else if i2c_reserved_addr(addr) {
            Err(Error::AddressReserved(addr))
        } else {
            Ok(())
        }
    }

    fn setup(&mut self, addr: u16) {
        self.i2c.ic_enable.write(|w| w.enable().disabled());
        self.i2c.ic_tar.write(|w| unsafe { w.ic_tar().bits(addr) });
        self.i2c.ic_enable.write(|w| w.enable().enabled());
    }

    fn read_and_clear_abort_reason(&mut self) -> Option<u32> {
        let abort_reason = self.i2c.ic_tx_abrt_source.read().bits();
        if abort_reason != 0 {
            // Note clearing the abort flag also clears the reason, and
            // this instance of flag is clear-on-read! Note also the
            // IC_CLR_TX_ABRT register always reads as 0.
            self.i2c.ic_clr_tx_abrt.read();
            Some(abort_reason)
        } else {
            None
        }
    }

    fn read_internal(
        &mut self,
        buffer: &mut [u8],
        force_restart: bool,
        do_stop: bool,
    ) -> Result<(), Error> {
        let lastindex = buffer.len() - 1;
        for (i, byte) in buffer.iter_mut().enumerate() {
            let first = i == 0;
            let last = i == lastindex;

            // wait until there is space in the FIFO to write the next byte
            while self.tx_fifo_full() {}

            self.i2c.ic_data_cmd.write(|w| {
                if force_restart && first {
                    w.restart().enable();
                } else {
                    w.restart().disable();
                }

                if do_stop && last {
                    w.stop().enable();
                } else {
                    w.stop().disable();
                }

                w.cmd().read()
            });

            while self.i2c.ic_rxflr.read().bits() == 0 {
                if let Some(abort_reason) = self.read_and_clear_abort_reason() {
                    return Err(Error::Abort(abort_reason));
                }
            }

            *byte = self.i2c.ic_data_cmd.read().dat().bits();
        }

        Ok(())
    }

    fn write_internal(&mut self, bytes: &[u8], do_stop: bool) -> Result<(), Error> {
        for (i, byte) in bytes.iter().enumerate() {
            let last = i == bytes.len() - 1;

            self.i2c.ic_data_cmd.write(|w| {
                if do_stop && last {
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

            let abort_reason = self.read_and_clear_abort_reason();

            if abort_reason.is_some() || (do_stop && last) {
                // If the transaction was aborted or if it completed
                // successfully wait until the STOP condition has occured.

                while self.i2c.ic_raw_intr_stat.read().stop_det().is_inactive() {}

                self.i2c.ic_clr_stop_det.read().clr_stop_det();
            }

            // Note the hardware issues a STOP automatically on an abort condition.
            // Note also the hardware clears RX FIFO as well as TX on abort,
            // ecause we set hwparam IC_AVOID_RX_FIFO_FLUSH_ON_TX_ABRT to 0.
            if let Some(abort_reason) = abort_reason {
                return Err(Error::Abort(abort_reason));
            }
        }
        Ok(())
    }
}
impl<T: Deref<Target = Block>, PINS> Read for I2C<T, PINS, Controller> {
    type Error = Error;

    fn read(&mut self, addr: u8, buffer: &mut [u8]) -> Result<(), Error> {
        let addr: u16 = addr.into();

        Self::validate(addr, None, Some(buffer.is_empty()))?;

        self.setup(addr);
        self.read_internal(buffer, true, true)
    }
}
impl<T: Deref<Target = Block>, PINS> WriteRead for I2C<T, PINS, Controller> {
    type Error = Error;

    fn write_read(&mut self, addr: u8, tx: &[u8], rx: &mut [u8]) -> Result<(), Error> {
        let addr: u16 = addr.into();

        Self::validate(addr, Some(tx.is_empty()), Some(rx.is_empty()))?;
        self.setup(addr);

        self.write_internal(tx, false)?;
        self.read_internal(rx, true, true)
    }
}
impl<T: Deref<Target = Block>, PINS> Write for I2C<T, PINS, Controller> {
    type Error = Error;

    fn write(&mut self, addr: u8, tx: &[u8]) -> Result<(), Error> {
        let addr: u16 = addr.into();
        Self::validate(addr, Some(tx.is_empty()), None)?;
        self.setup(addr);

        self.write_internal(tx, true)
    }
}

#[cfg(feature = "eh1_0_alpha")]
impl<T: Deref<Target = Block>, PINS> eh1::ErrorType for I2C<T, PINS, Controller> {
    type Error = Error;
}

#[cfg(feature = "eh1_0_alpha")]
impl<T: Deref<Target = Block>, PINS> eh1::blocking::I2c for I2C<T, PINS, Controller> {
    fn write(&mut self, addr: u8, bytes: &[u8]) -> Result<(), Self::Error> {
        Write::write(self, addr, bytes)
    }

    fn write_iter<B>(&mut self, address: u8, bytes: B) -> Result<(), Self::Error>
    where
        B: IntoIterator<Item = u8>,
    {
        let mut peekable = bytes.into_iter().peekable();
        let addr: u16 = address.into();
        Self::validate(addr, Some(peekable.peek().is_none()), None)?;
        self.setup(addr);

        while let Some(tx) = peekable.next() {
            self.write_internal(&[tx], peekable.peek().is_none())?
        }
        Ok(())
    }

    fn write_read(&mut self, addr: u8, bytes: &[u8], buffer: &mut [u8]) -> Result<(), Error> {
        WriteRead::write_read(self, addr, bytes, buffer)
    }

    fn write_iter_read<B>(
        &mut self,
        address: u8,
        bytes: B,
        buffer: &mut [u8],
    ) -> Result<(), Self::Error>
    where
        B: IntoIterator<Item = u8>,
    {
        let mut peekable = bytes.into_iter().peekable();
        let addr: u16 = address.into();
        Self::validate(addr, Some(peekable.peek().is_none()), None)?;
        self.setup(addr);

        for tx in peekable {
            self.write_internal(&[tx], false)?
        }
        self.read_internal(buffer, true, true)
    }

    fn read(&mut self, addr: u8, buffer: &mut [u8]) -> Result<(), Error> {
        Read::read(self, addr, buffer)
    }

    fn transaction<'a>(
        &mut self,
        address: u8,
        operations: &mut [eh1::blocking::Operation<'a>],
    ) -> Result<(), Self::Error> {
        let addr: u16 = address.into();
        self.setup(addr);
        for i in 0..operations.len() {
            let last = i == operations.len() - 1;
            match &mut operations[i] {
                eh1::blocking::Operation::Read(buf) => self.read_internal(buf, false, last)?,
                eh1::blocking::Operation::Write(buf) => self.write_internal(buf, last)?,
            }
        }
        Ok(())
    }

    fn transaction_iter<'a, O>(&mut self, address: u8, operations: O) -> Result<(), Self::Error>
    where
        O: IntoIterator<Item = eh1::blocking::Operation<'a>>,
    {
        let addr: u16 = address.into();
        self.setup(addr);
        let mut peekable = operations.into_iter().peekable();
        while let Some(operation) = peekable.next() {
            let last = peekable.peek().is_none();
            match operation {
                eh1::blocking::Operation::Read(buf) => self.read_internal(buf, false, last)?,
                eh1::blocking::Operation::Write(buf) => self.write_internal(buf, last)?,
            }
        }
        Ok(())
    }
}
