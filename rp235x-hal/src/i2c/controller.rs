use core::{ops::Deref, task::Poll};
use embedded_hal_0_2::blocking::i2c::{Read, Write, WriteIter, WriteIterRead, WriteRead};
use fugit::HertzU32;

use embedded_hal::i2c as eh1;

use crate::{
    i2c::{Controller, Error, ValidAddress, ValidPinScl, ValidPinSda, I2C},
    pac::{i2c0::RegisterBlock as Block, RESETS},
    resets::SubsystemReset,
};

pub(crate) mod non_blocking;

impl<T, Sda, Scl> I2C<T, (Sda, Scl), Controller>
where
    T: SubsystemReset + Deref<Target = Block>,
    Sda: ValidPinSda<T>,
    Scl: ValidPinScl<T>,
{
    /// Configures the I2C peripheral to work in controller mode
    pub fn new_controller(
        i2c: T,
        sda_pin: Sda,
        scl_pin: Scl,
        freq: HertzU32,
        resets: &mut RESETS,
        system_clock: HertzU32,
    ) -> Self {
        let freq = freq.to_Hz();
        assert!(freq <= 1_000_000);
        assert!(freq > 0);

        i2c.reset_bring_down(resets);
        i2c.reset_bring_up(resets);

        i2c.ic_enable().write(|w| w.enable().disabled());

        // select controller mode & speed
        i2c.ic_con().modify(|_, w| {
            w.speed().fast();
            w.master_mode().enabled();
            w.ic_slave_disable().slave_disabled();
            w.ic_restart_en().enabled();
            w.tx_empty_ctrl().enabled()
        });

        // Clear FIFO threshold
        i2c.ic_tx_tl().write(|w| unsafe { w.tx_tl().bits(0) });
        i2c.ic_rx_tl().write(|w| unsafe { w.rx_tl().bits(0) });

        let freq_in = system_clock.to_Hz();

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
            i2c.ic_fs_scl_hcnt()
                .write(|w| w.ic_fs_scl_hcnt().bits(hcnt as u16));
            i2c.ic_fs_scl_lcnt()
                .write(|w| w.ic_fs_scl_lcnt().bits(lcnt as u16));
            // spike filter duration
            i2c.ic_fs_spklen().write(|w| {
                let ticks = if lcnt < 16 { 1 } else { (lcnt / 16) as u8 };
                w.ic_fs_spklen().bits(ticks)
            });
            // sda hold time
            i2c.ic_sda_hold()
                .modify(|_r, w| w.ic_sda_tx_hold().bits(sda_tx_hold_count as u16));

            // make TX_EMPTY raise when the tx fifo is not full
            i2c.ic_tx_tl()
                .write(|w| w.tx_tl().bits(Self::TX_FIFO_DEPTH));
            // make RX_FULL raise when the rx fifo contains at least 1 byte
            i2c.ic_rx_tl().write(|w| w.rx_tl().bits(0));
            // Enable clock stretching.
            // Will hold clock when:
            // - receiving and rx fifo is full
            // - writing and tx fifo is empty
            i2c.ic_con()
                .modify(|_, w| w.rx_fifo_full_hld_ctrl().enabled());
        }

        // Enable I2C block
        i2c.ic_enable().write(|w| w.enable().enabled());

        Self {
            i2c,
            pins: (sda_pin, scl_pin),
            mode: Controller {},
        }
    }
}

impl<T: Deref<Target = Block>, PINS> I2C<T, PINS, Controller> {
    fn validate_buffer<U>(
        &mut self,
        first: bool,
        buf: &mut core::iter::Peekable<U>,
        err: Error,
    ) -> Result<(), Error>
    where
        U: Iterator,
    {
        if buf.peek().is_some() {
            Ok(())
        } else {
            if !first {
                self.abort();
            }
            Err(err)
        }
    }

    fn setup<A: ValidAddress>(&mut self, addr: A) -> Result<(), Error> {
        addr.is_valid()?;

        self.i2c.ic_enable().write(|w| w.enable().disabled());
        self.i2c
            .ic_con()
            .modify(|_, w| w.ic_10bitaddr_master().variant(A::BIT_ADDR_M));

        let addr = addr.into();
        self.i2c
            .ic_tar()
            .write(|w| unsafe { w.ic_tar().bits(addr) });
        self.i2c.ic_enable().write(|w| w.enable().enabled());
        Ok(())
    }

    #[inline]
    fn read_and_clear_abort_reason(&mut self) -> Result<(), Error> {
        let abort_reason = self.i2c.ic_tx_abrt_source().read().bits();
        if abort_reason != 0 {
            // Note clearing the abort flag also clears the reason, and
            // this instance of flag is clear-on-read! Note also the
            // IC_CLR_TX_ABRT register always reads as 0.
            self.i2c.ic_clr_tx_abrt().read();
            Err(Error::Abort(abort_reason))
        } else {
            Ok(())
        }
    }

    #[inline]
    fn poll_tx_not_full(&mut self) -> Poll<Result<(), Error>> {
        self.read_and_clear_abort_reason()?;
        if !self.tx_fifo_full() {
            Poll::Ready(Ok(()))
        } else {
            Poll::Pending
        }
    }

    #[inline]
    fn poll_tx_empty(&mut self) -> Poll<()> {
        if self.i2c.ic_raw_intr_stat().read().tx_empty().is_inactive() {
            Poll::Pending
        } else {
            Poll::Ready(())
        }
    }

    #[inline]
    fn poll_stop_detected(&mut self) -> Poll<()> {
        if self.i2c.ic_raw_intr_stat().read().stop_det().is_inactive() {
            Poll::Pending
        } else {
            Poll::Ready(())
        }
    }

    #[inline]
    fn abort(&mut self) {
        self.i2c.ic_enable().modify(|_, w| w.abort().set_bit());
        while self.i2c.ic_enable().read().abort().bit_is_set() {
            crate::arch::nop()
        }
        while self.i2c.ic_raw_intr_stat().read().tx_abrt().bit_is_clear() {
            crate::arch::nop()
        }
        // clear tx_abort interrupt flags (might have already been clear by irq)
        self.i2c.ic_clr_tx_abrt().read();
        // clear tx_abrt_source by reading it
        self.i2c.ic_tx_abrt_source().read();
    }

    fn read_internal(
        &mut self,
        first_transaction: bool,
        buffer: &mut [u8],
        do_stop: bool,
    ) -> Result<(), Error> {
        self.validate_buffer(
            first_transaction,
            &mut buffer.iter().peekable(),
            Error::InvalidReadBufferLength,
        )?;

        let lastindex = buffer.len() - 1;
        let mut first_byte = true;
        for (i, byte) in buffer.iter_mut().enumerate() {
            let last_byte = i == lastindex;

            // wait until there is space in the FIFO to write the next byte
            while self.i2c.ic_status().read().tfnf().bit_is_clear() {}

            self.i2c.ic_data_cmd().write(|w| {
                if first_byte {
                    if !first_transaction {
                        w.restart().enable();
                    }
                    first_byte = false;
                }

                w.stop().bit(do_stop && last_byte);
                w.cmd().read()
            });

            while self.i2c.ic_rxflr().read().bits() == 0 {
                self.read_and_clear_abort_reason()?;
            }

            *byte = self.i2c.ic_data_cmd().read().dat().bits();
        }

        Ok(())
    }

    fn write_internal(
        &mut self,
        first_transaction: bool,
        bytes: impl IntoIterator<Item = u8>,
        do_stop: bool,
    ) -> Result<(), Error> {
        let mut peekable = bytes.into_iter().peekable();
        self.validate_buffer(
            first_transaction,
            &mut peekable,
            Error::InvalidWriteBufferLength,
        )?;

        let mut abort_reason = Ok(());
        let mut first_byte = true;
        'outer: while let Some(byte) = peekable.next() {
            if self.tx_fifo_full() {
                // wait for more room in the fifo
                loop {
                    match self.poll_tx_not_full() {
                        Poll::Pending => continue,
                        Poll::Ready(Ok(())) => break,
                        Poll::Ready(r) => {
                            abort_reason = r;
                            break 'outer;
                        }
                    }
                }
            }

            // else enqueue
            let last = peekable.peek().is_none();
            self.i2c.ic_data_cmd().write(|w| {
                if first_byte {
                    if !first_transaction {
                        w.restart().enable();
                    }
                    first_byte = false;
                }
                w.stop().bit(do_stop && last);
                unsafe { w.dat().bits(byte) }
            });
        }

        if abort_reason.is_err() {
            // Wait until the transmission of the address/data from the internal
            // shift register has completed.
            while self.poll_tx_empty().is_pending() {}
            abort_reason = self.read_and_clear_abort_reason();
        }

        if abort_reason.is_err() || do_stop {
            // If the transaction was aborted or if it completed
            // successfully wait until the STOP condition has occurred.
            while self.poll_stop_detected().is_pending() {}
            self.i2c.ic_clr_stop_det().read().clr_stop_det();
        }
        // Note: the hardware issues a STOP automatically on an abort condition.
        // Note: the hardware also clears RX FIFO as well as TX on abort

        abort_reason
    }
}

impl<T: Deref<Target = Block>, PINS> I2C<T, PINS, Controller> {
    /// Writes bytes to slave with address `address`
    ///
    /// # I2C Events (contract)
    ///
    /// Same as the `write` method
    pub fn write_iter<A: ValidAddress, B>(&mut self, address: A, bytes: B) -> Result<(), Error>
    where
        B: IntoIterator<Item = u8>,
    {
        self.setup(address)?;
        self.write_internal(true, bytes, true)
    }

    /// Writes bytes to slave with address `address` and then reads enough bytes to fill `buffer` *in a
    /// single transaction*
    ///
    /// # I2C Events (contract)
    ///
    /// Same as the `write_read` method
    pub fn write_iter_read<A: ValidAddress, B>(
        &mut self,
        address: A,
        bytes: B,
        buffer: &mut [u8],
    ) -> Result<(), Error>
    where
        B: IntoIterator<Item = u8>,
    {
        self.setup(address)?;

        self.write_internal(true, bytes, false)?;
        self.read_internal(false, buffer, true)
    }

    /// Execute the provided operations on the I2C bus (iterator version).
    ///
    /// Transaction contract:
    /// - Before executing the first operation an ST is sent automatically. This is followed by SAD+R/W as appropriate.
    /// - Data from adjacent operations of the same type are sent after each other without an SP or SR.
    /// - Between adjacent operations of a different type an SR and SAD+R/W is sent.
    /// - After executing the last operation an SP is sent automatically.
    /// - If the last operation is a `Read` the master does not send an acknowledge for the last byte.
    ///
    /// - `ST` = start condition
    /// - `SAD+R/W` = slave address followed by bit 1 to indicate reading or 0 to indicate writing
    /// - `SR` = repeated start condition
    /// - `SP` = stop condition
    fn transaction<'op: 'iter, 'iter, A: ValidAddress>(
        &mut self,
        address: A,
        operations: impl IntoIterator<Item = &'iter mut eh1::Operation<'op>>,
    ) -> Result<(), Error> {
        self.setup(address)?;

        let mut first = true;
        let mut operations = operations.into_iter().peekable();
        while let Some(operation) = operations.next() {
            let last = operations.peek().is_none();
            match operation {
                eh1::Operation::Read(buf) => self.read_internal(first, buf, last)?,
                eh1::Operation::Write(buf) => {
                    self.write_internal(first, buf.iter().cloned(), last)?
                }
            }
            first = false;
        }
        Ok(())
    }

    #[cfg(feature = "i2c-write-iter")]
    fn transaction_iter<'op, A, O, B>(&mut self, address: A, operations: O) -> Result<(), Error>
    where
        A: ValidAddress,
        O: IntoIterator<Item = i2c_write_iter::Operation<'op, B>>,
        B: IntoIterator<Item = u8>,
    {
        use i2c_write_iter::Operation;
        self.setup(address)?;

        let mut first = true;
        let mut operations = operations.into_iter().peekable();
        while let Some(operation) = operations.next() {
            let last = operations.peek().is_none();
            match operation {
                Operation::Read(buf) => self.read_internal(first, buf, last)?,
                Operation::WriteIter(buf) => self.write_internal(first, buf, last)?,
            }
            first = false;
        }
        Ok(())
    }
}

impl<A: ValidAddress, T: Deref<Target = Block>, PINS> Read<A> for I2C<T, PINS, Controller> {
    type Error = Error;

    fn read(&mut self, addr: A, buffer: &mut [u8]) -> Result<(), Error> {
        self.setup(addr)?;
        self.read_internal(true, buffer, true)
    }
}
impl<A: ValidAddress, T: Deref<Target = Block>, PINS> WriteRead<A> for I2C<T, PINS, Controller> {
    type Error = Error;

    fn write_read(&mut self, addr: A, tx: &[u8], rx: &mut [u8]) -> Result<(), Error> {
        self.setup(addr)?;

        self.write_internal(true, tx.iter().cloned(), false)?;
        self.read_internal(false, rx, true)
    }
}

impl<A: ValidAddress, T: Deref<Target = Block>, PINS> Write<A> for I2C<T, PINS, Controller> {
    type Error = Error;

    fn write(&mut self, addr: A, tx: &[u8]) -> Result<(), Error> {
        self.setup(addr)?;
        self.write_internal(true, tx.iter().cloned(), true)
    }
}

impl<A: ValidAddress, T: Deref<Target = Block>, PINS> WriteIter<A> for I2C<T, PINS, Controller> {
    type Error = Error;

    fn write<B>(&mut self, address: A, bytes: B) -> Result<(), Self::Error>
    where
        B: IntoIterator<Item = u8>,
    {
        self.write_iter(address, bytes)
    }
}

impl<A: ValidAddress, T: Deref<Target = Block>, PINS> WriteIterRead<A>
    for I2C<T, PINS, Controller>
{
    type Error = Error;

    fn write_iter_read<B>(
        &mut self,
        address: A,
        bytes: B,
        buffer: &mut [u8],
    ) -> Result<(), Self::Error>
    where
        B: IntoIterator<Item = u8>,
    {
        self.write_iter_read(address, bytes, buffer)
    }
}

impl<T: Deref<Target = Block>, PINS> eh1::ErrorType for I2C<T, PINS, Controller> {
    type Error = Error;
}

impl<A: ValidAddress, T: Deref<Target = Block>, PINS> eh1::I2c<A> for I2C<T, PINS, Controller> {
    fn transaction(
        &mut self,
        address: A,
        operations: &mut [eh1::Operation<'_>],
    ) -> Result<(), Self::Error> {
        self.transaction(address, operations.iter_mut())
    }
}

#[cfg(feature = "i2c-write-iter")]
impl<A: i2c_write_iter::AddressMode + ValidAddress, T: Deref<Target = Block>, PINS>
    i2c_write_iter::I2cIter<A> for I2C<T, PINS, Controller>
{
    fn transaction_iter<'a, O, B>(&mut self, address: A, operations: O) -> Result<(), Self::Error>
    where
        O: IntoIterator<Item = i2c_write_iter::Operation<'a, B>>,
        B: IntoIterator<Item = u8>,
    {
        self.transaction_iter(address, operations)
    }
}
