use core::{ops::Deref, task::Poll};
use embedded_hal_async::i2c::{AddressMode, Operation};

use crate::{
    async_utils::{sealed::Wakeable, AsyncPeripheral, CancellablePollFn as CPFn},
    i2c::{Controller, Error, ValidAddress, I2C},
    pac::i2c0::RegisterBlock,
};

macro_rules! impl_async_traits {
    ($i2c:path) => {
        impl<P> AsyncPeripheral for I2C<$i2c, P, Controller>
        where
            Self: $crate::async_utils::sealed::Wakeable,
        {
            fn on_interrupt() {
                unsafe {
                    // This is equivalent to stealing from pac::Peripherals
                    let i2c = &*<$i2c>::ptr();

                    // Mask all interrupt flags. This does not clear the flags.
                    // Clearing is done by the driver after it wakes up.
                    i2c.ic_intr_mask().write_with_zero(|w| w);
                }
                // interrupts are now masked, we can wake the task and return from this handler.
                Self::waker().wake();
            }
        }
    };
}

impl_async_traits!(rp235x_pac::I2C0);
impl_async_traits!(rp235x_pac::I2C1);

enum TxEmptyConfig {
    Empty,
    NotFull,
}

impl<T, PINS> I2C<T, PINS, Controller>
where
    T: Deref<Target = RegisterBlock>,
    Self: AsyncPeripheral,
{
    /// `tx_empty`: true to unmask tx_empty
    #[inline]
    fn unmask_intr(&mut self, tx_empty: bool) {
        unsafe {
            self.i2c.ic_intr_mask().write_with_zero(|w| {
                w.m_tx_empty()
                    .bit(tx_empty)
                    .m_rx_full()
                    .disabled()
                    .m_tx_abrt()
                    .disabled()
                    .m_stop_det()
                    .disabled()
            });
        }
    }
    #[inline]
    fn configure_tx_empty(&mut self, cfg: TxEmptyConfig) {
        self.i2c
            .ic_tx_tl()
            // SAFETY: we are within [0; TX_FIFO_DEPTH)
            .write(|w| unsafe {
                w.tx_tl().bits(match cfg {
                    TxEmptyConfig::Empty => 1,
                    TxEmptyConfig::NotFull => Self::TX_FIFO_DEPTH - 1,
                })
            });
    }

    #[inline]
    fn unmask_tx_empty(&mut self) {
        self.configure_tx_empty(TxEmptyConfig::Empty);
        self.unmask_intr(true)
    }

    #[inline]
    fn unmask_tx_not_full(&mut self) {
        self.configure_tx_empty(TxEmptyConfig::NotFull);
        self.unmask_intr(true)
    }

    #[inline]
    fn unmask_stop_det(&mut self) {
        self.unmask_intr(false);
    }

    #[inline]
    fn poll_rx_not_empty_or_abrt(&mut self) -> Poll<Result<(), Error>> {
        self.read_and_clear_abort_reason()?;
        if self.i2c.ic_raw_intr_stat().read().rx_full().bit_is_set() {
            Poll::Ready(Ok(()))
        } else {
            Poll::Pending
        }
    }

    #[inline]
    fn cancel(&mut self) {
        unsafe {
            self.i2c.ic_intr_mask().write_with_zero(|w| w);
        }

        self.abort();
    }

    async fn non_blocking_read_internal(
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
            // cannot abort during read, so ignore the result
            let _ = CPFn::new(
                self,
                Self::poll_tx_not_full,
                Self::unmask_tx_not_full,
                Self::cancel,
            )
            .await;

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

            CPFn::new(
                self,
                Self::poll_rx_not_empty_or_abrt,
                Self::unmask_tx_empty,
                Self::cancel,
            )
            .await?;

            *byte = self.i2c.ic_data_cmd().read().dat().bits();
        }

        Ok(())
    }

    async fn non_blocking_write_internal(
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
        while let Some(byte) = peekable.next() {
            if self.tx_fifo_full() {
                // wait for more room in the fifo
                abort_reason = CPFn::new(
                    self,
                    Self::poll_tx_not_full,
                    Self::unmask_tx_not_full,
                    Self::cancel,
                )
                .await;
                if abort_reason.is_err() {
                    break;
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
            CPFn::new(
                self,
                Self::poll_tx_empty,
                Self::unmask_tx_empty,
                Self::cancel,
            )
            .await;
            abort_reason = self.read_and_clear_abort_reason();
        }

        if abort_reason.is_err() || do_stop {
            // If the transaction was aborted or if it completed
            // successfully wait until the STOP condition has occurred.
            CPFn::new(
                self,
                Self::poll_stop_detected,
                Self::unmask_stop_det,
                Self::cancel,
            )
            .await;
            self.i2c.ic_clr_stop_det().read().clr_stop_det();
        }
        // Note: the hardware issues a STOP automatically on an abort condition.
        // Note: the hardware also clears RX FIFO as well as TX on abort

        abort_reason
    }

    /// Writes to the i2c bus consuming bytes for the given iterator.
    pub async fn write_iter_async<A, U>(&mut self, address: A, bytes: U) -> Result<(), super::Error>
    where
        U: IntoIterator<Item = u8>,
        A: ValidAddress,
    {
        self.setup(address)?;
        self.non_blocking_write_internal(true, bytes, true).await
    }

    /// Writes to the i2c bus consuming bytes for the given iterator.
    pub async fn write_iter_read_async<A, U>(
        &mut self,
        address: A,
        bytes: U,
        read: &mut [u8],
    ) -> Result<(), Error>
    where
        U: IntoIterator<Item = u8>,
        A: ValidAddress,
    {
        self.setup(address)?;
        self.non_blocking_write_internal(true, bytes, false).await?;
        self.non_blocking_read_internal(false, read, true).await
    }

    /// Writes to the i2c bus taking operations from and iterator, writing from iterator of bytes,
    /// reading to slices of bytes.
    #[cfg(feature = "i2c-write-iter")]
    pub async fn transaction_iter_async<'b, A, O, B>(
        &mut self,
        address: A,
        operations: O,
    ) -> Result<(), super::Error>
    where
        A: ValidAddress,
        O: IntoIterator<Item = i2c_write_iter::Operation<'b, B>>,
        B: IntoIterator<Item = u8>,
    {
        self.setup(address)?;

        let mut first = true;
        let mut operations = operations.into_iter().peekable();
        while let Some(operation) = operations.next() {
            let last = operations.peek().is_none();
            match operation {
                i2c_write_iter::Operation::Read(buf) => {
                    self.non_blocking_read_internal(first, buf, last).await?
                }
                i2c_write_iter::Operation::WriteIter(buf) => {
                    self.non_blocking_write_internal(first, buf, last).await?
                }
            }
            first = false;
        }
        Ok(())
    }
}

impl<T, PINS, A> embedded_hal_async::i2c::I2c<A> for I2C<T, PINS, Controller>
where
    Self: AsyncPeripheral,
    A: ValidAddress + AddressMode,
    T: Deref<Target = RegisterBlock>,
{
    async fn transaction(
        &mut self,
        addr: A,
        operations: &mut [Operation<'_>],
    ) -> Result<(), Error> {
        self.setup(addr)?;

        let mut first = true;
        let mut operations = operations.iter_mut().peekable();
        while let Some(op) = operations.next() {
            let last = operations.peek().is_none();
            match op {
                Operation::Read(buffer) => {
                    self.non_blocking_read_internal(first, buffer, last).await?;
                }
                Operation::Write(buffer) => {
                    self.non_blocking_write_internal(first, buffer.iter().cloned(), last)
                        .await?;
                }
            }
            first = false;
        }
        Ok(())
    }
}

#[cfg(feature = "i2c-write-iter")]
impl<T, PINS, A> i2c_write_iter::non_blocking::I2cIter<A> for I2C<T, PINS, Controller>
where
    Self: AsyncPeripheral,
    A: 'static + ValidAddress + AddressMode,
    T: Deref<Target = RegisterBlock>,
{
    async fn transaction_iter<'a, O, B>(
        &mut self,
        address: A,
        operations: O,
    ) -> Result<(), Self::Error>
    where
        O: IntoIterator<Item = i2c_write_iter::Operation<'a, B>>,
        B: IntoIterator<Item = u8>,
    {
        self.transaction_iter_async(address, operations).await
    }
}
