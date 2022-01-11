use core::{future::Future, iter::Peekable, ops::Deref, task::Poll};

use super::{Block, Controller, Error, I2C};

impl<T: Deref<Target = Block>, PINS> I2C<T, PINS, Controller> {
    async fn non_blocking_read_internal<'a, U: Iterator<Item = &'a mut u8> + 'a>(
        &mut self,
        mut buffer: Peekable<U>,
    ) -> Result<(), Error> {
        let mut first = true;
        while let Some(byte) = buffer.next() {
            let last = buffer.peek().is_none();

            // wait until there is space in the FIFO to write the next byte
            block_on(|| {
                if self.tx_fifo_full() {
                    Poll::Pending
                } else {
                    Poll::Ready(())
                }
            })
            .await;

            self.i2c.ic_data_cmd.write(|w| {
                if first {
                    w.restart().enable();
                    first = false;
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

            block_on(|| {
                if let Some(abort_reason) = self.read_and_clear_abort_reason() {
                    Poll::Ready(Err(Error::Abort(abort_reason)))
                } else if self.i2c.ic_rxflr.read().bits() != 0 {
                    Poll::Ready(Ok(()))
                } else {
                    Poll::Pending
                }
            })
            .await?;

            *byte = self.i2c.ic_data_cmd.read().dat().bits();
        }

        Ok(())
    }

    async fn non_blocking_write_internal(
        &mut self,
        bytes: impl IntoIterator<Item = u8>,
        do_stop: bool,
    ) -> Result<(), Error> {
        let mut bytes = bytes.into_iter().peekable();
        while let Some(byte) = bytes.next() {
            let last = bytes.peek().is_none();

            self.i2c.ic_data_cmd.write(|w| {
                if do_stop && last {
                    w.stop().enable();
                } else {
                    w.stop().disable();
                }
                unsafe { w.dat().bits(byte) }
            });

            // Wait until the transmission of the address/data from the internal
            // shift register has completed. For this to function correctly, the
            // TX_EMPTY_CTRL flag in IC_CON must be set. The TX_EMPTY_CTRL flag
            // was set in i2c_init.
            block_on(|| {
                if self.i2c.ic_raw_intr_stat.read().tx_empty().is_inactive() {
                    Poll::Pending
                } else {
                    Poll::Ready(())
                }
            })
            .await;

            let abort_reason = self.read_and_clear_abort_reason();

            if abort_reason.is_some() || (do_stop && last) {
                // If the transaction was aborted or if it completed
                // successfully wait until the STOP condition has occured.

                block_on(|| {
                    if self.i2c.ic_raw_intr_stat.read().stop_det().is_inactive() {
                        Poll::Pending
                    } else {
                        Poll::Ready(())
                    }
                })
                .await;

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
async fn block_on<F: FnMut() -> Poll<T>, T>(mut f: F) -> T {
    futures::future::poll_fn(|cx| {
        // always ready to scan
        cx.waker().wake_by_ref();

        f()
    })
    .await
}

impl<T, PINS, A> embassy_traits::i2c::I2c<A> for I2C<T, PINS, Controller>
where
    T: Deref<Target = Block>,
    A: embassy_traits::i2c::AddressMode + 'static + Into<u16>,
{
    type Error = Error;

    #[rustfmt::skip]
    type WriteFuture<'a>
    where
        Self: 'a = impl Future<Output = Result<(), Self::Error>> + 'a;

    #[rustfmt::skip]
    type ReadFuture<'a>
    where
        Self: 'a = impl Future<Output = Result<(), Self::Error>> + 'a;

    #[rustfmt::skip]
    type WriteReadFuture<'a>
    where
        Self: 'a = impl Future<Output = Result<(), Self::Error>> + 'a;

    fn read<'a>(&'a mut self, address: A, buffer: &'a mut [u8]) -> Self::ReadFuture<'a> {
        let mut buffer = buffer.iter_mut().peekable();
        let addr: u16 = address.into();

        async move {
            self.setup(addr);

            Self::validate(addr, None, Some(buffer.peek().is_none()))?;

            self.non_blocking_read_internal(buffer).await
        }
    }

    fn write<'a>(&'a mut self, address: A, bytes: &'a [u8]) -> Self::WriteFuture<'a> {
        async move {
            let addr: u16 = address.into();
            Self::validate(addr, Some(bytes.is_empty()), None)?;
            self.setup(addr);

            self.non_blocking_write_internal(bytes.iter().cloned(), true)
                .await
        }
    }

    fn write_read<'a>(
        &'a mut self,
        address: A,
        bytes: &'a [u8],
        buffer: &'a mut [u8],
    ) -> Self::WriteReadFuture<'a> {
        async move {
            let addr: u16 = address.into();
            Self::validate(addr, Some(bytes.is_empty()), Some(buffer.is_empty()))?;
            self.setup(addr);

            self.non_blocking_write_internal(bytes.iter().cloned(), false)
                .await?;
            self.non_blocking_read_internal(buffer.iter_mut().peekable())
                .await
        }
    }
}
impl<T, PINS, A> embassy_traits::i2c::WriteIter<A> for I2C<T, PINS, Controller>
where
    T: Deref<Target = Block>,
    A: embassy_traits::i2c::AddressMode + 'static + Into<u16>,
{
    type Error = Error;

    #[rustfmt::skip]
    type WriteIterFuture<'a, U>
    where
        U: 'a + IntoIterator<Item = u8>,
        Self: 'a = impl Future<Output = Result<(), Self::Error>> + 'a;

    fn write_iter<'a, U>(&'a mut self, address: A, bytes: U) -> Self::WriteIterFuture<'a, U>
    where
        U: IntoIterator<Item = u8> + 'a,
    {
        let addr: u16 = address.into();
        async move {
            let mut bytes = bytes.into_iter().peekable();
            Self::validate(addr, Some(bytes.peek().is_none()), None)?;

            self.setup(addr);

            self.non_blocking_write_internal(bytes, true).await
        }
    }
}
