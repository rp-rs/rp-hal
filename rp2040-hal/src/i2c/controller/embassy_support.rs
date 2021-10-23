use core::{future::Future, ops::Deref, task::Poll};

use super::{Block, Controller, Error, I2C};

impl<T: Deref<Target = Block>, PINS> I2C<T, PINS, Controller> {
    async fn non_blocking_read_internal(&mut self, buffer: &mut [u8]) -> Result<(), Error> {
        let lastindex = buffer.len() - 1;
        for (i, byte) in buffer.iter_mut().enumerate() {
            let first = i == 0;
            let last = i == lastindex;

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
        bytes: &[u8],
        do_stop: bool,
    ) -> Result<(), Error> {
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
        async move {
            let addr: u16 = address.into();
            Self::validate(addr, None, Some(buffer))?;
            self.setup(addr);

            self.non_blocking_read_internal(buffer).await
        }
    }

    fn write<'a>(&'a mut self, address: A, bytes: &'a [u8]) -> Self::WriteFuture<'a> {
        async move {
            let addr: u16 = address.into();
            Self::validate(addr, Some(bytes), None)?;
            self.setup(addr);

            self.non_blocking_write_internal(bytes, true).await
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
            Self::validate(addr, Some(bytes), Some(buffer))?;
            self.setup(addr);

            self.non_blocking_write_internal(bytes, false).await?;
            self.non_blocking_read_internal(buffer).await
        }
    }
}
