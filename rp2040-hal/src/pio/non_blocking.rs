use core::task::Poll;

use crate::async_utils::{
    sealed::{IrqWaker, Wakeable},
    CancellablePollFn,
};
use crate::dma::TransferSize;

use super::{Interrupt, PioIRQ, Rx, Tx};
use super::{PIOExt, StateMachineIndex, ValidStateMachine};
use super::{PIO0, PIO1, SM0, SM1, SM2, SM3};

macro_rules! impl_wakeable {
    ($($pio:ident => ( $($sm:ident),* )),*) => {
        $(
            impl<const IRQ: usize> Wakeable for Interrupt<'_, $pio, IRQ> {
                fn waker() -> &'static IrqWaker {
                    static WAKER: IrqWaker = IrqWaker::new();
                    &WAKER
                }
            }
            $(
                impl_wakeable!(Rx, 0, "Rx Not Empty", $pio, $sm);
                impl_wakeable!(Tx, 4, "Tx Not Full", $pio, $sm);
            )*
        )*
    };
    ($t:ident, $offset:expr, $doc:expr, $pio:ident, $sm:ident) => {
        impl<W> Wakeable for $t<($pio, $sm), W>
        where
            W: TransferSize,
        {
            fn waker() -> &'static IrqWaker {
                static WAKER: IrqWaker = IrqWaker::new();
                &WAKER
            }
        }
    };
}
impl_wakeable!(
  PIO0 => ( SM0, SM1, SM2, SM3 ),
  PIO1 => ( SM0, SM1, SM2, SM3 )
);

impl<const IRQ: usize, P> Interrupt<'_, P, IRQ>
where
    Self: Wakeable,
    P: PIOExt,
{
    /// Wakes a task awaiting of this interrupt.
    pub fn on_interrupt() {
        use crate::atomic_register_access::write_bitmask_clear;
        unsafe {
            let mask = 1 << (IRQ + 8);
            let sm_irq = (*P::ptr()).sm_irq(IRQ);
            if sm_irq.irq_ints().read().bits() & mask == mask {
                write_bitmask_clear(sm_irq.irq_inte().as_ptr(), mask);
            }
        }
        Self::waker().wake();
    }
}

macro_rules! on_interrupts {
    ($t:ident, $offset:expr, $event:expr) => {
        impl<W, P, SM> $t<(P, SM), W>
        where
            W: TransferSize,
            P: PIOExt,
            SM: StateMachineIndex,
            Self: Wakeable,
        {
            /// Wakes a task awaiting on
            #[doc = $event]
            pub fn on_interrupt(irq: PioIRQ) {
                use crate::atomic_register_access::write_bitmask_clear;
                unsafe {
                    let mask = 1 << ($offset + <(P, SM)>::id());
                    let irq = (&*P::ptr()).sm_irq(irq as usize);
                    if irq.irq_ints().read().bits() & mask == mask {
                        write_bitmask_clear(irq.irq_inte().as_ptr(), mask);
                    }
                }
                Self::waker().wake();
            }
        }
    };
}
on_interrupts!(Rx, 0, "Rx Not Empty");
on_interrupts!(Tx, 4, "Tx Not Full");

impl<const IRQ: usize, P> Interrupt<'_, P, IRQ>
where
    P: PIOExt,
    Self: Wakeable,
{
    /// Stalls until the IRQ fires
    pub async fn sm_interrupt(&mut self, id: u8) {
        assert!(id < 4, "invalid state machine interrupt number");
        let mask = 1 << id;

        CancellablePollFn::new(
            self,
            |me| unsafe {
                if (me.block().irq().read().irq().bits() & mask) == mask {
                    Poll::Ready(())
                } else {
                    Poll::Pending
                }
            },
            |me| me.enable_sm_interrupt(id),
            |me| {
                me.disable_sm_interrupt(id);
            },
        )
        .await
    }
}

impl<SM: ValidStateMachine, W: TransferSize> Rx<SM, W>
where
    Self: Wakeable,
{
    /// Reads a u32 from the FIFO.
    pub async fn async_read(&mut self, irq: PioIRQ) -> u32 {
        self.rx_not_empty(irq).await;
        let fifo_address = self.fifo_address();
        unsafe { core::ptr::read_volatile(fifo_address) }
    }

    /// Used to `.await` until the fifo is no longer empty.
    pub async fn rx_not_empty(&mut self, irq: PioIRQ) {
        CancellablePollFn::new(
            self,
            |me| {
                if me.is_empty() {
                    Poll::Pending
                } else {
                    Poll::Ready(())
                }
            },
            |me| me.enable_rx_not_empty_interrupt(irq),
            |me| {
                me.disable_rx_not_empty_interrupt(irq);
            },
        )
        .await
    }
}

impl<SM: ValidStateMachine, W: TransferSize> Tx<SM, W>
where
    Self: Wakeable,
{
    /// Writes a u8 to the FIFO replicated 4 times in a 32bits word.
    pub async fn async_write_u8_replicated(&mut self, irq: PioIRQ, value: u8) {
        self.async_write_generic(irq, value).await
    }
    /// Writes a u16 to the FIFO replicated 2 times in a 32bits word.
    pub async fn async_write_u16_replicated(&mut self, irq: PioIRQ, value: u16) {
        self.async_write_generic(irq, value).await
    }

    /// Writes a u32 to the FIFO.
    pub async fn async_write(&mut self, irq: PioIRQ, value: u32) {
        self.async_write_generic(irq, value).await
    }

    async fn async_write_generic<T>(&mut self, irq: PioIRQ, value: T) {
        self.tx_not_full(irq).await;
        // Safety: Only accessed by this instance (unless DMA is used).
        unsafe {
            let reg_ptr = self.fifo_address() as *mut T;
            reg_ptr.write_volatile(value);
        }
    }

    /// Used to `.await` until the fifo is no longer full.
    pub async fn tx_not_full(&mut self, irq: PioIRQ) {
        CancellablePollFn::new(
            self,
            |me| {
                if me.is_full() {
                    Poll::Pending
                } else {
                    Poll::Ready(())
                }
            },
            |me| me.enable_tx_not_full_interrupt(irq),
            |me| {
                me.disable_tx_not_full_interrupt(irq);
            },
        )
        .await
    }
}
