use crate::{
    async_utils::{
        sealed::{IrqWaker, Wakeable},
        AsyncPeripheral, CancellablePollFn as CPFn,
    },
    atomic_register_access::{write_bitmask_clear, write_bitmask_set},
    gpio::{
        func::{FunctionSio, SioConfig},
        pin::pin_sealed::{PinIdOps, TypeLevelPinId},
        Error, Interrupt, Pin, PinId, PullType,
    },
    sio::Sio,
};
use core::task::Poll;
use embedded_hal_async::digital::Wait;

const EDGE_INTERRUPTS_MASK: u32 = Interrupt::EdgeLow.mask() | Interrupt::EdgeHigh.mask();

impl<I, P, S> AsyncPeripheral for Pin<I, FunctionSio<S>, P>
where
    I: PinId + TypeLevelPinId,
    P: PullType,
    S: SioConfig,
    Self: crate::async_utils::sealed::Wakeable,
{
    fn on_interrupt() {
        const INTERRUPTS_MASK: u32 = Interrupt::LevelLow.mask()
            | Interrupt::LevelHigh.mask()
            | Interrupt::EdgeLow.mask()
            | Interrupt::EdgeHigh.mask();

        let pin_id = I::ID;

        let (ints_reg, ints_offset) = pin_id.proc_ints(Sio::core());
        let ints_bits = ints_reg.read().bits() >> ints_offset;

        // Check if any interrupt is active for Pin
        if ints_bits & INTERRUPTS_MASK != 0 {
            // Disable interrupts for Pin
            let (inte_reg, inte_offset) = pin_id.proc_inte(Sio::core());
            unsafe {
                write_bitmask_clear(inte_reg.as_ptr(), INTERRUPTS_MASK << inte_offset);
            }

            // Don't need to clear any interrupts here. Level interrupts are not latched,
            // Edge interrupts are cleared in the relevant poll functions

            Self::waker().wake();
        }
    }
}

impl<I, P, S> Wakeable for Pin<I, FunctionSio<S>, P>
where
    I: PinId + Wakeable,
    P: PullType,
    S: SioConfig,
{
    fn waker() -> &'static IrqWaker {
        I::waker()
    }
}

impl<I, P, S> Pin<I, FunctionSio<S>, P>
where
    I: PinId + Wakeable,
    P: PullType,
    S: SioConfig,
{
    fn poll_is_high(&mut self) -> Poll<Result<(), Error>> {
        if self._is_high() {
            Poll::Ready(Ok(()))
        } else {
            Poll::Pending
        }
    }

    fn poll_is_low(&mut self) -> Poll<Result<(), Error>> {
        if self._is_low() {
            Poll::Ready(Ok(()))
        } else {
            Poll::Pending
        }
    }

    fn poll_rising_edge(&mut self) -> Poll<Result<(), Error>> {
        // read raw interrupt status, because interrupt is disabled in on_interrupt(), but is not cleared
        if self.raw_interrupt_status(Interrupt::EdgeHigh) {
            self.clear_interrupt(Interrupt::EdgeHigh);

            Poll::Ready(Ok(()))
        } else {
            Poll::Pending
        }
    }

    fn poll_falling_edge(&mut self) -> Poll<Result<(), Error>> {
        // read raw interrupt status, because interrupt is disabled in on_interrupt(), but is not cleared
        if self.raw_interrupt_status(Interrupt::EdgeLow) {
            self.clear_interrupt(Interrupt::EdgeLow);

            Poll::Ready(Ok(()))
        } else {
            Poll::Pending
        }
    }

    fn poll_any_edge(&mut self) -> Poll<Result<(), Error>> {
        // read raw interrupt status, because interrupt is disabled in on_interrupt(), but is not cleared
        let (reg, offset) = self.id.intr();
        let bits = reg.read().bits() >> offset;

        if bits & EDGE_INTERRUPTS_MASK != 0 {
            reg.write(|w| unsafe { w.bits(EDGE_INTERRUPTS_MASK << offset) });

            Poll::Ready(Ok(()))
        } else {
            Poll::Pending
        }
    }

    fn disable_level_high_irq(&mut self) {
        self.set_interrupt_enabled(Interrupt::LevelHigh, false);
    }

    fn enable_level_high_irq(&mut self) {
        self.set_interrupt_enabled(Interrupt::LevelHigh, true);
    }

    fn disable_level_low_irq(&mut self) {
        self.set_interrupt_enabled(Interrupt::LevelLow, false);
    }

    fn enable_level_low_irq(&mut self) {
        self.set_interrupt_enabled(Interrupt::LevelLow, true);
    }

    fn disable_rising_edge_irq(&mut self) {
        self.set_interrupt_enabled(Interrupt::EdgeHigh, false)
    }

    fn enable_rising_edge_irq(&mut self) {
        self.set_interrupt_enabled(Interrupt::EdgeHigh, true)
    }

    fn disable_falling_edge_irq(&mut self) {
        self.set_interrupt_enabled(Interrupt::EdgeLow, false)
    }

    fn enable_falling_edge_irq(&mut self) {
        self.set_interrupt_enabled(Interrupt::EdgeLow, true)
    }

    fn disable_any_edge_irq(&mut self) {
        let (reg, offset) = self.id.proc_inte(Sio::core());
        unsafe {
            write_bitmask_clear(reg.as_ptr(), EDGE_INTERRUPTS_MASK << offset);
        }
    }

    fn enable_any_edge_irq(&mut self) {
        let (reg, offset) = self.id.proc_inte(Sio::core());
        unsafe {
            write_bitmask_set(reg.as_ptr(), EDGE_INTERRUPTS_MASK << offset);
        }
    }

    fn raw_interrupt_status(&self, interrupt: Interrupt) -> bool {
        let (reg, offset) = self.id.intr();
        let mask = interrupt.mask();
        (reg.read().bits() >> offset) & mask == mask
    }

    async fn _wait_for_high(&mut self) {
        let _ = CPFn::new(
            self,
            Self::poll_is_high,
            Self::enable_level_high_irq,
            Self::disable_level_high_irq,
        )
        .await;
    }

    async fn _wait_for_low(&mut self) {
        let _ = CPFn::new(
            self,
            Self::poll_is_low,
            Self::enable_level_low_irq,
            Self::disable_level_low_irq,
        )
        .await;
    }

    async fn _wait_for_rising_edge(&mut self) {
        let _ = CPFn::new(
            self,
            Self::poll_rising_edge,
            Self::enable_rising_edge_irq,
            Self::disable_rising_edge_irq,
        )
        .await;
    }

    async fn _wait_for_falling_edge(&mut self) {
        let _ = CPFn::new(
            self,
            Self::poll_falling_edge,
            Self::enable_falling_edge_irq,
            Self::disable_falling_edge_irq,
        )
        .await;
    }

    async fn _wait_for_any_edge(&mut self) {
        let _ = CPFn::new(
            self,
            Self::poll_any_edge,
            Self::enable_any_edge_irq,
            Self::disable_any_edge_irq,
        )
        .await;
    }
}

impl<I, P, S> Wait for Pin<I, FunctionSio<S>, P>
where
    I: PinId + Wakeable,
    P: PullType,
    S: SioConfig,
    Self: AsyncPeripheral,
{
    async fn wait_for_high(&mut self) -> Result<(), Self::Error> {
        self._wait_for_high().await;
        Ok(())
    }

    async fn wait_for_low(&mut self) -> Result<(), Self::Error> {
        self._wait_for_low().await;
        Ok(())
    }

    async fn wait_for_rising_edge(&mut self) -> Result<(), Self::Error> {
        self._wait_for_rising_edge().await;
        Ok(())
    }

    async fn wait_for_falling_edge(&mut self) -> Result<(), Self::Error> {
        self._wait_for_falling_edge().await;
        Ok(())
    }

    async fn wait_for_any_edge(&mut self) -> Result<(), Self::Error> {
        self._wait_for_any_edge().await;
        Ok(())
    }
}
