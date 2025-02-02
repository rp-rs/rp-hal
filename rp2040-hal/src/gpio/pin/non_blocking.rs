use embedded_hal_async::digital::Wait;
use core::task::Poll;
use crate::{
    async_utils::{sealed::{IrqWaker, Wakeable}, AsyncPeripheral, CancellablePollFn as CPFn},
    gpio::{
        Error,
        func::{ FunctionSio, SioConfig },
        Interrupt,
        Pin, PinId, PullType,
        pin::pin_sealed::TypeLevelPinId,
        new_pin,
    },
};

impl<I, P, S> AsyncPeripheral for Pin<I, FunctionSio<S>, P>
where
    I: PinId + TypeLevelPinId,
    P: PullType,
    S: SioConfig,
    Self: crate::async_utils::sealed::Wakeable,
{
    fn on_interrupt() {
        let pin_id = I::ID;
        let mut pin = unsafe { new_pin(pin_id) };

        if pin.interrupt_status(Interrupt::LevelLow)
            || pin.interrupt_status(Interrupt::LevelHigh)
            || pin.interrupt_status(Interrupt::EdgeLow)
            || pin.interrupt_status(Interrupt::EdgeHigh)
        {
            pin.set_interrupt_enabled(Interrupt::LevelLow, false);
            pin.set_interrupt_enabled(Interrupt::LevelHigh, false);
            pin.set_interrupt_enabled(Interrupt::EdgeLow, false);
            pin.set_interrupt_enabled(Interrupt::EdgeHigh, false);

            pin.clear_interrupt(Interrupt::LevelLow);
            pin.clear_interrupt(Interrupt::LevelHigh);
            pin.clear_interrupt(Interrupt::EdgeLow);
            pin.clear_interrupt(Interrupt::EdgeHigh);

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

    async fn _wait_for_high(&mut self) {
        let _ = CPFn::new(
            self,
            Self::poll_is_high,
            Self::enable_level_high_irq,
            Self::disable_level_high_irq,
        ).await;
    }

    async fn _wait_for_low(&mut self) {
        let _ = CPFn::new(
            self,
            Self::poll_is_low,
            Self::enable_level_low_irq,
            Self::disable_level_low_irq,
        ).await;
    }

    async fn _wait_for_rising_edge(&mut self) {
        if self._is_high() {
            self._wait_for_low().await;
        }

        let _ = CPFn::new(
            self,
            Self::poll_is_high,
            Self::enable_rising_edge_irq,
            Self::disable_rising_edge_irq,
        ).await;
    }

    async fn _wait_for_falling_edge(&mut self) {
        if self._is_low() {
            self._wait_for_high().await;
        }

        let _ = CPFn::new(
            self,
            Self::poll_is_high,
            Self::enable_falling_edge_irq,
            Self::disable_falling_edge_irq,
        ).await;
    }

    async fn _wait_for_any_edge(&mut self) {
        if self._is_high() {
            self._wait_for_falling_edge().await;
        } else {
            self._wait_for_rising_edge().await;
        }
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
