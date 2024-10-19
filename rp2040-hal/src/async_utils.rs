//! Commonly used in async implementations.

use core::{marker::PhantomData, task::Poll};

pub(crate) mod sealed {
    use core::{cell::Cell, task::Waker};
    use critical_section::Mutex;

    pub trait Wakeable {
        /// Returns the waker associated with driver instance.
        fn waker() -> &'static IrqWaker;
    }

    /// This type wraps a `Waker` in a `Mutex<Cell<Option<_>>`.
    ///
    /// While `critical_section::Mutex` intregrates nicely with RefCell, RefCell adds a borrow
    /// counter that is not necessary for this usecase.
    ///
    /// This type is kept sealed to prevent user from mistakenly messing with the waker such as
    /// clearing it while the driver is parked.
    pub struct IrqWaker {
        waker: Mutex<Cell<Option<Waker>>>,
    }

    impl Default for IrqWaker {
        fn default() -> Self {
            Self::new()
        }
    }

    impl IrqWaker {
        pub const fn new() -> Self {
            Self {
                waker: Mutex::new(Cell::new(None)),
            }
        }
        pub fn wake(&self) {
            critical_section::with(|cs| {
                if let Some(waker) = self.waker.borrow(cs).take() {
                    Waker::wake(waker);
                }
            });
        }
        pub fn register(&self, waker: &Waker) {
            critical_section::with(|cs| {
                self.waker.borrow(cs).replace(Some(waker.clone()));
            });
        }
        pub fn clear(&self) {
            critical_section::with(|cs| {
                self.waker.borrow(cs).take();
            });
        }
    }
}

/// Marks driver instances that can be bound to an interrupt to wake async tasks.
pub trait AsyncPeripheral: sealed::Wakeable {
    /// Signals the driver of an interrupt.
    fn on_interrupt();
}

#[must_use = "Future do nothing unless they are polled on."]
pub(crate) struct CancellablePollFn<'periph, Periph, PFn, EnIrqFn, CancelFn, OutputTy>
where
    Periph: sealed::Wakeable,
    CancelFn: FnMut(&mut Periph),
{
    periph: &'periph mut Periph,
    poll: PFn,
    enable_irq: EnIrqFn,
    cancel: CancelFn,
    done: bool,
    // captures F's return type.
    phantom: PhantomData<OutputTy>,
}
impl<'p, Periph, PFn, EnIrqFn, CancelFn, OutputTy>
    CancellablePollFn<'p, Periph, PFn, EnIrqFn, CancelFn, OutputTy>
where
    Periph: sealed::Wakeable,
    PFn: FnMut(&mut Periph) -> Poll<OutputTy>,
    EnIrqFn: FnMut(&mut Periph),
    CancelFn: FnMut(&mut Periph),
{
    pub(crate) fn new(
        periph: &'p mut Periph,
        poll: PFn,
        enable_irq: EnIrqFn,
        cancel: CancelFn,
    ) -> Self {
        Self {
            periph,
            poll,
            enable_irq,
            cancel,
            done: false,
            phantom: PhantomData,
        }
    }
}

impl<Periph, PFn, EnIrqFn, CancelFn, OutputTy> core::future::Future
    for CancellablePollFn<'_, Periph, PFn, EnIrqFn, CancelFn, OutputTy>
where
    Periph: sealed::Wakeable,
    PFn: FnMut(&mut Periph) -> Poll<OutputTy>,
    EnIrqFn: FnMut(&mut Periph),
    CancelFn: FnMut(&mut Periph),
{
    type Output = OutputTy;

    fn poll(self: core::pin::Pin<&mut Self>, cx: &mut core::task::Context<'_>) -> Poll<OutputTy> {
        // SAFETY: We are not moving anything.
        let Self {
            ref mut periph,
            poll: ref mut is_ready,
            enable_irq: ref mut setup_flags,
            ref mut done,
            ..
        } = unsafe { self.get_unchecked_mut() };
        let r = (is_ready)(periph);
        if r.is_pending() {
            Periph::waker().register(cx.waker());
            (setup_flags)(periph);
        } else {
            *done = true;
        }
        r
    }
}
impl<Periph, PFn, EnIrqFn, CancelFn, OutputTy> Drop
    for CancellablePollFn<'_, Periph, PFn, EnIrqFn, CancelFn, OutputTy>
where
    Periph: sealed::Wakeable,
    CancelFn: FnMut(&mut Periph),
{
    fn drop(&mut self) {
        if !self.done {
            Periph::waker().clear();
            (self.cancel)(self.periph);
        }
    }
}
