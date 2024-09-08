//! Simplistic test executor
//!
//! Compared to a real executor, this has some limitations:
//!
//! - Can only run to completion (like block_on, but without busy polling)
//! - Can't spawn additional tasks
//! - Must not be called multiple times concurrently

use core::{
    future::Future,
    pin::{self, Pin},
    ptr::addr_of,
    sync::atomic::{AtomicBool, Ordering},
    task::{Context, Poll, RawWaker, RawWakerVTable, Waker},
};

use once_cell::sync::OnceCell;

static WOKE: AtomicBool = AtomicBool::new(false);
static POLLING: AtomicBool = AtomicBool::new(false);

fn wake_fn(_data: *const ()) {
    if !POLLING.load(Ordering::Relaxed) {
        defmt::info!("waker called while not polling");
    }
    WOKE.store(true, Ordering::Relaxed);
}

const fn clone_fn(data: *const ()) -> RawWaker {
    RawWaker::new(data, raw_waker_vtable())
}

fn drop_fn(_data: *const ()) {}

const fn raw_waker_vtable() -> &'static RawWakerVTable {
    const VTABLE: RawWakerVTable = RawWakerVTable::new(clone_fn, wake_fn, wake_fn, drop_fn);
    &VTABLE
}

fn context() -> Context<'static> {
    static DATA: () = ();

    static WAKER: OnceCell<Waker> = OnceCell::new();
    // Safety: The functions in the vtable of this executor only modify static atomics.
    let waker = WAKER.get_or_init(|| unsafe { Waker::from_raw(clone_fn(addr_of!(DATA))) });

    // Starting from rust 1.82.0, this could be used:
    // static WAKER: Waker = unsafe { Waker::from_raw(clone_fn(addr_of!(DATA))) };

    Context::from_waker(waker)
}

/// Run future to completion
///
/// poll() will only be called when the waker was invoked, so this is suitable to test
/// if the waker is properly triggered from an interrupt.
///
/// This won't work as expected of multiple calls to `execute` happen concurrently.
pub fn execute<T>(future: impl Future<Output = T>) -> T {
    let mut pinned: Pin<&mut _> = pin::pin!(future);
    if WOKE.load(Ordering::Relaxed) {
        defmt::info!("woken before poll - ignoring");
    }
    POLLING.store(true, Ordering::Relaxed);
    loop {
        WOKE.store(false, Ordering::Relaxed);
        if let Poll::Ready(result) = pinned.as_mut().poll(&mut context()) {
            WOKE.store(false, Ordering::Relaxed);
            POLLING.store(false, Ordering::Relaxed);
            break result;
        }
        while !WOKE.load(Ordering::Relaxed) {
            core::hint::spin_loop();
            // TODO WFI or similar?
            // But probably not important for this test executor
        }
    }
}
