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
    ptr,
    sync::atomic::{AtomicBool, Ordering},
    task::{Context, Poll, RawWaker, RawWakerVTable, Waker},
};

use once_cell::sync::OnceCell;

static WOKE: AtomicBool = AtomicBool::new(false);
static POLLING: AtomicBool = AtomicBool::new(false);

static VTABLE: RawWakerVTable = RawWakerVTable::new(clone_fn, wake_fn, wake_fn, drop_fn);

fn wake_fn(_data: *const ()) {
    if !POLLING.load(Ordering::Relaxed) {
        defmt::info!("waker called while not polling");
    }
    WOKE.store(true, Ordering::Relaxed);
}

fn clone_fn(data: *const ()) -> RawWaker {
    RawWaker::new(data, &VTABLE)
}

fn drop_fn(_data: *const ()) {}

fn context() -> Context<'static> {
    static WAKER: OnceCell<Waker> = OnceCell::new();
    // Safety: The functions in the vtable of this executor only modify static atomics.
    let waker =
        WAKER.get_or_init(|| unsafe { Waker::from_raw(RawWaker::new(ptr::null(), &VTABLE)) });

    // Starting from rust 1.82.0, this could be used:
    // static WAKER: Waker = unsafe { Waker::from_raw(RawWaker::new(ptr::null(), &VTABLE)) };
    // (stabilized by https://github.com/rust-lang/rust/pull/128228)

    Context::from_waker(waker)
}

/// Run future to completion
///
/// poll() will only be called when the waker was invoked, so this is suitable to test
/// if the waker is properly triggered from an interrupt.
///
/// This won't work as expected of multiple calls to `execute` happen concurrently.
///
/// (Calling this function from multiple threads concurrently doesn't violate any
/// safety guarantees, but wakers may wake the wrong task, making futures stall.)
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
            // In a real executor, there should be a WFI/WFE or similar here, to avoid
            // busy looping.
            // As this is only a test executor, we don't care.
            core::hint::spin_loop();
        }
    }
}
