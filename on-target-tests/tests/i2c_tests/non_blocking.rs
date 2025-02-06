use core::{
    cell::RefCell,
    future::Future,
    ops::{Deref, RangeInclusive},
    task::Poll,
};

use fugit::{HertzU32, RateExtU32};
use futures::FutureExt;
use heapless::Vec;

use hal::{
    clocks::init_clocks_and_plls,
    gpio::{FunctionI2C, Pin, PullUp},
    i2c::{Error, ValidAddress},
    pac,
    watchdog::Watchdog,
    Clock,
};
#[cfg(feature = "rp2040")]
use rp2040_hal as hal;
#[cfg(feature = "rp235x")]
use rp235x_hal as hal;

use super::{Controller, FIFOBuffer, Generator, Target, TargetState};

pub struct State {
    controller: Option<Controller>,
    target: Option<Target>,
    resets: hal::pac::RESETS,
    ref_clock_freq: HertzU32,
    payload: RefCell<TargetState>,
}

pub fn run_test(f: impl Future) {
    super::test_executor::execute(f);
}
async fn wait_with(payload: &RefCell<TargetState>, mut f: impl FnMut(&TargetState) -> bool) {
    while f(payload.borrow().deref()) {
        let mut done = false;
        core::future::poll_fn(|cx| {
            cx.waker().wake_by_ref();
            if !done {
                done = true;
                Poll::Pending
            } else {
                Poll::Ready(())
            }
        })
        .await;
    }
}

pub fn setup<T: ValidAddress>(xtal_freq_hz: u32, addr: T) -> State {
    let mut pac = pac::Peripherals::take().unwrap();
    let mut watchdog = Watchdog::new(pac.WATCHDOG);

    let clocks = init_clocks_and_plls(
        xtal_freq_hz,
        pac.XOSC,
        pac.CLOCKS,
        pac.PLL_SYS,
        pac.PLL_USB,
        &mut pac.RESETS,
        &mut watchdog,
    )
    .ok()
    .unwrap();

    // The single-cycle I/O block controls our GPIO pins
    let sio = hal::Sio::new(pac.SIO);
    let pins = hal::gpio::Pins::new(
        pac.IO_BANK0,
        pac.PADS_BANK0,
        sio.gpio_bank0,
        &mut pac.RESETS,
    );

    // Configure two pins as being I²C, not GPIO
    let ctrl_sda_pin: Pin<_, FunctionI2C, PullUp> = pins.gpio0.reconfigure();
    let ctrl_scl_pin: Pin<_, FunctionI2C, PullUp> = pins.gpio1.reconfigure();

    let trg_sda_pin: Pin<_, FunctionI2C, PullUp> = pins.gpio2.reconfigure();
    let trg_scl_pin: Pin<_, FunctionI2C, PullUp> = pins.gpio3.reconfigure();

    let i2c_ctrl = hal::I2C::new_controller(
        pac.I2C0,
        ctrl_sda_pin,
        ctrl_scl_pin,
        400.kHz(),
        &mut pac.RESETS,
        clocks.system_clock.freq(),
    );
    let i2c_target = hal::I2C::new_peripheral_event_iterator(
        pac.I2C1,
        trg_sda_pin,
        trg_scl_pin,
        &mut pac.RESETS,
        addr,
    );

    unsafe {
        cortex_m::peripheral::NVIC::unpend(hal::pac::Interrupt::I2C0_IRQ);
        cortex_m::peripheral::NVIC::unmask(hal::pac::Interrupt::I2C0_IRQ);
        cortex_m::peripheral::NVIC::unpend(hal::pac::Interrupt::I2C1_IRQ);
        cortex_m::peripheral::NVIC::unmask(hal::pac::Interrupt::I2C1_IRQ);
    }

    State {
        controller: Some(i2c_ctrl),
        target: Some(i2c_target),
        resets: pac.RESETS,
        ref_clock_freq: clocks.system_clock.freq(),
        payload: RefCell::new(TargetState::new()),
    }
}

pub fn reset<T: ValidAddress>(state: &mut State, addr: T) {
    // reset controller
    let (i2c, (sda, scl)) = state
        .controller
        .take()
        .expect("controller's missing.")
        .free(&mut state.resets);

    let (i2c_t, (sda_t, scl_t)) = state
        .target
        .take()
        .expect("target's missing")
        .free(&mut state.resets);

    state.payload.replace(Default::default());

    state.target = Some(hal::I2C::new_peripheral_event_iterator(
        i2c_t,
        sda_t,
        scl_t,
        &mut state.resets,
        addr,
    ));

    state.controller = Some(hal::I2C::new_controller(
        i2c,
        sda,
        scl,
        400.kHz(),
        &mut state.resets,
        state.ref_clock_freq,
    ));
}

pub async fn target_handler(payload: &RefCell<TargetState>, target: &mut Target) -> (u32, u32) {
    loop {
        let evt = target.wait_next().await;

        super::target_handler(target, evt, &mut *payload.borrow_mut(), false);
    }
}

async fn embedded_hal_case<A: ValidAddress>(
    controller: &mut Controller,
    addr: A,
    v: &mut ([u8; 25], [u8; 25], [u8; 25], [u8; 14], [u8; 14]),
    payload: &RefCell<TargetState>,
) -> Result<(), Error> {
    use embedded_hal_async::i2c::I2c;
    let sample1: FIFOBuffer = Generator::seq().take(25).collect();
    let sample2: FIFOBuffer = Generator::fib().take(14).collect();

    // we need to wait for stop to be registered between each operations otherwise we have no
    // way to know when the Target side has finished processing the last request.
    controller.write(addr, &sample1).await?;
    wait_with(payload, |p| p.stop_cnt != 1).await;

    controller.read(addr, &mut v.0).await?;
    wait_with(payload, |p| p.stop_cnt != 2).await;

    controller.read(addr, &mut v.1).await?;
    wait_with(payload, |p| p.stop_cnt != 3).await;

    controller.read(addr, &mut v.2).await?;
    wait_with(payload, |p| p.stop_cnt != 4).await;

    controller.write_read(addr, &sample2, &mut v.3).await?;
    wait_with(payload, |p| p.stop_cnt != 5).await;

    controller.write(addr, &sample2).await?;
    wait_with(payload, |p| p.stop_cnt != 6).await;

    controller.write(addr, &sample1).await?;
    wait_with(payload, |p| p.stop_cnt != 7).await;

    controller.write_read(addr, &sample1, &mut v.4).await?;
    wait_with(payload, |p| p.stop_cnt != 8).await;
    Ok::<(), Error>(())
}
pub async fn embedded_hal<T: ValidAddress>(
    state: &mut State,
    addr: T,
    restart_count: RangeInclusive<u32>,
) {
    // Throttling is important for this test as it also ensures that the Target implementation
    // does not "waste" bytes that would be discarded otherwise.
    //
    // One down side of this is that the Target implementation is unable to detect restarts
    // between consicutive write operations
    reset(state, addr);

    // Test
    let mut v = Default::default();
    let ctrl = embedded_hal_case(
        state.controller.as_mut().expect("controller's missing."),
        addr,
        &mut v,
        &state.payload,
    );
    let trgt = target_handler(
        &state.payload,
        state.target.as_mut().take().expect("target’s missing"),
    );
    futures::select_biased! {
        r = ctrl.fuse() => r.expect("Controller test success"),
        _ = trgt.fuse() => {}
    }

    // Validate

    // There are 14restarts in this sequence but because of latency in the target handling, it
    // may only detect 7.
    let actual_restart_count = state.payload.borrow().restart_cnt;
    assert!(
        restart_count.contains(&actual_restart_count),
        "restart count out of range {} ∉ {:?}",
        actual_restart_count,
        restart_count
    );

    // assert writes
    let sample1: FIFOBuffer = Generator::seq().take(25).collect();
    let sample2: FIFOBuffer = Generator::fib().take(14).collect();
    let e: FIFOBuffer = itertools::chain!(
        sample1.iter(),
        sample2.iter(),
        sample2.iter(),
        sample1.iter(),
        sample1.iter(),
    )
    .cloned()
    .collect();
    assert_eq!(state.payload.borrow().vec, e);
    // assert reads
    let g: FIFOBuffer = itertools::chain!(
        Generator::fib().take(25),
        Generator::fib().skip(25 + 7).take(25),
        Generator::fib().skip(2 * (25 + 7)).take(25),
        Generator::seq().take(14),
        Generator::fib().take(14)
    )
    .collect();
    let h: FIFOBuffer = itertools::chain!(
        v.0.into_iter(),
        v.1.into_iter(),
        v.2.into_iter(),
        v.3.into_iter(),
        v.4.into_iter()
    )
    .collect();
    assert_eq!(g, h);
}

pub async fn transaction<A: ValidAddress>(
    state: &mut State,
    addr: A,
    restart_count: RangeInclusive<u32>,
) {
    use embedded_hal::i2c::Operation;
    use embedded_hal_async::i2c::I2c;
    reset(state, addr);

    // Throttling is important for this test as it also ensures that the Target implementation
    // does not "waste" bytes that would be discarded otherwise.
    //
    // One down side of this is that the Target implementation is unable to detect restarts
    // between consicutive write operations
    let sample1: Vec<u8, 25> = Generator::seq().take(25).collect();
    let sample2: Vec<u8, 14> = Generator::fib().take(14).collect();

    // Test
    let mut v: ([u8; 25], [u8; 25], [u8; 25], [u8; 14], [u8; 14]) = Default::default();
    let mut ops = [
        Operation::Write(&sample1), // goes to v2
        Operation::Read(&mut v.0),
        Operation::Read(&mut v.1),
        Operation::Read(&mut v.2),
        Operation::Write(&sample2), // goes to v3
        Operation::Read(&mut v.3),
        Operation::Write(&sample2), // goes to v4
        Operation::Write(&sample1), // remains in buffer
        Operation::Write(&sample1), // remains in buffer
        Operation::Read(&mut v.4),
    ];

    let case = async {
        state
            .controller
            .as_mut()
            .expect("controller's missing.")
            .transaction(addr, &mut ops)
            .await
            .expect("Controller test success");
        wait_with(&state.payload, |p| p.stop_cnt != 1).await;
    };
    futures::select_biased! {
        _ = case.fuse() => {}
        _ = target_handler(
            &state.payload,
            state.target.as_mut().take().expect("target’s missing"),
        ).fuse() => {}
    }

    // Validate

    // There are 14restarts in this sequence but because of latency in the target handling, it
    // may only detect 7.
    let actual_restart_count = state.payload.borrow().restart_cnt;
    assert!(
        restart_count.contains(&actual_restart_count),
        "restart count out of range {} ∉ {:?}",
        actual_restart_count,
        restart_count
    );
    // assert writes
    let e: FIFOBuffer = itertools::chain!(
        Generator::seq().take(25),
        Generator::fib().take(14),
        Generator::fib().take(14),
        Generator::seq().take(25),
        Generator::seq().take(25),
    )
    .collect();
    assert_eq!(e, state.payload.borrow().vec);
    // assert reads
    let g: FIFOBuffer = itertools::chain!(
        Generator::fib().take(25),
        Generator::fib().skip(32).take(25),
        Generator::fib().skip(64).take(25),
        Generator::fib().skip(96).take(14),
        Generator::fib().skip(112).take(14),
    )
    .collect();
    let h: FIFOBuffer = itertools::chain!(
        v.0.into_iter(),
        v.1.into_iter(),
        v.2.into_iter(),
        v.3.into_iter(),
        v.4.into_iter()
    )
    .collect();
    assert_eq!(g, h);
}

pub async fn transaction_iter<A: ValidAddress>(state: &mut State, addr: A) {
    use i2c_write_iter::non_blocking::I2cIter;
    reset(state, addr);

    let samples: FIFOBuffer = Generator::seq().take(25).collect();
    let controller = state.controller.as_mut().expect("controller's missing.");
    let case = async {
        controller
            .transaction_iter(
                addr,
                [i2c_write_iter::Operation::WriteIter(
                    samples.iter().cloned(),
                )],
            )
            .await
            .expect("Successful write_iter");
        wait_with(&state.payload, |p| p.stop_cnt != 1).await;
    };

    futures::select_biased! {
        _ = case.fuse() => {}
        _ = target_handler(
            &state.payload,
            state.target.as_mut().take().expect("target’s missing"),
        ).fuse() => {}
    }

    assert_eq!(samples, state.payload.borrow().vec);
}
