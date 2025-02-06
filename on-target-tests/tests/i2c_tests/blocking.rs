use core::{cell::RefCell, ops::RangeInclusive};

use critical_section::Mutex;
use fugit::{HertzU32, RateExtU32};
use hal::{
    clocks::init_clocks_and_plls,
    gpio::{FunctionI2C, Pin, PullUp},
    i2c::{Error, ValidAddress},
    pac,
    watchdog::Watchdog,
    Clock, Timer,
};
#[cfg(feature = "rp2040")]
use rp2040_hal as hal;
#[cfg(feature = "rp235x")]
use rp235x_hal as hal;

use super::{Controller, FIFOBuffer, Generator, MutexCell, Target, TargetState};

pub struct State {
    controller: Option<Controller>,
    #[cfg(feature = "rp2040")]
    timer: Timer,
    #[cfg(feature = "rp235x")]
    timer: Timer<hal::timer::CopyableTimer0>,
    resets: hal::pac::RESETS,
    ref_clock_freq: HertzU32,
}

static TARGET: MutexCell<Option<Target>> = Mutex::new(RefCell::new(None));

static PAYLOAD: MutexCell<TargetState> = MutexCell::new(RefCell::new(TargetState::new()));
#[cfg(feature = "rp2040")]
static TIMER: MutexCell<Option<Timer>> = MutexCell::new(RefCell::new(None));
#[cfg(feature = "rp235x")]
static TIMER: MutexCell<Option<Timer<hal::timer::CopyableTimer0>>> =
    MutexCell::new(RefCell::new(None));

macro_rules! assert_vec_eq {
    ($e:expr) => {
        critical_section::with(|cs| {
            let v = &mut PAYLOAD.borrow_ref_mut(cs).vec;
            assert_eq!(*v, $e, "FIFO");
            v.clear();
        });
    };
}
macro_rules! assert_restart_count {
    ($e:expr) => {{
        let restart_cnt: u32 = critical_section::with(|cs| PAYLOAD.borrow_ref(cs).restart_cnt);
        defmt::assert!(
            $e.contains(&restart_cnt),
            "restart count out of range {} ∉ {}",
            restart_cnt,
            $e
        );
    }};
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

    #[cfg(feature = "rp2040")]
    let timer = hal::Timer::new(pac.TIMER, &mut pac.RESETS, &clocks);
    #[cfg(feature = "rp235x")]
    let timer = hal::Timer::new_timer0(pac.TIMER0, &mut pac.RESETS, &clocks);

    // The single-cycle I/O block controls our GPIO pins
    let mut sio = hal::Sio::new(pac.SIO);
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

    critical_section::with(|cs| TARGET.replace(cs, Some(i2c_target)));

    static STACK: hal::multicore::Stack<10240> = hal::multicore::Stack::new();
    unsafe {
        // delegate I2C1 irqs to core 1
        hal::multicore::Multicore::new(&mut pac.PSM, &mut pac.PPB, &mut sio.fifo)
            .cores()
            .get_mut(1)
            .expect("core 1 is not available")
            .spawn(STACK.take().unwrap(), || {
                cortex_m::peripheral::NVIC::unpend(hal::pac::Interrupt::I2C1_IRQ);
                cortex_m::peripheral::NVIC::unmask(hal::pac::Interrupt::I2C1_IRQ);

                loop {
                    cortex_m::asm::wfi()
                }
            })
            .expect("failed to start second core.");
    }

    State {
        controller: Some(i2c_ctrl),
        timer,
        resets: pac.RESETS,
        ref_clock_freq: clocks.system_clock.freq(),
    }
}

pub fn reset<T: ValidAddress>(state: &mut State, addr: T, throttling: bool) -> &mut Controller {
    // reset controller
    let (i2c, (sda, scl)) = state
        .controller
        .take()
        .expect("State contains a controller")
        .free(&mut state.resets);

    // TARGET is shared with core1. Therefore this needs to happen in a cross-core
    // critical-section.
    critical_section::with(|cs| {
        // reset peripheral
        let (i2c, (sda, scl)) = TARGET
            .replace(cs, None)
            .expect("State contains a target")
            .free(&mut state.resets);

        // reset payload storage
        PAYLOAD.replace_with(cs, |_| TargetState::new());

        // remove timer/disable throttling
        TIMER.replace(cs, throttling.then_some(state.timer));

        //
        TARGET.replace(
            cs,
            Some(hal::I2C::new_peripheral_event_iterator(
                i2c,
                sda,
                scl,
                &mut state.resets,
                addr,
            )),
        );
    });

    state.controller = Some(hal::I2C::new_controller(
        i2c,
        sda,
        scl,
        400.kHz(),
        &mut state.resets,
        state.ref_clock_freq,
    ));
    state
        .controller
        .as_mut()
        .expect("State contains a controller")
}

/// Wait for the expected count of Stop event to ensure the target side has finished processing
/// requests.
///
/// If a test ends with a write command, there is a risk that the test will check the content of
/// the shared buffer while the target handler hasn't finished processing its fifo.
pub fn wait_stop_count(stop_cnt: u32) {
    while critical_section::with(|cs| PAYLOAD.borrow_ref(cs).stop_cnt) < stop_cnt {
        cortex_m::asm::wfe();
    }
    defmt::flush();
}

pub fn peripheral_handler() {
    critical_section::with(|cs| {
        let Some(ref mut target) = *TARGET.borrow_ref_mut(cs) else {
            return;
        };

        let mut timer = TIMER.borrow_ref_mut(cs);

        while let Some(evt) = target.next_event() {
            if let Some(t) = timer.as_mut() {
                use embedded_hal_0_2::blocking::delay::DelayUs;
                t.delay_us(50);
            }

            super::target_handler(
                target,
                evt,
                &mut *PAYLOAD.borrow_ref_mut(cs),
                timer.is_some(),
            );
        }
    })
}

pub fn write<T: ValidAddress>(state: &mut State, addr: T) {
    use embedded_hal_0_2::blocking::i2c::Write;
    let controller = reset(state, addr, false);

    let samples: FIFOBuffer = Generator::seq().take(25).collect();
    assert_eq!(controller.write(addr, &samples).is_ok(), true);
    wait_stop_count(1);

    assert_restart_count!((0..=0));
    assert_vec_eq!(samples);
}
pub fn write_iter<T: ValidAddress>(state: &mut State, addr: T) {
    let controller = reset(state, addr, false);

    let samples: FIFOBuffer = Generator::seq().take(25).collect();
    controller
        .write_iter(addr, samples.iter().cloned())
        .expect("Successful write_iter");
    wait_stop_count(1);

    assert_restart_count!((0..=0));
    assert_vec_eq!(samples);
}

pub fn write_iter_read<T: ValidAddress>(
    state: &mut State,
    addr: T,
    restart_count: RangeInclusive<u32>,
) {
    let controller = reset(state, addr, false);

    let samples_seq: FIFOBuffer = Generator::seq().take(25).collect();
    let samples_fib: FIFOBuffer = Generator::fib().take(25).collect();
    let mut v = [0u8; 25];
    controller
        .write_iter_read(addr, samples_fib.iter().cloned(), &mut v)
        .expect("Successful write_iter_read");
    wait_stop_count(1);

    assert_restart_count!(restart_count);
    assert_eq!(v, samples_seq);
    assert_vec_eq!(samples_fib);
}

pub fn write_read<T: ValidAddress>(state: &mut State, addr: T, restart_count: RangeInclusive<u32>) {
    use embedded_hal_0_2::blocking::i2c::WriteRead;
    let controller = reset(state, addr, false);

    let samples_seq: FIFOBuffer = Generator::seq().take(25).collect();
    let samples_fib: FIFOBuffer = Generator::fib().take(25).collect();
    let mut v = [0u8; 25];
    controller
        .write_read(addr, &samples_fib, &mut v)
        .expect("successfully write_read");
    wait_stop_count(1);

    assert_restart_count!(restart_count);
    assert_eq!(v, samples_seq);
    assert_vec_eq!(samples_fib);
}

pub fn read<T: ValidAddress>(state: &mut State, addr: T, restart_count: RangeInclusive<u32>) {
    use embedded_hal_0_2::blocking::i2c::Read;
    let controller = reset(state, addr, false);

    let mut v = [0u8; 25];
    controller.read(addr, &mut v).expect("successfully read");
    wait_stop_count(1);

    let samples: FIFOBuffer = Generator::fib().take(25).collect();
    assert_restart_count!(restart_count);
    assert_eq!(v, samples);
    assert_vec_eq!([]);
}

pub fn transactions_read<T: ValidAddress>(
    state: &mut State,
    addr: T,
    restart_count: RangeInclusive<u32>,
) {
    use embedded_hal::i2c::{I2c, Operation};
    let controller = reset(state, addr, false);

    let mut v = [0u8; 25];
    controller
        .transaction(addr, &mut [Operation::Read(&mut v)])
        .expect("successfully write_read");
    wait_stop_count(1);

    let samples: FIFOBuffer = Generator::fib().take(25).collect();
    assert_restart_count!(restart_count);
    assert_eq!(v, samples);
    assert_vec_eq!([]);
}

pub fn transactions_write<T: ValidAddress>(state: &mut State, addr: T) {
    use embedded_hal::i2c::{I2c, Operation};
    let controller = reset(state, addr, false);

    let samples: FIFOBuffer = Generator::seq().take(25).collect();
    controller
        .transaction(addr, &mut [Operation::Write(&samples)])
        .expect("successfully write_read");
    wait_stop_count(1);

    assert_restart_count!((0..=0));
    assert_vec_eq!(samples);
}

pub fn transactions_read_write<T: ValidAddress>(
    state: &mut State,
    addr: T,
    restart_count: RangeInclusive<u32>,
) {
    use embedded_hal::i2c::{I2c, Operation};
    let controller = reset(state, addr, true);

    let samples_seq: FIFOBuffer = Generator::seq().take(25).collect();
    let samples_fib: FIFOBuffer = Generator::fib().take(25).collect();
    let mut v = [0u8; 25];
    controller
        .transaction(
            addr,
            &mut [Operation::Read(&mut v), Operation::Write(&samples_seq)],
        )
        .expect("successfully write_read");
    wait_stop_count(1);

    assert_restart_count!(restart_count);
    assert_eq!(v, samples_fib);
    assert_vec_eq!(samples_seq);
}

pub fn transactions_write_read<T: ValidAddress>(
    state: &mut State,
    addr: T,
    restart_count: RangeInclusive<u32>,
) {
    use embedded_hal::i2c::{I2c, Operation};
    let controller = reset(state, addr, false);

    let samples_seq: FIFOBuffer = Generator::seq().take(25).collect();
    let mut v = [0u8; 25];

    controller
        .transaction(
            addr,
            &mut [Operation::Write(&samples_seq), Operation::Read(&mut v)],
        )
        .expect("successfully write_read");
    wait_stop_count(1);

    assert_restart_count!(restart_count);
    assert_eq!(v, samples_seq);
    assert_vec_eq!(samples_seq);
}

pub fn transaction<T: ValidAddress>(
    state: &mut State,
    addr: T,
    restart_count: RangeInclusive<u32>,
) {
    // Throttling is important for this test as it also ensures that the Target implementation
    // does not "waste" bytes that would be discarded otherwise.
    //
    // One down side of this is that the Target implementation is unable to detect restarts
    // between consicutive write operations
    use embedded_hal::i2c::{I2c, Operation};
    let controller = reset(state, addr, true);

    let mut v = ([0u8; 14], [0u8; 25], [0u8; 25], [0u8; 14], [0u8; 14]);
    let samples: FIFOBuffer = Generator::seq().take(25).collect();
    controller
        .transaction(
            addr,
            &mut [
                Operation::Write(&samples), // goes to v2
                Operation::Read(&mut v.0),
                Operation::Read(&mut v.1),
                Operation::Read(&mut v.2),
                Operation::Write(&samples), // goes to v3
                Operation::Read(&mut v.3),
                Operation::Write(&samples), // goes to v4
                Operation::Write(&samples), // remains in buffer
                Operation::Write(&samples), // remains in buffer
                Operation::Read(&mut v.4),
            ],
        )
        .expect("successfully write_read");
    wait_stop_count(1);

    // There are 14restarts in this sequence but because of latency in the target handling, it
    // may only detect 7.
    assert_restart_count!(restart_count);

    // assert writes
    let e: FIFOBuffer = itertools::chain!(
        samples.iter(),
        samples.iter(),
        samples.iter(),
        samples.iter(),
        samples.iter(),
    )
    .cloned()
    .collect();
    assert_vec_eq!(e);

    // assert reads
    let g: FIFOBuffer = Generator::seq().take(92).collect();
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

pub fn transactions_iter<T: ValidAddress>(
    state: &mut State,
    addr: T,
    restart_count: RangeInclusive<u32>,
) {
    use embedded_hal::i2c::{I2c, Operation};
    let controller = reset(state, addr, false);

    let samples: FIFOBuffer = Generator::seq().take(25).collect();
    let mut v = [0u8; 25];
    controller
        .transaction(
            addr,
            &mut [Operation::Write(&samples), Operation::Read(&mut v)],
        )
        .expect("successfully write_read");
    wait_stop_count(1);

    assert_restart_count!(restart_count);
    assert_eq!(v, samples);
    assert_vec_eq!(samples);
}

pub fn embedded_hal<T: ValidAddress>(
    state: &mut State,
    addr: T,
    restart_count: RangeInclusive<u32>,
) {
    // Throttling is important for this test as it also ensures that the Target implementation
    // does not "waste" bytes that would be discarded otherwise.
    //
    // One down side of this is that the Target implementation is unable to detect restarts
    // between consicutive write operations
    use embedded_hal::i2c::I2c;
    let controller = reset(state, addr, true);

    let samples1: FIFOBuffer = Generator::seq().take(25).collect();
    let samples2: FIFOBuffer = Generator::fib().take(14).collect();
    let mut v = ([0; 14], [0; 25], [0; 25], [0; 14], [0; 14]);

    let mut case = || {
        controller.write(addr, &samples1)?;
        wait_stop_count(1);
        controller.read(addr, &mut v.0)?;
        wait_stop_count(2);
        controller.read(addr, &mut v.1)?;
        wait_stop_count(3);
        controller.read(addr, &mut v.2)?;
        wait_stop_count(4);
        controller.write_read(addr, &samples2, &mut v.3)?;
        wait_stop_count(5);
        controller.write(addr, &samples2)?;
        wait_stop_count(6);
        controller.write(addr, &samples1)?;
        wait_stop_count(7);
        controller.write_read(addr, &samples1, &mut v.4)?;
        wait_stop_count(8);
        Ok::<(), Error>(())
    };
    case().expect("Successful test");

    // There are 14restarts in this sequence but because of latency in the target handling, it
    // may only detect 7.
    assert_restart_count!(restart_count);

    // assert writes
    let e: FIFOBuffer = itertools::chain!(
        Generator::seq().take(25),
        Generator::fib().take(14),
        Generator::fib().take(14),
        Generator::seq().take(25),
        Generator::seq().take(25),
    )
    .collect();
    assert_vec_eq!(e);

    // assert reads
    let g: FIFOBuffer = itertools::chain!(
        Generator::seq().take(64),
        Generator::fib().take(14),
        Generator::seq().take(14)
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
