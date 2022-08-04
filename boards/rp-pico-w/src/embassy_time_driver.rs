// copied from embassy-rp

use core::cell::Cell;
use core::cell::RefCell;
use core::ops::DerefMut;

use atomic_polyfill::{AtomicU8, Ordering};
use critical_section::CriticalSection;
use embassy_sync::blocking_mutex::raw::CriticalSectionRawMutex;
use embassy_sync::blocking_mutex::Mutex;
use embassy_time::driver::{AlarmHandle, Driver};

use rp2040_hal::pac;
use rp2040_hal::pac::interrupt;
use rp2040_hal::timer::Alarm;
use rp2040_hal::timer::Alarm0;
use rp2040_hal::timer::Timer;

use defmt::*;

struct AlarmState {
    timestamp: Cell<u64>,
    #[allow(clippy::type_complexity)]
    callback: Cell<Option<(fn(*mut ()), *mut ())>>,
}
unsafe impl Send for AlarmState {}

const ALARM_COUNT: usize = 4;
#[allow(clippy::declare_interior_mutable_const)]
const DUMMY_ALARM: AlarmState = AlarmState {
    timestamp: Cell::new(0),
    callback: Cell::new(None),
};

struct TimerDriver {
    timer: Mutex<CriticalSectionRawMutex, RefCell<Option<Timer>>>,
    alarm: Mutex<CriticalSectionRawMutex, RefCell<Option<Alarm0>>>,
    alarms: Mutex<CriticalSectionRawMutex, [AlarmState; ALARM_COUNT]>,
    next_alarm: AtomicU8,
}

embassy_time::time_driver_impl!(static DRIVER: TimerDriver = TimerDriver{
    timer: Mutex::const_new(CriticalSectionRawMutex::new(), RefCell::new(None)),
    alarm:  Mutex::const_new(CriticalSectionRawMutex::new(), RefCell::new(None)),
    alarms:  Mutex::const_new(CriticalSectionRawMutex::new(), [DUMMY_ALARM; ALARM_COUNT]),
    next_alarm: AtomicU8::new(0),
});

impl Driver for TimerDriver {
    fn now(&self) -> u64 {
        critical_section::with(|cs| {
            self.timer
                .borrow(cs)
                .borrow()
                .as_ref()
                .unwrap()
                .get_counter()
        })
    }

    unsafe fn allocate_alarm(&self) -> Option<AlarmHandle> {
        let id = self
            .next_alarm
            .fetch_update(Ordering::AcqRel, Ordering::Acquire, |x| {
                if x < ALARM_COUNT as u8 {
                    Some(x + 1)
                } else {
                    None
                }
            });

        match id {
            Ok(id) => Some(AlarmHandle::new(id)),
            Err(_) => None,
        }
    }

    fn set_alarm_callback(&self, alarm: AlarmHandle, callback: fn(*mut ()), ctx: *mut ()) {
        let n = alarm.id() as usize;
        critical_section::with(|cs| {
            let alarm = &self.alarms.borrow(cs)[n];
            alarm.callback.set(Some((callback, ctx)));
        })
    }

    fn set_alarm(&self, alarm: AlarmHandle, timestamp: u64) {
        let n = alarm.id() as usize;
        trace!("set alarm {} to {}", n, timestamp);
        critical_section::with(|cs| {
            let alarms = self.alarms.borrow(cs);
            let alarm = &alarms[n];
            //let timer = self.timer.borrow(cs).borrow_mut().as_mut();
            alarm.timestamp.set(timestamp);

            let min_timestamp = self.next_scheduled_alarm().1;

            // Arm it.
            // Note that we're not checking the high bits at all. This means the irq may fire early
            // if the alarm is more than 72 minutes (2^32 us) in the future. This is OK, since on irq fire
            // it is checked if the alarm time has passed.
            let now = self.now();

            if min_timestamp >= now {
                use fugit::MicrosDurationU32;
                self.alarm
                    .borrow(cs)
                    .borrow_mut()
                    .deref_mut()
                    .as_mut()
                    .unwrap()
                    .schedule(MicrosDurationU32::micros((min_timestamp - now) as u32))
                    .ok(); // TODO: ok to ignore?
                trace!("alarm armed for {}", min_timestamp - now);
            }
            let now = self.now();

            // If alarm timestamp has passed, trigger it instantly.
            // This disarms it.
            if min_timestamp <= now {
                self.trigger_alarm(n, cs);
            }
        })
    }
}

impl TimerDriver {
    /*
    fn arm(&self) {
        info!("arm!");
        critical_section::with(|cs| {
            //let alarms = self.alarms.borrow(cs);
            let next = self.next_scheduled_alarm();
            let min_timestamp = next.1;
            if min_timestamp != u64::MAX {
                //let timer = self.timer.borrow(cs).borrow_mut().as_mut();
                let now = self.now();
                if min_timestamp <= now {
                    self.trigger_alarm(next.0, cs)
                } else {
                    use embedded_time::{duration::*};
                    self.alarm.borrow(cs).borrow_mut().deref_mut().as_mut().unwrap().schedule(((min_timestamp - now) as u32).microseconds()).unwrap();
                }
            } else {
                // ignore... for now unsafe { pac::TIMER.armed().write(|w| w.set_armed(1 << n)) }
            }
        });
    }
    */

    fn check_alarm(&self) {
        trace!("checking alarm");
        let next = self.next_scheduled_alarm();
        if next.1 == u64::MAX {
            critical_section::with(|cs| {
                self.alarm
                    .borrow(cs)
                    .borrow_mut()
                    .deref_mut()
                    .as_mut()
                    .unwrap()
                    .clear_interrupt();
            });
            return;
        }
        let n = next.0;
        critical_section::with(|cs| {
            let alarm = &self.alarms.borrow(cs)[n];
            //let timer = self.timer.borrow(cs).borrow_mut().as_mut();
            let timestamp = alarm.timestamp.get();
            let now = self.now();
            if timestamp <= now {
                self.trigger_alarm(n, cs)
            } else {
                // Not elapsed, arm it again.
                // This can happen if it was set more than 2^32 us in the future.
                use fugit::MicrosDurationU32;
                self.alarm
                    .borrow(cs)
                    .borrow_mut()
                    .deref_mut()
                    .as_mut()
                    .unwrap()
                    .schedule(MicrosDurationU32::micros((timestamp - now) as u32))
                    .unwrap();
            }
            self.alarm
                .borrow(cs)
                .borrow_mut()
                .deref_mut()
                .as_mut()
                .unwrap()
                .clear_interrupt();
        });
    }

    fn trigger_alarm(&self, n: usize, cs: CriticalSection) {
        trace!("triggered alarm {}", n);
        // disarm
        // ignore... for now unsafe { pac::TIMER.armed().write(|w| w.set_armed(1 << n)) }

        let alarm = &self.alarms.borrow(cs)[n];
        alarm.timestamp.set(u64::MAX);

        // Call after clearing alarm, so the callback can set another alarm.
        if let Some((f, ctx)) = alarm.callback.get() {
            trace!("callback:");
            f(ctx);
        }
        trace!("callback executed");
    }

    fn next_scheduled_alarm(&self) -> (usize, u64) {
        critical_section::with(|cs| {
            self.alarms
                .borrow(cs)
                .iter()
                .map(|a| a.timestamp.get())
                .enumerate()
                .min_by_key(|p| p.1)
                .unwrap()
        })
    }
}

/// # Safety
/// must be called exactly once at bootup
pub unsafe fn init(mut timer: Timer) {
    // init alarms
    critical_section::with(|cs| {
        // make sure the alarm is not yet taken,
        // and leak it, so it can be used safely
        let mut alarm = timer.alarm_0().unwrap();
        alarm.enable_interrupt();
        //unsafe {
        pac::NVIC::unmask(pac::Interrupt::TIMER_IRQ_0);
        //}
        trace!("interrupt enabled!");
        DRIVER.alarm.borrow(cs).replace(Some(alarm));
        DRIVER.timer.borrow(cs).replace(Some(timer));
        let alarms = DRIVER.alarms.borrow(cs);
        for a in alarms {
            a.timestamp.set(u64::MAX);
        }
    });

    /*
    // enable all irqs
    pac::TIMER.inte().write(|w| {
        w.set_alarm(0, true);
        w.set_alarm(1, true);
        w.set_alarm(2, true);
        w.set_alarm(3, true);
    });

    interrupt::TIMER_IRQ_0::steal().enable();
    interrupt::TIMER_IRQ_1::steal().enable();
    interrupt::TIMER_IRQ_2::steal().enable();
    interrupt::TIMER_IRQ_3::steal().enable();
    */
}

#[interrupt]
unsafe fn TIMER_IRQ_0() {
    trace!("Interrupt!");
    DRIVER.check_alarm()
}

/*
#[interrupt]
unsafe fn TIMER_IRQ_1() {
    DRIVER.check_alarm(1)
}

#[interrupt]
unsafe fn TIMER_IRQ_2() {
    DRIVER.check_alarm(2)
}

#[interrupt]
unsafe fn TIMER_IRQ_3() {
    DRIVER.check_alarm(3)
}
*/
