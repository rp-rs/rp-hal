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
use rp2040_hal::timer::Instant;
use rp2040_hal::timer::ScheduleAlarmError;
use rp2040_hal::timer::Timer;

use defmt::*;

struct AlarmState {
    timestamp: Cell<Instant>,
    #[allow(clippy::type_complexity)]
    callback: Cell<Option<(fn(*mut ()), *mut ())>>,
}
unsafe impl Send for AlarmState {}

const ALARM_COUNT: usize = 4;
const NO_ALARM: Instant = Instant::from_ticks(u64::MAX);

#[allow(clippy::declare_interior_mutable_const)]
const DUMMY_ALARM: AlarmState = AlarmState {
    timestamp: Cell::new(NO_ALARM),
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
    alarm: Mutex::const_new(CriticalSectionRawMutex::new(), RefCell::new(None)),
    alarms: Mutex::const_new(CriticalSectionRawMutex::new(), [DUMMY_ALARM; ALARM_COUNT]),
    next_alarm: AtomicU8::new(0),
});

impl Driver for TimerDriver {
    fn now(&self) -> u64 {
        self.now_instant().ticks()
    }

    unsafe fn allocate_alarm(&self) -> Option<AlarmHandle> {
        let id = self
            .next_alarm
            .fetch_update(Ordering::AcqRel, Ordering::Acquire, |x| {
                if x < ALARM_COUNT as u8 {
                    Some(x + 1)
                } else {
                    warn!("No alarm left");
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
        critical_section::with(|cs| {
            let alarms = self.alarms.borrow(cs);
            let alarm = &alarms[n];
            if alarm.timestamp.get().ticks() == timestamp {
                // redundant set_alarm - ignore
                return;
            }
            alarm.timestamp.set(Instant::from_ticks(timestamp));

            self.arm(cs);
            trace!("set alarm {} to {}", n, timestamp);
        })
    }
}

impl TimerDriver {
    fn now_instant(&self) -> Instant {
        critical_section::with(|cs| {
            self.timer
                .borrow(cs)
                .borrow()
                .as_ref()
                .unwrap()
                .get_counter()
        })
    }

    fn arm(&self, cs: CriticalSection) {
        let (n, min_timestamp) = self.next_scheduled_alarm();
        if min_timestamp == NO_ALARM {
            // no alarm set
            trace!("Trying to arm non-existing alarm");
            return;
        }
        // Arm it.
        trace!("arm!");
        // Note that we're not checking the high bits at all. This means the irq may fire early
        // if the alarm is more than 72 minutes (2^32 us) in the future. This is OK, since on irq fire
        // it is checked if the alarm time has passed.
        let now = self.now_instant();
        use fugit::MicrosDurationU32;
        match self
            .alarm
            .borrow(cs)
            .borrow_mut()
            .deref_mut()
            .as_mut()
            .unwrap()
            .schedule_at(min_timestamp)
        {
            Result::Err(ScheduleAlarmError::AlarmTooSoon) => {
                // If alarm timestamp has passed, trigger it instantly.
                // This disarms it.
                trace!(
                    "timestamp has passed, trigger now! timestamp {} <= now {}",
                    min_timestamp.ticks(),
                    now.ticks()
                );
                self.trigger_alarm(n, cs);
            }
            Result::Err(ScheduleAlarmError::AlarmTooLate) => {
                // Duration >72 minutes. Reschedule 1 hour later.
                trace!("reschedule in 1h");
                self.alarm
                    .borrow(cs)
                    .borrow_mut()
                    .deref_mut()
                    .as_mut()
                    .unwrap()
                    .schedule(MicrosDurationU32::hours(1))
                    .unwrap();
            }
            Ok(_) => {
                trace!("min: {}, now: {}", min_timestamp.ticks(), now.ticks());
                trace!("alarm armed for {}", (min_timestamp - now).ticks());
            }
            Err(_) => core::panic!("Unknown error scheduling an alarm"),
        }
    }

    fn check_alarm(&self) {
        trace!("checking alarm");
        critical_section::with(|cs| {
            self.alarm
                .borrow(cs)
                .borrow_mut()
                .deref_mut()
                .as_mut()
                .unwrap()
                .clear_interrupt();
            let next = self.next_scheduled_alarm();
            if next.1 == NO_ALARM {
                warn!("No next alarm. Spurious interrupt?");
                return;
            }
            let (n, timestamp) = next;
            let now = self.now_instant();
            // alarm peripheral has only 32 bits, so might have triggered early
            if timestamp.const_cmp(now) != core::cmp::Ordering::Less {
                self.trigger_alarm(n, cs)
            }
            // next alarm could have changed - rearm
            self.arm(cs);
        });
    }

    fn trigger_alarm(&self, n: usize, cs: CriticalSection) {
        trace!("triggered alarm {}", n);
        // disarm
        // ignore... for now unsafe { pac::TIMER.armed().write(|w| w.set_armed(1 << n)) }

        let alarm = &self.alarms.borrow(cs)[n];
        alarm.timestamp.set(NO_ALARM);

        // Call after clearing alarm, so the callback can set another alarm.
        if let Some((f, ctx)) = alarm.callback.get() {
            trace!("callback:");
            f(ctx);
        }
        trace!("callback executed");
    }

    fn next_scheduled_alarm(&self) -> (usize, Instant) {
        critical_section::with(|cs| {
            self.alarms
                .borrow(cs)
                .iter()
                .map(|a| a.timestamp.get())
                .enumerate()
                .min_by_key(|p| p.1.ticks())
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
            a.timestamp.set(NO_ALARM);
        }
    });
}

#[interrupt]
unsafe fn TIMER_IRQ_0() {
    trace!("Interrupt!");
    DRIVER.check_alarm()
}
