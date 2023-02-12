//! Timer Peripheral
//!
//! The Timer peripheral on RP2040 consists of a 64-bit counter and 4 alarms.  
//! The Counter is incremented once per microsecond. It obtains its clock source from the watchdog peripheral, you must enable the watchdog before using this peripheral.  
//! Since it would take thousands of years for this counter to overflow you do not need to write logic for dealing with this if using get_counter.  
//!
//! Each of the 4 alarms can match on the lower 32 bits of Counter and trigger an interrupt.
//!
//! See [Chapter 4 Section 6](https://datasheets.raspberrypi.org/rp2040/rp2040_datasheet.pdf) of the datasheet for more details.

use fugit::{MicrosDurationU32, MicrosDurationU64, TimerInstantU64};

use crate::atomic_register_access::{write_bitmask_clear, write_bitmask_set};
use crate::pac::{RESETS, TIMER};
use crate::resets::SubsystemReset;
use crate::typelevel::Sealed;
use core::marker::PhantomData;
use core::sync::atomic::{AtomicU8, Ordering};

/// Instant type used by the Timer & Alarm methods.
pub type Instant = TimerInstantU64<1_000_000>;

static ALARMS: AtomicU8 = AtomicU8::new(0x0F);
fn take_alarm(mask: u8) -> bool {
    critical_section::with(|_| {
        let alarms = ALARMS.load(Ordering::Relaxed);
        ALARMS.store(alarms & !mask, Ordering::Relaxed);
        (alarms & mask) != 0
    })
}
fn release_alarm(mask: u8) {
    critical_section::with(|_| {
        let alarms = ALARMS.load(Ordering::Relaxed);
        ALARMS.store(alarms | mask, Ordering::Relaxed);
    });
}

fn get_counter(timer: &crate::pac::timer::RegisterBlock) -> Instant {
    let mut hi0 = timer.timerawh.read().bits();
    let timestamp = loop {
        let low = timer.timerawl.read().bits();
        let hi1 = timer.timerawh.read().bits();
        if hi0 == hi1 {
            break (u64::from(hi0) << 32) | u64::from(low);
        }
        hi0 = hi1;
    };
    TimerInstantU64::from_ticks(timestamp)
}
/// Timer peripheral
pub struct Timer {
    timer: TIMER,
}

impl Timer {
    /// Create a new [`Timer`]
    pub fn new(timer: TIMER, resets: &mut RESETS) -> Self {
        timer.reset_bring_down(resets);
        timer.reset_bring_up(resets);
        Self { timer }
    }

    /// Get the current counter value.
    pub fn get_counter(&self) -> Instant {
        get_counter(&self.timer)
    }

    /// Get the value of the least significant word of the counter.
    pub fn get_counter_low(&self) -> u32 {
        self.timer.timerawl.read().bits()
    }

    /// Initialized a Count Down instance without starting it.
    pub fn count_down(&self) -> CountDown<'_> {
        CountDown {
            timer: self,
            period: MicrosDurationU64::nanos(0),
            next_end: None,
        }
    }
    /// Retrieve a reference to alarm 0. Will only return a value the first time this is called
    pub fn alarm_0(&mut self) -> Option<Alarm0> {
        take_alarm(1 << 0).then_some(Alarm0(PhantomData))
    }

    /// Retrieve a reference to alarm 1. Will only return a value the first time this is called
    pub fn alarm_1(&mut self) -> Option<Alarm1> {
        take_alarm(1 << 1).then_some(Alarm1(PhantomData))
    }

    /// Retrieve a reference to alarm 2. Will only return a value the first time this is called
    pub fn alarm_2(&mut self) -> Option<Alarm2> {
        take_alarm(1 << 2).then_some(Alarm2(PhantomData))
    }

    /// Retrieve a reference to alarm 3. Will only return a value the first time this is called
    pub fn alarm_3(&mut self) -> Option<Alarm3> {
        take_alarm(1 << 3).then_some(Alarm3(PhantomData))
    }
}

// safety: all write operations are synchronised and all reads are atomic
unsafe impl Sync for Timer {}

/// Implementation of the embedded_hal::Timer traits using rp2040_hal::timer counter
///
/// ## Usage
/// ```no_run
/// use embedded_hal::timer::{CountDown, Cancel};
/// use fugit::ExtU32;
/// use rp2040_hal;
/// let mut pac = rp2040_hal::pac::Peripherals::take().unwrap();
/// // Configure the Timer peripheral in count-down mode
/// let timer = rp2040_hal::Timer::new(pac.TIMER, &mut pac.RESETS);
/// let mut count_down = timer.count_down();
/// // Create a count_down timer for 500 milliseconds
/// count_down.start(500.millis());
/// // Block until timer has elapsed
/// let _ = nb::block!(count_down.wait());
/// // Restart the count_down timer with a period of 100 milliseconds
/// count_down.start(100.millis());
/// // Cancel it immediately
/// count_down.cancel();
/// ```
pub struct CountDown<'timer> {
    timer: &'timer Timer,
    period: MicrosDurationU64,
    next_end: Option<u64>,
}

impl embedded_hal::timer::CountDown for CountDown<'_> {
    type Time = MicrosDurationU64;

    fn start<T>(&mut self, count: T)
    where
        T: Into<Self::Time>,
    {
        self.period = count.into();
        self.next_end = Some(
            self.timer
                .get_counter()
                .ticks()
                .wrapping_add(self.period.to_micros()),
        );
    }

    fn wait(&mut self) -> nb::Result<(), void::Void> {
        if let Some(end) = self.next_end {
            let ts = self.timer.get_counter().ticks();
            if ts >= end {
                self.next_end = Some(end.wrapping_add(self.period.to_micros()));
                Ok(())
            } else {
                Err(nb::Error::WouldBlock)
            }
        } else {
            panic!("CountDown is not running!");
        }
    }
}

impl embedded_hal::timer::Periodic for CountDown<'_> {}

impl embedded_hal::timer::Cancel for CountDown<'_> {
    type Error = &'static str;

    fn cancel(&mut self) -> Result<(), Self::Error> {
        if self.next_end.is_none() {
            Err("CountDown is not running.")
        } else {
            self.next_end = None;
            Ok(())
        }
    }
}

/// Alarm abstraction.
pub trait Alarm: Sealed {
    /// Clear the interrupt flag.
    ///
    /// The interrupt is unable to trigger a 2nd time until this interrupt is cleared.
    fn clear_interrupt(&mut self);

    /// Enable this alarm to trigger an interrupt.
    ///
    /// After this interrupt is triggered, make sure to clear the interrupt with [clear_interrupt].
    ///
    /// [clear_interrupt]: #method.clear_interrupt
    fn enable_interrupt(&mut self);

    /// Disable this alarm, preventing it from triggering an interrupt.
    fn disable_interrupt(&mut self);

    /// Schedule the alarm to be finished after `countdown`. If [enable_interrupt] is called,
    /// this will trigger interrupt whenever this time elapses.
    ///
    /// [enable_interrupt]: #method.enable_interrupt
    fn schedule(&mut self, countdown: MicrosDurationU32) -> Result<(), ScheduleAlarmError>;

    /// Schedule the alarm to be finished at the given timestamp. If [enable_interrupt] is
    /// called, this will trigger interrupt whenever this timestamp is reached.
    ///
    /// The RP2040 is unable to schedule an event taking place in more than
    /// `u32::max_value()` microseconds.
    ///
    /// [enable_interrupt]: #method.enable_interrupt
    fn schedule_at(&mut self, timestamp: Instant) -> Result<(), ScheduleAlarmError>;

    /// Return true if this alarm is finished. The returned value is undefined if the alarm
    /// has not been scheduled yet.
    fn finished(&self) -> bool;

    /// Cancel an activated alarm.
    fn cancel(&mut self) -> Result<(), ScheduleAlarmError>;
}

macro_rules! impl_alarm {
    ($name:ident  { rb: $timer_alarm:ident, int: $int_alarm:ident, int_name: $int_name:tt, armed_bit_mask: $armed_bit_mask: expr }) => {
        /// An alarm that can be used to schedule events in the future. Alarms can also be configured to trigger interrupts.
        pub struct $name(PhantomData<()>);
        impl $name {
            fn schedule_internal(
                &mut self,
                timer: &crate::pac::timer::RegisterBlock,
                timestamp: Instant,
            ) -> Result<(), ScheduleAlarmError> {
                let timestamp_low = (timestamp.ticks() & 0xFFFF_FFFF) as u32;

                // This lock is for time-criticality
                cortex_m::interrupt::free(|_| {
                    let alarm = &timer.$timer_alarm;

                    // safety: This is the only code in the codebase that accesses memory address $timer_alarm
                    alarm.write(|w| unsafe { w.bits(timestamp_low) });

                    // If it is not set, it has already triggered.
                    let now = get_counter(timer);
                    if now > timestamp && (timer.armed.read().bits() & $armed_bit_mask) != 0 {
                        // timestamp was set to a value in the past

                        // safety: TIMER.armed is a write-clear register, and there can only be
                        // 1 instance of AlarmN so we can safely atomically clear this bit.
                        unsafe {
                            timer.armed.write_with_zero(|w| w.bits($armed_bit_mask));
                            crate::atomic_register_access::write_bitmask_set(
                                timer.intf.as_ptr(),
                                $armed_bit_mask,
                            );
                        }
                    }
                    Ok(())
                })
            }
        }

        impl Alarm for $name {
            /// Clear the interrupt flag. This should be called after interrupt `
            #[doc = $int_name]
            /// ` is called.
            ///
            /// The interrupt is unable to trigger a 2nd time until this interrupt is cleared.
            fn clear_interrupt(&mut self) {
                // safety: TIMER.intr is a write-clear register, so we can atomically clear our interrupt
                // by writing its value to this field
                // Only one instance of this alarm index can exist, and only this alarm interacts with this bit
                // of the TIMER.inte register
                unsafe {
                    let timer = &(*pac::TIMER::ptr());
                    crate::atomic_register_access::write_bitmask_clear(
                        timer.intf.as_ptr(),
                        $armed_bit_mask,
                    );
                    timer.intr.write_with_zero(|w| w.$int_alarm().set_bit());
                }
            }

            /// Enable this alarm to trigger an interrupt. This alarm will trigger `
            #[doc = $int_name]
            /// `.
            ///
            /// After this interrupt is triggered, make sure to clear the interrupt with [clear_interrupt].
            ///
            /// [clear_interrupt]: #method.clear_interrupt
            fn enable_interrupt(&mut self) {
                // safety: using the atomic set alias means we can atomically set our interrupt enable bit.
                // Only one instance of this alarm can exist, and only this alarm interacts with this bit
                // of the TIMER.inte register
                unsafe {
                    let timer = &(*pac::TIMER::ptr());
                    let reg = (&timer.inte).as_ptr();
                    write_bitmask_set(reg, $armed_bit_mask);
                }
            }

            /// Disable this alarm, preventing it from triggering an interrupt.
            fn disable_interrupt(&mut self) {
                // safety: using the atomic set alias means we can atomically clear our interrupt enable bit.
                // Only one instance of this alarm can exist, and only this alarm interacts with this bit
                // of the TIMER.inte register
                unsafe {
                    let timer = &(*pac::TIMER::ptr());
                    let reg = (&timer.inte).as_ptr();
                    write_bitmask_clear(reg, $armed_bit_mask);
                }
            }

            /// Schedule the alarm to be finished after `countdown`. If [enable_interrupt] is called,
            /// this will trigger interrupt `
            #[doc = $int_name]
            /// ` whenever this time elapses.
            ///
            /// [enable_interrupt]: #method.enable_interrupt
            fn schedule(&mut self, countdown: MicrosDurationU32) -> Result<(), ScheduleAlarmError> {
                // safety: Only read operations are made on the timer and they should not have any UB
                let timer = unsafe { &*TIMER::ptr() };
                let timestamp = get_counter(timer) + countdown;

                self.schedule_internal(timer, timestamp)
            }

            /// Schedule the alarm to be finished at the given timestamp. If [enable_interrupt] is
            /// called, this will trigger interrupt `
            #[doc = $int_name]
            /// ` whenever this timestamp is reached.
            ///
            /// The RP2040 is unable to schedule an event taking place in more than
            /// `u32::max_value()` microseconds.
            ///
            /// [enable_interrupt]: #method.enable_interrupt
            fn schedule_at(&mut self, timestamp: Instant) -> Result<(), ScheduleAlarmError> {
                // safety: Only read operations are made on the timer and they should not have any UB
                let timer = unsafe { &*TIMER::ptr() };
                let now = get_counter(timer);
                let duration = timestamp.ticks().saturating_sub(now.ticks());
                if duration > u32::max_value().into() {
                    return Err(ScheduleAlarmError::AlarmTooLate);
                }

                self.schedule_internal(timer, timestamp)
            }

            /// Return true if this alarm is finished. The returned value is undefined if the alarm
            /// has not been scheduled yet.
            fn finished(&self) -> bool {
                // safety: This is a read action and should not have any UB
                let bits: u32 = unsafe { &*TIMER::ptr() }.armed.read().bits();
                (bits & $armed_bit_mask) == 0
            }

            /// Cancel an activated Alarm. No negative effects if it's already disabled.
            /// Unlike `timer::cancel` trait, this only cancels the alarm and keeps the timer running
            /// if it's already active.
            fn cancel(&mut self) -> Result<(), ScheduleAlarmError> {
                unsafe {
                    let timer = &*TIMER::ptr();
                    timer.armed.write_with_zero(|w| w.bits($armed_bit_mask));
                    crate::atomic_register_access::write_bitmask_clear(
                        timer.intf.as_ptr(),
                        $armed_bit_mask,
                    );
                }

                Ok(())
            }
        }

        impl Sealed for $name {}

        impl Drop for $name {
            fn drop(&mut self) {
                self.disable_interrupt();
                release_alarm($armed_bit_mask)
            }
        }
    };
}

/// Errors that can be returned from any of the `AlarmX::schedule` methods.
#[derive(Copy, Clone, Debug, PartialEq, Eq, Hash)]
pub enum ScheduleAlarmError {
    /// Alarm time is too high. Should not be more than `u32::max_value()` in the future.
    AlarmTooLate,
}

impl_alarm!(Alarm0 {
    rb: alarm0,
    int: alarm_0,
    int_name: "TIMER_IRQ_0",
    armed_bit_mask: 0b0001
});

impl_alarm!(Alarm1 {
    rb: alarm1,
    int: alarm_1,
    int_name: "TIMER_IRQ_1",
    armed_bit_mask: 0b0010
});

impl_alarm!(Alarm2 {
    rb: alarm2,
    int: alarm_2,
    int_name: "TIMER_IRQ_2",
    armed_bit_mask: 0b0100
});

impl_alarm!(Alarm3 {
    rb: alarm3,
    int: alarm_3,
    int_name: "TIMER_IRQ_3",
    armed_bit_mask: 0b1000
});

/// Support for RTIC monotonic trait.
#[cfg(feature = "rtic-monotonic")]
pub mod monotonic {
    use super::{Alarm, Instant, Timer};
    use fugit::ExtU32;

    /// RTIC Monotonic Implementation
    pub struct Monotonic<A>(pub Timer, A);
    impl<A: Alarm> Monotonic<A> {
        /// Creates a new monotonic.
        pub const fn new(timer: Timer, alarm: A) -> Self {
            Self(timer, alarm)
        }
    }
    impl<A: Alarm> rtic_monotonic::Monotonic for Monotonic<A> {
        type Instant = Instant;
        type Duration = fugit::MicrosDurationU64;

        const DISABLE_INTERRUPT_ON_EMPTY_QUEUE: bool = false;

        fn now(&mut self) -> Instant {
            self.0.get_counter()
        }

        fn set_compare(&mut self, instant: Instant) {
            // The alarm can only trigger up to 2^32 - 1 ticks in the future.
            // So, if `instant` is more than 2^32 - 2 in the future, we use `max_instant` instead.
            let max_instant = self.0.get_counter() + 0xFFFF_FFFE.micros();
            let wake_at = core::cmp::min(instant, max_instant);

            // Cannot fail
            let _ = self.1.schedule_at(wake_at);
            self.1.enable_interrupt();
        }

        fn clear_compare_flag(&mut self) {
            self.1.clear_interrupt();
        }

        fn zero() -> Self::Instant {
            Instant::from_ticks(0)
        }

        unsafe fn reset(&mut self) {}
    }
}
