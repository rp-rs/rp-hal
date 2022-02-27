//! Timer Peripheral
// See [Chapter 4 Section 6](https://datasheets.raspberrypi.org/rp2040/rp2040_datasheet.pdf) for more details

use embedded_time::duration::Microseconds;

use crate::atomic_register_access::{write_bitmask_clear, write_bitmask_set};
use crate::pac::{RESETS, TIMER};
use crate::resets::SubsystemReset;
use core::marker::PhantomData;

/// Timer peripheral
pub struct Timer {
    timer: TIMER,
    alarms: [bool; 4],
}

impl Timer {
    /// Create a new [`Timer`]
    pub fn new(timer: TIMER, resets: &mut RESETS) -> Self {
        timer.reset_bring_up(resets);
        Self {
            timer,
            alarms: [true; 4],
        }
    }

    /// Get the current counter value.
    pub fn get_counter(&self) -> u64 {
        let mut hi0 = self.timer.timerawh.read().bits();
        loop {
            let low = self.timer.timerawl.read().bits();
            let hi1 = self.timer.timerawh.read().bits();
            if hi0 == hi1 {
                break (u64::from(hi0) << 32) | u64::from(low);
            }
            hi0 = hi1;
        }
    }

    /// Get the value of the least significant word of the counter.
    pub fn get_counter_low(&self) -> u32 {
        self.timer.timerawl.read().bits()
    }

    /// Initialized a Count Down instance without starting it.
    pub fn count_down(&self) -> CountDown<'_> {
        CountDown {
            timer: self,
            period: Microseconds::new(0),
            next_end: None,
        }
    }

    /// Retrieve a reference to alarm 0. Will only return a value the first time this is called
    pub fn alarm_0(&mut self) -> Option<Alarm0> {
        cortex_m::interrupt::free(|_| {
            if self.alarms[0] {
                self.alarms[0] = false;
                Some(Alarm0(PhantomData))
            } else {
                None
            }
        })
    }

    /// Retrieve a reference to alarm 1. Will only return a value the first time this is called
    pub fn alarm_1(&mut self) -> Option<Alarm1> {
        cortex_m::interrupt::free(|_| {
            if self.alarms[1] {
                self.alarms[1] = false;
                Some(Alarm1(PhantomData))
            } else {
                None
            }
        })
    }

    /// Retrieve a reference to alarm 2. Will only return a value the first time this is called
    pub fn alarm_2(&mut self) -> Option<Alarm2> {
        cortex_m::interrupt::free(|_| {
            if self.alarms[2] {
                self.alarms[2] = false;
                Some(Alarm2(PhantomData))
            } else {
                None
            }
        })
    }

    /// Retrieve a reference to alarm 3. Will only return a value the first time this is called
    pub fn alarm_3(&mut self) -> Option<Alarm3> {
        cortex_m::interrupt::free(|_| {
            if self.alarms[3] {
                self.alarms[3] = false;
                Some(Alarm3(PhantomData))
            } else {
                None
            }
        })
    }
}

/// Delay implementation
pub struct CountDown<'timer> {
    timer: &'timer Timer,
    period: embedded_time::duration::Microseconds<u64>,
    next_end: Option<u64>,
}

impl embedded_hal::timer::CountDown for CountDown<'_> {
    type Time = embedded_time::duration::Microseconds<u64>;

    fn start<T>(&mut self, count: T)
    where
        T: Into<Self::Time>,
    {
        self.period = count.into();
        self.next_end = Some(self.timer.get_counter().wrapping_add(self.period.0));
    }

    fn wait(&mut self) -> nb::Result<(), void::Void> {
        if let Some(end) = self.next_end {
            let ts = self.timer.get_counter();
            if ts >= end {
                self.next_end = Some(end.wrapping_add(self.period.0));
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

macro_rules! impl_alarm {
    ($name:ident  { rb: $timer_alarm:ident, int: $int_alarm:ident, int_name: $int_name:tt, armed_bit_mask: $armed_bit_mask: expr }) => {
        /// An alarm that can be used to schedule events in the future. Alarms can also be configured to trigger interrupts.
        pub struct $name(PhantomData<()>);

        impl $name {
            /// Clear the interrupt flag. This should be called after interrupt `
            #[doc = $int_name]
            /// ` is called.
            ///
            /// The interrupt is unable to trigger a 2nd time until this interrupt is cleared.
            pub fn clear_interrupt(&mut self) {
                // safety: TIMER.intr is a write-clear register, so we can atomically clear our interrupt
                // by writing its value to this field
                // Only one instance of this alarm index can exist, and only this alarm interacts with this bit
                // of the TIMER.inte register
                unsafe {
                    let timer = &(*pac::TIMER::ptr());
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
            pub fn enable_interrupt(&mut self) {
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
            pub fn disable_interrupt(&mut self) {
                // safety: using the atomic set alias means we can atomically clear our interrupt enable bit.
                // Only one instance of this alarm can exist, and only this alarm interacts with this bit
                // of the TIMER.inte register
                unsafe {
                    let timer = &(*pac::TIMER::ptr());
                    let reg = (&timer.inte).as_ptr();
                    write_bitmask_clear(reg, $armed_bit_mask);
                }
            }

            /// Schedule the alarm to be finished after `countdown`. If [enable_interrupt] is called, this will trigger interrupt `
            #[doc = $int_name]
            /// ` whenever this time elapses.
            ///
            /// The RP2040 has been observed to take a little while to schedule an alarm. For this reason, the minimum time that this function accepts is `10.microseconds()`
            ///
            /// [enable_interrupt]: #method.enable_interrupt
            pub fn schedule<TIME: Into<Microseconds>>(
                &mut self,
                countdown: TIME,
            ) -> Result<(), ScheduleAlarmError> {
                let duration = countdown.into().0;

                const MIN_MICROSECONDS: u32 = 10;
                if duration < MIN_MICROSECONDS {
                    return Err(ScheduleAlarmError::AlarmTooSoon);
                } else {
                    cortex_m::interrupt::free(|_| {
                        // safety: This is a read action and should not have any UB
                        let target_time = unsafe { &*TIMER::ptr() }
                            .timelr
                            .read()
                            .bits()
                            .wrapping_add(duration);

                        // safety: This is the only code in the codebase that accesses memory address $timer_alarm
                        unsafe { &*TIMER::ptr() }
                            .$timer_alarm
                            .write(|w| unsafe { w.bits(target_time) });
                    });
                    Ok(())
                }
            }

            /// Return true if this alarm is finished.
            pub fn finished(&self) -> bool {
                // safety: This is a read action and should not have any UB
                let bits: u32 = unsafe { &*TIMER::ptr() }.armed.read().bits();
                (bits & $armed_bit_mask) == 0
            }
        }
    };
}

/// Errors that can be returned from any of the `AlarmX::schedule` methods.
#[non_exhaustive]
#[derive(Copy, Clone, Debug, PartialEq, Eq, Hash)]
pub enum ScheduleAlarmError {
    /// Alarm time is too low. Should be at least 10 microseconds.
    AlarmTooSoon,
}

impl_alarm!(Alarm0 {
    rb: alarm0,
    int: alarm_0,
    int_name: "IRQ_TIMER_0",
    armed_bit_mask: 0b0001
});

impl_alarm!(Alarm1 {
    rb: alarm1,
    int: alarm_1,
    int_name: "IRQ_TIMER_1",
    armed_bit_mask: 0b0010
});

impl_alarm!(Alarm2 {
    rb: alarm2,
    int: alarm_2,
    int_name: "IRQ_TIMER_2",
    armed_bit_mask: 0b0100
});

impl_alarm!(Alarm3 {
    rb: alarm3,
    int: alarm_3,
    int_name: "IRQ_TIMER_3",
    armed_bit_mask: 0b1000
});
