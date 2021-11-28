//! Real time clock functionality
//!
//! A [`RealTimeClock`] can be configured with an initial [`DateTime`]. Afterwards the clock will track time automatically. The current `DateTime` can be retrieved by [`RealTimeClock::now()`].
//!
//! With the **chrono** feature enabled, the following types will be alias for chrono types:
//! - `DateTime`: `chrono::NaiveDateTime`
//! - `DayOfWeek`: `chrono::Weekday`
//!
//! # Notes
//!
//! There are some things to take into account. As per the datasheet:
//!
//! - **Day of week**: The RTC will not compute the correct day of the week; it will only increment the existing value.
//!   - With the `chrono` feature, the day of week is calculated by chrono and should be correct. The value from the rp2040 itself is not used.
//! - **Leap year**: If the current year is evenly divisible by 4, a leap year is detected, then Feb 28th is followed by Feb 29th instead  of  March  1st.
//!   - There are cases where this is incorrect, e.g. century years have no leap day, but the chip will still add a Feb 29th.
//!   - To disable leap year checking and never have a Feb 29th, call `RealTimeClock::set_leap_year_check(false)`.
//!
//! Other limitations:
//!
//! - **Leap seconds**: The rp2040 will not take leap seconds into account
//!   - With the `chrono` feature, leap seconds will be silently handled by `chrono`. This means there might be a slight difference between the value of [`RealTimeClock::now()`] and adding 2 times together in code.

use crate::clocks::Clock;
use crate::clocks::RtcClock;
use embedded_time::fixed_point::FixedPoint;
use rp2040_pac::{RESETS, RTC};

mod filter;

pub use self::filter::DateTimeFilter;

#[cfg_attr(feature = "chrono", path = "datetime_chrono.rs")]
#[cfg_attr(not(feature = "chrono"), path = "datetime_no_deps.rs")]
mod datetime;

pub use self::datetime::{DateTime, DayOfWeek, Error as DateTimeError};

/// A reference to the real time clock of the system
pub struct RealTimeClock {
    rtc: RTC,
}

impl RealTimeClock {
    /// Create a new instance of the real time clock, with the given date as an initial value.
    ///
    /// Note that the [`ClocksManager`] should be enabled first. See the [`clocks`] module for more information.
    ///
    /// # Errors
    ///
    /// Will return `RtcError::InvalidDateTime` if the datetime is not a valid range.
    ///
    /// [`ClocksManager`]: ../clocks/struct.ClocksManager.html
    /// [`clocks`]: ../clocks/index.html
    pub fn new(
        rtc: RTC,
        clock: RtcClock,
        resets: &mut RESETS,
        initial_date: DateTime,
    ) -> Result<Self, RtcError> {
        // Toggle the RTC reset
        resets.reset.modify(|_, w| w.rtc().set_bit());
        resets.reset.modify(|_, w| w.rtc().clear_bit());
        while resets.reset_done.read().rtc().bit_is_clear() {
            core::hint::spin_loop();
        }

        // Set the RTC divider
        let freq = clock.freq().integer() - 1;
        rtc.clkdiv_m1.write(|w| unsafe { w.bits(freq) });

        let mut result = Self { rtc };
        result.set_leap_year_check(true); // should be on by default, make sure this is the case.
        result.set_datetime(initial_date)?;
        Ok(result)
    }

    /// Enable or disable the leap year check. The rp2040 chip will always add a Feb 29th on every year that is divisable by 4, but this may be incorrect (e.g. on century years). This function allows you to disable this check.
    ///
    /// Leap year checking is enabled by default.
    pub fn set_leap_year_check(&mut self, leap_year_check_enabled: bool) {
        self.rtc
            .ctrl
            .modify(|_, w| w.force_notleapyear().bit(!leap_year_check_enabled));
    }

    /// Checks to see if this RealTimeClock is running
    pub fn is_running(&self) -> bool {
        self.rtc.ctrl.read().rtc_active().bit_is_set()
    }

    /// Set the datetime to a new value.
    ///
    /// # Errors
    ///
    /// Will return `RtcError::InvalidDateTime` if the datetime is not a valid range.
    pub fn set_datetime(&mut self, t: DateTime) -> Result<(), RtcError> {
        self::datetime::validate_datetime(&t).map_err(RtcError::InvalidDateTime)?;

        // disable RTC while we configure it
        self.rtc.ctrl.modify(|_, w| w.rtc_enable().clear_bit());
        while self.rtc.ctrl.read().rtc_active().bit_is_set() {
            core::hint::spin_loop();
        }

        self.rtc.setup_0.write(|w| {
            self::datetime::write_setup_0(&t, w);
            w
        });
        self.rtc.setup_1.write(|w| {
            self::datetime::write_setup_1(&t, w);
            w
        });

        // Load the new datetime and re-enable RTC
        self.rtc.ctrl.write(|w| w.load().set_bit());
        self.rtc.ctrl.write(|w| w.rtc_enable().set_bit());
        while self.rtc.ctrl.read().rtc_active().bit_is_clear() {
            core::hint::spin_loop();
        }

        Ok(())
    }

    /// Return the current datetime.
    ///
    /// # Errors
    ///
    /// Will return an `RtcError::InvalidDateTime` if the stored value in the system is not a valid [`DayOfWeek`].
    pub fn now(&self) -> Result<DateTime, RtcError> {
        if !self.is_running() {
            return Err(RtcError::NotRunning);
        }

        let rtc_0 = self.rtc.rtc_0.read();
        let rtc_1 = self.rtc.rtc_1.read();

        self::datetime::datetime_from_registers(rtc_0, rtc_1).map_err(RtcError::InvalidDateTime)
    }

    /// Disable the alarm that was scheduled with [`schedule_alarm`].
    ///
    /// [`schedule_alarm`]: #method.schedule_alarm
    pub fn disable_alarm(&mut self) {
        self.rtc
            .irq_setup_0
            .modify(|_, s| s.match_ena().clear_bit());

        while self.rtc.irq_setup_0.read().match_active().bit() {
            core::hint::spin_loop();
        }
    }

    /// Schedule an alarm. The `filter` determines at which point in time this alarm is set.
    ///
    /// Keep in mind that the filter only triggers on the specified time. If you want to schedule this alarm every minute, you have to call:
    /// ```no_run
    /// # #[cfg(feature = "chrono")]
    /// # fn main() { }
    /// # #[cfg(not(feature = "chrono"))]
    /// # fn main() {
    /// # use rp2040_hal::rtc::{RealTimeClock, DateTimeFilter};
    /// # let mut real_time_clock: RealTimeClock = unsafe { core::mem::zeroed() };
    /// let now = real_time_clock.now().unwrap();
    /// real_time_clock.schedule_alarm(
    ///     DateTimeFilter::default()
    ///         .minute(if now.minute == 59 { 0 } else { now.minute + 1 })
    /// );
    /// # }
    /// ```
    pub fn schedule_alarm(&mut self, filter: DateTimeFilter) {
        self.disable_alarm();

        self.rtc.irq_setup_0.write(|w| {
            filter.write_setup_0(w);
            w
        });
        self.rtc.irq_setup_1.write(|w| {
            filter.write_setup_1(w);
            w
        });

        // Set the enable bit and check if it is set
        self.rtc.irq_setup_0.modify(|_, w| w.match_ena().set_bit());
        while self.rtc.irq_setup_0.read().match_active().bit_is_clear() {
            core::hint::spin_loop();
        }
    }

    /// Clear the interrupt. This should be called every time the `RTC_IRQ` interrupt is triggered,
    /// or the next [`schedule_alarm`] will never fire.
    ///
    /// [`schedule_alarm`]: #method.schedule_alarm
    pub fn clear_interrupt(&mut self) {
        self.disable_alarm();
    }
}

/// Errors that can occur on methods on [RtcClock]
#[derive(Clone, Debug, PartialEq, Eq)]
pub enum RtcError {
    /// An invalid DateTime was given or stored on the hardware.
    InvalidDateTime(DateTimeError),

    /// The RTC clock is not running
    NotRunning,
}
