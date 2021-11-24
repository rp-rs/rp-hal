//! Real Time Clock (RTC)
// See [Chapter 4 Section 8](https://datasheets.raspberrypi.org/rp2040/rp2040_datasheet.pdf) for more details

pub use crate::clocks::RtcClock;

use crate::clocks::Clock;
use embedded_time::fixed_point::FixedPoint;
use rp2040_pac::{RESETS, RTC};

/// Structure containing date and time information
pub struct DateTime {
    /// 0..4095
    pub year: u16,
    /// 1..12, 1 is January
    pub month: u8,
    /// 1..28,29,30,31 depending on month
    pub day: u8,
    ///
    pub day_of_week: DayOfWeek,
    /// 0..23
    pub hour: u8,
    /// 0..59
    pub minute: u8,
    /// 0..59
    pub second: u8,
}

impl DateTime {
    /// Validate the datetime, making sure it's a valid value.
    ///
    /// Note that the day of month is not checked currently. This means that you can set e.g. February 31th, and this function will still pass correctly.
    pub fn validate(&self) -> Result<(), DateTimeError> {
        if self.year > 4095 {
            Err(DateTimeError::InvalidYear)
        } else if self.month < 1 || self.month > 12 {
            Err(DateTimeError::InvalidMonth)
        } else if self.day < 1 || self.day > 31 {
            Err(DateTimeError::InvalidDay)
        } else if self.hour > 23 {
            Err(DateTimeError::InvalidHour)
        } else if self.minute > 59 {
            Err(DateTimeError::InvalidMinute)
        } else if self.second > 59 {
            Err(DateTimeError::InvalidSecond)
        } else {
            Ok(())
        }
    }
}

/// A day of the week
#[repr(u8)]
#[derive(Copy, Clone, Debug, PartialEq, Eq, Ord, PartialOrd, Hash)]
#[allow(missing_docs)]
pub enum DayOfWeek {
    Sunday = 0,
    Monday = 1,
    Tuesday = 2,
    Wednesday = 3,
    Thursday = 4,
    Friday = 5,
    Saturday = 6,
}

impl DayOfWeek {
    fn from_u8(v: u8) -> Result<Self, DateTimeError> {
        Ok(match v {
            0 => Self::Sunday,
            1 => Self::Monday,
            2 => Self::Tuesday,
            3 => Self::Wednesday,
            4 => Self::Thursday,
            5 => Self::Friday,
            6 => Self::Saturday,
            x => return Err(DateTimeError::InvalidDayOfWeek(x)),
        })
    }
}

/// A filter used for [`RtcClock::schedule_alarm`].
#[derive(Default)]
pub struct DateTimeFilter {
    /// The year that this alarm should trigger on, `None` if the RTC alarm should not trigger on a year value.
    pub year: Option<u16>,
    /// The month that this alarm should trigger on, `None` if the RTC alarm should not trigger on a month value.
    pub month: Option<u8>,
    /// The day that this alarm should trigger on, `None` if the RTC alarm should not trigger on a day value.
    pub day: Option<u8>,
    /// The day of week that this alarm should trigger on, `None` if the RTC alarm should not trigger on a day of week value.
    pub day_of_week: Option<DayOfWeek>,
    /// The hour that this alarm should trigger on, `None` if the RTC alarm should not trigger on a hour value.
    pub hour: Option<u8>,
    /// The minute that this alarm should trigger on, `None` if the RTC alarm should not trigger on a minute value.
    pub minute: Option<u8>,
    /// The second that this alarm should trigger on, `None` if the RTC alarm should not trigger on a second value.
    pub second: Option<u8>,
}

impl DateTimeFilter {
    /// Set a filter on the given year
    pub fn year(mut self, year: u16) -> Self {
        self.year = Some(year);
        self
    }
    /// Set a filter on the given month
    pub fn month(mut self, month: u8) -> Self {
        self.month = Some(month);
        self
    }
    /// Set a filter on the given day
    pub fn day(mut self, day: u8) -> Self {
        self.day = Some(day);
        self
    }
    /// Set a filter on the given day of the week
    pub fn day_of_week(mut self, day_of_week: DayOfWeek) -> Self {
        self.day_of_week = Some(day_of_week);
        self
    }
    /// Set a filter on the given hour
    pub fn hour(mut self, hour: u8) -> Self {
        self.hour = Some(hour);
        self
    }
    /// Set a filter on the given minute
    pub fn minute(mut self, minute: u8) -> Self {
        self.minute = Some(minute);
        self
    }
    /// Set a filter on the given second
    pub fn second(mut self, second: u8) -> Self {
        self.second = Some(second);
        self
    }
}

impl RtcClock {
    /// Init the RTC clock. This should be called exactly once at the start of the application.
    ///
    /// [`init_clocks_and_plls`] will call this internally so you often won't have to call this.
    ///
    /// [`init_clocks_and_plls`]: fn.init_clocks_and_plls.html
    pub fn init(&mut self, resets: &mut RESETS) {
        // reset RTC
        resets.reset.modify(|_, w| w.rtc().set_bit());
        resets.reset.modify(|_, w| w.rtc().clear_bit());
        while resets.reset_done.read().rtc().bit_is_clear() {
            core::hint::spin_loop();
        }
        // Set the RTC divider
        let freq = self.freq().integer() - 1;
        unsafe { &*RTC::ptr() }
            .clkdiv_m1
            .write(|w| unsafe { w.bits(freq) });
    }
    /// Checks to see if this RtcClock is running
    pub fn is_running(&mut self) -> bool {
        // Safety: We only read from the given address
        let rtc = unsafe { &*RTC::ptr() };
        let ctrl = rtc.ctrl.read().bits();
        (ctrl & consts::RTC_CTRL_RTC_ACTIVE_BITS) > 0
    }

    /// Set the RTC clock to the given DateTime. After this is set, the RtcClock will automatically keep the date synchronized.
    ///
    /// To retrieve the current DateTime, see [`now`]
    ///
    /// This DateTime persists through sleep mode and dormant mode, but not through chip resets.
    ///
    /// Note that the datetime calculation may be incorrect, especially when dealing with leapyears.
    /// See [Chapter 4 Section 8](https://datasheets.raspberrypi.org/rp2040/rp2040_datasheet.pdf) for more details
    ///
    /// [`now`]: #method.now
    pub fn set_datetime(&mut self, t: &DateTime) -> Result<(), RtcError> {
        t.validate().map_err(RtcError::InvalidDateTime)?;

        // NOTE: No functions should be called in this block as we take an unsafe ownership of `RTC::ptr()`
        {
            // Safety: We should have unique access to this RtcClock because we have `&mut self`
            // and this file is the only file that messes with these addresses.
            let rtc = unsafe { &*RTC::ptr() };

            rtc.ctrl.modify(|_, w| w.rtc_enable().clear_bit());
            // TODO: Add a timeout to this?
            while rtc.ctrl.read().rtc_active().bit_is_set() {
                core::hint::spin_loop();
            }

            let setup_0 = ((t.year as u32) << consts::RTC_SETUP_0_YEAR_LSB)
                | ((t.month as u32) << consts::RTC_SETUP_0_MONTH_LSB)
                | ((t.day as u32) << consts::RTC_SETUP_0_DAY_LSB);
            let setup_1 = ((t.day_of_week as u32) << consts::RTC_SETUP_1_DOTW_LSB)
                | ((t.hour as u32) << consts::RTC_SETUP_1_HOUR_LSB)
                | ((t.minute as u32) << consts::RTC_SETUP_1_MIN_LSB)
                | ((t.second as u32) << consts::RTC_SETUP_1_SEC_LSB);

            rtc.setup_0.write(|w| unsafe { w.bits(setup_0) });
            rtc.setup_1.write(|w| unsafe { w.bits(setup_1) });

            rtc.ctrl.write(|w| w.load().set_bit());
            rtc.ctrl.write(|w| w.rtc_enable().set_bit());
            // TODO: Add a timeout to this?
            while rtc.ctrl.read().rtc_active().bit_is_clear() {
                core::hint::spin_loop();
            }
        }
        // The Rtc RegisterBlock is no longer available here, so we can call other methods again

        Ok(())
    }

    /// Get the current date.
    ///
    /// Note that when this value is NOT set with [`RtcClock::set_datetime`], the returned value is undefined.
    ///
    /// This DateTime persists through sleep mode and dormant mode, but not through chip resets.
    ///
    /// Note that the datetime calculation may be incorrect, especially when dealing with leapyears.
    /// See [Chapter 4 Section 8](https://datasheets.raspberrypi.org/rp2040/rp2040_datasheet.pdf) for more details
    ///
    /// [`RtcClock::set_datetime`]: #method.set_datetime
    pub fn now(&mut self) -> Result<DateTime, RtcError> {
        if !self.is_running() {
            return Err(RtcError::NotRunning);
        }

        let rtc_0;
        let rtc_1;

        // NOTE: No functions should be called in this block as we take an unsafe ownership of `RTC::ptr()`
        {
            // Safety: We should have unique access to this RtcClock because we have `&mut self`
            // and this file is the only file that messes with these addresses.
            let rtc = unsafe { &*RTC::ptr() };

            rtc_0 = rtc.rtc_0.read().bits();
            rtc_1 = rtc.rtc_1.read().bits();
        }

        let day_of_week = DayOfWeek::from_u8(
            ((rtc_0 & consts::RTC_RTC_0_DOTW_BITS) >> consts::RTC_RTC_0_DOTW_LSB) as u8,
        )
        .map_err(RtcError::InvalidDateTime)?;
        let hour = ((rtc_0 & consts::RTC_RTC_0_HOUR_BITS) >> consts::RTC_RTC_0_HOUR_LSB) as u8;
        let minute = ((rtc_0 & consts::RTC_RTC_0_MIN_BITS) >> consts::RTC_RTC_0_MIN_LSB) as u8;
        let second = ((rtc_0 & consts::RTC_RTC_0_SEC_BITS) >> consts::RTC_RTC_0_SEC_LSB) as u8;
        let year = ((rtc_1 & consts::RTC_RTC_1_YEAR_BITS) >> consts::RTC_RTC_1_YEAR_LSB) as u16;
        let month = ((rtc_1 & consts::RTC_RTC_1_MONTH_BITS) >> consts::RTC_RTC_1_MONTH_LSB) as u8;
        let day = ((rtc_1 & consts::RTC_RTC_1_DAY_BITS) >> consts::RTC_RTC_1_DAY_LSB) as u8;

        Ok(DateTime {
            year,
            month,
            day,
            day_of_week,
            hour,
            minute,
            second,
        })
    }

    /// Disables the alarm. Will also clear the IRQ flag.
    pub fn disable_alarm(&mut self) {
        unsafe {
            // Safety: We should have unique access to this RtcClock because we have `&mut self`
            // and this file is the only file that messes with these addresses.
            let rtc = &*RTC::ptr();

            rtc.irq_setup_0.modify(|_, s| s.match_ena().clear_bit());

            // TODO: Add a timeout to this?
            while rtc.irq_setup_0.read().match_active().bit() {
                core::hint::spin_loop();
            }
        }
    }

    /// Schedule an alarm with the given [`DateTimeFilter`].
    /// This alarm will trigger [`Interrupt::RTC_IRQ`] the next time the filter matches the value of [`now`].
    ///
    /// After the interrupt is triggered, you need to first call [`RtcClock::clear_interrupt`], and after that re-schedule an alarm.
    /// Repeating alarms are currently *not* supported.
    ///
    /// [`now`]: #method.now
    /// [`Interrupt::RTC_IRQ`]: https://docs.rs/rp2040-pac/*/rp2040_pac/enum.Interrupt.html#variant.RTC_IRQ
    pub fn schedule_alarm(&mut self, filter: DateTimeFilter) {
        self.disable_alarm();

        let mut irq_setup_0 = 0;
        if let Some(year) = filter.year {
            irq_setup_0 |= consts::RTC_IRQ_SETUP_0_YEAR_ENA_BITS;
            irq_setup_0 |= (year as u32) << consts::RTC_IRQ_SETUP_0_YEAR_LSB;
        }
        if let Some(month) = filter.month {
            irq_setup_0 |= consts::RTC_IRQ_SETUP_0_MONTH_ENA_BITS;
            irq_setup_0 |= (month as u32) << consts::RTC_IRQ_SETUP_0_MONTH_LSB;
        }
        if let Some(day) = filter.day {
            irq_setup_0 |= consts::RTC_IRQ_SETUP_0_DAY_ENA_BITS;
            irq_setup_0 |= (day as u32) << consts::RTC_IRQ_SETUP_0_DAY_LSB;
        }

        let mut irq_setup_1 = 0;
        if let Some(day_of_week) = filter.day_of_week {
            irq_setup_1 |= consts::RTC_IRQ_SETUP_1_DOTW_ENA_BITS;
            irq_setup_1 |= (day_of_week as u32) << consts::RTC_IRQ_SETUP_1_DOTW_LSB;
        }
        if let Some(hour) = filter.hour {
            irq_setup_1 |= consts::RTC_IRQ_SETUP_1_HOUR_ENA_BITS;
            irq_setup_1 |= (hour as u32) << consts::RTC_IRQ_SETUP_1_HOUR_LSB;
        }
        if let Some(minute) = filter.minute {
            irq_setup_1 |= consts::RTC_IRQ_SETUP_1_MIN_ENA_BITS;
            irq_setup_1 |= (minute as u32) << consts::RTC_IRQ_SETUP_1_MIN_LSB;
        }
        if let Some(second) = filter.second {
            irq_setup_1 |= consts::RTC_IRQ_SETUP_1_SEC_ENA_BITS;
            irq_setup_1 |= (second as u32) << consts::RTC_IRQ_SETUP_1_SEC_LSB;
        }

        // NOTE: No functions should be called in this block as we take an unsafe ownership of `RTC::ptr()`
        unsafe {
            // Safety: We should have unique access to this RtcClock because we have `&mut self`
            // and this file is the only file that messes with these addresses.
            let rtc = &*RTC::ptr();

            // Safety: We know the RTC IRQ_SETUP_x has atomic updates
            crate::atomic_register_access::write_bitmask_set(
                rtc.irq_setup_0.as_ptr() as *mut u32,
                irq_setup_0,
            );
            crate::atomic_register_access::write_bitmask_set(
                rtc.irq_setup_1.as_ptr() as *mut u32,
                irq_setup_1,
            );

            rtc.inte.write(|w| w.rtc().set_bit());

            crate::atomic_register_access::write_bitmask_set(
                rtc.irq_setup_0.as_ptr() as *mut u32,
                consts::RTC_IRQ_SETUP_0_MATCH_ENA_BITS,
            );

            while !rtc.irq_setup_0.read().match_active().bit_is_set() {
                core::hint::spin_loop();
            }
        }
    }

    /// Clear the interrupt flag. This should be called every time [`Interrupt::RTC_IRQ`] has been triggered.
    ///
    /// [`Interrupt::RTC_IRQ`]: https://docs.rs/rp2040-pac/*/rp2040_pac/enum.Interrupt.html#variant.RTC_IRQ
    pub fn clear_interrupt(&mut self) {
        self.disable_alarm();
    }
}

/// Errors regarding the [`DateTime`] and [`DateTimeFilter`] structs.
#[derive(Clone, Debug, PartialEq, Eq)]
pub enum DateTimeError {
    /// The [DateTime] contains an invalid year value. Must be between `0..=4095`.
    InvalidYear,
    /// The [DateTime] contains an invalid month value. Must be between `1..=12`.
    InvalidMonth,
    /// The [DateTime] contains an invalid day value. Must be between `1..=31`.
    InvalidDay,
    /// The [DateTime] contains an invalid day of week. Must be between `0..=6` where 0 is Sunday.
    InvalidDayOfWeek(
        /// The value of the DayOfWeek that was given.
        u8,
    ),
    /// The [DateTime] contains an invalid hour value. Must be between `0..=23`.
    InvalidHour,
    /// The [DateTime] contains an invalid minute value. Must be between `0..=59`.
    InvalidMinute,
    /// The [DateTime] contains an invalid second value. Must be between `0..=59`.
    InvalidSecond,
}

/// Errors that can occur on methods on [RtcClock]
#[derive(Clone, Debug, PartialEq, Eq)]
pub enum RtcError {
    /// An invalid DateTime was given or stored on the hardware.
    InvalidDateTime(DateTimeError),

    /// The RTC clock is not running
    NotRunning,
}

mod consts {
    // helper constants, taken from https://github.com/raspberrypi/pico-sdk/blob/debef7471ef154135abe92eb74377f4ecbd80cdc/src/rp2040/hardware_regs/include/hardware/regs/rtc.h

    // Because we want to keep a 1:1 reference with the original source file, we have a lot of fields that aren't used (yet).
    #![allow(dead_code)]

    // -----------------------------------------------------------------------------
    // Field       : RTC_IRQ_SETUP_0_YEAR_ENA
    // Description : Enable year matching
    pub const RTC_IRQ_SETUP_0_YEAR_ENA_RESET: u32 = 0x0;
    pub const RTC_IRQ_SETUP_0_YEAR_ENA_BITS: u32 = 0x04000000;
    pub const RTC_IRQ_SETUP_0_YEAR_ENA_MSB: u32 = 26;
    pub const RTC_IRQ_SETUP_0_YEAR_ENA_LSB: u32 = 26;
    // -----------------------------------------------------------------------------
    // Field       : RTC_IRQ_SETUP_0_MONTH_ENA
    // Description : Enable month matching
    pub const RTC_IRQ_SETUP_0_MONTH_ENA_RESE: u32 = 0x0;
    pub const RTC_IRQ_SETUP_0_MONTH_ENA_BITS: u32 = 0x02000000;
    pub const RTC_IRQ_SETUP_0_MONTH_ENA_MSB: u32 = 25;
    pub const RTC_IRQ_SETUP_0_MONTH_ENA_LSB: u32 = 25;
    // -----------------------------------------------------------------------------
    // Field       : RTC_IRQ_SETUP_0_DAY_ENA
    // Description : Enable day matching
    pub const RTC_IRQ_SETUP_0_DAY_ENA_RESE: u32 = 0x0;
    pub const RTC_IRQ_SETUP_0_DAY_ENA_BITS: u32 = 0x01000000;
    pub const RTC_IRQ_SETUP_0_DAY_ENA_MSB: u32 = 24;
    pub const RTC_IRQ_SETUP_0_DAY_ENA_LSB: u32 = 24;

    // -----------------------------------------------------------------------------
    // Field       : RTC_SETUP_0_YEAR
    // Description : Year
    pub const RTC_SETUP_0_YEAR_RESET: u32 = 0x000;
    pub const RTC_SETUP_0_YEAR_BITS: u32 = 0x00fff000;
    pub const RTC_SETUP_0_YEAR_MSB: u32 = 23;
    pub const RTC_SETUP_0_YEAR_LSB: u32 = 12;

    // -----------------------------------------------------------------------------
    // Field       : RTC_SETUP_0_MONTH
    // Description : Month (1..12)
    pub const RTC_SETUP_0_MONTH_RESET: u32 = 0x0;
    pub const RTC_SETUP_0_MONTH_BITS: u32 = 0x00000f00;
    pub const RTC_SETUP_0_MONTH_MSB: u32 = 11;
    pub const RTC_SETUP_0_MONTH_LSB: u32 = 8;

    // -----------------------------------------------------------------------------
    // Field       : RTC_SETUP_0_DAY
    // Description : Day of the month (1..31)
    pub const RTC_SETUP_0_DAY_RESET: u32 = 0x00;
    pub const RTC_SETUP_0_DAY_BITS: u32 = 0x0000001f;
    pub const RTC_SETUP_0_DAY_MSB: u32 = 4;
    pub const RTC_SETUP_0_DAY_LSB: u32 = 0;

    // -----------------------------------------------------------------------------
    // Field       : RTC_IRQ_SETUP_1_DOTW_ENA
    // Description : Enable day of the week matching
    pub const RTC_IRQ_SETUP_1_DOTW_ENA_RESE: u32 = 0x0;
    pub const RTC_IRQ_SETUP_1_DOTW_ENA_BITS: u32 = 0x80000000;
    pub const RTC_IRQ_SETUP_1_DOTW_ENA_MSB: u32 = 31;
    pub const RTC_IRQ_SETUP_1_DOTW_ENA_LSB: u32 = 31;
    // -----------------------------------------------------------------------------
    // Field       : RTC_IRQ_SETUP_1_HOUR_ENA
    // Description : Enable hour matching
    pub const RTC_IRQ_SETUP_1_HOUR_ENA_RESET: u32 = 0x0;
    pub const RTC_IRQ_SETUP_1_HOUR_ENA_BITS: u32 = 0x40000000;
    pub const RTC_IRQ_SETUP_1_HOUR_ENA_MSB: u32 = 30;
    pub const RTC_IRQ_SETUP_1_HOUR_ENA_LSB: u32 = 30;
    // -----------------------------------------------------------------------------
    // Field       : RTC_IRQ_SETUP_1_MIN_ENA
    // Description : Enable minute matching
    pub const RTC_IRQ_SETUP_1_MIN_ENA_RESET: u32 = 0x0;
    pub const RTC_IRQ_SETUP_1_MIN_ENA_BITS: u32 = 0x20000000;
    pub const RTC_IRQ_SETUP_1_MIN_ENA_MSB: u32 = 29;
    pub const RTC_IRQ_SETUP_1_MIN_ENA_LSB: u32 = 29;
    // -----------------------------------------------------------------------------
    // Field       : RTC_IRQ_SETUP_1_SEC_ENA
    // Description : Enable second matching
    pub const RTC_IRQ_SETUP_1_SEC_ENA_RESET: u32 = 0x0;
    pub const RTC_IRQ_SETUP_1_SEC_ENA_BITS: u32 = 0x10000000;
    pub const RTC_IRQ_SETUP_1_SEC_ENA_MSB: u32 = 28;
    pub const RTC_IRQ_SETUP_1_SEC_ENA_LSB: u32 = 28;

    // -----------------------------------------------------------------------------
    // Field       : RTC_SETUP_1_DOTW
    // Description : Day of the week: 1-Monday...0-Sunday ISO 8601 mod 7
    pub const RTC_SETUP_1_DOTW_RESET: u32 = 0x0;
    pub const RTC_SETUP_1_DOTW_BITS: u32 = 0x07000000;
    pub const RTC_SETUP_1_DOTW_MSB: u32 = 26;
    pub const RTC_SETUP_1_DOTW_LSB: u32 = 24;

    // -----------------------------------------------------------------------------
    // Field       : RTC_SETUP_1_HOUR
    // Description : Hours
    pub const RTC_SETUP_1_HOUR_RESET: u32 = 0x00;
    pub const RTC_SETUP_1_HOUR_BITS: u32 = 0x001f0000;
    pub const RTC_SETUP_1_HOUR_MSB: u32 = 20;
    pub const RTC_SETUP_1_HOUR_LSB: u32 = 16;

    // -----------------------------------------------------------------------------
    // Field       : RTC_SETUP_1_MIN
    // Description : Minutes
    pub const RTC_SETUP_1_MIN_RESET: u32 = 0x00;
    pub const RTC_SETUP_1_MIN_BITS: u32 = 0x00003f00;
    pub const RTC_SETUP_1_MIN_MSB: u32 = 13;
    pub const RTC_SETUP_1_MIN_LSB: u32 = 8;

    // -----------------------------------------------------------------------------
    // Field       : RTC_SETUP_1_SEC
    // Description : Seconds
    pub const RTC_SETUP_1_SEC_RESET: u32 = 0x00;
    pub const RTC_SETUP_1_SEC_BITS: u32 = 0x0000003f;
    pub const RTC_SETUP_1_SEC_MSB: u32 = 5;
    pub const RTC_SETUP_1_SEC_LSB: u32 = 0;

    // -----------------------------------------------------------------------------
    // Field       : RTC_CTRL_LOAD
    // Description : Load RTC
    pub const RTC_CTRL_LOAD_RESET: u32 = 0x0;
    pub const RTC_CTRL_LOAD_BITS: u32 = 0x00000010;
    pub const RTC_CTRL_LOAD_MSB: u32 = 4;
    pub const RTC_CTRL_LOAD_LSB: u32 = 4;

    // -----------------------------------------------------------------------------
    // Field       : RTC_CTRL_RTC_ACTIVE
    // Description : RTC enabled (running)
    pub const RTC_CTRL_RTC_ACTIVE_BITS: u32 = 0x00000002;
    pub const RTC_CTRL_RTC_ACTIVE_MSB: u32 = 1;
    pub const RTC_CTRL_RTC_ACTIVE_LSB: u32 = 1;

    // -----------------------------------------------------------------------------
    // Field       : RTC_RTC_1_YEAR
    // Description : Year
    pub const RTC_RTC_1_YEAR_BITS: u32 = 0x00fff000;
    pub const RTC_RTC_1_YEAR_MSB: u32 = 23;
    pub const RTC_RTC_1_YEAR_LSB: u32 = 12;
    // -----------------------------------------------------------------------------
    // Field       : RTC_RTC_1_MONTH
    // Description : Month (1..12)
    pub const RTC_RTC_1_MONTH_BITS: u32 = 0x00000f00;
    pub const RTC_RTC_1_MONTH_MSB: u32 = 11;
    pub const RTC_RTC_1_MONTH_LSB: u32 = 8;
    // -----------------------------------------------------------------------------
    // Field       : RTC_RTC_1_DAY
    // Description : Day of the month (1..31)
    pub const RTC_RTC_1_DAY_BITS: u32 = 0x0000001f;
    pub const RTC_RTC_1_DAY_MSB: u32 = 4;
    pub const RTC_RTC_1_DAY_LSB: u32 = 0;

    // -----------------------------------------------------------------------------
    // Field       : RTC_RTC_0_DOTW
    // Description : Day of the week
    pub const RTC_RTC_0_DOTW_BITS: u32 = 0x07000000;
    pub const RTC_RTC_0_DOTW_MSB: u32 = 26;
    pub const RTC_RTC_0_DOTW_LSB: u32 = 24;
    // -----------------------------------------------------------------------------
    // Field       : RTC_RTC_0_HOUR
    // Description : Hours
    pub const RTC_RTC_0_HOUR_BITS: u32 = 0x001f0000;
    pub const RTC_RTC_0_HOUR_MSB: u32 = 20;
    pub const RTC_RTC_0_HOUR_LSB: u32 = 16;
    // -----------------------------------------------------------------------------
    // Field       : RTC_RTC_0_MIN
    // Description : Minutes
    pub const RTC_RTC_0_MIN_BITS: u32 = 0x00003f00;
    pub const RTC_RTC_0_MIN_MSB: u32 = 13;
    pub const RTC_RTC_0_MIN_LSB: u32 = 8;
    // -----------------------------------------------------------------------------
    // Field       : RTC_RTC_0_SEC
    // Description : Seconds
    pub const RTC_RTC_0_SEC_BITS: u32 = 0x0000003f;
    pub const RTC_RTC_0_SEC_MSB: u32 = 5;
    pub const RTC_RTC_0_SEC_LSB: u32 = 0;
    // -----------------------------------------------------------------------------
    // Field       : RTC_IRQ_SETUP_0_MATCH_ENA
    // Description : Global match enable. Don't change any other value while this
    //               one is enabled
    pub const RTC_IRQ_SETUP_0_MATCH_ENA_RESET: u32 = 0x0;
    pub const RTC_IRQ_SETUP_0_MATCH_ENA_BITS: u32 = 0x10000000;
    pub const RTC_IRQ_SETUP_0_MATCH_ENA_MSB: u32 = 28;
    pub const RTC_IRQ_SETUP_0_MATCH_ENA_LSB: u32 = 28;

    // -----------------------------------------------------------------------------
    // Field       : RTC_IRQ_SETUP_0_YEAR
    // Description : Year
    pub const RTC_IRQ_SETUP_0_YEAR_RESET: u32 = 0x000;
    pub const RTC_IRQ_SETUP_0_YEAR_BITS: u32 = 0x00fff000;
    pub const RTC_IRQ_SETUP_0_YEAR_MSB: u32 = 23;
    pub const RTC_IRQ_SETUP_0_YEAR_LSB: u32 = 12;
    // -----------------------------------------------------------------------------
    // Field       : RTC_IRQ_SETUP_0_MONTH
    // Description : Month (1..12)
    pub const RTC_IRQ_SETUP_0_MONTH_RESET: u32 = 0x0;
    pub const RTC_IRQ_SETUP_0_MONTH_BITS: u32 = 0x00000f00;
    pub const RTC_IRQ_SETUP_0_MONTH_MSB: u32 = 11;
    pub const RTC_IRQ_SETUP_0_MONTH_LSB: u32 = 8;
    // -----------------------------------------------------------------------------
    // Field       : RTC_IRQ_SETUP_0_DAY
    // Description : Day of the month (1..31)
    pub const RTC_IRQ_SETUP_0_DAY_RESET: u32 = 0x00;
    pub const RTC_IRQ_SETUP_0_DAY_BITS: u32 = 0x0000001f;
    pub const RTC_IRQ_SETUP_0_DAY_MSB: u32 = 4;
    pub const RTC_IRQ_SETUP_0_DAY_LSB: u32 = 0;

    // -----------------------------------------------------------------------------
    // Field       : RTC_IRQ_SETUP_1_DOTW
    // Description : Day of the week
    pub const RTC_IRQ_SETUP_1_DOTW_RESET: u32 = 0x0;
    pub const RTC_IRQ_SETUP_1_DOTW_BITS: u32 = 0x07000000;
    pub const RTC_IRQ_SETUP_1_DOTW_MSB: u32 = 26;
    pub const RTC_IRQ_SETUP_1_DOTW_LSB: u32 = 24;
    // -----------------------------------------------------------------------------
    // Field       : RTC_IRQ_SETUP_1_HOUR
    // Description : Hours
    pub const RTC_IRQ_SETUP_1_HOUR_RESET: u32 = 0x00;
    pub const RTC_IRQ_SETUP_1_HOUR_BITS: u32 = 0x001f0000;
    pub const RTC_IRQ_SETUP_1_HOUR_MSB: u32 = 20;
    pub const RTC_IRQ_SETUP_1_HOUR_LSB: u32 = 16;
    // -----------------------------------------------------------------------------
    // Field       : RTC_IRQ_SETUP_1_MIN
    // Description : Minutes
    pub const RTC_IRQ_SETUP_1_MIN_RESET: u32 = 0x00;
    pub const RTC_IRQ_SETUP_1_MIN_BITS: u32 = 0x00003f00;
    pub const RTC_IRQ_SETUP_1_MIN_MSB: u32 = 13;
    pub const RTC_IRQ_SETUP_1_MIN_LSB: u32 = 8;
    // -----------------------------------------------------------------------------
    // Field       : RTC_IRQ_SETUP_1_SEC
    // Description : Seconds
    pub const RTC_IRQ_SETUP_1_SEC_RESET: u32 = 0x00;
    pub const RTC_IRQ_SETUP_1_SEC_BITS: u32 = 0x0000003f;
    pub const RTC_IRQ_SETUP_1_SEC_MSB: u32 = 5;
    pub const RTC_IRQ_SETUP_1_SEC_LSB: u32 = 0;
}
