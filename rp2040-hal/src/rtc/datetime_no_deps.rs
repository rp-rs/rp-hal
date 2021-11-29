use rp2040_pac::rtc::{rtc_0, rtc_1, setup_0, setup_1};

/// Errors regarding the [`DateTime`] and [`DateTimeFilter`] structs.
///
/// [`DateTimeFilter`]: struct.DateTimeFilter.html
#[derive(Clone, Debug, PartialEq, Eq)]
pub enum Error {
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

fn day_of_week_from_u8(v: u8) -> Result<DayOfWeek, Error> {
    Ok(match v {
        0 => DayOfWeek::Sunday,
        1 => DayOfWeek::Monday,
        2 => DayOfWeek::Tuesday,
        3 => DayOfWeek::Wednesday,
        4 => DayOfWeek::Thursday,
        5 => DayOfWeek::Friday,
        6 => DayOfWeek::Saturday,
        x => return Err(Error::InvalidDayOfWeek(x)),
    })
}

pub(super) fn day_of_week_to_u8(dotw: DayOfWeek) -> u8 {
    dotw as u8
}

pub(super) fn validate_datetime(dt: &DateTime) -> Result<(), Error> {
    if dt.year > 4095 {
        Err(Error::InvalidYear)
    } else if dt.month < 1 || dt.month > 12 {
        Err(Error::InvalidMonth)
    } else if dt.day < 1 || dt.day > 31 {
        Err(Error::InvalidDay)
    } else if dt.hour > 23 {
        Err(Error::InvalidHour)
    } else if dt.minute > 59 {
        Err(Error::InvalidMinute)
    } else if dt.second > 59 {
        Err(Error::InvalidSecond)
    } else {
        Ok(())
    }
}

pub(super) fn write_setup_0(dt: &DateTime, w: &mut setup_0::W) {
    // Safety: the `.bits()` fields are marked `unsafe` but all bit values are valid
    unsafe {
        w.year().bits(dt.year);
        w.month().bits(dt.month);
        w.day().bits(dt.day);
    }
}

pub(super) fn write_setup_1(dt: &DateTime, w: &mut setup_1::W) {
    // Safety: the `.bits()` fields are marked `unsafe` but all bit values are valid
    unsafe {
        w.dotw().bits(dt.day_of_week as u8);
        w.hour().bits(dt.hour);
        w.min().bits(dt.minute);
        w.sec().bits(dt.second);
    }
}

pub(super) fn datetime_from_registers(rtc_0: rtc_0::R, rtc_1: rtc_1::R) -> Result<DateTime, Error> {
    let year = rtc_1.year().bits();
    let month = rtc_1.month().bits();
    let day = rtc_1.day().bits();

    let day_of_week = rtc_0.dotw().bits();
    let hour = rtc_0.hour().bits();
    let minute = rtc_0.min().bits();
    let second = rtc_0.sec().bits();

    let day_of_week = day_of_week_from_u8(day_of_week)?;
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
