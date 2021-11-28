use chrono::{Datelike, Timelike};
use rp2040_pac::rtc::{rtc_0, rtc_1, setup_0, setup_1};

/// Alias for [`chrono::NaiveDateTime`]
pub type DateTime = chrono::NaiveDateTime;
/// Alias for [`chrono::Weekday`]
pub type DayOfWeek = chrono::Weekday;

/// Errors regarding the [`DateTime`] and [`DateTimeFilter`] structs.
///
/// [`DateTimeFilter`]: struct.DateTimeFilter.html
#[derive(Clone, Debug, PartialEq, Eq)]
pub enum Error {
    /// The [DateTime] has an invalid year. The year must be between 0 and 4095.
    InvalidYear,
    /// The [DateTime] contains an invalid date.
    InvalidDate,
    /// The [DateTime] contains an invalid time.
    InvalidTime,
}

pub(super) fn day_of_week_to_u8(dotw: DayOfWeek) -> u8 {
    dotw.num_days_from_sunday() as u8
}

pub(crate) fn validate_datetime(dt: &DateTime) -> Result<(), Error> {
    if dt.year() < 0 || dt.year() > 4095 {
        // rp2040 can't hold these years
        Err(Error::InvalidYear)
    } else {
        // The rest of the chrono date is assumed to be valid
        Ok(())
    }
}

pub(super) fn write_setup_0(dt: &DateTime, w: &mut setup_0::W) {
    // Safety: the `.bits()` fields are marked `unsafe` but all bit values are valid
    unsafe {
        w.year().bits(dt.year() as u16);
        w.month().bits(dt.month() as u8);
        w.day().bits(dt.day() as u8);
    }
}

pub(super) fn write_setup_1(dt: &DateTime, w: &mut setup_1::W) {
    // Safety: the `.bits()` fields are marked `unsafe` but all bit values are valid
    unsafe {
        w.dotw().bits(dt.weekday().num_days_from_sunday() as u8);
        w.hour().bits(dt.hour() as u8);
        w.min().bits(dt.minute() as u8);
        w.sec().bits(dt.second() as u8);
    }
}

pub(super) fn datetime_from_registers(rtc_0: rtc_0::R, rtc_1: rtc_1::R) -> Result<DateTime, Error> {
    let year = rtc_1.year().bits() as i32;
    let month = rtc_1.month().bits() as u32;
    let day = rtc_1.day().bits() as u32;

    let hour = rtc_0.hour().bits() as u32;
    let minute = rtc_0.min().bits() as u32;
    let second = rtc_0.sec().bits() as u32;

    let date = chrono::NaiveDate::from_ymd_opt(year, month, day).ok_or(Error::InvalidDate)?;
    let time = chrono::NaiveTime::from_hms_opt(hour, minute, second).ok_or(Error::InvalidTime)?;
    Ok(DateTime::new(date, time))
}
