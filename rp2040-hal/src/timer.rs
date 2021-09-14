//! Timer Peripheral
// See [Chapter 4 Section 6](https://datasheets.raspberrypi.org/rp2040/rp2040_datasheet.pdf) for more details

use embedded_time::duration::Microseconds;

use crate::pac::TIMER;

/// Timer peripheral
pub struct Timer {
    timer: TIMER,
}

impl Timer {
    /// Create a new [`Timer`]
    pub fn new(timer: TIMER) -> Self {
        Self { timer }
    }

    /// Get the current counter value.
    pub fn get_counter(&self) -> u64 {
        // latched read, low before high
        let lo = self.timer.timelr.read().bits();
        let hi = self.timer.timehr.read().bits();
        (hi as u64) << 32 | lo as u64
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
