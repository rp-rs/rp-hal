//! POWMAN Support
//!
//! The POWMAN peripheral contains a mixture of functionality, including:
//!
//! * [x] An always-on timer (AOT) with alarm
//! * [ ] Voltage Regulator Control
//! * [ ] Brown-out detection
//! * [ ] Low-power oscillator control
//! * [ ] Using as GPIO as a time reference or wake-up signal
//! * [ ] The power-on statemachine, including last-power-on reason
//!
//! See [Section 6.5](https://rptl.io/rp2350-datasheet) of the RP2350 datasheet
//! for more details

use crate::{
    gpio::{
        bank0::{Gpio12, Gpio14, Gpio20, Gpio22},
        DynPullType, FunctionSioInput, Pin,
    },
    pac,
};

/// A frequency in kHz, represented as a fixed point 16.16 value
///
/// The RP2350 needs the frequency in `(integer_kHz, frac_kHz)` where `frac_kHz`
/// goes from 0 to 65535 (and 32768 represents 0.5 kHz). This might seem odd,
/// but the AOT counts in milliseconds, so this is basically *integer number of
/// ticks per millisecond* and *fractional number of ticks per millisecond*.
///
/// This type represents a frequency converted into that format.
///
/// The easiest way to construct one is with [`FractionalFrequency::from_hz`]:
#[derive(Debug, Copy, Clone, PartialEq, Eq)]
pub struct FractionalFrequency(u32);

impl FractionalFrequency {
    /// Create a new Fractional Frequency from any `fugit::Rate`.
    pub const fn new<const NOM: u32, const DENOM: u32>(
        rate: fugit::Rate<u32, NOM, DENOM>,
    ) -> FractionalFrequency {
        // fugit has a `convert` method, but it has rounding problems.
        // So we do it by hand.
        let rd_times_ln: u64 = 65536u64 * NOM as u64;
        let ld_times_rn: u64 = DENOM as u64 * 1000u64;
        let divisor: u64 = gcd::binary_u64(ld_times_rn, rd_times_ln);
        let rd_times_ln: u64 = rd_times_ln / divisor;
        let ld_times_rn: u64 = ld_times_rn / divisor;
        let raw = rate.raw() as u64 * rd_times_ln;
        let raw = (raw + (ld_times_rn / 2)) / ld_times_rn;
        FractionalFrequency(raw as u32)
    }

    /// Create a new Fractional Frequency from an integer number in Hertz
    ///
    /// ```rust
    /// # use rp235x_hal::powman::{AotClockSource, FractionalFrequency};
    /// let source = AotClockSource::Xosc(FractionalFrequency::from_hz(12_000_000));
    /// ```
    pub const fn from_hz(hz: u32) -> FractionalFrequency {
        let hz: fugit::HertzU32 = fugit::HertzU32::from_raw(hz);
        FractionalFrequency::new(hz)
    }

    /// Convert to an integer value in Hz
    ///
    /// Note: this involves a loss of precision
    pub const fn as_int_hz(self) -> u32 {
        let (i, f) = self.to_registers();
        (i as u32 * 1000) + ((f as u32 * 1000) / 65536)
    }

    /// Convert to an floating point value in Hz
    pub fn as_float_hz(self) -> f32 {
        let (i, f) = self.to_registers();
        (i as f32 + (f as f32) / 65536.0) * 1000.0
    }

    /// Construct a fractional frequency from the raw register contents
    const fn from_registers(i: u16, f: u16) -> FractionalFrequency {
        let raw = ((i as u32) << 16) | (f as u32);
        FractionalFrequency(raw)
    }

    /// Convert to the raw register values
    const fn to_registers(self) -> (u16, u16) {
        let raw = self.0;
        ((raw >> 16) as u16, raw as u16)
    }
}

impl core::fmt::Display for FractionalFrequency {
    fn fmt(&self, f: &mut core::fmt::Formatter<'_>) -> core::fmt::Result {
        write!(f, "{} Hz", self.as_float_hz())
    }
}

#[cfg(feature = "defmt")]
impl defmt::Format for FractionalFrequency {
    fn format(&self, f: defmt::Formatter) {
        defmt::write!(f, "{=u32} Hz", self.as_int_hz())
    }
}

/// Specify which clock source the POWMAN Always-On-Timer is clocked from.
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[derive(Debug, Copy, Clone, PartialEq, Eq)]
pub enum AotClockSource {
    /// On-Chip Crystal Oscillator (XOSC).
    ///
    /// When the chip core is powered, the tick source can be switched the
    /// on-chip crystal oscillator for greater precision.
    Xosc(FractionalFrequency),
    /// On-Chip Low-Power Oscillator (LPOSC).
    ///
    /// The LPOSC frequency is not precise and may vary with voltage and
    /// temperature
    Lposc(FractionalFrequency),
    /// External GPIO pin input to the Low-Power Oscillator (LPOSC).
    ///
    /// Uses the pin provided when the POWMAN driver was constructed.
    GpioLpOsc(FractionalFrequency),
    /// External 1 kHz GPIO pin.
    ///
    /// Uses the pin provided when the POWMAN driver was constructed.
    Gpio1kHz,
    /// External 1 Hz GPIO pin
    ///
    /// Uses the pin provided when the POWMAN driver was constructed.
    Gpio1Hz,
}

/// The ways in which frequency conversion can fail
#[derive(Debug, Copy, Clone, PartialEq, Eq)]
pub enum ClockSourceError {
    /// Tried to construct an LPOSC frequency but it was too high
    InvalidFrequency(FractionalFrequency),
    /// Tried to pick an external clock source with no GPIO pin
    MissingGpioPin,
}

impl AotClockSource {
    /// Create a clock source, using the XOSC running at the given frequency.
    pub const fn new_xosc(freq: FractionalFrequency) -> AotClockSource {
        AotClockSource::Xosc(freq)
    }

    /// Create a clock source, using the LPOSC running at the given frequency.
    ///
    /// Gives an error if the frequency is above 256 kHz.
    pub const fn new_lposc(freq: FractionalFrequency) -> Result<AotClockSource, ClockSourceError> {
        if freq.0 >= 0x0100_0000 {
            // LPOSC only goes up to 256 kHz
            return Err(ClockSourceError::InvalidFrequency(freq));
        }
        Ok(AotClockSource::Lposc(freq))
    }
}

impl core::fmt::Display for AotClockSource {
    fn fmt(&self, f: &mut core::fmt::Formatter<'_>) -> core::fmt::Result {
        match self {
            AotClockSource::Xosc(frac) => {
                write!(f, "Crystal @ {}", frac)
            }
            AotClockSource::Lposc(frac) => {
                write!(f, "On-Chip LowPower @ {}", frac)
            }
            AotClockSource::GpioLpOsc(frac) => {
                write!(f, "GPIO LowPower @ {}", frac)
            }
            AotClockSource::Gpio1kHz => write!(f, "GPIO 1kHz"),
            AotClockSource::Gpio1Hz => write!(f, "GPIO 1 Hz"),
        }
    }
}

/// A bit in the POWMAN.TIMER register.
///
/// We can't use the svd2rust API because we need to also set a key.
#[repr(u16)]
enum TimerBit {
    Run = 1 << 1,
    Clear = 1 << 2,
    // 3 is reserved
    AlarmEnable = 1 << 4,
    // TIMER_POWERUP_ON_ALARM = 1 << 5,
    AlarmRinging = 1 << 6,
    // 7 is reserved
    UseLposc = 1 << 8,
    UseXosc = 1 << 9,
    Use1khz = 1 << 10,
    // 11:12 is reserved
    Use1hz = 1 << 13,
}

/// Describes a pin we can use as an AOT clock input
pub enum ClockPin {
    /// Using GPIO 12 as clock input
    Pin12(Pin<Gpio12, FunctionSioInput, DynPullType>),
    /// Using GPIO 14 as clock input
    Pin14(Pin<Gpio14, FunctionSioInput, DynPullType>),
    /// Using GPIO 20 as clock input
    Pin20(Pin<Gpio20, FunctionSioInput, DynPullType>),
    /// Using GPIO 22 as clock input
    Pin22(Pin<Gpio22, FunctionSioInput, DynPullType>),
}

/// The Power Management device
pub struct Powman {
    device: pac::POWMAN,
    aot_clock_pin: Option<ClockPin>,
}

impl Powman {
    const KEY_VALUE: u32 = 0x5AFE_0000;
    const EXT_TIME_REF_DRIVE_LPCLK: u16 = 1 << 4;
    const INT_TIMER: u32 = 1 << 1;

    /// Create a new AOT from the from the underlying POWMAN device.
    pub fn new(device: pac::POWMAN, aot_clock_pin: Option<ClockPin>) -> Powman {
        // we set this once, and can ignore it throughout the rest of the driver
        let ext_time_ref = device.ext_time_ref().as_ptr();
        match aot_clock_pin {
            None => {}
            Some(ClockPin::Pin12(_)) => {
                unsafe { Self::powman_write(ext_time_ref, 0) };
            }
            Some(ClockPin::Pin14(_)) => {
                unsafe { Self::powman_write(ext_time_ref, 2) };
            }
            Some(ClockPin::Pin20(_)) => {
                unsafe { Self::powman_write(ext_time_ref, 1) };
            }
            Some(ClockPin::Pin22(_)) => {
                unsafe { Self::powman_write(ext_time_ref, 3) };
            }
        }

        Powman {
            device,
            aot_clock_pin,
        }
    }

    /// Releases the underlying device.
    pub fn free(self) -> (pac::POWMAN, Option<ClockPin>) {
        (self.device, self.aot_clock_pin)
    }

    /// Start the Always-On-Timer
    pub fn aot_start(&mut self) {
        self.timer_set_bit(TimerBit::Run);
    }

    /// Configure the clock source for the Always-On-Timer
    ///
    /// If the timer is running, it will be stopped and restarted.
    pub fn aot_set_clock(&mut self, source: AotClockSource) -> Result<(), ClockSourceError> {
        // only allowed to change the clock speed when the timer is stopped
        let was_running = self.aot_is_running();
        if was_running {
            self.aot_stop();
        }
        match source {
            AotClockSource::Xosc(freq) => {
                let (int_portion, frac_portion) = freq.to_registers();
                unsafe {
                    Self::powman_write(self.device.xosc_freq_khz_int().as_ptr(), int_portion);
                    Self::powman_write(self.device.xosc_freq_khz_frac().as_ptr(), frac_portion);
                    self.timer_set_bit(TimerBit::UseXosc);
                }
                if was_running {
                    self.aot_start();
                    // wait for change
                    while self.device.timer().read().using_xosc() != true {
                        core::hint::spin_loop();
                    }
                }
            }
            AotClockSource::Lposc(freq) => {
                let (int_portion, frac_portion) = freq.to_registers();
                unsafe {
                    Self::powman_write(self.device.lposc_freq_khz_int().as_ptr(), int_portion);
                    Self::powman_write(self.device.lposc_freq_khz_frac().as_ptr(), frac_portion);
                    self.timer_set_bit(TimerBit::UseLposc);
                    Self::powman_clear_bits(
                        self.device.ext_time_ref().as_ptr(),
                        Self::EXT_TIME_REF_DRIVE_LPCLK,
                    );
                }
                if was_running {
                    self.aot_start();
                    // wait for change
                    while self.device.timer().read().using_lposc() != true {
                        core::hint::spin_loop();
                    }
                }
            }
            AotClockSource::GpioLpOsc(freq) => {
                if self.aot_clock_pin.is_none() {
                    return Err(ClockSourceError::MissingGpioPin);
                }
                let (int_portion, frac_portion) = freq.to_registers();
                unsafe {
                    Self::powman_write(self.device.lposc_freq_khz_int().as_ptr(), int_portion);
                    Self::powman_write(self.device.lposc_freq_khz_frac().as_ptr(), frac_portion);
                    self.timer_set_bit(TimerBit::UseLposc);
                    // Use the selected GPIO to drive the 32kHz low power clock, in place of LPOSC.
                    Self::powman_set_bits(
                        self.device.ext_time_ref().as_ptr(),
                        Self::EXT_TIME_REF_DRIVE_LPCLK,
                    );
                }
                if was_running {
                    self.aot_start();
                    // wait for change
                    while self.device.timer().read().using_gpio_1khz() != true {
                        core::hint::spin_loop();
                    }
                }
            }
            AotClockSource::Gpio1kHz => {
                if self.aot_clock_pin.is_none() {
                    return Err(ClockSourceError::MissingGpioPin);
                }
                self.timer_set_bit(TimerBit::Use1khz);
                if was_running {
                    self.aot_start();
                    // wait for change
                    while self.device.timer().read().using_gpio_1khz() != true {
                        core::hint::spin_loop();
                    }
                }
            }
            AotClockSource::Gpio1Hz => {
                if self.aot_clock_pin.is_none() {
                    return Err(ClockSourceError::MissingGpioPin);
                }
                self.timer_set_bit(TimerBit::Use1hz);
                if was_running {
                    self.aot_start();
                    // wait for change
                    while self.device.timer().read().using_gpio_1hz() != true {
                        core::hint::spin_loop();
                    }
                }
            }
        }
        Ok(())
    }

    /// Get the current Always-On-Timer clock source
    ///
    /// You may get `None` if the AOT is not running.
    pub fn aot_get_clock(&mut self) -> Option<AotClockSource> {
        let status = self.device.timer().read();
        if status.using_gpio_1hz().bit_is_set() {
            Some(AotClockSource::Gpio1Hz)
        } else if status.using_gpio_1khz().bit_is_set() {
            Some(AotClockSource::Gpio1kHz)
        } else if status.using_lposc().bit_is_set() {
            let int_portion = self.device.lposc_freq_khz_int().read().bits() as u16;
            let frac_portion = self.device.lposc_freq_khz_frac().read().bits() as u16;
            Some(AotClockSource::Lposc(FractionalFrequency::from_registers(
                int_portion,
                frac_portion,
            )))
        } else if status.using_xosc().bit_is_set() {
            let int_portion = self.device.xosc_freq_khz_int().read().bits() as u16;
            let frac_portion = self.device.xosc_freq_khz_frac().read().bits() as u16;
            Some(AotClockSource::Xosc(FractionalFrequency::from_registers(
                int_portion,
                frac_portion,
            )))
        } else {
            None
        }
    }

    /// Stop the Always-On-Timer
    pub fn aot_stop(&self) {
        self.timer_clear_bit(TimerBit::Run);
    }

    /// Is the Always-On-Timer running
    pub fn aot_is_running(&self) -> bool {
        let status = self.device.timer().read();
        status.run().bit_is_set()
    }

    /// Get the time from the Always-On-Timer
    pub fn aot_get_time(&self) -> u64 {
        // keep reading until we do NOT cross a 32-bit boundary
        loop {
            let upper1 = self.device.read_time_upper().read().bits();
            let lower = self.device.read_time_lower().read().bits();
            let upper2 = self.device.read_time_upper().read().bits();
            if upper1 == upper2 {
                // we did not cross a boundary
                return (u64::from(upper1) << 32) | u64::from(lower);
            }
        }
    }

    /// Get the alarm time from the Always-On-Timer
    pub fn aot_get_alarm(&self) -> u64 {
        let alarm0 = self
            .device
            .alarm_time_15to0()
            .read()
            .alarm_time_15to0()
            .bits() as u64;
        let alarm1 = self
            .device
            .alarm_time_31to16()
            .read()
            .alarm_time_31to16()
            .bits() as u64;
        let alarm2 = self
            .device
            .alarm_time_47to32()
            .read()
            .alarm_time_47to32()
            .bits() as u64;
        let alarm3 = self
            .device
            .alarm_time_63to48()
            .read()
            .alarm_time_63to48()
            .bits() as u64;
        (alarm3 << 48) | (alarm2 << 32) | (alarm1 << 16) | alarm0
    }

    /// Clear the Always-On-Timer
    ///
    /// If the timer is running, it will NOT be stopped.
    pub fn aot_clear(&self) {
        self.timer_set_bit(TimerBit::Clear);
    }

    /// Enable the alarm
    pub fn aot_alarm_enable(&self) {
        self.timer_set_bit(TimerBit::AlarmEnable);
    }

    /// Disable the alarm
    ///
    /// Note: this is not the same as clearing a ringing alarm. This just makes
    /// sure it won't fire in the future.
    pub fn aot_alarm_disable(&self) {
        self.timer_clear_bit(TimerBit::AlarmEnable);
    }

    /// Is the alarm currently ringing?
    ///
    /// In other words, has it fired since we last cleared it?
    pub fn aot_alarm_ringing(&self) -> bool {
        self.device.timer().read().alarm().bit_is_set()
    }

    /// Clear a ringing alarm
    ///
    /// Also disables the alarm, otherwise the alarm is likely to go off again
    /// right away.
    pub fn aot_alarm_clear(&self) {
        self.aot_alarm_disable();
        // Early datasheets said write a 1 bit to clear the ringing alarm, but
        // they're lying. You have to write a zero.
        self.timer_clear_bit(TimerBit::AlarmRinging);
    }

    /// Enable the AOT Alarm interrupt
    pub fn aot_alarm_interrupt_enable(&self) {
        // Doesn't need a key to write
        let inte = self.device.inte().as_ptr();
        unsafe {
            crate::atomic_register_access::write_bitmask_set(inte, Self::INT_TIMER);
        }
    }

    /// Disable the AOT Alarm interrupt
    pub fn aot_alarm_interrupt_disable(&self) {
        // Doesn't need a key to write
        let inte = self.device.inte().as_ptr();
        unsafe {
            crate::atomic_register_access::write_bitmask_clear(inte, Self::INT_TIMER);
        }
    }

    /// Disable the AOT Alarm interrupt, from a free function
    pub fn static_aot_alarm_interrupt_disable() {
        unsafe {
            let ptr = pac::POWMAN::PTR as *mut u32;
            let inte = ptr.offset(0xe4 >> 2);
            crate::atomic_register_access::write_bitmask_clear(inte, Self::INT_TIMER);
        }
    }

    /// Has the AOT Alarm interrupt fired?
    pub fn aot_alarm_interrupt_status(&self) -> bool {
        self.device.ints().read().timer().bit_is_set()
    }

    /// Set the time in the Always-On-Timer
    ///
    /// If the timer is running, it will be stopped and restarted.
    pub fn aot_set_time(&mut self, time: u64) {
        // only allowed to change the time when the timer is stopped
        let was_running = self.aot_is_running();
        if was_running {
            self.aot_stop();
        }
        self.device.set_time_15to0().modify(|_r, w| {
            unsafe {
                w.bits(Self::KEY_VALUE);
                w.set_time_15to0().bits(time as u16);
            }
            w
        });
        self.device.set_time_31to16().modify(|_r, w| {
            unsafe {
                w.bits(Self::KEY_VALUE);
                w.set_time_31to16().bits((time >> 16) as u16);
            }
            w
        });
        self.device.set_time_47to32().modify(|_r, w| {
            unsafe {
                w.bits(Self::KEY_VALUE);
                w.set_time_47to32().bits((time >> 32) as u16);
            }
            w
        });
        self.device.set_time_63to48().modify(|_r, w| {
            unsafe {
                w.bits(Self::KEY_VALUE);
                w.set_time_63to48().bits((time >> 48) as u16);
            }
            w
        });
        if was_running {
            self.aot_start();
        }
    }

    /// Set the alarm in the Always-On-Timer and start it.
    pub fn aot_set_alarm(&mut self, time: u64) {
        self.aot_alarm_disable();
        self.device.alarm_time_15to0().modify(|_r, w| {
            unsafe {
                w.bits(Self::KEY_VALUE);
                w.alarm_time_15to0().bits(time as u16);
            }
            w
        });
        self.device.alarm_time_31to16().modify(|_r, w| {
            unsafe {
                w.bits(Self::KEY_VALUE);
                w.alarm_time_31to16().bits((time >> 16) as u16);
            }
            w
        });
        self.device.alarm_time_47to32().modify(|_r, w| {
            unsafe {
                w.bits(Self::KEY_VALUE);
                w.alarm_time_47to32().bits((time >> 32) as u16);
            }
            w
        });
        self.device.alarm_time_63to48().modify(|_r, w| {
            unsafe {
                w.bits(Self::KEY_VALUE);
                w.alarm_time_63to48().bits((time >> 48) as u16);
            }
            w
        });
    }

    /// Clear the specified bits in the TIMER register.
    ///
    /// For every `1` bit in `bitmask`, that bit is set to `0` in `POWMAN.TIMER`.
    fn timer_clear_bit(&self, bit: TimerBit) {
        let reg = self.device.timer().as_ptr();
        // # Safety
        //
        // svd2rust generated the address for us, so we know it's right
        //
        // This is an atomic operation, so &self is fine
        unsafe {
            Self::powman_clear_bits(reg, bit as u16);
        }
    }

    /// Set the specified bits in the TIMER register.
    ///
    /// For every `1` bit in `bitmask`, that bit is set to `1` in `POWMAN.TIMER`.
    fn timer_set_bit(&self, bit: TimerBit) {
        let reg = self.device.timer().as_ptr();
        // # Safety
        //
        // svd2rust generated the address for us, so we know it's right
        //
        // This is an atomic operation, so &self is fine
        unsafe {
            Self::powman_set_bits(reg, bit as u16);
        }
    }

    /// Write to a POWMAN protected register, using the key
    unsafe fn powman_write(addr: *mut u32, bits: u16) {
        let bits_with_key: u32 = Self::KEY_VALUE | u32::from(bits);
        addr.write_volatile(bits_with_key);
    }

    /// Set bits in a POWMAN protected register, using the key
    unsafe fn powman_set_bits(addr: *mut u32, bits: u16) {
        let bits_with_key: u32 = Self::KEY_VALUE | u32::from(bits);
        crate::atomic_register_access::write_bitmask_set(addr, bits_with_key);
    }

    /// Clear bits in a POWMAN protected register, using the key
    unsafe fn powman_clear_bits(addr: *mut u32, bits: u16) {
        let bits_with_key: u32 = Self::KEY_VALUE | u32::from(bits);
        crate::atomic_register_access::write_bitmask_clear(addr, bits_with_key);
    }
}

#[cfg(test)]
mod tests {
    use crate::fugit::RateExtU32;

    use super::*;

    #[test]
    fn test_split_join_12mhz() {
        let clock_speed: fugit::HertzU32 = 12_000_000.Hz();
        let freq = FractionalFrequency::new(clock_speed);
        // As given in the datasheet as the default values for 12 MHz XOSC
        assert_eq!(freq.to_registers(), (0x2ee0, 0x0000));
        assert_eq!(freq.as_int_hz(), 12_000_000);
        assert_eq!(freq.as_float_hz(), 12_000_000.0);
        let freq2 = FractionalFrequency::from_registers(0x2ee0, 0x0000);
        assert_eq!(freq, freq2);
        let freq3 = FractionalFrequency::from_hz(12_000_000);
        assert_eq!(freq, freq3);
    }

    #[test]
    fn test_split_32768() {
        let clock_speed: fugit::HertzU32 = 32768.Hz();
        let freq = FractionalFrequency::new(clock_speed);
        // As given in the datasheet as the default values for 32.768 kHz LPOSC
        assert_eq!(freq.to_registers(), (0x20, 0xc49c));
        assert_eq!(freq.as_int_hz(), 32768);
        let delta = freq.as_float_hz() - 32768.0;
        assert!(delta < 0.01, "{} != 32768.0", freq.as_float_hz());
        let freq2 = FractionalFrequency::from_registers(0x20, 0xc49c);
        assert_eq!(freq, freq2);
        let freq3 = FractionalFrequency::from_hz(32768);
        assert_eq!(freq, freq3);
    }
}
