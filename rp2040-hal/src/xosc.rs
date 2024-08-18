//! Crystal Oscillator (XOSC)
//!
//! See [Chapter 2 Section 16](https://datasheets.raspberrypi.org/rp2040/rp2040-datasheet.pdf) for more details.

use core::{convert::Infallible, ops::RangeInclusive};

use fugit::HertzU32;
use nb::Error::WouldBlock;

use crate::{pac::XOSC, typelevel::Sealed};

/// State of the Crystal Oscillator (typestate trait)
pub trait State: Sealed {}

/// XOSC is disabled (typestate)
pub struct Disabled;

/// XOSC is initialized but has not yet stabilized (typestate)
pub struct Unstable {
    freq_hz: HertzU32,
}

/// XOSC is stable (typestate)
pub struct Stable {
    freq_hz: HertzU32,
}

impl State for Disabled {}
impl Sealed for Disabled {}
impl State for Unstable {}
impl Sealed for Unstable {}
impl State for Stable {}
impl Sealed for Stable {}

/// Possible errors when initializing the CrystalOscillator
#[derive(Debug)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum Error {
    /// Frequency is out of the 1-15MHz range (see datasheet)
    FrequencyOutOfRange,

    /// Argument is bad : overflows, ...
    BadArgument,
}

/// Blocking helper method to setup the XOSC without going through all the steps.
///
/// This uses a startup_delay_multiplier of 64, which is a rather conservative value
/// that should work even if the XOSC starts up slowly. In case you need a fast boot
/// sequence, and your XOSC starts up quickly enough, use [`setup_xosc_blocking_custom_delay`].
pub fn setup_xosc_blocking(
    xosc_dev: XOSC,
    frequency: HertzU32,
) -> Result<CrystalOscillator<Stable>, Error> {
    let initialized_xosc = CrystalOscillator::new(xosc_dev).initialize(frequency, 64)?;

    let stable_xosc_token = nb::block!(initialized_xosc.await_stabilization()).unwrap();

    Ok(initialized_xosc.get_stable(stable_xosc_token))
}

/// Blocking helper method to setup the XOSC without going through all the steps.
///
/// This function allows setting a startup_delay_multiplier to tune the amount of time
/// the chips waits for the XOSC to stabilize.
/// The default value in the C SDK is 1, which should work on the Raspberry Pico, and many
/// third-party boards.
/// [`setup_xosc_blocking`], uses a conservative value of 64, which is the value commonly
/// used on slower-starting oscillators.
pub fn setup_xosc_blocking_custom_delay(
    xosc_dev: XOSC,
    frequency: HertzU32,
    startup_delay_multiplier: u32,
) -> Result<CrystalOscillator<Stable>, Error> {
    let initialized_xosc =
        CrystalOscillator::new(xosc_dev).initialize(frequency, startup_delay_multiplier)?;

    let stable_xosc_token = nb::block!(initialized_xosc.await_stabilization()).unwrap();

    Ok(initialized_xosc.get_stable(stable_xosc_token))
}

/// A Crystal Oscillator.
pub struct CrystalOscillator<S: State> {
    device: XOSC,
    state: S,
}

impl<S: State> CrystalOscillator<S> {
    /// Transitions the oscillator to another state.
    fn transition<To: State>(self, state: To) -> CrystalOscillator<To> {
        CrystalOscillator {
            device: self.device,
            state,
        }
    }

    /// Releases the underlying device.
    pub fn free(self) -> XOSC {
        self.device
    }
}

impl CrystalOscillator<Disabled> {
    /// Creates a new CrystalOscillator from the underlying device.
    pub fn new(dev: XOSC) -> Self {
        CrystalOscillator {
            device: dev,
            state: Disabled,
        }
    }

    /// Initializes the XOSC : frequency range is set, startup delay is calculated and set.
    /// Set startup_delay_multiplier to a value > 1 when using a slow-starting oscillator.
    pub fn initialize(
        self,
        frequency: HertzU32,
        startup_delay_multiplier: u32,
    ) -> Result<CrystalOscillator<Unstable>, Error> {
        const ALLOWED_FREQUENCY_RANGE: RangeInclusive<HertzU32> =
            HertzU32::MHz(1)..=HertzU32::MHz(15);
        //1 ms = 10e-3 sec and Freq = 1/T where T is in seconds so 1ms converts to 1000Hz
        const STABLE_DELAY_AS_HZ: HertzU32 = HertzU32::Hz(1000);
        const DIVIDER: u32 = 256;

        if !ALLOWED_FREQUENCY_RANGE.contains(&frequency) {
            return Err(Error::FrequencyOutOfRange);
        }

        if startup_delay_multiplier == 0 {
            return Err(Error::BadArgument);
        }

        self.device.ctrl().write(|w| {
            w.freq_range()._1_15mhz();
            w
        });

        //startup_delay = ((freq_hz * STABLE_DELAY) / 256) = ((freq_hz / delay_to_hz) / 256)
        //              = freq_hz / (delay_to_hz * 256)
        //See Chapter 2, Section 16, §3)
        //We do the calculation first.
        let startup_delay = frequency.to_Hz() / (STABLE_DELAY_AS_HZ.to_Hz() * DIVIDER);
        let startup_delay = startup_delay.saturating_mul(startup_delay_multiplier);

        //Then we check if it fits into an u16.
        let startup_delay: u16 = startup_delay.try_into().map_err(|_| Error::BadArgument)?;

        self.device.startup().write(|w| unsafe {
            w.delay().bits(startup_delay);
            w
        });

        self.device.ctrl().write(|w| {
            w.enable().enable();
            w
        });

        Ok(self.transition(Unstable { freq_hz: frequency }))
    }
}

/// A token that's given when the oscillator is stabilized, and can be exchanged to proceed to the next stage.
pub struct StableOscillatorToken {
    _private: (),
}

impl CrystalOscillator<Unstable> {
    /// One has to wait for the startup delay before using the oscillator, ie awaiting stabilization of the XOSC
    pub fn await_stabilization(&self) -> nb::Result<StableOscillatorToken, Infallible> {
        if self.device.status().read().stable().bit_is_clear() {
            return Err(WouldBlock);
        }

        Ok(StableOscillatorToken { _private: () })
    }

    /// Returns the stabilized oscillator
    pub fn get_stable(self, _token: StableOscillatorToken) -> CrystalOscillator<Stable> {
        let freq_hz = self.state.freq_hz;
        self.transition(Stable { freq_hz })
    }
}

impl CrystalOscillator<Stable> {
    /// Operating frequency of the XOSC in hertz
    pub fn operating_frequency(&self) -> HertzU32 {
        self.state.freq_hz
    }

    /// Disables the XOSC
    pub fn disable(self) -> CrystalOscillator<Disabled> {
        self.device.ctrl().modify(|_r, w| {
            w.enable().disable();
            w
        });

        self.transition(Disabled)
    }

    /// Put the XOSC in DORMANT state. The method returns after the processor awakens.
    ///
    /// After waking up from the DORMANT state, XOSC needs to re-stabilise.
    ///
    /// # Safety
    /// This method is marked unsafe because prior to switch the XOSC into DORMANT state,
    /// PLLs must be stopped and IRQs have to be properly configured.
    /// This method does not do any of that, it merely switches the XOSC to DORMANT state.
    /// It should only be called if this oscillator is the clock source for the system clock.
    /// See Chapter 2, Section 16, §5) for details.
    pub unsafe fn dormant(self) -> CrystalOscillator<Unstable> {
        //taken from the C SDK
        const XOSC_DORMANT_VALUE: u32 = 0x636f6d61;

        self.device.dormant().write(|w| {
            w.bits(XOSC_DORMANT_VALUE);
            w
        });

        let freq_hz = self.state.freq_hz;
        self.transition(Unstable { freq_hz })
    }
}
