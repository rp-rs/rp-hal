//! Crystal Oscillator (XOSC)
// See [Chapter 2 Section 16](https://datasheets.raspberrypi.org/rp2040/rp2040_datasheet.pdf) for more details

use core::convert::TryInto;
use core::{convert::Infallible, ops::RangeInclusive};

use embedded_time::{
    duration::{Duration, Milliseconds},
    fixed_point::FixedPoint,
    fraction::Fraction,
    rate::{Hertz, Megahertz, Rate},
};

use nb::Error::WouldBlock;

/// State of the Crystal Oscillator (typestate trait)
pub trait State {}

/// XOSC is disabled (typestate)
pub struct Disabled;

/// XOSC is initialized, ie we've given parameters (typestate)
pub struct Initialized {
    freq_hz: Hertz,
}

/// Stable state (typestate)
pub struct Stable {
    freq_hz: Hertz,
}

/// XOSC is in dormant mode (see Chapter 2, Section 16, §5)
pub struct Dormant;

impl State for Disabled {}
impl State for Initialized {}
impl State for Stable {}
impl State for Dormant {}

/// Possible errors when initializing the CrystalOscillator
pub enum Error {
    /// Frequency is out of the 1-15MHz range (see datasheet)
    FrequencyOutOfRange,

    /// Argument is bad : overflows, ...
    BadArgument,
}

/// Blocking helper method to setup the XOSC without going through all the steps.
pub fn setup_xosc_blocking(
    xosc_dev: rp2040_pac::XOSC,
    frequency: Hertz,
) -> Result<CrystalOscillator<Stable>, Error> {
    let initialized_xosc = CrystalOscillator::new(xosc_dev).initialize(frequency)?;

    let stable_xosc_token = nb::block!(initialized_xosc.await_stabilization()).unwrap();

    Ok(initialized_xosc.get_stable(stable_xosc_token))
}

/// A Crystal Oscillator.
pub struct CrystalOscillator<S: State> {
    device: rp2040_pac::XOSC,
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
    pub fn free(self) -> rp2040_pac::XOSC {
        self.device
    }
}

impl CrystalOscillator<Disabled> {
    /// Creates a new CrystalOscillator from the underlying device.
    pub fn new(dev: rp2040_pac::XOSC) -> Self {
        CrystalOscillator {
            device: dev,
            state: Disabled,
        }
    }

    /// Initializes the XOSC : frequency range is set, startup delay is calculated and set.
    pub fn initialize(self, frequency: Hertz) -> Result<CrystalOscillator<Initialized>, Error> {
        const ALLOWED_FREQUENCY_RANGE: RangeInclusive<Megahertz<u32>> =
            Megahertz(1)..=Megahertz(15);
        const STABLE_DELAY: Milliseconds = Milliseconds(1_u32);
        const DIVIDER: Fraction = Fraction::new(256, 1);

        let freq_mhz: Megahertz = frequency.into();

        if !ALLOWED_FREQUENCY_RANGE.contains(&freq_mhz) {
            return Err(Error::FrequencyOutOfRange);
        }

        self.device.ctrl.write(|w| {
            w.freq_range()._1_15mhz();
            w
        });

        //1 ms = 10e-3 sec and Freq = 1/T where T is in seconds so 1ms converts to 1000Hz
        let delay_to_hz: Hertz = STABLE_DELAY.to_rate().map_err(|_| Error::BadArgument)?;

        //startup_delay = ((freq_hz * 10e-3) / 256) = ((freq_hz / 1000) / 256)
        //See Chapter 2, Section 16, §3)
        //We do the calculation first.
        let startup_delay = frequency
            .checked_div(&delay_to_hz.integer())
            .and_then(|r| r.to_generic::<u32>(DIVIDER).ok())
            .ok_or(Error::BadArgument)?;

        //Then we check if it fits into an u16.
        let startup_delay: u16 = (startup_delay.integer())
            .try_into()
            .map_err(|_| Error::BadArgument)?;

        self.device.startup.write(|w| unsafe {
            w.delay().bits(startup_delay);
            w
        });

        self.device.ctrl.write(|w| {
            w.enable().enable();
            w
        });

        Ok(self.transition(Initialized { freq_hz: frequency }))
    }
}

/// A token that's given when the oscillator is stablilzed, and can be exchanged to proceed to the next stage.
pub struct StableOscillatorToken {
    _private: (),
}

impl CrystalOscillator<Initialized> {
    /// One has to wait for the startup delay before using the oscillator, ie awaiting stablilzation of the XOSC
    pub fn await_stabilization(&self) -> nb::Result<StableOscillatorToken, Infallible> {
        if self.device.status.read().stable().bit_is_clear() {
            return Err(WouldBlock);
        }

        Ok(StableOscillatorToken { _private: () })
    }

    /// Returns the stablilzed oscillator
    pub fn get_stable(self, _token: StableOscillatorToken) -> CrystalOscillator<Stable> {
        let freq_hz = self.state.freq_hz;
        self.transition(Stable { freq_hz })
    }
}

impl CrystalOscillator<Stable> {
    /// Operating frequency of the XOSC in hertz
    pub fn operating_frequency(&self) -> Hertz {
        self.state.freq_hz
    }

    /// Disables the XOSC
    pub fn disable(self) -> CrystalOscillator<Disabled> {
        self.device.ctrl.modify(|_r, w| {
            w.enable().disable();
            w
        });

        self.transition(Disabled)
    }

    /// Put the XOSC in DORMANT state.
    ///
    /// # Safety
    /// This method is marked unsafe because prior to switch the XOSC into DORMANT state,
    /// PLLs must be stopped and IRQs have to be properly configured.
    /// This method does not do any of that, it merely switches the XOSC to DORMANT state.
    /// See Chapter 2, Section 16, §5) for details.
    pub unsafe fn dormant(self) -> CrystalOscillator<Dormant> {
        //taken from the C SDK
        const XOSC_DORMANT_VALUE: u32 = 0x636f6d61;

        self.device.dormant.write(|w| {
            w.bits(XOSC_DORMANT_VALUE);
            w
        });

        self.transition(Dormant)
    }
}
