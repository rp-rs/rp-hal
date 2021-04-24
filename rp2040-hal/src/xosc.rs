//! Crystal Oscillator (XOSC)
// See [Chapter 2 Section 16](https://datasheets.raspberrypi.org/rp2040/rp2040_datasheet.pdf) for more details

use core::{
    convert::Infallible,
    ops::RangeInclusive
};

use embedded_time::{
    fraction::Fraction,
    fixed_point::FixedPoint,
    rate::{
        Hertz,
        Megahertz,
    },
    duration::{
        Seconds,
        Milliseconds,
        Duration
    }
};

use nb::Error::WouldBlock;

/// State of the Crystal Oscillator (typestate trait)
pub trait State {}

/// XOSC is disabled (typestate)
pub struct Disabled;

/// XOSC is initialized, ie we've given parameters (typestate)
pub struct Initialized {
    freq_hz: Hertz<u64>
}

/// Stable state (typestate)
pub struct Stable{
    freq_hz: Hertz<u64>
}

/// XOSC is disabling (typestate)
pub struct Disabling;

/// XOSC is in dormant mode (see Chapter 2, Section 16, §5)
pub struct Dormant;

impl State for Disabled {}
impl State for Initialized {}
impl State for Stable {}
impl State for Disabling {}
impl State for Dormant {}

/// Possible errors when initializing the CrystalOscillator
pub enum Error {
    /// Frequency is out of the 1-15MHz range (see datasheet)
    FrequencyOutOfRange,

    /// Argument is bad : overflows, ...
    BadArgument
}

/// A Crystal Oscillator.
pub struct CrystalOscillator<S: State> {
    device: rp2040_pac::XOSC,
    state: S
}

impl<S: State> CrystalOscillator<S> {
    /// Transitions the oscillator to another state.
    fn transition<To: State>(self, state: To) -> CrystalOscillator<To> {
        CrystalOscillator {
            device: self.device,
            state: state
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
            state: Disabled
        }
    }

    /// Initializes the XOSC : frequency range is set, startup delay is calculated and set.
    pub fn initialize(self, frequency: Megahertz<u32>) -> Result<CrystalOscillator<Initialized>, Error> {

        const ALLOWED_FREQUENCY_RANGE: RangeInclusive<Megahertz<u32>> = Megahertz(1)..=Megahertz(15);
        const STABLE_DELAY: Milliseconds<u64> = Milliseconds(1_u64);
        const DIVIDER: Fraction = Fraction::new(1, 256);

        if !ALLOWED_FREQUENCY_RANGE.contains(&frequency) {
            return Err(Error::FrequencyOutOfRange)
        }

        self.device.ctrl.write(|w| {
            w.freq_range()._1_15mhz();
            w
        });

        let freq_hz: Hertz<u64> = frequency.into();
        let delay_sec: Seconds<u64> = STABLE_DELAY.into();

        //let startup_delay = ((freq_hz / 1000) + 128) / 256;
        let startup_delay = delay_sec.
            checked_mul(freq_hz.integer()).and_then(|r|
                 r.to_generic::<u64>(DIVIDER).ok()
             ).
            ok_or(Error::BadArgument)?;

        self.device.startup.write(|w| unsafe {
            w.delay().bits(*startup_delay.integer() as u16);
            w
        });

        self.device.ctrl.write(|w| {
            w.enable().enable();
            w
        });

        Ok(self.transition(Initialized {
            freq_hz: freq_hz
        }))
    }
}

/// A token that's given when the oscillator is stablilzed, and can be exchanged to proceed to the next stage.
pub struct StableOscillatorToken {
    _private : ()
}

impl CrystalOscillator<Initialized> {

    /// One has to wait for the startup delay before using the oscillator, ie awaiting stablilzation of the XOSC
    pub fn await_stabilization(&self) -> nb::Result<StableOscillatorToken, Infallible> {

        if self.device.status.read().stable().bit_is_clear() {
            return Err(WouldBlock)
        }

        Ok(StableOscillatorToken {
            _private: ()
        })
    }

    /// Returns the stablilzed oscillator
    pub fn get_stable(self, _token: StableOscillatorToken) -> CrystalOscillator<Stable> {
        let freq_hz = self.state.freq_hz;
        self.transition(Stable {
            freq_hz:freq_hz
        })
    }
}

impl CrystalOscillator<Stable> {

    /// Operating frequency of the XOSC in hertz
    pub fn operating_frequency(&self) -> Hertz<u64> {
        self.state.freq_hz
    }

    /// Disables the XOSC
    pub fn disable(self) -> CrystalOscillator<Disabling> {
        self.device.ctrl.modify(|_r,w| {
            w.enable().disable();
            w
        });

        self.transition(Disabling)
    }

    /// Put the XOSC in dormant state
    pub fn dormant(self) -> CrystalOscillator<Dormant> {
        //taken from the C SDK
        const XOSC_DORMANT_VALUE: u32 = 0x636f6d61;

        self.device.dormant.write(|w| unsafe {
            w.bits(XOSC_DORMANT_VALUE);
            w
        });

        self.transition(Dormant)
    }
}
