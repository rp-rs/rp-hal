//! Ring Oscillator (ROSC)
//!
//! See [Section 8.3](https://rptl.io/rp2350-datasheet#section_rosc) for more details.
//!
//! In addition to its obvious role as a clock source, [`RingOscillator`] can also be used as a random number source
//! for the [`rand`] crate:
//!
//! ```no_run
//! # let mut pac = rp235x_pac::Peripherals::take().unwrap();
//! use rand::Rng;
//! use rp235x_hal::rosc::RingOscillator;
//! let mut rnd = RingOscillator::new(pac.ROSC).initialize();
//! let random_value: u32 = rnd.gen();
//! ```
//! [`rand`]: https://docs.rs/rand
use fugit::HertzU32;

use crate::{pac::ROSC, typelevel::Sealed};

/// State of the Ring Oscillator (typestate trait)
pub trait State: Sealed {}

/// ROSC is disabled (typestate)
pub struct Disabled;

/// ROSC is initialized, ie we've given parameters (typestate)
pub struct Enabled {
    freq_hz: HertzU32,
}

impl State for Disabled {}
impl Sealed for Disabled {}
impl State for Enabled {}
impl Sealed for Enabled {}

/// A Ring Oscillator.
pub struct RingOscillator<S: State> {
    device: ROSC,
    state: S,
}

impl<S: State> RingOscillator<S> {
    /// Transitions the oscillator to another state.
    fn transition<To: State>(self, state: To) -> RingOscillator<To> {
        RingOscillator {
            device: self.device,
            state,
        }
    }

    /// Releases the underlying device.
    pub fn free(self) -> ROSC {
        self.device
    }
}

impl RingOscillator<Disabled> {
    /// Creates a new RingOscillator from the underlying device.
    pub fn new(dev: ROSC) -> Self {
        RingOscillator {
            device: dev,
            state: Disabled,
        }
    }

    /// Initializes the ROSC : frequency range is set, startup delay is calculated and set.
    pub fn initialize(self) -> RingOscillator<Enabled> {
        self.device.ctrl().write(|w| w.enable().enable());

        use fugit::RateExtU32;
        self.transition(Enabled {
            freq_hz: 6_500_000u32.Hz(),
        })
    }

    /// Initializes the ROSC with a known frequency.
    ///
    /// See Sections 8.3.4 "Modifying the frequency", and 8.3.8 "Using the
    /// frequency counter" in the [RP2350 datasheet](https://rptl.io/rp2350-datasheet)
    /// for guidance on how to do this before initialising the ROSC. Also see
    /// `rosc_as_system_clock` example for usage.
    pub fn initialize_with_freq(self, known_freq: HertzU32) -> RingOscillator<Enabled> {
        self.device.ctrl().write(|w| w.enable().enable());
        self.transition(Enabled {
            freq_hz: known_freq,
        })
    }
}

impl RingOscillator<Enabled> {
    /// Approx operating frequency of the ROSC in hertz
    pub fn operating_frequency(&self) -> HertzU32 {
        self.state.freq_hz
    }

    /// Disables the ROSC
    pub fn disable(self) -> RingOscillator<Disabled> {
        self.device.ctrl().modify(|_r, w| w.enable().disable());

        self.transition(Disabled)
    }

    /// Generate random bit based on the Ring oscillator
    /// This is not suited for security purposes
    pub fn get_random_bit(&self) -> bool {
        self.device.randombit().read().randombit().bit()
    }

    /// Put the ROSC in DORMANT state. The method returns after the processor awakens.
    ///
    /// After waking up from the DORMANT state, ROSC restarts in approximately 1µs.
    ///
    /// # Safety
    /// This method is marked unsafe because prior to switch the ROSC into DORMANT state,
    /// PLLs must be stopped and IRQs have to be properly configured.
    /// This method does not do any of that, it merely switches the ROSC to DORMANT state.
    /// It should only be called if this oscillator is the clock source for the system clock.
    ///
    /// See [Section 6.5.3](https://rptl.io/rp2350-datasheet#section_bootrom) of the RP2350
    /// datasheet.
    pub unsafe fn dormant(&self) {
        //taken from the C SDK
        const ROSC_DORMANT_VALUE: u32 = 0x636f6d61;

        self.device.dormant().write(|w| w.bits(ROSC_DORMANT_VALUE));
    }
}

impl rand_core::RngCore for RingOscillator<Enabled> {
    fn next_u32(&mut self) -> u32 {
        rand_core::impls::next_u32_via_fill(self)
    }

    fn next_u64(&mut self) -> u64 {
        rand_core::impls::next_u64_via_fill(self)
    }

    fn fill_bytes(&mut self, dest: &mut [u8]) {
        for chunk in dest.iter_mut() {
            *chunk = 0_u8;
            for _ in 0..8 {
                *chunk <<= 1;
                *chunk ^= self.get_random_bit() as u8;
            }
        }
    }

    fn try_fill_bytes(&mut self, dest: &mut [u8]) -> Result<(), rand_core::Error> {
        self.fill_bytes(dest);
        Ok(())
    }
}
