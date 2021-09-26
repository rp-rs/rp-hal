//! Ring Oscillator (ROSC)
// See [Chapter 2 Section 17](https://datasheets.raspberrypi.org/rp2040/rp2040_datasheet.pdf) for more details

use embedded_time::rate::Extensions;
use embedded_time::rate::Hertz;

/// State of the Ring Oscillator (typestate trait)
pub trait State {}

/// ROSC is disabled (typestate)
pub struct Disabled;

/// ROSC is initialized, ie we've given parameters (typestate)
pub struct Enabled {
    freq_hz: Hertz,
}

/// ROSC is in dormant mode (see Chapter 2, Section 17, §7)
pub struct Dormant;

impl State for Disabled {}
impl State for Enabled {}
impl State for Dormant {}

/// A Ring Oscillator.
pub struct RingOscillator<S: State> {
    device: rp2040_pac::ROSC,
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
    pub fn free(self) -> rp2040_pac::ROSC {
        self.device
    }
}

impl RingOscillator<Disabled> {
    /// Creates a new RingOscillator from the underlying device.
    pub fn new(dev: rp2040_pac::ROSC) -> Self {
        RingOscillator {
            device: dev,
            state: Disabled,
        }
    }

    /// Initializes the ROSC : frequency range is set, startup delay is calculated and set.
    pub fn initialize(self) -> RingOscillator<Enabled> {
        self.device.ctrl.write(|w| w.enable().enable());

        self.transition(Enabled {
            freq_hz: 6_500_000u32.Hz(),
        })
    }
}

impl RingOscillator<Enabled> {
    /// Approx operating frequency of the ROSC in hertz
    pub fn operating_frequency(&self) -> Hertz {
        self.state.freq_hz
    }

    /// Disables the ROSC
    pub fn disable(self) -> RingOscillator<Disabled> {
        self.device.ctrl.modify(|_r, w| w.enable().disable());

        self.transition(Disabled)
    }

    /// Generate random bit based on the Ring oscillator
    /// This is not suited for security purposes
    pub fn get_random_bit(&self) -> bool {
        self.device.randombit.read().randombit().bit()
    }

    /// Put the ROSC in DORMANT state.
    ///
    /// # Safety
    /// This method is marked unsafe because prior to switch the ROSC into DORMANT state,
    /// PLLs must be stopped and IRQs have to be properly configured.
    /// This method does not do any of that, it merely switches the ROSC to DORMANT state.
    /// See Chapter 2, Section 16, §5) for details.
    pub unsafe fn dormant(self) -> RingOscillator<Dormant> {
        //taken from the C SDK
        const ROSC_DORMANT_VALUE: u32 = 0x636f6d61;

        self.device.dormant.write(|w| w.bits(ROSC_DORMANT_VALUE));

        self.transition(Dormant)
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
