//! Pulse Width Modulation (PWM)
//!
//! To access the PWM pins you must call the 'split' method on the PWM. This will return a
//! `_____` struct with access to each PWM pin:
//!
//! ```rust
//! use rp2040_hal::prelude::*;
//! let mut pac = rp2040_pac::Peripherals::take().unwrap();
//! let pin_num = 0;
//! let mut pwm_pin = Pwm0::new(pin_num);
//! ```
//!
//! Once you have the PWM pins struct, you can take individual pins and configure them:
//!
//! ```rust
//! pwm_pin.default_config(&mut pac.PWM, &mut pac.PAD_BANK0, &mut pac.IO_BANK0, &mut pac.RESETS);
//! pwm_pin.set_duty(32767);
//! pwm_pin.enable();
//! ```
//!
//! The following configuration options are also available:
//!
//! ```rust
//! pwm_pin.min_config(&pac.PWM, &pac.PAD_BANK0, &pac.IO_BANK0, &mut pac.RESETS);
//!
//! pwm_pin.get_duty();
//! pwm_pin.get_max_duty();
//!
//! pwm_pin.set_ph_correct().unwrap(); // Run in phase correct mode
//! pwm_pin.clr_ph_correct().unwrap(); // Don't run in phase correct mode
//!
//! pwm_pin.set_div_int(value: u8).unwrap(); // To set integer part of clock divider
//! pwm_pin.set_div_frac(value: u8).unwrap(); // To set fractional part of clock divider
//!
//! pwm_pin.set_inv().unwrap(); // Invert the output
//! pwm_pin.clr_inv().unwrap(); // Don't invert the output
//!
//! pwm_pin.set_top(value: u16).unwrap(); // To set the TOP register 
//!
//! pwm_pin.divmode_div().unwrap(); // Default divmode. Counts up at a rate dictated by div.
//! pwm_pin.divmode_level().unwrap(); // These 3 divmodes can be used with a PWM B pin to read PWM inputs.
//! pwm_pin.divmode_rise().unwrap();
//! pwm_pin.divmode_fall().unwrap();
//! ```
//!
//! default_config() sets ph_correct to false, the clock divider to 1, does not invert the output, sets top to 65535, and resets the counter.
//! min_config() leaves those registers in the state they were before it was called (Careful, this can lead to unexpected behavior)
//! It's recommended to only call min_config() after calling default_config() on a pin that shares a PWM block.

use embedded_hal::PwmPin;
use crate::resets::SubsystemReset;
use super::*;

macro_rules! pwm {
    ($PWMX:ident, $pwmx:ident, [
    $($PXi:ident: ($pxi:ident, $pwms:expr, $pins:expr, $CCI:ident, $cci:ident, $CSRI:ident, $csri:ident, $CTRI:ident, $ctri:ident, $DIVI:ident, $divi:ident, $TOPI:ident, $topi:ident),)+]) => {
        $(


#[doc = "Struct for any of the "]
#[doc = $pwms]
#[doc = " pins"]
pub struct $PXi {
    pin: usize
}

impl $PXi {
    #[doc = "Constructor for a PWM pin struct"]
    pub fn new (pin: usize) -> Self {
        let mut pin_num : usize = 255;
        for i in 0..$pins.len() {
            if (pin == $pins[i]) {
                pin_num = pin;
            }
        }

        if (pin_num == 255) {
            pin_num = $pins[0];
        }

        Self {
            pin: pin_num,
        }
    }

    // TODO: This function contains all the methods that required the PWM to have an instance of PADS_BANK0, RESETS, or IO_BANK0.
    // Since the GPIO pins take ownership of these, after the GPIO refactor, this method should be moved into gpio.rs, and the PWM
    // will instead receive a single gpio pin.
    fn init_io(&self, pwm: &mut pac::$PWMX, pad : &mut pac::PADS_BANK0, io : &mut pac::IO_BANK0, resets: &mut pac::RESETS) -> () {
        //TODO: Merge these into gpio.rs split function after GPIO refactor. At the moment, this is here because these need to be reset for
        // the PWM to work. However, because they're here, they'll be reset every time a new PWM pin is created (which is bad). Definitely needs to change once GPIO is redone.
        pwm.reset_bring_up(resets);
        io.reset_bring_up(resets);

        pad.gpio[self.pin].write(|w| w.ie().set_bit());
        pad.gpio[self.pin].write(|w| w.od().clear_bit());
        io.gpio[self.pin].gpio_ctrl.write_with_zero(|w| w.funcsel().pwm_a_0());
    }

    fn cc(&self) -> &pac::$pwmx::$CCI {
        unsafe {
            &(*pac::$PWMX::ptr()).$cci
        }
    }

    fn csr(&self) -> &pac::$pwmx::$CSRI {
        unsafe {
            &(*pac::$PWMX::ptr()).$csri
        }
    }

    fn ctr(&self) -> &pac::$pwmx::$CTRI {
        unsafe {
            &(*pac::$PWMX::ptr()).$ctri
        }
    }

    fn div(&self) -> &pac::$pwmx::$DIVI {
        unsafe {
            &(*pac::$PWMX::ptr()).$divi
        }
    }

    fn top(&self) -> &pac::$pwmx::$TOPI {
        unsafe {
            &(*pac::$PWMX::ptr()).$topi
        }
    }

    #[doc = "Sets up a pin with the default configurations"]
    pub fn default_config(&mut self, pwm: &mut pac::$PWMX, pad: &mut pac::PADS_BANK0, io: &mut pac::IO_BANK0, resets: &mut pac::RESETS) -> () {
        self.init_io(pwm, pad, io, resets);

        self.clr_ph_correct();
        self.set_div_int(1);
        self.set_div_frac(0);
        self.divmode_div();
        self.set_top(0xffffu16);
        self.ctr().write(|w| unsafe { w.$ctri().bits(0x0000u16) }); //Reset the counter

        self.set_duty(0); //Default duty cycle of 0%
        self.clr_inv(); //Don't invert the channel
    }

    #[doc = "Sets up a pin with minimum configurations"]
    pub fn min_config(&mut self, pwm: &mut pac::$PWMX, pad: &mut pac::PADS_BANK0, io: &mut pac::IO_BANK0, resets: &mut pac::RESETS) -> () {
        self.init_io(pwm, pad, io, resets);
    }

    #[doc = "Enables phase correct mode"]
    pub fn set_ph_correct(&self) {
        self.csr().write(|w| w.ph_correct().set_bit());
    }

    #[doc = "Disales phase correct mode"]
    pub fn clr_ph_correct(&self) {
        self.csr().write(|w| w.ph_correct().clear_bit());
    }

    #[doc = "Sets the integer part of the clock divider"]
    pub fn set_div_int(&self, value: u8) {
        self.div().write(|w| unsafe { w.int().bits(value) });
    }

    #[doc = "Sets the fractional part of the clock divider"]
    pub fn set_div_frac(&self, value: u8) {
        self.div().write(|w| unsafe { w.frac().bits(value) });
    }

    #[doc = "Enables output inversion"]
    pub fn set_inv(&self) {
        if (self.pin % 2 == 0) {
            self.csr().write(|w| w.a_inv().set_bit());
        } else {
            self.csr().write(|w| w.b_inv().set_bit());
        }
    }

    #[doc = "Disables output inversion"]
    pub fn clr_inv(&self) {
        if (self.pin % 2 == 0) {
            self.csr().write(|w| w.a_inv().clear_bit());
        } else {
            self.csr().write(|w| w.b_inv().clear_bit());
        }
    }

    #[doc = "Sets the top register value"]
    pub fn set_top(&self, value: u16) {
        self.top().write(|w| unsafe { w.$topi().bits(value) });
    }

    #[doc = "Sets the divmode to div. Use this if you aren't reading a PWM input."]
    pub fn divmode_div(&self) {
        self.csr().write(|w| w.divmode().div());
    }

    #[doc = "Sets the divmode to level."]
    pub fn divmode_level(&self) {
        self.csr().write(|w| w.divmode().level());
    }

    #[doc = "Sets the divmode to rise."]
    pub fn divmode_rise(&self) {
        self.csr().write(|w| w.divmode().rise());
    }

    #[doc = "Sets the divmode to fall."]
    pub fn divmode_fall(&self) {
        self.csr().write(|w| w.divmode().div());
    }
}

impl PwmPin for $PXi {
    type Duty = u16;

    fn disable(&mut self) -> () {
        self.csr().write(|w| w.en().clear_bit());
    }

    fn enable(&mut self) -> () {
        self.csr().write(|w| w.en().set_bit());
    }

    fn get_duty(&self) -> Self::Duty {
        if (self.pin % 2 == 0) {
            self.cc().read().a().bits()
        } else {
            self.cc().read().b().bits()
        }
    }

    fn get_max_duty(&self) -> Self::Duty {
        self.top().read().$topi().bits()
    }

    fn set_duty(&mut self, duty: Self::Duty) {
        if (self.pin % 2 == 0) {
            self.cc().write(|w| unsafe { w.a().bits(duty) });
        } else {
            self.cc().write(|w| unsafe { w.b().bits(duty) });
        }
    }
}

)+}}

// This is stupidly long because rust's concat_idents feature is only available in nightly builds.
// TODO: See if this can be condensed by creating an array of idents somehow? (no idea if that's possible)
pwm! {
    PWM, pwm, [
        Pwm0: (pwm0, "pwm0", [0, 1, 16, 18], CH0_CC, ch0_cc, CH0_CSR, ch0_csr, CH0_CTR, ch0_ctr, CH0_DIV, ch0_div, CH0_TOP, ch0_top),
        Pwm1: (pwm1, "pwm1", [2, 3, 18, 19], CH1_CC, ch1_cc, CH1_CSR, ch1_csr, CH1_CTR, ch1_ctr, CH1_DIV, ch1_div, CH1_TOP, ch1_top),
        Pwm2: (pwm2, "pwm2", [4, 5, 20, 21], CH2_CC, ch2_cc, CH2_CSR, ch2_csr, CH2_CTR, ch2_ctr, CH2_DIV, ch2_div, CH2_TOP, ch2_top),
        Pwm3: (pwm3, "pwm3", [6, 7, 22], CH3_CC, ch3_cc, CH3_CSR, ch3_csr, CH3_CTR, ch3_ctr, CH3_DIV, ch3_div, CH3_TOP, ch3_top),
        Pwm4: (pwm4, "pwm4", [8, 9], CH4_CC, ch4_cc, CH4_CSR, ch4_csr, CH4_CTR, ch4_ctr, CH4_DIV, ch4_div, CH4_TOP, ch4_top),
        Pwm5: (pwm5, "pwm5", [10, 11, 26, 27], CH5_CC, ch5_cc, CH5_CSR, ch5_csr, CH5_CTR, ch5_ctr, CH5_DIV, ch5_div, CH5_TOP, ch5_top),
        Pwm6: (pwm6, "pwm6", [12, 13, 28], CH6_CC, ch6_cc, CH6_CSR, ch6_csr, CH6_CTR, ch6_ctr, CH6_DIV, ch6_div, CH6_TOP, ch6_top),
        Pwm7: (pwm7, "pwm7", [14, 15], CH7_CC, ch7_cc, CH7_CSR, ch7_csr, CH7_CTR, ch7_ctr, CH7_DIV, ch7_div, CH7_TOP, ch7_top),
    ]
}

