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

use super::*;
use crate::resets::SubsystemReset;
use embedded_hal::PwmPin;

macro_rules! pwm {
    ($PWMX:ident, $pwmx:ident, [
    $($PXi:ident: ($pxi:ident, $pwms:expr, $pins:expr, $i:expr),)+]) => {
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
        // the PWM to work. However, because they're here, they'll be reset every time a new PWM pin is created (BAD).
        pwm.reset_bring_up(resets);
        io.reset_bring_up(resets);

        pad.gpio[self.pin].write(|w| w.ie().set_bit());
        pad.gpio[self.pin].write(|w| w.od().clear_bit());
        unsafe {
            io.gpio[self.pin].gpio_ctrl.write_with_zero(|w| w.funcsel().pwm_a_0());
        }
    }

    fn cc(&self) -> &pac::$pwmx::ch::CC {
        unsafe {
            &(*pac::$PWMX::ptr()).ch[$i].cc
        }
    }

    fn csr(&self) -> &pac::$pwmx::ch::CSR {
        unsafe {
            &(*pac::$PWMX::ptr()).ch[$i].csr
        }
    }

    fn ctr(&self) -> &pac::$pwmx::ch::CTR {
        unsafe {
            &(*pac::$PWMX::ptr()).ch[$i].ctr
        }
    }

    fn div(&self) -> &pac::$pwmx::ch::DIV {
        unsafe {
            &(*pac::$PWMX::ptr()).ch[$i].div
        }
    }

    fn top(&self) -> &pac::$pwmx::ch::TOP {
        unsafe {
            &(*pac::$PWMX::ptr()).ch[$i].top
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
        self.ctr().write(|w| unsafe { w.ctr().bits(0x0000u16) }); //Reset the counter

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
        self.top().write(|w| unsafe { w.top().bits(value) });
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
        self.top().read().top().bits()
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

pwm! {
    PWM, pwm, [
        Pwm0: (pwm0, "pwm0", [0, 1, 16, 18], 0),
        Pwm1: (pwm1, "pwm1", [2, 3, 18, 19], 1),
        Pwm2: (pwm2, "pwm2", [4, 5, 20, 21], 2),
        Pwm3: (pwm3, "pwm3", [6, 7, 22, 23], 3),
        Pwm4: (pwm4, "pwm4", [8, 9, 24, 25], 4),
        Pwm5: (pwm5, "pwm5", [10, 11, 26, 27], 5),
        Pwm6: (pwm6, "pwm6", [12, 13, 28, 29], 6),
        Pwm7: (pwm7, "pwm7", [14, 15], 7),
    ]
}
