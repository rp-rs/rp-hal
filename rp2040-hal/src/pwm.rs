//! Pulse Width Modulation (PWM)
//!
//! To access the PWM pins you must call the 'split' method on the PWM. This will return a
//! `_____` struct with access to each PWM pin:
//!
//! ```rust
//! use rp2040_hal::prelude::*;
//! let mut pac = rp2040_pac::Peripherals::take().unwrap();
//! let pins = pac.PWM.split(pac.PADS_BANK0, pac.IO_BANK0, &mut pac.RESETS);
//! ```
//!
//! Once you have the PWM pins struct, you can take individual pins and configure them:
//!
//! ```rust
//! let mut pwm_pin = pin.pwm0.default_config();
//! pwm_pin.set_duty(32767);
//! pwm_pin.enable();
//! ```
//!
//! The following configuration options are also available:
//!
//! ```rust
//! pin.pwm0.min_config(); // Doesn't change config settings that could be shared between pins
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

#[doc = "Trait to be implemented on the HAL, containing methods to extract PWM pins"]
pub trait PwmExt<PADS, IO> {
    #[doc = "The type that PWM pins are stored in"]
    type Parts;

    #[doc = "Method that extracts PWM pins into HAL objects"]
    fn split(self, pads: PADS, io: IO, reset: &mut rp2040_pac::RESETS) -> Self::Parts;
}

macro_rules! pwm {
    ($PWMX:ident, $pwmx:ident, $GPIOX:ident, $gpiox:ident, $PADSX:ident, $padsx:ident, $pwmxs:expr, [
        $($PXi:ident: ($pxi:ident, $CCI:ident, $cci:ident, $CSRI:ident, $csri:ident, $CTRI:ident, $ctri:ident, $DIVI:ident, $divi:ident, $TOPI:ident, $topi:ident, $i:expr, $is:expr),)+
    ]) => {
        #[doc = "HAL objects for the "]
        #[doc = $pwmxs]
        #[doc = " bank of PWM pins"]
        pub mod $pwmx {
            use core::convert::Infallible;
            use embedded_hal::PwmPin;
            use super::*;

            use crate::resets::SubsystemReset;

            impl PwmExt<pac::$PADSX, pac::$GPIOX> for pac::$PWMX {
                type Parts = Parts;

                fn split(self, pads: pac::$PADSX, io: pac::$GPIOX, resets: &mut pac::RESETS) -> Parts {
                    self.reset_bring_up(resets);
                    io.reset_bring_up(resets);

                    Parts {
                        _pads: pads,
                        _io: io,
                        $(
                            $pxi: $PXi {},
                        )+
                    }
                }
            }

            #[doc = "Struct containing HAL objects for all the "]
            #[doc = $pwmxs]
            #[doc = " pins"]
            pub struct Parts {
                _pads: pac::$PADSX,
                _io: pac::$GPIOX,
                $(
                    #[doc = "PWM pin "]
                    #[doc = $is]
                    pub $pxi: $PXi,
                )+
            }

            $(
                #[doc = "HAL object for PWM pin "]
                #[doc = $is]
                pub struct $PXi {}

                impl $PXi {
                    fn pad(&self) -> &pac::$padsx::GPIO {
                        unsafe {
                            &(*pac::$PADSX::ptr()).gpio[$i]
                        }
                    }

                    fn gpio_ctrl(&self) -> &pac::$gpiox::gpio::GPIO_CTRL {
                        unsafe {
                            &(*pac::$GPIOX::ptr()).gpio[$i].gpio_ctrl
                        }
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
                    pub fn default_config(self) -> $PXi {
                        self.csr().write(|w| w.ph_correct().clear_bit()); //Disable ph_correct
                        self.div().write(|w| unsafe { w.int().bits(0x01u8) }); //Set clock divider to 1
                        self.csr().write(|w| w.divmode().div()); //Sets counter to go at rate dictated by fractional divider (not gated)                        
                        self.top().write(|w| unsafe { w.$topi().bits(0xffffu16) }); //Set TOP register to max value
                        self.ctr().write(|w| unsafe { w.$ctri().bits(0x0000u16) }); //Reset the counter

                        if ($i % 2 == 0) {
                            self.cc().write(|w| unsafe { w.a().bits(0x0000u16) }); //Default duty cycle of 0%
                            self.csr().write(|w| w.a_inv().clear_bit()); //Don't invert the channel
                        } else {
                            self.cc().write(|w| unsafe { w.b().bits(0x0000u16) });
                            self.csr().write(|w| w.b_inv().clear_bit());
                        }

                        self.pad().write(|w| w.ie().set_bit());
                        self.pad().write(|w| w.od().clear_bit());
                        self.gpio_ctrl().write_with_zero(|w| w.funcsel().pwm_a_0());

                        $PXi {}
                    }

                    #[doc = "Sets up a pin with minimum configurations"]
                    pub fn min_config(self) -> $PXi {
                        self.pad().write(|w| w.ie().set_bit());
                        self.pad().write(|w| w.od().clear_bit());
                        self.gpio_ctrl().write_with_zero(|w| w.funcsel().pwm_a_0());

                        $PXi {}
                    }

                    #[doc = "Enables phase correct mode"]
                    pub fn set_ph_correct(&self) -> Result<(), Infallible> {
                        self.csr().write(|w| w.ph_correct().set_bit());
                        Ok(())
                    }

                    #[doc = "Disales phase correct mode"]
                    pub fn clr_ph_correct(&self) -> Result<(), Infallible> {
                        self.csr().write(|w| w.ph_correct().clear_bit());
                        Ok(())
                    }

                    #[doc = "Sets the integer part of the clock divider"]
                    pub fn set_div_int(&self, value: u8) -> Result<(), Infallible> {
                        self.div().write(|w| unsafe { w.int().bits(value) });
                        Ok(())
                    }

                    #[doc = "Sets the fractional part of the clock divider"]
                    pub fn set_div_frac(&self, value: u8) -> Result<(), Infallible> {
                        self.div().write(|w| unsafe { w.frac().bits(value) });
                        Ok(())
                    }

                    #[doc = "Enables output inversion"]
                    pub fn set_inv(&self) -> Result<(), Infallible> {
                        if ($i % 2 == 0) {
                            self.csr().write(|w| w.a_inv().set_bit());
                        } else {
                            self.csr().write(|w| w.b_inv().set_bit());
                        }
                        Ok(())
                    }

                    #[doc = "Disables output inversion"]
                    pub fn clr_inv(&self) -> Result<(), Infallible> {
                        if ($i % 2 == 0) {
                            self.csr().write(|w| w.a_inv().clear_bit());
                        } else {
                            self.csr().write(|w| w.b_inv().clear_bit());
                        }
                        Ok(())
                    }

                    #[doc = "Sets the top register value"]
                    pub fn set_top(&self, value: u16) -> Result<(), Infallible> {
                        self.top().write(|w| unsafe { w.$topi().bits(value) });
                        Ok(())
                    }

                    #[doc = "Sets the divmode to div. Use this if you aren't reading a PWM input."]
                    pub fn divmode_div(&self) -> Result<(), Infallible> {
                        self.csr().write(|w| w.divmode().div());
                        Ok(())
                    }

                    #[doc = "Sets the divmode to level."]
                    pub fn divmode_level(&self) -> Result<(), Infallible> {
                        self.csr().write(|w| w.divmode().level());
                        Ok(())
                    }

                    #[doc = "Sets the divmode to rise."]
                    pub fn divmode_rise(&self) -> Result<(), Infallible> {
                        self.csr().write(|w| w.divmode().rise());
                        Ok(())
                    }

                    #[doc = "Sets the divmode to fall."]
                    pub fn divmode_fall(&self) -> Result<(), Infallible> {
                        self.csr().write(|w| w.divmode().div());
                        Ok(())
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
                        if ($i % 2 == 0) {
                            self.cc().read().a().bits()
                        } else {
                            self.cc().read().b().bits()
                        }
                    }

                    fn get_max_duty(&self) -> Self::Duty {
                        self.top().read().$topi().bits()
                    }

                    fn set_duty(&mut self, duty: Self::Duty) {
                        if ($i % 2 == 0) {
                            self.cc().write(|w| unsafe { w.a().bits(duty) });
                        } else {
                            self.cc().write(|w| unsafe { w.b().bits(duty) });
                        }
                    }
                }
            )+
        }
    };
}

// This is stupidly long because rust's concat_idents feature is only available in nightly builds.
// TODO: See if this can be condensed by creating an array of idents somehow? (no idea if that's possible)
pwm!(
    PWM, pwm, IO_BANK0, io_bank0, PADS_BANK0, pads_bank0, "PWM", [
        Pwm0: (pwm0, CH0_CC, ch0_cc, CH0_CSR, ch0_csr, CH0_CTR, ch0_ctr, CH0_DIV, ch0_div, CH0_TOP, ch0_top, 0, "0"),
        Pwm1: (pwm1, CH0_CC, ch0_cc, CH0_CSR, ch0_csr, CH0_CTR, ch0_ctr, CH0_DIV, ch0_div, CH0_TOP, ch0_top, 1, "1"),
        Pwm2: (pwm2, CH1_CC, ch1_cc, CH1_CSR, ch1_csr, CH1_CTR, ch1_ctr, CH1_DIV, ch1_div, CH1_TOP, ch1_top, 2, "2"),
        Pwm3: (pwm3, CH1_CC, ch1_cc, CH1_CSR, ch1_csr, CH1_CTR, ch1_ctr, CH1_DIV, ch1_div, CH1_TOP, ch1_top, 3, "3"),
        Pwm4: (pwm4, CH2_CC, ch2_cc, CH2_CSR, ch2_csr, CH2_CTR, ch2_ctr, CH2_DIV, ch2_div, CH2_TOP, ch2_top, 4, "4"),
        Pwm5: (pwm5, CH2_CC, ch2_cc, CH2_CSR, ch2_csr, CH2_CTR, ch2_ctr, CH2_DIV, ch2_div, CH2_TOP, ch2_top, 5, "5"),
        Pwm6: (pwm6, CH3_CC, ch3_cc, CH3_CSR, ch3_csr, CH3_CTR, ch3_ctr, CH3_DIV, ch3_div, CH3_TOP, ch3_top, 6, "6"),
        Pwm7: (pwm7, CH3_CC, ch3_cc, CH3_CSR, ch3_csr, CH3_CTR, ch3_ctr, CH3_DIV, ch3_div, CH3_TOP, ch3_top, 7, "7"),
        Pwm8: (pwm8, CH4_CC, ch4_cc, CH4_CSR, ch4_csr, CH4_CTR, ch4_ctr, CH4_DIV, ch4_div, CH4_TOP, ch4_top, 8, "8"),
        Pwm9: (pwm9, CH4_CC, ch4_cc, CH4_CSR, ch4_csr, CH4_CTR, ch4_ctr, CH4_DIV, ch4_div, CH4_TOP, ch4_top, 9, "9"),
        Pwm10: (pwm10, CH5_CC, ch5_cc, CH5_CSR, ch5_csr, CH5_CTR, ch5_ctr, CH5_DIV, ch5_div, CH5_TOP, ch5_top, 10, "10"),
        Pwm11: (pwm11, CH5_CC, ch5_cc, CH5_CSR, ch5_csr, CH5_CTR, ch5_ctr, CH5_DIV, ch5_div, CH5_TOP, ch5_top, 11, "11"),
        Pwm12: (pwm12, CH6_CC, ch6_cc, CH6_CSR, ch6_csr, CH6_CTR, ch6_ctr, CH6_DIV, ch6_div, CH6_TOP, ch6_top, 12, "12"),
        Pwm13: (pwm13, CH6_CC, ch6_cc, CH6_CSR, ch6_csr, CH6_CTR, ch6_ctr, CH6_DIV, ch6_div, CH6_TOP, ch6_top, 13, "13"),
        Pwm14: (pwm14, CH7_CC, ch7_cc, CH7_CSR, ch7_csr, CH7_CTR, ch7_ctr, CH7_DIV, ch7_div, CH7_TOP, ch7_top, 14, "14"),
        Pwm15: (pwm15, CH7_CC, ch7_cc, CH7_CSR, ch7_csr, CH7_CTR, ch7_ctr, CH7_DIV, ch7_div, CH7_TOP, ch7_top, 15, "15"),
        Pwm16: (pwm16, CH0_CC, ch0_cc, CH0_CSR, ch0_csr, CH0_CTR, ch0_ctr, CH0_DIV, ch0_div, CH0_TOP, ch0_top, 16, "16"),
        Pwm17: (pwm17, CH0_CC, ch0_cc, CH0_CSR, ch0_csr, CH0_CTR, ch0_ctr, CH0_DIV, ch0_div, CH0_TOP, ch0_top, 17, "17"),
        Pwm18: (pwm18, CH1_CC, ch1_cc, CH1_CSR, ch1_csr, CH1_CTR, ch1_ctr, CH1_DIV, ch1_div, CH1_TOP, ch1_top, 18, "18"),
        Pwm19: (pwm19, CH1_CC, ch1_cc, CH1_CSR, ch1_csr, CH1_CTR, ch1_ctr, CH1_DIV, ch1_div, CH1_TOP, ch1_top, 19, "19"),
        Pwm20: (pwm20, CH2_CC, ch2_cc, CH2_CSR, ch2_csr, CH2_CTR, ch2_ctr, CH2_DIV, ch2_div, CH2_TOP, ch2_top, 20, "20"),
        Pwm21: (pwm21, CH2_CC, ch2_cc, CH2_CSR, ch2_csr, CH2_CTR, ch2_ctr, CH2_DIV, ch2_div, CH2_TOP, ch2_top, 21, "21"),
        Pwm22: (pwm22, CH3_CC, ch3_cc, CH3_CSR, ch3_csr, CH3_CTR, ch3_ctr, CH3_DIV, ch3_div, CH3_TOP, ch3_top, 22, "22"),
        Pwm26: (pwm26, CH5_CC, ch5_cc, CH5_CSR, ch5_csr, CH5_CTR, ch5_ctr, CH5_DIV, ch5_div, CH5_TOP, ch5_top, 26, "26"),
        Pwm27: (pwm27, CH5_CC, ch5_cc, CH5_CSR, ch5_csr, CH5_CTR, ch5_ctr, CH5_DIV, ch5_div, CH5_TOP, ch5_top, 27, "27"),
        Pwm28: (pwm28, CH6_CC, ch6_cc, CH6_CSR, ch6_csr, CH6_CTR, ch6_ctr, CH6_DIV, ch6_div, CH6_TOP, ch6_top, 28, "28"),
    ]
);
