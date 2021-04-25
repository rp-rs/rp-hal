//! General Purpose Input and Output (GPIO)
//!
//! To access the GPIO pins you must call the `split` method on the IO bank. This will return a
//! `Parts` struct with access to the individual pins:
//!
//! ```rust
//! use rp2040_hal::prelude::*;
//! let mut pac = rp2040_pac::Peripherals::take().unwrap();
//! let pins = pac.IO_BANK0.split(pac.PADS_BANK0, pac.SIO, &mut pac.RESETS);
//! ```
//!
//! Once you have the GPIO pins struct, you can take individual pins and configure them:
//!
//! ```rust
//! let mut led_pin = pins.gpio25.into_output();
//! led_pin.set_high().unwrap();
//! ```
//!
//! Input pins support the following options:
//! - Pull high, pull low, or floating
//! - Schmitt trigger
//!
//! Output pins support the following options:
//! - Slew rate (fast or slow)
//! - Drive strength (2, 4, 8 or 12 mA)

/// Mode marker for an input pin
pub struct Input;
/// Mode marker for an output pin
pub struct Output;
/// Mode marker for a pin in an unknown state (generally happens at startup)
pub struct Unknown;

/// This trait adds a method to extract pins from an IO bank and convert them into HAL objects
pub trait GpioExt<PADS, SIO> {
    /// The type of struct that will hold the pins once they're converted to HAL objects
    type Parts;

    /// Convert the IO bank into a struct of HAL pin objects
    // TODO: Do we need a marker to check that clocks are up?
    fn split(self, pads: PADS, sio: SIO, reset: &mut rp2040_pac::RESETS) -> Self::Parts;
}

#[derive(Clone, Copy, Eq, PartialEq, Debug)]
/// The amount of current that a pin can drive when used as an output
pub enum OutputDriveStrength {
    /// 2 mA
    TwoMilliAmps,
    /// 4 mA
    FourMilliAmps,
    /// 8 mA
    EightMilliAmps,
    /// 12 mA
    TwelveMilliAmps,
}

#[derive(Clone, Copy, Eq, PartialEq, Debug)]
/// The slew rate of a pin when used as an output
pub enum OutputSlewRate {
    /// Slew slow
    Slow,
    /// Slew fast
    Fast,
}

macro_rules! gpio {
    ($GPIOX:ident, $gpiox:ident, $PADSX:ident, $padsx:ident, $gpioxs:expr, [
        $($PXi:ident: ($pxi:ident, $i:expr, $is:expr),)+
    ]) => {
        #[doc = "HAL objects for the "]
        #[doc = $gpioxs]
        #[doc = " bank of GPIO pins"]
        pub mod $gpiox {
            use core::convert::Infallible;
            use core::marker::PhantomData;
            use embedded_hal::digital::v2::{InputPin, OutputPin, StatefulOutputPin};
            use super::*;

            impl GpioExt<pac::$PADSX, pac::SIO> for pac::$GPIOX {
                type Parts = Parts;

                fn split(self, pads: pac::$PADSX, sio: pac::SIO, resets: &mut pac::RESETS) -> Parts {
                    resets.reset.modify(|_, w| w.$gpiox().clear_bit().$padsx().clear_bit());
                    while resets.reset_done.read().$gpiox().bit_is_clear() {
                        cortex_m::asm::delay(10);
                    }
                    while resets.reset_done.read().$padsx().bit_is_clear() {
                        cortex_m::asm::delay(10);
                    }
                    Parts {
                        _pads: pads,
                        _sio: sio,
                        $(
                            $pxi: $PXi { _mode: PhantomData },
                        )+
                    }
                }
            }

            #[doc = "Struct containing HAL objects for all the "]
            #[doc = $gpioxs]
            #[doc = " pins"]
            pub struct Parts {
                _pads: pac::$PADSX,
                _sio: pac::SIO,
                $(
                    #[doc = "GPIO pin "]
                    #[doc = $is]
                    pub $pxi: $PXi<Unknown>,
                )+
            }

            type PacDriveStrength = pac::$padsx::gpio::DRIVE_A;

            $(
                #[doc = "HAL object for GPIO pin "]
                #[doc = $is]
                pub struct $PXi<MODE> {
                    _mode: PhantomData<MODE>,
                }

                impl<MODE> $PXi<MODE> {
                    // This is safe because Parts owns the pads, and each pin is responsible
                    // for its own pad
                    fn pad(&self) -> &pac::$padsx::GPIO {
                        unsafe {
                            &(*pac::$PADSX::ptr()).gpio[$i]
                        }
                    }

                    // This is safe because Parts owns the SIO. But callers must only touch their
                    // own pin
                    fn sio(&self) -> &pac::sio::RegisterBlock {
                        unsafe {
                            &(*pac::SIO::ptr())
                        }
                    }

                    // This is safe because Parts owns the bank, and each pin is responsible
                    // for its own slice of the bank
                    fn gpio_ctrl(&self) -> &pac::$gpiox::gpio::GPIO_CTRL {
                        unsafe {
                            &(*pac::$GPIOX::ptr()).gpio[$i].gpio_ctrl
                        }
                    }

                    #[doc = "Configure this pin as an output"]
                    pub fn into_output(self)-> $PXi<Output> {
                        self.pad().reset();
                        self.gpio_ctrl().write_with_zero(|x| { x.funcsel().sio_0() });
                        // TODO: Can we update the PAC to give us a safe register field
                        //       instead of `bits`?
                        self.sio().gpio_oe_set.write(|x| unsafe { x.bits(1 << $i) });
                        $PXi { _mode: PhantomData }
                    }

                    #[doc = "Configure this pin as an input"]
                    pub fn into_input(self) -> $PXi<Input> {
                        self.pad().reset();
                        self.gpio_ctrl().write_with_zero(|x| { x.funcsel().sio_0() });
                        self.sio().gpio_oe_clr.write(|x| unsafe { x.bits(1 << $i) });

                        $PXi { _mode: PhantomData }
                    }
                }

                impl OutputPin for $PXi<Output> {
                    type Error = Infallible;

                    fn set_low(&mut self) -> Result<(), Self::Error> {
                        self.sio().gpio_out_clr.write(|x| unsafe { x.bits(1 << $i) });
                        Ok(())
                    }

                    fn set_high(&mut self) -> Result<(), Self::Error> {
                        self.sio().gpio_out_set.write(|x| unsafe { x.bits(1 << $i) });
                        Ok(())
                    }
                }

                impl StatefulOutputPin for $PXi<Output> {
                    fn is_set_low(&self) -> Result<bool, Self::Error> {
                        Ok(!self.is_set_high()?)
                    }

                    fn is_set_high(&self) -> Result<bool, Self::Error> {
                        Ok(self.sio().gpio_out.read().bits() & (1 << $i) != 0)
                    }
                }

                macro_rules! impl_input_for {
                    ($MODE:ident) => {
                        impl InputPin for $PXi<$MODE> {
                            type Error = Infallible;

                            fn is_low(&self) -> Result<bool, Self::Error> {
                                Ok(!self.is_high()?)
                            }

                            fn is_high(&self) -> Result<bool, Self::Error> {
                                Ok(self.sio().gpio_in.read().bits() & (1 << $i) != 0)
                            }
                        }
                    };
                }
                // Not allowed for Unknown since we don't know what state the pad is in
                impl_input_for!(Input);
                impl_input_for!(Output);

                impl $PXi<Output> {
                    #[doc = "Configure the drive strength for this output pin"]
                    pub fn drive_strength(self, strength: OutputDriveStrength) -> Self {
                        let converted = match strength {
                            OutputDriveStrength::TwoMilliAmps => PacDriveStrength::_2MA,
                            OutputDriveStrength::FourMilliAmps => PacDriveStrength::_4MA,
                            OutputDriveStrength::EightMilliAmps => PacDriveStrength::_8MA,
                            OutputDriveStrength::TwelveMilliAmps => PacDriveStrength::_12MA,
                        };
                        self.pad().modify(|_, w| w.drive().variant(converted));
                        self
                    }

                    #[doc = "Configure the slew rate for this output pin"]
                    pub fn slew_rate(self, slew_rate: OutputSlewRate) -> Self {
                        self.pad().modify(|_, w| w.slewfast().bit(slew_rate == OutputSlewRate::Fast));
                        self
                    }
                }

                impl $PXi<Input> {
                    #[doc = "Pull this input pin high using internal resistors"]
                    pub fn pull_up(self) -> Self {
                        self.pad().modify(|_, w| w.pue().set_bit().pde().clear_bit());
                        self
                    }

                    #[doc = "Pull this input pin low using internal resistors"]
                    pub fn pull_down(self) -> Self {
                        self.pad().modify(|_, w| w.pue().clear_bit().pde().set_bit());
                        self
                    }

                    #[doc = "Allow this input pin to float (i.e. don't pull it high or low)"]
                    pub fn float(self) -> Self {
                        self.pad().modify(|_, w| w.pue().clear_bit().pde().clear_bit());
                        self
                    }

                    #[doc = "Enable the schmitt trigger for this input pin"]
                    pub fn enable_schmitt_trigger(self) -> Self {
                        self.pad().modify(|_, w| w.schmitt().set_bit());
                        self
                    }

                    #[doc = "Disable the schmitt trigger for this input pin"]
                    pub fn disable_schmitt_trigger(self) -> Self {
                        self.pad().modify(|_, w| w.schmitt().clear_bit());
                        self
                    }
                }
            )+
        }
    };
}

gpio!(
    IO_BANK0, io_bank0, PADS_BANK0, pads_bank0, "IO_BANK0", [
        Gpio0: (gpio0, 0, "0"),
        Gpio1: (gpio1, 1, "1"),
        Gpio2: (gpio2, 2, "2"),
        Gpio3: (gpio3, 3, "3"),
        Gpio4: (gpio4, 4, "4"),
        Gpio5: (gpio5, 5, "5"),
        Gpio6: (gpio6, 6, "6"),
        Gpio7: (gpio7, 7, "7"),
        Gpio8: (gpio8, 8, "8"),
        Gpio9: (gpio9, 9, "9"),
        Gpio10: (gpio10, 10, "10"),
        Gpio11: (gpio11, 11, "11"),
        Gpio12: (gpio12, 12, "12"),
        Gpio13: (gpio13, 13, "13"),
        Gpio14: (gpio14, 14, "14"),
        Gpio15: (gpio15, 15, "15"),
        Gpio16: (gpio16, 16, "16"),
        Gpio17: (gpio17, 17, "17"),
        Gpio18: (gpio18, 18, "18"),
        Gpio19: (gpio19, 19, "19"),
        Gpio20: (gpio20, 20, "20"),
        Gpio21: (gpio21, 21, "21"),
        Gpio22: (gpio22, 22, "22"),
        Gpio23: (gpio23, 23, "23"),
        Gpio24: (gpio24, 24, "24"),
        Gpio25: (gpio25, 25, "25"),
        Gpio26: (gpio26, 26, "26"),
        Gpio27: (gpio27, 27, "27"),
        Gpio28: (gpio28, 28, "28"),
        Gpio29: (gpio29, 29, "29"),
    ]
);
