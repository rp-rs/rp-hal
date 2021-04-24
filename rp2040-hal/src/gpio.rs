//! General Purpose Input and Output (GPIO)
//!
//! TODO: Docs for this whole module
//!
//! For example, to turn on the LED on a Pico board:
//!
//! ```rust
//! let mut pac = rp2040_pac::Peripherals::take().unwrap();
//!
//! let pins = pac.IO_BANK0.split(pac.PADS_BANK0, pac.SIO, &mut pac.RESETS);
//! let mut led_pin = pins.gpio25.into_output();
//! led_pin.set_high().unwrap();
//! ```
pub struct Input;
pub struct Output;
pub struct Unknown;

pub trait GpioExt<PADS, SIO> {
    type Parts;

    // TODO: Do we need a marker to check that clocks are up?
    fn split(self, pads: PADS, sio: SIO, reset: &mut rp2040_pac::RESETS) -> Self::Parts;
}

// Magic numbers from the datasheet
// const FUNCTION_SPI: u8 = 1;
// const FUNCTION_UART: u8 = 2;
// const FUNCTION_I2C: u8 = 3;
// const FUNCTION_PWM: u8 = 4;
const FUNCTION_SIO: u8 = 5;
// const FUNCTION_PIO0: u8 = 6;
// const FUNCTION_PIO1: u8 = 7;
// const FUNCTION_CLOCK: u8 = 8;
// const FUNCTION_USB: u8 = 9;

macro_rules! gpio {
    ($GPIOX:ident, $gpiox:ident, $PADSX:ident, $padsx:ident, [
        $($PXi:ident: ($pxi:ident, $i:expr),)+
    ]) => {
        mod $gpiox {
            use core::convert::Infallible;
            use core::marker::PhantomData;
            use embedded_hal::digital::v2::{InputPin, OutputPin, StatefulOutputPin};
            use super::{GpioExt, Input, Output, Unknown, FUNCTION_SIO};

            impl GpioExt<pac::$PADSX, pac::SIO> for pac::$GPIOX {
                type Parts = Parts;

                fn split(self, pads: pac::$PADSX, sio: pac::SIO, resets: &mut pac::RESETS) -> Parts {
                    resets.reset.modify(|_, w| w.$gpiox().clear_bit());
                    Parts {
                        _pads: pads,
                        _sio: sio,
                        $(
                            $pxi: $PXi { _mode: PhantomData },
                        )+
                    }
                }
            }

            pub struct Parts {
                _pads: pac::$PADSX,
                _sio: pac::SIO,
                $(
                    pub $pxi: $PXi<Unknown>,
                )+
            }

            // Puts pad in default state as far as this crate is concerned:
            // - Input is enabled
            // - Output is enabled (if also set in SIO)
            // - Pull up/down is disabled
            //
            // TODO: Drive strength, smitty, slewing
            fn setup_pad_io(pads: &pac::$padsx::RegisterBlock, index: usize) {
                pads.gpio[index].modify(|_, w| w.ie().set_bit().od().clear_bit().pue().clear_bit().pde().clear_bit());
            }

            fn set_gpio_function(gpios: &pac::$gpiox::RegisterBlock, index: usize, function: u8) {
                gpios.gpio[index]
                    .gpio_ctrl
                    .write_with_zero(|x| unsafe { x.funcsel().bits(function) });
            }

            $(
                pub struct $PXi<MODE> {
                    _mode: PhantomData<MODE>,
                }

                // Safety: We own our $i slice of padsx, gpiox, and sio because the
                // construction of Parts assumes ownership of all 3 and will not release
                // them. Thus several of the methods below will reconstruct these objects
                // as-needed

                impl<MODE> $PXi<MODE> {
                    pub fn into_output(
                        self,
                    ) -> $PXi<Output> {
                        unsafe {
                            setup_pad_io(&*pac::$PADSX::ptr(), $i);
                        }
                        unsafe {
                            set_gpio_function(&*pac::$GPIOX::ptr(), $i, FUNCTION_SIO);
                        }
                        unsafe {
                            (*pac::SIO::ptr()).gpio_oe_set.write(|x| { x.bits(1 << $i) });
                        }
                        $PXi { _mode: PhantomData }
                    }

                    pub fn into_input(
                        self,
                    ) -> $PXi<Input> {
                        unsafe {
                            setup_pad_io(&*pac::$PADSX::ptr(), $i);
                        }
                        unsafe {
                            set_gpio_function(&*pac::$GPIOX::ptr(), $i, FUNCTION_SIO);
                        }
                        unsafe {
                            (*pac::SIO::ptr()).gpio_oe_clr.write(|x| { x.bits(1 << $i) });
                        }
                        $PXi { _mode: PhantomData }
                    }
                }

                impl OutputPin for $PXi<Output> {
                    type Error = Infallible;

                    fn set_low(&mut self) -> Result<(), Self::Error> {
                        unsafe {
                            (*pac::SIO::ptr()).gpio_out_clr.write(|x| x.bits(1 << $i));
                        }
                        Ok(())
                    }

                    fn set_high(&mut self) -> Result<(), Self::Error> {
                        unsafe {
                            (*pac::SIO::ptr()).gpio_out_set.write(|x| x.bits(1 << $i));
                        }
                        Ok(())
                    }
                }

                impl StatefulOutputPin for $PXi<Output> {
                    fn is_set_low(&self) -> Result<bool, Self::Error> {
                        Ok(!self.is_set_high()?)
                    }

                    fn is_set_high(&self) -> Result<bool, Self::Error> {
                        unsafe {
                            Ok((*pac::SIO::ptr()).gpio_out_set.read().bits() & (1 << $i) != 0)
                        }
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
                                unsafe {
                                    Ok((*pac::SIO::ptr()).gpio_in.read().bits() & (1 << $i) != 0)
                                }
                            }
                        }
                    };
                }
                // Not allowed for Unknown since we don't know what state the pad is in
                impl_input_for!(Input);
                impl_input_for!(Output);

                impl $PXi<Input> {
                    pub fn pull_high(&mut self) {
                        unsafe {
                            (*pac::$PADSX::ptr()).gpio[$i].modify(|_, w| w.pue().set_bit().pde().clear_bit());
                        }
                    }

                    pub fn pull_low(&mut self) {
                        unsafe {
                            (*pac::$PADSX::ptr()).gpio[$i].modify(|_, w| w.pue().clear_bit().pde().set_bit());
                        }
                    }
                }
            )+
        }
    };
}

gpio!(
    IO_BANK0, io_bank0, PADS_BANK0, pads_bank0, [
        Gpio0: (gpio0, 0),
        Gpio1: (gpio1, 1),
        Gpio2: (gpio2, 2),
        Gpio3: (gpio3, 3),
        Gpio4: (gpio4, 4),
        Gpio5: (gpio5, 5),
        Gpio6: (gpio6, 6),
        Gpio7: (gpio7, 7),
        Gpio8: (gpio8, 8),
        Gpio9: (gpio9, 9),
        Gpio10: (gpio10, 10),
        Gpio11: (gpio11, 11),
        Gpio12: (gpio12, 12),
        Gpio13: (gpio13, 13),
        Gpio14: (gpio14, 14),
        Gpio15: (gpio15, 15),
        Gpio16: (gpio16, 16),
        Gpio17: (gpio17, 17),
        Gpio18: (gpio18, 18),
        Gpio19: (gpio19, 19),
        Gpio20: (gpio20, 20),
        Gpio21: (gpio21, 21),
        Gpio22: (gpio22, 22),
        Gpio23: (gpio23, 23),
        Gpio24: (gpio24, 24),
        Gpio25: (gpio25, 25),
        Gpio26: (gpio26, 26),
        Gpio27: (gpio27, 27),
        Gpio28: (gpio28, 28),
        Gpio29: (gpio29, 29),
    ]
);
