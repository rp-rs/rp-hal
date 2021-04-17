//! General Purpose Input and Output (GPIO)
//!
//! TODO: Docs for this whole module
//!
//! For example, to turn on the LED on a Pico board:
//!
//! ```rust
//! let mut pac = rp2040_pac::Peripherals::take().unwrap();
//!
//! let pins = pac.IO_BANK0.split(&mut pac.RESETS);
//! let mut led_pin = pins.gpio25.into_output(&mut pac.PADS_BANK0, &mut pac.SIO);
//!  led_pin.set_low().unwrap();
//! ```
use core::convert::Infallible;
use core::marker::PhantomData;
use embedded_hal::digital::v2::OutputPin;

pub struct Floating;
pub struct Input<MODE> {
    _mode: PhantomData<MODE>,
}
pub struct Output;

pub trait GpioExt {
    type Parts;

    // TODO: Do we need a marker to check that clocks are up?
    fn split(self, reset: &mut rp2040_pac::RESETS) -> Self::Parts;
}

const FUNCTION_SIO: u8 = 5;

macro_rules! gpio {
    ($GPIOX:ident, $gpiox:ident, $PADSX:ident, [
        $($PXi:ident: ($pxi:ident, $i:expr),)+
    ]) => {
        impl GpioExt for pac::$GPIOX {
            type Parts = Parts;

            fn split(self, resets: &mut pac::RESETS) -> Parts {
                resets.reset.modify(|_, w| w.$gpiox().clear_bit());
                Parts {
                    $(
                        $pxi: $PXi { _mode: PhantomData },
                    )+
                }
            }
        }

        pub struct Parts {
            $(
                pub $pxi: $PXi<Input<Floating>>,
            )+
        }

        $(
            pub struct $PXi<MODE> {
                _mode: PhantomData<MODE>,
            }

            impl<MODE> $PXi<MODE> {
                pub fn into_output(
                    self,
                    pads: &mut pac::$PADSX,
                    sio: &mut pac::SIO,
                ) -> $PXi<Output> {
                    pads.gpio[$i].modify(|_, w| w.ie().set_bit().od().clear_bit());
                    unsafe {
                        (*pac::$GPIOX::ptr()).gpio[$i]
                            .gpio_ctrl
                            .write_with_zero(|x| x.funcsel().bits(FUNCTION_SIO));
                    }
                    sio.gpio_oe.modify(|_, x| unsafe { x.bits(1 << $i) });
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
        )+
    };
}

gpio!(
    IO_BANK0, io_bank0, PADS_BANK0, [
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
