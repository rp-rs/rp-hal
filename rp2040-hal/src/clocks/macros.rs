#[macro_export]
/// Create clock structs
macro_rules! clock {
    {
        $(#[$attr:meta])*
        ($name:ident, $reg:ident, src=[$($src:ident),*], auxsrc=[$($auxsrc:ident),*])
     } => {
        $crate::clock!{ @base
            $(#[$attr])*
            ($name, $reg, auxsrc=[$($auxsrc),*])
        }
        $crate::clock!{ @division
            $(#[$attr])*
            ($name, $reg)
        }

        $crate::paste::paste!{
            impl GlitchlessClock for $name {
                type Clock = Self;

                fn await_select(&self, clock_token: &ChangingClockToken<Self>) -> nb::Result<(),()> {
                    let shared_dev = unsafe { self.shared_dev.get() };

                    let selected = shared_dev.[<$reg _selected>].read().bits();
                    if selected != 1 << clock_token.clock_nr {
                        return Err(nb::Error::WouldBlock);
                    }

                    Ok(())
                }
            }

            // Clock glitchless sources
            $([<$src _source>]!($name, [<$reg _ctrl>]);)*


            impl $name {
                /// WIP - Helper function to reset source (blocking)
                pub fn reset_source_await(&mut self) -> nb::Result<(), ()> {
                    let shared_dev = unsafe { self.shared_dev.get() };

                    shared_dev.[<$reg _ctrl>].modify(|_,w| {
                        w.src().variant(self.get_default_clock_source())
                    });

                    self.await_select(&ChangingClockToken{clock_nr:0, clock: PhantomData::<Self>})
                }
            }
        }
    };
    {
        $( #[$attr:meta])*
        ($name:ident, $reg:ident, auxsrc=[$($auxsrc:ident),*], nodiv)
    } => {
        $crate::clock!{ @base
            $(#[$attr])*
            ($name, $reg, auxsrc=[$($auxsrc),*])
        }

        $crate::paste::paste!{
            impl ClockGenerator for $name {
                fn enable(&mut self) {
                    unsafe { self.shared_dev.get() }.[<$reg _ctrl>].modify(|_, w| {
                        w.enable().set_bit()
                    });
                }

                fn disable(&mut self) {
                    unsafe { self.shared_dev.get() }.[<$reg _ctrl>].modify(|_, w| {
                        w.enable().clear_bit()
                    });
                }

                fn kill(&mut self) {
                    unsafe { self.shared_dev.get() }.[<$reg _ctrl>].modify(|_, w| {
                        w.kill().set_bit()
                    });
                }
            }
        }
    };
    {
        $( #[$attr:meta])*
        ($name:ident, $reg:ident, auxsrc=[$($auxsrc:ident),*])
    } => {
        $crate::clock!{ @base
            $(#[$attr])*
            ($name, $reg, auxsrc=[$($auxsrc),*])
        }

        $crate::clock!{ @division
            $(#[$attr])*
            ($name, $reg)
        }

        $crate::paste::paste!{
            impl ClockGenerator for $name {
                fn enable(&mut self) {
                    unsafe { self.shared_dev.get() }.[<$reg _ctrl>].modify(|_, w| {
                        w.enable().set_bit()
                    });
                }

                fn disable(&mut self) {
                    unsafe { self.shared_dev.get() }.[<$reg _ctrl>].modify(|_, w| {
                        w.enable().clear_bit()
                    });
                }

                fn kill(&mut self) {
                    unsafe { self.shared_dev.get() }.[<$reg _ctrl>].modify(|_, w| {
                        w.kill().set_bit()
                    });
                }
            }
        }
    };
    {@base
        $(#[$attr:meta])*
        ($name:ident, $reg:ident, auxsrc=[$($auxsrc:ident),*])

    } => {
        $crate::paste::paste!{
            impl ClocksManager {
                    #[ doc = "Getter for the" $name ]
                    pub fn [<$name:snake>](&self) -> $name {

                        //TODO: Init clock here
                        $name {
                            shared_dev: self.shared_clocks,
                            frequency: 0.Hz(),
                        }
                    }

            }
            $(#[$attr])*
            pub struct $name {
                shared_dev: ShareableClocks,
                frequency: Hertz,
            }

            impl $name {
                /// Returns the frequency of the configured clock
                pub fn freq(&self) -> Hertz {
                    self.frequency
                }
            }
            impl Into<Hertz> for $name {
                fn into(self) -> Hertz {
                    self.frequency
                }
            }

            // Clock aux sources
            $([<$auxsrc _auxsource>]!($name, [<$reg _ctrl>]);)*
        }
    };
    {@division
        $(#[$attr:meta])*
        ($name:ident, $reg:ident, nodiv)

    } => {};
    {@division
        $(#[$attr:meta])*
        ($name:ident, $reg:ident)

    } => {
        $crate::paste::paste!{
            impl ClockDivision for $name {
                fn set_div(&mut self, div: u32) {
                    unsafe { self.shared_dev.get() }.[<$reg _div>].modify(|_, w| unsafe {
                        w.bits(div);
                        w
                    });
                }
                fn get_div(&self) -> u32 {
                    unsafe { self.shared_dev.get() }.[<$reg _div>].read().bits()
                }
            }
        }
    };
}

macro_rules! xosc_source {
    ($name:ident, $ctrl:ident) => {
        impl XOSCClockSource<$name> for $name {
            fn set_xosc_src(&mut self) -> ChangingClockToken<$name> {
                unsafe { self.shared_dev.get() }
                    .$ctrl
                    .modify(|_, w| w.src().xosc_clksrc());

                ChangingClockToken {
                    clock: PhantomData::<$name>,
                    clock_nr: pac::clocks::clk_ref_ctrl::SRC_A::XOSC_CLKSRC as u8,
                }
            }
        }
    };
}

macro_rules! rosc_source {
    ($name:ident, $ctrl:ident) => {
        impl ROSCClockSource<$name> for $name {
            fn set_rosc_src(&mut self) -> ChangingClockToken<$name> {
                unsafe { self.shared_dev.get() }
                    .$ctrl
                    .modify(|_, w| w.src().rosc_clksrc_ph());

                ChangingClockToken {
                    clock: PhantomData::<$name>,
                    clock_nr: pac::clocks::clk_ref_ctrl::SRC_A::ROSC_CLKSRC_PH as u8,
                }
            }
        }
    };
}

macro_rules! selfaux_source {
    ($name:ident, $ctrl:ident) => {
        $crate::paste::paste! {
            impl SelfAuxClockSource<$name> for $name {
                fn set_self_aux_src(&mut self) -> ChangingClockToken<$name> {
                    unsafe { self.shared_dev.get() }.$ctrl.modify(|_, w| {
                        w.src().variant(self.get_aux_source())
                    });

                    ChangingClockToken{
                        clock: PhantomData::<$name>,
                        clock_nr: pac::clocks::clk_ref_ctrl::SRC_A::CLKSRC_CLK_REF_AUX as u8,
                    }
                }
            }
        }
    };
}

macro_rules! clockref_source {
    ($name:ident, $ctrl:ident) => {
        impl ClockREFClockSource<$name> for $name {
            fn set_clkref_src(&mut self) -> ChangingClockToken<$name> {
                unsafe { self.shared_dev.get() }
                    .$ctrl
                    .modify(|_, w| w.src().clk_ref());
                ChangingClockToken {
                    clock: PhantomData::<$name>,
                    clock_nr: pac::clocks::clk_sys_ctrl::SRC_A::CLK_REF as u8,
                }
            }
        }
    };
}

macro_rules! clocksys_auxsource {
    ($name:ident, $ctrl:ident) => {
        impl ClockSYSClockAuxSource for $name {
            fn set_clksys_auxsrc(&mut self) {
                unsafe { self.shared_dev.get() }
                    .$ctrl
                    .modify(|_, w| w.auxsrc().clk_sys());
            }
        }
    };
}

macro_rules! clockusb_auxsource {
    ($name:ident, $ctrl:ident) => {
        impl ClockUSBClockAuxSource for $name {
            fn set_clkusb_auxsrc(&mut self) {
                unsafe { self.shared_dev.get() }
                    .$ctrl
                    .modify(|_, w| w.auxsrc().clk_usb());
            }
        }
    };
}

macro_rules! clockadc_auxsource {
    ($name:ident, $ctrl:ident) => {
        impl ClockADCClockAuxSource for $name {
            fn set_clkadc_auxsrc(&mut self) {
                unsafe { self.shared_dev.get() }
                    .$ctrl
                    .modify(|_, w| w.auxsrc().clk_adc());
            }
        }
    };
}

macro_rules! clockrtc_auxsource {
    ($name:ident, $ctrl:ident) => {
        impl ClockRTCClockAuxSource for $name {
            fn set_clkrtc_auxsrc(&mut self) {
                unsafe { self.shared_dev.get() }
                    .$ctrl
                    .modify(|_, w| w.auxsrc().clk_rtc());
            }
        }
    };
}

macro_rules! clockref_auxsource {
    ($name:ident, $ctrl:ident) => {
        impl ClockRefClockAuxSource for $name {
            fn set_clkref_auxsrc(&mut self) {
                unsafe { self.shared_dev.get() }
                    .$ctrl
                    .modify(|_, w| w.auxsrc().clk_ref());
            }
        }
    };
}

macro_rules! xosc_auxsource {
    ($name:ident, $ctrl:ident) => {
        impl XOSCClockAuxSource for $name {
            fn set_xosc_auxsrc(&mut self) {
                unsafe { self.shared_dev.get() }
                    .$ctrl
                    .modify(|_, w| w.auxsrc().xosc_clksrc());
            }
        }
    };
}

macro_rules! rosc_auxsource {
    ($name:ident, $ctrl:ident) => {
        impl ROSCClockAuxSource for $name {
            fn set_rosc_auxsrc(&mut self) {
                unsafe { self.shared_dev.get() }
                    .$ctrl
                    .modify(|_, w| w.auxsrc().rosc_clksrc());
            }
        }
    };
}

macro_rules! rosc_ph_auxsource {
    ($name:ident, $ctrl:ident) => {
        impl ROSCPHClockAuxSource for $name {
            fn set_rosc_ph_auxsrc(&mut self) {
                unsafe { self.shared_dev.get() }
                    .$ctrl
                    .modify(|_, w| w.auxsrc().rosc_clksrc_ph());
            }
        }
    };
}

macro_rules! gpin0_auxsource {
    ($name:ident, $ctrl:ident) => {
        impl Gpin0ClockAuxSource for $name {
            fn set_gpin0_auxsrc(&mut self) {
                unsafe { self.shared_dev.get() }
                    .$ctrl
                    .modify(|_, w| w.auxsrc().clksrc_gpin0());
            }
        }
    };
}

macro_rules! gpin1_auxsource {
    ($name:ident, $ctrl:ident) => {
        impl Gpin1ClockAuxSource for $name {
            fn set_gpin1_auxsrc(&mut self) {
                unsafe { self.shared_dev.get() }
                    .$ctrl
                    .modify(|_, w| w.auxsrc().clksrc_gpin1());
            }
        }
    };
}

macro_rules! pll_usb_auxsource {
    ($name:ident, $ctrl:ident) => {
        impl PLLUSBClockAuxSource for $name {
            fn set_pll_usb_auxsrc(&mut self) {
                unsafe { self.shared_dev.get() }
                    .$ctrl
                    .modify(|_, w| w.auxsrc().clksrc_pll_usb());
            }
        }
    };
}

macro_rules! pll_sys_auxsource {
    ($name:ident, $ctrl:ident) => {
        impl PLLSYSClockAuxSource for $name {
            fn set_pll_sys_auxsrc(&mut self) {
                unsafe { self.shared_dev.get() }
                    .$ctrl
                    .modify(|_, w| w.auxsrc().clksrc_pll_sys());
            }
        }
    };
}
