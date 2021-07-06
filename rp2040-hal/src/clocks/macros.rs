macro_rules! int_division {
    ($name:ident, $div:ident, $u:ty) => {
        impl IntegerDivision for $name {
            fn set_int_div(&mut self, div: usize) {
                unsafe { self.shared_dev.get() }.$div.modify(|_, w| unsafe {
                    w.int().bits(div as $u)
                });
            }

            fn get_int_div(&self) -> usize {
                unsafe { self.shared_dev.get() }.$div.read().int().bits() as usize
            }
        }
    };
}

macro_rules! frac_division {
    ($name:ident, $div:ident, $u:ty) => {
        impl FractionDivision for $name {
            fn set_frac_div(&mut self, div: usize) {
                unsafe { self.shared_dev.get() }.$div.modify(|_, w| unsafe {
                    w.frac().bits(div as $u)
                });
            }

            fn get_frac_div(&self) -> usize {
                unsafe { self.shared_dev.get() }
                    .$div
                    .read()
                    .frac()
                    .bits()
                    .into()
            }
        }
    };
}

macro_rules! division {
    ($name:ident, $div:ident) => {
        impl ClockDivision for $name {
            fn set_div(&mut self, div: u32) {
                unsafe { self.shared_dev.get() }.$div.modify(|_, w| unsafe {
                    w.bits(div)
                });
            }

            fn get_div(&self) -> u32 {
                unsafe { self.shared_dev.get() }.$div.read().bits()
            }
        }
    };
}

macro_rules! clock_generator {
    ($name:ident, $ctrl:ident) => {
        impl ClockGenerator for $name {
            fn enable(&mut self) {
                unsafe { self.shared_dev.get() }.$ctrl.modify(|_, w| {
                    w.enable().set_bit()
                });
            }

            fn disable(&mut self) {
                unsafe { self.shared_dev.get() }.$ctrl.modify(|_, w| {
                    w.enable().clear_bit()
                });
            }

            fn kill(&mut self) {
                unsafe { self.shared_dev.get() }.$ctrl.modify(|_, w| {
                    w.kill().set_bit()
                });
            }
        }
    };
}

macro_rules! xosc_source {
    ($name:ident, $ctrl:ident) => {
        impl XOSCClockSource for $name {
            fn set_xosc_src(&mut self) {
                unsafe { self.shared_dev.get() }.$ctrl.modify(|_, w| {
                    w.src().xosc_clksrc()
                });
            }
        }
    };
}

macro_rules! rosc_source {
    ($name:ident, $ctrl:ident) => {
        impl ROSCClockSource for $name {
            fn set_rosc_src(&mut self) {
                unsafe { self.shared_dev.get() }.$ctrl.modify(|_, w| {
                    w.src().rosc_clksrc_ph()
                });
            }
        }
    };
}

macro_rules! selfaux_source {
    ($name:ident, $ctrl:ident, $self:ident) => {
        impl SelfAuxClockSource for $name {
            fn set_self_aux_src(&mut self) {
                unsafe { self.shared_dev.get() }.$ctrl.modify(|_, w| {
                    w.src().$self()
                });
            }
        }
    };
}

macro_rules! clockref_source {
    ($name:ident, $ctrl:ident) => {
        impl ClockREFClockSource for $name {
            fn set_clkref_src(&mut self) {
                unsafe { self.shared_dev.get() }.$ctrl.modify(|_, w| {
                    w.src().clk_ref()
                });
            }
        }
    };
}

macro_rules! clocksys_auxsource {
    ($name:ident, $ctrl:ident) => {
        impl ClockSYSClockAuxSource for $name {
            fn set_clksys_auxsrc(&mut self) {
                unsafe { self.shared_dev.get() }.$ctrl.modify(|_, w| {
                    w.auxsrc().clk_sys()
                });
            }
        }
    };
}

macro_rules! clockusb_auxsource {
    ($name:ident, $ctrl:ident) => {
        impl ClockUSBClockAuxSource for $name {
            fn set_clkusb_auxsrc(&mut self) {
                unsafe { self.shared_dev.get() }.$ctrl.modify(|_, w| {
                    w.auxsrc().clk_usb()
                });
            }
        }
    };
}

macro_rules! clockadc_auxsource {
    ($name:ident, $ctrl:ident) => {
        impl ClockADCClockAuxSource for $name {
            fn set_clkadc_auxsrc(&mut self) {
                unsafe { self.shared_dev.get() }.$ctrl.modify(|_, w| {
                    w.auxsrc().clk_adc()
                });
            }
        }
    };
}

macro_rules! clockrtc_auxsource {
    ($name:ident, $ctrl:ident) => {
        impl ClockRTCClockAuxSource for $name {
            fn set_clkrtc_auxsrc(&mut self) {
                unsafe { self.shared_dev.get() }.$ctrl.modify(|_, w| {
                    w.auxsrc().clk_rtc()
                });
            }
        }
    };
}

macro_rules! clockref_auxsource {
    ($name:ident, $ctrl:ident) => {
        impl ClockRefClockAuxSource for $name {
            fn set_clkref_auxsrc(&mut self) {
                unsafe { self.shared_dev.get() }.$ctrl.modify(|_, w| {
                    w.auxsrc().clk_ref()
                });
            }
        }
    };
}

macro_rules! xosc_auxsource {
    ($name:ident, $ctrl:ident) => {
        impl XOSCClockAuxSource for $name {
            fn set_xosc_auxsrc(&mut self) {
                unsafe { self.shared_dev.get() }.$ctrl.modify(|_, w| {
                    w.auxsrc().xosc_clksrc()
                });
            }
        }
    };
}

macro_rules! rosc_auxsource {
    ($name:ident, $ctrl:ident) => {
        impl ROSCClockAuxSource for $name {
            fn set_rosc_auxsrc(&mut self) {
                unsafe { self.shared_dev.get() }.$ctrl.modify(|_, w| {
                    w.auxsrc().rosc_clksrc()
                });
            }
        }
    };
}

macro_rules! rosc_ph_auxsource {
    ($name:ident, $ctrl:ident) => {
        impl ROSCPHClockAuxSource for $name {
            fn set_rosc_ph_auxsrc(&mut self) {
                unsafe { self.shared_dev.get() }.$ctrl.modify(|_, w| {
                    w.auxsrc().rosc_clksrc_ph()
                });
            }
        }
    };
}

macro_rules! gpin0_auxsource {
    ($name:ident, $ctrl:ident) => {
        impl Gpin0ClockAuxSource for $name {
            fn set_gpin0_auxsrc(&mut self) {
                unsafe { self.shared_dev.get() }.$ctrl.modify(|_, w| {
                    w.auxsrc().clksrc_gpin0()
                });
            }
        }
    };
}

macro_rules! gpin1_auxsource {
    ($name:ident, $ctrl:ident) => {
        impl Gpin1ClockAuxSource for $name {
            fn set_gpin1_auxsrc(&mut self) {
                unsafe { self.shared_dev.get() }.$ctrl.modify(|_, w| {
                    w.auxsrc().clksrc_gpin1()
                });
            }
        }
    };
}

macro_rules! pll_usb_auxsource {
    ($name:ident, $ctrl:ident) => {
        impl PLLUSBClockAuxSource for $name {
            fn set_pll_usb_auxsrc(&mut self) {
                unsafe { self.shared_dev.get() }.$ctrl.modify(|_, w| {
                    w.auxsrc().clksrc_pll_usb()
                });
            }
        }
    };
}

macro_rules! pll_sys_auxsource {
    ($name:ident, $ctrl:ident) => {
        impl PLLSYSClockAuxSource for $name {
            fn set_pll_sys_auxsrc(&mut self) {
                unsafe { self.shared_dev.get() }.$ctrl.modify(|_, w| {
                    w.auxsrc().clksrc_pll_sys()
                });
            }
        }
    };
}
