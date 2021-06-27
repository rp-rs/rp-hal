#[macro_use]

macro_rules! int_division {
    ($name:ident, $div:ident, $u:ty) => {
        impl IntegerDivision for $name {
            fn set_int_div(&mut self, div: usize) {
                unsafe { self.shared_dev.get() }.$div.modify(|_, w| unsafe {
                    w.int().bits(div as $u);
                    w
                });
            }
        }
    };
}

macro_rules! frac_division {
    ($name:ident, $div:ident, $u:ty) => {
        impl FractionDivision for $name {
            fn set_frac_div(&mut self, div: usize) {
                unsafe { self.shared_dev.get() }.$div.modify(|_, w| unsafe {
                    w.frac().bits(div as $u);
                    w
                });
            }
        }
    };
}

macro_rules! clock_generator {
    ($name:ident, $ctrl:ident) => {
        impl ClockGenerator for $name {
            fn enable(&mut self) {
                unsafe { self.shared_dev.get() }.$ctrl.modify(|_, w| {
                    w.enable().set_bit();
                    w
                });
            }

            fn disable(&mut self) {
                unsafe { self.shared_dev.get() }.$ctrl.modify(|_, w| {
                    w.enable().clear_bit();
                    w
                });
            }

            fn kill(&mut self) {
                unsafe { self.shared_dev.get() }.$ctrl.modify(|_, w| {
                    w.kill().set_bit();
                    w
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
                    w.src().xosc_clksrc();
                    w
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
                    w.src().rosc_clksrc_ph();
                    w
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
                    w.src().$self();
                    w
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
                    w.src().clk_ref();
                    w
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
                    w.auxsrc().clk_sys();
                    w
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
                    w.auxsrc().xosc_clksrc();
                    w
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
                    w.auxsrc().rosc_clksrc();
                    w
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
                    w.auxsrc().rosc_clksrc_ph();
                    w
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
                    w.auxsrc().clksrc_gpin0();
                    w
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
                    w.auxsrc().clksrc_gpin1();
                    w
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
                    w.auxsrc().clksrc_pll_usb();
                    w
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
                    w.auxsrc().clksrc_pll_sys();
                    w
                });
            }
        }
    };
}
