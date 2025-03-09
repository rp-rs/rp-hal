use super::{DynBankId, DynPinId};
use crate::{pac, sio::CoreId};

pub trait TypeLevelPinId: super::PinId {
    type Bank: super::BankId;

    const ID: DynPinId;

    fn new() -> Self;
}

pub trait PinIdOps {
    fn mask(&self) -> u32;
    fn io_status(&self) -> &pac::io_bank0::gpio::GPIO_STATUS;
    fn io_ctrl(&self) -> &pac::io_bank0::gpio::GPIO_CTRL;
    fn pad_ctrl(&self) -> &pac::pads_bank0::GPIO;

    fn sio_in(&self) -> &pac::sio::GPIO_IN;
    fn sio_out(&self) -> &pac::sio::GPIO_OUT;
    fn sio_out_set(&self) -> &pac::sio::GPIO_OUT_SET;
    fn sio_out_clr(&self) -> &pac::sio::GPIO_OUT_CLR;
    fn sio_out_xor(&self) -> &pac::sio::GPIO_OUT_XOR;
    fn sio_oe(&self) -> &pac::sio::GPIO_OE;
    fn sio_oe_set(&self) -> &pac::sio::GPIO_OE_SET;
    fn sio_oe_clr(&self) -> &pac::sio::GPIO_OE_CLR;
    fn sio_oe_xor(&self) -> &pac::sio::GPIO_OE_XOR;
    fn proc_in_by_pass(&self) -> &pac::syscfg::PROC_IN_SYNC_BYPASS;

    fn intr(&self) -> (&pac::io_bank0::INTR, usize);
    // These function signatures look as if they would return the PROC0 registers
    // even on core 1, which would be wrong.
    // But that's not true: If `proc` is `CoreId::Core1`, they return the
    // PROC1_* registers transmuted to PROC0_*. This works, because both
    // sets of registers are structured identically.
    fn proc_ints(&self, proc: CoreId) -> (&pac::io_bank0::PROC0_INTS, usize);
    fn proc_inte(&self, proc: CoreId) -> (&pac::io_bank0::PROC0_INTE, usize);
    fn proc_intf(&self, proc: CoreId) -> (&pac::io_bank0::PROC0_INTF, usize);
    fn dormant_wake_ints(&self) -> (&pac::io_bank0::DORMANT_WAKE_INTS, usize);
    fn dormant_wake_inte(&self) -> (&pac::io_bank0::DORMANT_WAKE_INTE, usize);
    fn dormant_wake_intf(&self) -> (&pac::io_bank0::DORMANT_WAKE_INTF, usize);
}

macro_rules! accessor_fns {
    (sio $reg:ident) => {
        paste::paste! {
            fn [<sio_ $reg:lower>](&self) -> &$crate::pac::sio::[<GPIO_ $reg:upper>] {
                let pin = self.as_dyn();
                unsafe {
                    let sio = &*$crate::pac::SIO::PTR;
                    match pin.bank {
                        DynBankId::Bank0 => sio.[<gpio_ $reg:lower>](),
                        DynBankId::Qspi => core::mem::transmute::<&$crate::pac::sio::[<GPIO_HI_ $reg:upper>],&$crate::pac::sio::[<GPIO_ $reg:upper>]>(sio.[<gpio_hi_ $reg:lower>]()),
                    }
                }
            }
        }
    };
    (io $reg:ident) => {
        paste::paste! {
            fn [<io_ $reg:lower>](&self) -> &$crate::pac::io_bank0::gpio::[<GPIO_ $reg:upper>] {
                let pin = self.as_dyn();
                match pin.bank {
                    DynBankId::Bank0 => {
                        let gpio = unsafe { &*$crate::pac::IO_BANK0::PTR };
                        gpio.gpio(usize::from(pin.num)).[<gpio_ $reg:lower>]()
                    }
                    DynBankId::Qspi => unsafe {
                        let qspi = &*$crate::pac::IO_QSPI::PTR;
                        core::mem::transmute::<&$crate::pac::io_qspi::gpio_qspi::[<GPIO_ $reg:upper>], &$crate::pac::io_bank0::gpio::[<GPIO_ $reg:upper>]>(qspi.gpio_qspi(usize::from(pin.num)).[<gpio_ $reg:lower>]())
                    },
                }
            }
        }
    };
    (int $reg:ident) => {
        paste::paste! {
            fn [<proc_ $reg:lower>](&self, proc: CoreId) -> (&$crate::pac::io_bank0::[<PROC0_ $reg:upper>], usize) {
                let pin = self.as_dyn();
                let (index, offset) = (pin.num / 8, pin.num % 8 * 4);
                unsafe {
                    let reg = match pin.bank {
                        DynBankId::Bank0 => {
                            let bank = &*$crate::pac::IO_BANK0::PTR;
                            match proc {
                                CoreId::Core0 => bank.[<proc0_ $reg:lower>](usize::from(index)),
                                CoreId::Core1 => core::mem::transmute::<&$crate::pac::io_bank0::[<PROC1_ $reg:upper>], &$crate::pac::io_bank0::[<PROC0_ $reg:upper>]>(bank.[<proc1_ $reg:lower>](usize::from(index))),
                            }
                        }
                        DynBankId::Qspi => {
                            let bank = &*$crate::pac::IO_QSPI::PTR;
                            match proc {
                                CoreId::Core0 => core::mem::transmute::<&$crate::pac::io_qspi::[<PROC0_ $reg:upper>], &$crate::pac::io_bank0::[<PROC0_ $reg:upper>]>(bank.[<proc0_ $reg:lower>]()),
                                CoreId::Core1 => core::mem::transmute::<&$crate::pac::io_qspi::[<PROC1_ $reg:upper>], &$crate::pac::io_bank0::[<PROC0_ $reg:upper>]>(bank.[<proc1_ $reg:lower>]()),
                            }
                        }
                    };
                    (reg, usize::from(offset))
                }
            }
        }
    };
    (dormant $reg:ident) => {
        paste::paste! {
            fn [< dormant_wake_ $reg:lower>](&self) -> (&$crate::pac::io_bank0::[< DORMANT_WAKE_ $reg:upper >], usize) {
                let pin = self.as_dyn();
                let (index, offset) = (pin.num / 8, pin.num % 8 * 4);
                unsafe {
                    let reg = match pin.bank {
                        DynBankId::Bank0 => {
                            let bank = &*$crate::pac::IO_BANK0::PTR;
                            bank.[< dormant_wake_ $reg:lower>](usize::from(index))
                        }
                        DynBankId::Qspi => {
                            let bank = &*$crate::pac::IO_QSPI::PTR;
                            core::mem::transmute::<&$crate::pac::io_qspi::[< DORMANT_WAKE_ $reg:upper >], &$crate::pac::io_bank0::[< DORMANT_WAKE_ $reg:upper >]>(bank.[< dormant_wake_ $reg:lower>]())
                        }
                    };
                    (reg, usize::from(offset))
                }
            }
        }
    };
}
impl<T> PinIdOps for T
where
    T: super::PinId,
{
    fn mask(&self) -> u32 {
        1 << self.as_dyn().num
    }
    fn pad_ctrl(&self) -> &pac::pads_bank0::GPIO {
        let pin = self.as_dyn();
        match pin.bank {
            DynBankId::Bank0 => {
                let gpio = unsafe { &*pac::PADS_BANK0::PTR };
                gpio.gpio(usize::from(pin.num))
            }
            DynBankId::Qspi => unsafe {
                let qspi = &*pac::PADS_QSPI::PTR;
                use rp2040_pac::{generic::Reg, pads_bank0, pads_qspi};
                match pin.num {
                    0 => core::mem::transmute::<
                        &Reg<pads_qspi::gpio_qspi_sclk::GPIO_QSPI_SCLK_SPEC>,
                        &Reg<pads_bank0::gpio::GPIO_SPEC>,
                    >(qspi.gpio_qspi_sclk()),
                    1 => core::mem::transmute::<
                        &Reg<pads_qspi::gpio_qspi_ss::GPIO_QSPI_SS_SPEC>,
                        &Reg<pads_bank0::gpio::GPIO_SPEC>,
                    >(qspi.gpio_qspi_ss()),
                    2 => core::mem::transmute::<
                        &Reg<pads_qspi::gpio_qspi_sd0::GPIO_QSPI_SD0_SPEC>,
                        &Reg<pads_bank0::gpio::GPIO_SPEC>,
                    >(qspi.gpio_qspi_sd0()),
                    3 => core::mem::transmute::<
                        &Reg<pads_qspi::gpio_qspi_sd1::GPIO_QSPI_SD1_SPEC>,
                        &Reg<pads_bank0::gpio::GPIO_SPEC>,
                    >(qspi.gpio_qspi_sd1()),
                    4 => core::mem::transmute::<
                        &Reg<pads_qspi::gpio_qspi_sd2::GPIO_QSPI_SD2_SPEC>,
                        &Reg<pads_bank0::gpio::GPIO_SPEC>,
                    >(qspi.gpio_qspi_sd2()),
                    5 => core::mem::transmute::<
                        &Reg<pads_qspi::gpio_qspi_sd3::GPIO_QSPI_SD3_SPEC>,
                        &Reg<pads_bank0::gpio::GPIO_SPEC>,
                    >(qspi.gpio_qspi_sd3()),
                    _ => unreachable!("Invalid QSPI bank pin number."),
                }
            },
        }
    }
    accessor_fns!(io ctrl);
    accessor_fns!(io status);

    accessor_fns!(sio in);
    accessor_fns!(sio out);
    accessor_fns!(sio out_set);
    accessor_fns!(sio out_clr);
    accessor_fns!(sio out_xor);
    accessor_fns!(sio oe);
    accessor_fns!(sio oe_set);
    accessor_fns!(sio oe_clr);
    accessor_fns!(sio oe_xor);

    fn proc_in_by_pass(&self) -> &pac::syscfg::PROC_IN_SYNC_BYPASS {
        let pin = self.as_dyn();
        unsafe {
            let syscfg = &*pac::SYSCFG::PTR;
            match pin.bank {
                DynBankId::Bank0 => syscfg.proc_in_sync_bypass(),
                DynBankId::Qspi => core::mem::transmute::<
                    &pac::syscfg::PROC_IN_SYNC_BYPASS_HI,
                    &pac::syscfg::PROC_IN_SYNC_BYPASS,
                >(syscfg.proc_in_sync_bypass_hi()),
            }
        }
    }

    fn intr(&self) -> (&pac::io_bank0::INTR, usize) {
        let pin = self.as_dyn();
        let (index, offset) = (pin.num / 8, pin.num % 8 * 4);
        unsafe {
            let reg = match pin.bank {
                DynBankId::Bank0 => {
                    let bank = &*pac::IO_BANK0::PTR;
                    bank.intr(usize::from(index))
                }
                DynBankId::Qspi => {
                    let bank = &*pac::IO_QSPI::PTR;
                    core::mem::transmute::<&pac::io_qspi::INTR, &pac::io_bank0::INTR>(bank.intr())
                }
            };

            (reg, usize::from(offset))
        }
    }

    accessor_fns!(int ints);
    accessor_fns!(int inte);
    accessor_fns!(int intf);

    accessor_fns!(dormant ints);
    accessor_fns!(dormant inte);
    accessor_fns!(dormant intf);
}
