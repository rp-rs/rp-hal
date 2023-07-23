use crate::sio::CoreId;

use super::{DynBankId, DynPinId};

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
            fn [<sio_ $reg:lower>](&self) -> &pac::sio::[<GPIO_ $reg:upper>] {
                let pin = self.as_dyn();
                unsafe {
                    let sio = &*pac::SIO::PTR;
                    match pin.bank {
                        DynBankId::Bank0 => &sio.[<gpio_ $reg:lower>],
                        DynBankId::Qspi => core::mem::transmute(&sio.[<gpio_hi_ $reg:lower>]),
                    }
                }
            }
        }
    };
    (io $reg:ident) => {
        paste::paste! {
            fn [<io_ $reg:lower>](&self) -> &pac::io_bank0::gpio::[<GPIO_ $reg:upper>] {
                let pin = self.as_dyn();
                match pin.bank {
                    DynBankId::Bank0 => {
                        let gpio = unsafe { &*pac::IO_BANK0::PTR };
                        &gpio.gpio[usize::from(pin.num)].[<gpio_ $reg:lower>]
                    }
                    DynBankId::Qspi => unsafe {
                        let qspi = &*pac::IO_QSPI::PTR;
                        match pin.num {
                            0 => core::mem::transmute(&qspi.gpio_qspisclk().[<gpio_ $reg:lower>]),
                            1 => core::mem::transmute(&qspi.gpio_qspiss().[<gpio_ $reg:lower>]),
                            2 => core::mem::transmute(&qspi.gpio_qspisd0().[<gpio_ $reg:lower>]),
                            3 => core::mem::transmute(&qspi.gpio_qspisd1().[<gpio_ $reg:lower>]),
                            4 => core::mem::transmute(&qspi.gpio_qspisd2().[<gpio_ $reg:lower>]),
                            5 => core::mem::transmute(&qspi.gpio_qspisd3().[<gpio_ $reg:lower>]),
                            _ => unreachable!("Invalid QSPI bank pin number."),
                        }
                    },
                }
            }
        }
    };
    (int $reg:ident) => {
        paste::paste! {
            fn [<proc_ $reg:lower>](&self, proc: CoreId) -> (&pac::io_bank0::[<PROC0_ $reg:upper>], usize) {
                let pin = self.as_dyn();
                let (index, offset) = (pin.num / 8, pin.num % 8 * 4);
                unsafe {
                    let reg = match pin.bank {
                        DynBankId::Bank0 => {
                            let bank = &*pac::IO_BANK0::PTR;
                            match proc {
                                CoreId::Core0 => &bank.[<proc0_ $reg:lower>][usize::from(index)],
                                CoreId::Core1 => core::mem::transmute(&bank.[<proc1_ $reg:lower>][usize::from(index)]),
                            }
                        }
                        DynBankId::Qspi => {
                            let bank = &*pac::IO_QSPI::PTR;
                            match proc {
                                CoreId::Core0 => core::mem::transmute(&bank.[<proc0_ $reg:lower>]),
                                CoreId::Core1 => core::mem::transmute(&bank.[<proc1_ $reg:lower>]),
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
            fn [< dormant_wake_ $reg:lower>](&self) -> (&pac::io_bank0::[< DORMANT_WAKE_ $reg:upper >], usize) {
                let pin = self.as_dyn();
                let (index, offset) = (pin.num / 8, pin.num % 8 * 4);
                unsafe {
                    let reg = match pin.bank {
                        DynBankId::Bank0 => {
                            let bank = &*pac::IO_BANK0::PTR;
                            &bank.[< dormant_wake_ $reg:lower>][usize::from(index)]
                        }
                        DynBankId::Qspi => {
                            let bank = &*pac::IO_QSPI::PTR;
                            core::mem::transmute(&bank.[< dormant_wake_ $reg:lower>])
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
                &gpio.gpio[usize::from(pin.num)]
            }
            DynBankId::Qspi => unsafe {
                let qspi = &*pac::PADS_QSPI::PTR;
                match pin.num {
                    0 => core::mem::transmute(&qspi.gpio_qspi_sclk),
                    1 => core::mem::transmute(&qspi.gpio_qspi_ss),
                    2 => core::mem::transmute(&qspi.gpio_qspi_sd0),
                    3 => core::mem::transmute(&qspi.gpio_qspi_sd1),
                    4 => core::mem::transmute(&qspi.gpio_qspi_sd2),
                    5 => core::mem::transmute(&qspi.gpio_qspi_sd3),
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

    fn proc_in_by_pass(&self) -> &rp2040_pac::syscfg::PROC_IN_SYNC_BYPASS {
        let pin = self.as_dyn();
        unsafe {
            let syscfg = &*pac::SYSCFG::PTR;
            match pin.bank {
                DynBankId::Bank0 => &syscfg.proc_in_sync_bypass,
                DynBankId::Qspi => core::mem::transmute(&syscfg.proc_in_sync_bypass_hi),
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
                    &bank.intr[usize::from(index)]
                }
                DynBankId::Qspi => {
                    let bank = &*pac::IO_QSPI::PTR;
                    core::mem::transmute(&bank.intr)
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
