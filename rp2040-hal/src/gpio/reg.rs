// Based heavily on and in some places copied from `atsamd-hal` gpio::v2
use super::dynpin::{DynGroup, DynPinId};
use super::{OutputDriveStrength, OutputSlewRate};
use crate::gpio::dynpin::{DynDisabled, DynFunction, DynInput, DynOutput, DynPinMode};
use crate::pac;

//==============================================================================
//  ModeFields
//==============================================================================

/// Collect all fields needed to set the [`PinMode`](super::PinMode)
#[derive(Default)]
struct ModeFields {
    inen: bool,
    pue: bool,
    pde: bool,

    sio_outen: bool,
    funcsel: u8,
}

const SIO_FUNCSEL: u8 = 5;
const NULL_FUNCSEL: u8 = 0x1f;

impl From<DynPinMode> for ModeFields {
    #[inline]
    fn from(mode: DynPinMode) -> Self {
        use DynPinMode::*;
        let mut fields = Self::default();
        match mode {
            Disabled(config) => {
                use DynDisabled::*;
                fields.funcsel = NULL_FUNCSEL;
                fields.sio_outen = false;
                fields.inen = false;

                match config {
                    Floating => (),
                    PullDown => {
                        fields.pde = true;
                    }
                    PullUp => {
                        fields.pue = true;
                    }
                }
            }
            Input(config) => {
                use DynInput::*;
                fields.funcsel = SIO_FUNCSEL;
                fields.sio_outen = false;
                fields.inen = true;

                match config {
                    Floating => (),
                    PullDown => {
                        fields.pde = true;
                    }
                    PullUp => {
                        fields.pue = true;
                    }
                }
            }
            Output(config) => {
                use DynOutput::*;
                fields.funcsel = SIO_FUNCSEL;
                fields.sio_outen = true;
                match config {
                    PushPull => {
                        fields.inen = false;
                    }
                    Readable => {
                        fields.inen = true;
                    }
                }
            }
            Function(func) => {
                use DynFunction::*;
                fields.funcsel = match func {
                    Xip => 0,
                    Spi => 1,
                    Uart => 2,
                    I2C => 3,
                    Pwm => 4,
                    // Pio => 5
                    Pio0 => 6,
                    Pio1 => 7,
                    Clock => 8,
                    UsbAux => 9,
                };
                fields.inen = true;
            }
        };

        fields
    }
}
pub(super) unsafe trait RegisterInterface {
    /// Provide a [`DynPinId`] identifying the set of registers controlled by
    /// this type.
    fn id(&self) -> DynPinId;

    #[inline]
    fn mask_32(&self) -> u32 {
        1 << self.id().num
    }

    /// Read the logic level of an input put
    #[inline]
    fn read_pin(&self) -> bool {
        let mask = self.mask_32();
        (match self.id().group {
            DynGroup::Bank0 => unsafe { &(*pac::SIO::ptr()) }.gpio_in.read().bits(),
            DynGroup::Qspi => unsafe { &(*pac::SIO::ptr()) }.gpio_hi_in.read().bits(),
        }) & mask
            != 0
    }

    /// Write the logic level of an output pin
    #[inline]
    fn write_pin(&mut self, bit: bool) {
        let mask = self.mask_32();
        // This ordering to try and make the group match inline if bit can't be
        unsafe {
            match self.id().group {
                DynGroup::Bank0 => {
                    if bit {
                        (*pac::SIO::ptr()).gpio_out_set.write(|w| w.bits(mask));
                    } else {
                        (*pac::SIO::ptr()).gpio_out_clr.write(|w| w.bits(mask));
                    }
                }
                DynGroup::Qspi => {
                    if bit {
                        (*pac::SIO::ptr()).gpio_hi_out_set.write(|w| w.bits(mask));
                    } else {
                        (*pac::SIO::ptr()).gpio_hi_out_clr.write(|w| w.bits(mask));
                    }
                }
            };
        }
    }

    /// Toggle the logic level of an output pin
    #[inline]
    fn toggle_pin(&mut self) {
        let mask = self.mask_32();
        match self.id().group {
            DynGroup::Bank0 => unsafe { (*pac::SIO::ptr()).gpio_out_xor.write(|w| w.bits(mask)) },
            DynGroup::Qspi => unsafe { (*pac::SIO::ptr()).gpio_hi_out_xor.write(|w| w.bits(mask)) },
        }
    }

    /// Read back the logic level of an output pin
    #[inline]
    fn read_out_pin(&self) -> bool {
        let mask = self.mask_32();
        (match self.id().group {
            DynGroup::Bank0 => unsafe { &(*pac::SIO::ptr()) }.gpio_out.read().bits(),
            DynGroup::Qspi => unsafe { &(*pac::SIO::ptr()) }.gpio_hi_out.read().bits(),
        }) & mask
            != 0
    }

    #[inline]
    fn read_drive_strength(&self) -> OutputDriveStrength {
        use OutputDriveStrength::*;
        let num = self.id().num as usize;
        let strength = match self.id().group {
            DynGroup::Bank0 => unsafe { &(*pac::PADS_BANK0::ptr()) }.gpio[num]
                .read()
                .drive()
                .bits(),
            DynGroup::Qspi => qspi_read_drive(num),
        };
        match strength {
            0x0 => TwoMilliAmps,
            0x1 => FourMilliAmps,
            0x2 => EightMilliAmps,
            0x3 => TwelveMilliAmps,
            _ => unreachable!("invalid drive strength"),
        }
    }

    #[inline]
    fn write_drive_strength(&self, strength: OutputDriveStrength) {
        use OutputDriveStrength::*;
        let num = self.id().num as usize;
        let strength = match strength {
            TwoMilliAmps => 0x0,
            FourMilliAmps => 0x1,
            EightMilliAmps => 0x2,
            TwelveMilliAmps => 0x3,
        };

        match self.id().group {
            DynGroup::Bank0 => unsafe { &(*pac::PADS_BANK0::ptr()) }.gpio[num]
                .modify(|_, w| w.drive().bits(strength)),
            DynGroup::Qspi => qspi_write_drive(num, strength),
        };
    }

    #[inline]
    fn read_slew_rate(&self) -> OutputSlewRate {
        let num = self.id().num as usize;
        let slew_fast = match self.id().group {
            DynGroup::Bank0 => unsafe { &(*pac::PADS_BANK0::ptr()) }.gpio[num]
                .read()
                .slewfast()
                .bit_is_set(),
            DynGroup::Qspi => qspi_read_slew(num),
        };
        if slew_fast {
            OutputSlewRate::Fast
        } else {
            OutputSlewRate::Slow
        }
    }

    #[inline]
    fn write_slew_rate(&self, rate: OutputSlewRate) {
        let num = self.id().num as usize;
        let slewfast = match rate {
            OutputSlewRate::Fast => true,
            OutputSlewRate::Slow => false,
        };

        match self.id().group {
            DynGroup::Bank0 => unsafe { &(*pac::PADS_BANK0::ptr()) }.gpio[num]
                .modify(|_, w| w.slewfast().bit(slewfast)),
            DynGroup::Qspi => qspi_write_slew(num, slewfast),
        };
    }

    // We have to duplicate code, maybe a fix in the HAL layer can prevent this
    #[inline]
    fn do_change_mode(&self, mode: DynPinMode) {
        let num = self.id().num as usize;
        match self.id().group {
            DynGroup::Bank0 => gpio_change_mode(num, mode),
            DynGroup::Qspi => qspi_change_mode(num, mode),
        }
    }
}

#[inline]
fn gpio_change_mode(num: usize, mode: DynPinMode) {
    let fields: ModeFields = mode.into();
    let io = unsafe { &(*pac::IO_BANK0::ptr()).gpio[num] };
    let pads = unsafe { &(*pac::PADS_BANK0::ptr()).gpio[num] };

    pads.write(|w| {
        w.pue().bit(fields.pue);
        w.pde().bit(fields.pde);
        w.ie().bit(fields.inen);
        w.od().bit(false) // the SIO oe bit will handle this instead
    });

    io.gpio_ctrl
        .write(|w| unsafe { w.funcsel().bits(fields.funcsel) });

    if fields.funcsel == SIO_FUNCSEL {
        if fields.sio_outen {
            unsafe {
                (*pac::SIO::ptr()).gpio_oe_set.write(|w| w.bits(1 << num));
            }
        } else {
            unsafe {
                (*pac::SIO::ptr()).gpio_oe_clr.write(|w| w.bits(1 << num));
            }
        }
    } else {
        unsafe {
            (*pac::SIO::ptr()).gpio_oe_clr.write(|w| w.bits(1 << num));
        }
    }
}

// TODO: This is really nasty, but there's no single type for the QSPI pins
//   I'm not sure if a svd change is even possible to fix this, as these do have
//   different reset values
macro_rules! qspi_bits {
    ( $( ($num : expr, $suffix : ident) ),+ ) => {
        $crate::paste::paste! {
            #[inline]
            fn qspi_read_drive(num: usize) -> u8 {
                match num {
                    $($num => unsafe { &(*pac::PADS_QSPI::ptr()).[<gpio_qspi_ $suffix>] }.read().drive().bits(), )+
                    _ => unreachable!("invalid ID for QSPI pin")
                }
            }

            #[inline]
            fn qspi_write_drive(num: usize, val : u8) {
                match num {
                    $($num => unsafe { &(*pac::PADS_QSPI::ptr()).[<gpio_qspi_ $suffix>] }.modify(|_,w| w.drive().bits(val) ), )+
                    _ => unreachable!("invalid ID for QSPI pin")
                }
            }

            #[inline]
            fn qspi_read_slew(num: usize) -> bool {
                match num {
                    $($num => unsafe { &(*pac::PADS_QSPI::ptr()).[<gpio_qspi_ $suffix>] }.read().slewfast().bit_is_set(), )+
                    _ => unreachable!("invalid ID for QSPI pin")
                }
            }

            #[inline]
            fn qspi_write_slew(num: usize, slewfast : bool) {
                match num {
                    $($num => unsafe { &(*pac::PADS_QSPI::ptr()).[<gpio_qspi_ $suffix>] }.modify(|_,w| w.slewfast().bit(slewfast) ), )+
                    _ => unreachable!("invalid ID for QSPI pin")
                }
            }

            #[inline]
            fn qspi_change_mode(num: usize, mode: DynPinMode) {
                let fields : ModeFields = mode.into();

                match num {
                    $($num => {
                        let io = unsafe { &(*pac::IO_QSPI::ptr()).[<gpio_qspi $suffix>] };
                        let pads = unsafe { &(*pac::PADS_QSPI::ptr()).[<gpio_qspi_ $suffix>] };

                        pads.write(|w| {
                            w.pue().bit(fields.pue);
                            w.pde().bit(fields.pde);
                            w.ie().bit(fields.inen);
                            w.od().bit(false)
                        });

                        io.gpio_ctrl.write(|w| unsafe { w.funcsel().bits(fields.funcsel) } );
                    }, )+
                    _ => unreachable!("invalid ID for QSPI pin")
                }


                // outen is only on SIO
                if fields.funcsel == SIO_FUNCSEL {
                    if fields.sio_outen {
                        unsafe { (*pac::SIO::ptr()).gpio_hi_oe_set.write(|w| w.bits(1 << num)); }
                    } else {
                        unsafe { (*pac::SIO::ptr()).gpio_hi_oe_clr.write(|w| w.bits(1 << num)); }
                    }
                } else {
                    unsafe { (*pac::SIO::ptr()).gpio_hi_oe_clr.write(|w| w.bits(1 << num)); }
                }
            }
        }
    }
}

qspi_bits!((0, sclk), (1, ss), (2, sd0), (3, sd1), (4, sd2), (5, sd3));
