// Based heavily on and in some places copied from `atsamd-hal` gpio::v2
use super::dynpin::{DynGroup, DynPinId};
use super::{
    InputOverride, Interrupt, InterruptOverride, OutputDriveStrength, OutputEnableOverride,
    OutputOverride, OutputSlewRate,
};
use crate::atomic_register_access::{write_bitmask_clear, write_bitmask_set};
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
                    BusKeep => {
                        fields.pde = true;
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
                    BusKeep => {
                        fields.pde = true;
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
                    // SIO is 5, but isn't an alternate function but is instead for using the pin as GPIO
                    Pio0 => 6,
                    Pio1 => 7,
                    Clock => 8,
                    UsbAux => 9,
                };
                fields.inen = true;
                if func == I2C {
                    fields.pue = true;
                }
            }
        };

        fields
    }
}

/// # Safety
///
/// Users should only implement the [`id`] function. No default function
/// implementations should be overridden. The implementing type must also have
/// "control" over the corresponding pin ID, i.e. it must guarantee that each
/// pin ID is a singleton
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

    /// Clear interrupt.
    #[inline]
    fn clear_interrupt(&self, interrupt: Interrupt) {
        let num = self.id().num as usize;
        unsafe {
            let io = &(*pac::IO_BANK0::ptr());
            // There are four bits for each GPIO pin (one for each enumerator
            // in the `Interrupt` enum). There are therefore eight pins per
            // 32-bit register, and four registers in total.
            let bit_in_reg = num % 8 * 4 + interrupt as usize;
            io.intr[num >> 3].write(|w| w.bits(1 << bit_in_reg));
        }
    }

    /// Interrupt status.
    #[inline]
    fn interrupt_status(&self, interrupt: Interrupt) -> bool {
        let num = self.id().num as usize;
        unsafe {
            let io = &(*pac::IO_BANK0::ptr());
            let cpuid = *(pac::SIO::ptr() as *const u32);
            // There are four bits for each GPIO pin (one for each enumerator
            // in the `Interrupt` enum). There are therefore eight pins per
            // 32-bit register, and four registers per CPU.
            let bit_in_reg = ((num % 8) * 4) + (interrupt as usize);
            if cpuid == 0 {
                (io.proc0_ints[num >> 3].read().bits() & (1 << bit_in_reg)) != 0
            } else {
                (io.proc1_ints[num >> 3].read().bits() & (1 << bit_in_reg)) != 0
            }
        }
    }

    /// Is interrupt enabled.
    #[inline]
    fn is_interrupt_enabled(&self, interrupt: Interrupt) -> bool {
        let num = self.id().num as usize;
        unsafe {
            let io = &(*pac::IO_BANK0::ptr());
            let cpuid = *(pac::SIO::ptr() as *const u32);
            // There are four bits for each GPIO pin (one for each enumerator
            // in the `Interrupt` enum). There are therefore eight pins per
            // 32-bit register, and four registers per CPU.
            let bit_in_reg = num % 8 * 4 + interrupt as usize;
            if cpuid == 0 {
                (io.proc0_inte[num >> 3].read().bits() & (1 << bit_in_reg)) != 0
            } else {
                (io.proc1_inte[num >> 3].read().bits() & (1 << bit_in_reg)) != 0
            }
        }
    }

    /// Enable or disable interrupt.
    #[inline]
    fn set_interrupt_enabled(&self, interrupt: Interrupt, enabled: bool) {
        let num = self.id().num as usize;
        unsafe {
            let cpuid = *(pac::SIO::ptr() as *const u32);
            let io = &(*pac::IO_BANK0::ptr());
            // There are four bits for each GPIO pin (one for each enumerator
            // in the `Interrupt` enum). There are therefore eight pins per
            // 32-bit register, and four registers per CPU.
            let reg = if cpuid == 0 {
                io.proc0_inte[num >> 3].as_ptr()
            } else {
                io.proc1_inte[num >> 3].as_ptr()
            };
            let bit_in_reg = num % 8 * 4 + interrupt as usize;
            if enabled {
                write_bitmask_set(reg, 1 << bit_in_reg);
            } else {
                write_bitmask_clear(reg, 1 << bit_in_reg);
            }
        }
    }

    /// Is interrupt forced.
    #[inline]
    fn is_interrupt_forced(&self, interrupt: Interrupt) -> bool {
        let num = self.id().num as usize;
        unsafe {
            let cpuid = *(pac::SIO::ptr() as *const u32);
            let io = &(*pac::IO_BANK0::ptr());
            // There are four bits for each GPIO pin (one for each enumerator
            // in the `Interrupt` enum). There are therefore eight pins per
            // 32-bit register, and four registers per CPU.
            let bit_in_reg = num % 8 * 4 + interrupt as usize;
            if cpuid == 0 {
                (io.proc0_intf[num >> 3].read().bits() & (1 << bit_in_reg)) != 0
            } else {
                (io.proc1_intf[num >> 3].read().bits() & (1 << bit_in_reg)) != 0
            }
        }
    }

    /// Force or release interrupt.
    #[inline]
    fn set_interrupt_forced(&self, interrupt: Interrupt, forced: bool) {
        let num = self.id().num as usize;
        unsafe {
            let cpuid = *(pac::SIO::ptr() as *const u32);
            let io = &(*pac::IO_BANK0::ptr());
            // There are four bits for each GPIO pin (one for each enumerator
            // in the `Interrupt` enum). There are therefore eight pins per
            // 32-bit register, and four registers per CPU.
            let reg = if cpuid == 0 {
                io.proc0_intf[num >> 3].as_ptr()
            } else {
                io.proc1_intf[num >> 3].as_ptr()
            };
            let bit_in_reg = num % 8 * 4 + interrupt as usize;
            if forced {
                write_bitmask_set(reg, 1 << bit_in_reg);
            } else {
                write_bitmask_clear(reg, 1 << bit_in_reg);
            }
        }
    }

    /// Set the interrupt override.
    #[inline]
    fn set_interrupt_override(&self, override_value: InterruptOverride) {
        let num = self.id().num as usize;
        unsafe { &(*pac::IO_BANK0::ptr()) }.gpio[num]
            .gpio_ctrl
            .modify(|_, w| w.irqover().bits(override_value as u8));
    }

    /// Set the input override.
    #[inline]
    fn set_input_override(&self, override_value: InputOverride) {
        let num = self.id().num as usize;
        unsafe { &(*pac::IO_BANK0::ptr()) }.gpio[num]
            .gpio_ctrl
            .modify(|_, w| w.inover().bits(override_value as u8));
    }

    /// Set the output enable override.
    #[inline]
    fn set_output_enable_override(&self, override_value: OutputEnableOverride) {
        let num = self.id().num as usize;
        unsafe { &(*pac::IO_BANK0::ptr()) }.gpio[num]
            .gpio_ctrl
            .modify(|_, w| w.oeover().bits(override_value as u8));
    }

    /// Set the output override.
    #[inline]
    fn set_output_override(&self, override_value: OutputOverride) {
        let num = self.id().num as usize;
        unsafe { &(*pac::IO_BANK0::ptr()) }.gpio[num]
            .gpio_ctrl
            .modify(|_, w| w.outover().bits(override_value as u8));
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
