//! After coming out of reset, the hardware expects 800us of LS_J (linestate J) time
//! before it will move to the connected state. However on a hub that broadcasts packets
//! for other devices this isn't the case. The plan here is to wait for the end of the bus
//! reset, force an LS_J for 1ms and then switch control back to the USB phy. Unfortunately
//! this requires us to use GPIO15 as there is no other way to force the input path.
//! We only need to force DP as DM can be left at zero. It will be gated off by GPIO
//! logic if it isn't func selected.

use crate::atomic_register_access::{write_bitmask_clear, write_bitmask_set};
use rp2040_pac::Peripherals;

pub struct ForceLineStateJ {
    prev_pads: u32,
    prev_io_ctrls: u32,
}
pub enum Errata5State {
    WaitEndOfReset,
    ForceLineStateJ(ForceLineStateJ),
}

impl Errata5State {
    pub fn start() -> Self {
        Self::WaitEndOfReset
    }
    /// SAFETY: This method steals the peripherals.
    /// It makes read only use of TIMER and read/write access to USBCTRL_REGS.
    /// Both peripherals must be initialized & running.
    pub unsafe fn update(self) -> Option<Self> {
        let pac = crate::pac::Peripherals::steal();
        match self {
            Self::WaitEndOfReset => {
                if pac.USBCTRL_REGS.sie_status.read().line_state().is_se0() {
                    Some(self)
                } else {
                    let reset_state = pac.RESETS.reset.read();
                    assert!(
                        reset_state.io_bank0().bit_is_clear()
                            && reset_state.pads_bank0().bit_is_clear(),
                        "IO Bank 0 must be out of reset for this work around to function properly."
                    );
                    Some(Self::ForceLineStateJ(start_force_j(&pac)))
                }
            }
            Self::ForceLineStateJ(ref state) => {
                if pac
                    .USBCTRL_REGS
                    .sie_status
                    .read()
                    .connected()
                    .bit_is_clear()
                {
                    Some(self)
                } else {
                    finish(&pac, state.prev_pads, state.prev_io_ctrls);
                    None
                }
            }
        }
    }
}

const DP_PULLUP_EN_FLAG: u32 = 0x0000_0002;
const DP_PULLUP_EN_OVERRIDE_FLAG: u32 = 0x0000_0004;

fn start_force_j(pac: &Peripherals) -> ForceLineStateJ {
    let pads = &pac.PADS_BANK0.gpio[15];
    let io = &pac.IO_BANK0.gpio[15];
    let usb_ctrl = &pac.USBCTRL_REGS;

    assert!(!usb_ctrl.sie_status.read().line_state().is_se0());
    assert!(
        pac.IO_BANK0.gpio[16].gpio_ctrl.read().funcsel().bits() != 8,
        "Not expecting DM to be function 8"
    );

    // backup io ctrl & pad ctrl
    let prev_pads = pads.read().bits();
    let prev_io_ctrls = io.gpio_ctrl.read().bits();

    // Enable bus keep and force pin to tristate, so USB DP muxing doesn't affect
    // pin state
    pads.modify(|_, w| w.pue().set_bit().pde().set_bit());
    io.gpio_ctrl.modify(|_, w| w.oeover().disable());

    // Select function 8 (USB debug muxing) without disturbing other controls
    io.gpio_ctrl.modify(|_, w| unsafe { w.funcsel().bits(8) });

    // J state is a differential 1 for a full speed device so
    // DP = 1 and DM = 0. Don't actually need to set DM low as it
    // is already gated assuming it isn't funcseld.
    io.gpio_ctrl.modify(|_, w| w.inover().high());

    // Force PHY pull up to stay before switching away from the phy
    unsafe {
        let usbphy_direct = usb_ctrl.usbphy_direct.as_ptr();
        let usbphy_direct_override = usb_ctrl.usbphy_direct_override.as_ptr();
        write_bitmask_set(usbphy_direct, DP_PULLUP_EN_FLAG);
        write_bitmask_set(usbphy_direct_override, DP_PULLUP_EN_OVERRIDE_FLAG);
    }

    // Switch to GPIO phy with LS_J forced
    unsafe {
        usb_ctrl
            .usb_muxing
            .write_with_zero(|w| w.to_digital_pad().set_bit().softcon().set_bit());
    }

    // LS_J is now forced, wait until the signal propagates through the usb logic.
    loop {
        let status = usb_ctrl.sie_status.read();
        if status.line_state().is_j() {
            break;
        }
    }

    ForceLineStateJ {
        prev_pads,
        prev_io_ctrls,
    }
}
fn finish(pac: &Peripherals, prev_pads: u32, prev_io_ctrls: u32) {
    let pads = &pac.PADS_BANK0.gpio[15];
    let io = &pac.IO_BANK0.gpio[15];

    // Switch back to USB phy
    pac.USBCTRL_REGS
        .usb_muxing
        .write(|w| w.to_phy().set_bit().softcon().set_bit());

    // Get rid of DP pullup override
    unsafe {
        let usbphy_direct_override = pac.USBCTRL_REGS.usbphy_direct_override.as_ptr();
        write_bitmask_clear(usbphy_direct_override, DP_PULLUP_EN_OVERRIDE_FLAG);

        // Finally, restore the gpio ctrl value back to GPIO15
        io.gpio_ctrl.write(|w| w.bits(prev_io_ctrls));
        // Restore the pad ctrl value
        pads.write(|w| w.bits(prev_pads));
    }
}
