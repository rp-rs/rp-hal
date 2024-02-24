//! On-chip voltage regulator (VREG)
//!
//! See [Chapter 2, Section 10](https://datasheets.raspberrypi.com/rp2040/rp2040-datasheet.pdf) of the datasheet for more details
//!
//! ## Usage
//! ```no_run
//! use rp2040_hal::pac::vreg_and_chip_reset::vreg::VSEL_A;
//! use rp2040_hal::{vreg::set_voltage, pac};
//! let mut pac = pac::Peripherals::take().unwrap();
//! // Set voltage to 1.20V
//! set_voltage(&mut pac.VREG_AND_CHIP_RESET, VSEL_A::VOLTAGE1_20);
//! ```

use crate::pac::vreg_and_chip_reset::vreg::VSEL_A;
use crate::pac::VREG_AND_CHIP_RESET;

/// Set voltage to the on-chip voltage regulator.
///
/// There is no guarantee that the device will operate at all of the available voltages.
/// Appropriate values should be selected in consideration of the system clock frequency and other factors to be set.
///
/// # Arguments
///
/// * `vreg_dev` - VREG peripheral
/// * `voltage` - Voltage to set
pub fn set_voltage(vreg_dev: &mut VREG_AND_CHIP_RESET, voltage: VSEL_A) {
    vreg_dev.vreg().write(|w| w.vsel().variant(voltage));
}

/// Get voltage from the on-chip voltage regulator
///
/// # Arguments
///
/// * `vreg_dev` - VREG peripheral
pub fn get_voltage(vreg_dev: &VREG_AND_CHIP_RESET) -> Option<VSEL_A> {
    vreg_dev.vreg().read().vsel().variant()
}
