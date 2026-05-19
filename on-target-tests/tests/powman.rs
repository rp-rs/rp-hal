#![no_std]
#![no_main]
#![cfg(test)]

use defmt_rtt as _; // defmt transport
use defmt_test as _;
use panic_probe as _;
// This test only runs on rp235x
use rp235x_hal as hal;

#[link_section = ".start_block"]
#[used]
pub static IMAGE_DEF: hal::block::ImageDef = hal::block::ImageDef::secure_exe();

/// External high-speed crystal on the Raspberry Pi Pico board is 12 MHz. Adjust
/// if your board has a different frequency
const XTAL_FREQ_HZ: u32 = 12_000_000u32;

mod init;

#[defmt_test::tests]
mod tests {
    use crate::hal;
    use crate::hal::clocks::init_clocks_and_plls;
    use crate::hal::pac;
    use crate::XTAL_FREQ_HZ;
    use hal::watchdog::Watchdog;
    use rp235x_hal::powman::{Powman, VRegError};
    use rp235x_hal::vreg::VRegVoltage;

    #[init]
    fn setup() -> () {
        unsafe {
            crate::init::reset_cleanup();
        }
        let mut pac = pac::Peripherals::take().unwrap();
        let mut watchdog = Watchdog::new(pac.WATCHDOG);

        let _clocks = init_clocks_and_plls(
            XTAL_FREQ_HZ,
            pac.XOSC,
            pac.CLOCKS,
            pac.PLL_SYS,
            pac.PLL_USB,
            &mut pac.RESETS,
            &mut watchdog,
        )
        .ok()
        .unwrap();
    }

    #[test]
    fn check_set_vreg() {
        // Safety: Test cases do not run in parallel
        let pac = unsafe { pac::Peripherals::steal() };
        let mut powman = Powman::new(pac.POWMAN, None);

        let before = powman.get_vreg_voltage().expect("Failed to read vreg");

        // Set to default value
        powman
            .set_vreg_voltage(VRegVoltage::V1100mV)
            .expect("Failed to set vreg to 1.1V");
        assert_eq!(Ok(VRegVoltage::V1100mV), powman.get_vreg_voltage());

        // Set to the maximum safe value
        powman
            .set_vreg_voltage(VRegVoltage::V1200mV)
            .expect("Failed to set vreg to 1.2V");
        assert_eq!(Ok(VRegVoltage::V1200mV), powman.get_vreg_voltage());

        // This should fail because we don't set DISABLE_VOLTAGE_LIMIT
        assert_eq!(
            Err(VRegError::VRegReadbackMismatchError),
            powman.set_vreg_voltage(VRegVoltage::V1350mV)
        );
        // The failed call should not have changed the voltage.
        assert_eq!(Ok(VRegVoltage::V1300mV), powman.get_vreg_voltage());

        powman
            .set_vreg_voltage(before)
            .expect("Failed to set vreg back to initial value");
        assert_eq!(Ok(before), powman.get_vreg_voltage());
    }
}
