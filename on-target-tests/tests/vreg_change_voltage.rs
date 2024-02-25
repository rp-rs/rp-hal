#![no_std]
#![no_main]
#![cfg(test)]

use defmt_rtt as _; // defmt transport
use defmt_test as _;
use panic_probe as _; // panic handler
use rp2040_hal as hal; // memory layout

/// The linker will place this boot block at the start of our program image. We
/// need this to help the ROM bootloader get our code up and running.
/// Note: This boot block is not necessary when using a rp-hal based BSP
/// as the BSPs already perform this step.
#[link_section = ".boot2"]
#[used]
pub static BOOT2: [u8; 256] = rp2040_boot2::BOOT_LOADER_GENERIC_03H;

/// External high-speed crystal on the Raspberry Pi Pico board is 12 MHz. Adjust
/// if your board has a different frequency
const XTAL_FREQ_HZ: u32 = 12_000_000u32;

struct State {
    vreg: hal::pac::VREG_AND_CHIP_RESET,
}

#[defmt_test::tests]
mod tests {
    use crate::{State, XTAL_FREQ_HZ};
    use defmt::assert;
    use defmt_rtt as _;
    use panic_probe as _;
    use rp2040_hal as hal;

    use hal::pac::vreg_and_chip_reset::vreg::VSEL_A;
    use hal::vreg;
    use hal::{clocks::init_clocks_and_plls, pac, watchdog::Watchdog};

    #[init]
    fn setup() -> State {
        unsafe {
            hal::sio::spinlock_reset();
        }
        let mut pac = pac::Peripherals::take().unwrap();
        let _core = pac::CorePeripherals::take().unwrap();
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
        State {
            vreg: pac.VREG_AND_CHIP_RESET,
        }
    }

    #[test]
    fn change_onchip_regulator_voltage(state: &mut State) {
        // Set the voltage to 1.05V
        let target_voltage = VSEL_A::VOLTAGE1_05;
        vreg::set_voltage(&mut state.vreg, target_voltage);
        let v = vreg::get_voltage(&mut state.vreg).unwrap();
        assert!(v == target_voltage);

        // Set the voltage to 1.10V
        let target_voltage = VSEL_A::VOLTAGE1_10;
        vreg::set_voltage(&mut state.vreg, target_voltage);
        let v = vreg::get_voltage(&mut state.vreg).unwrap();
        assert!(v == target_voltage);

        // Set the voltage to 1.15V
        let target_voltage = VSEL_A::VOLTAGE1_15;
        vreg::set_voltage(&mut state.vreg, target_voltage);
        let v = vreg::get_voltage(&mut state.vreg).unwrap();
        assert!(v == target_voltage);

        // Set the voltage to 1.20V
        let target_voltage = VSEL_A::VOLTAGE1_20;
        vreg::set_voltage(&mut state.vreg, target_voltage);
        let v = vreg::get_voltage(&mut state.vreg).unwrap();
        assert!(v == target_voltage);

        // Reset to default voltage (1.10V)
        let target_voltage = VSEL_A::VOLTAGE1_10;
        vreg::set_voltage(&mut state.vreg, target_voltage);
        let v = vreg::get_voltage(&mut state.vreg).unwrap();
        assert!(v == target_voltage);
    }
}
