#![no_std]
#![no_main]
#![cfg(test)]

use defmt_rtt as _; // defmt transport
use defmt_test as _;
use panic_probe as _;
use rp2040_hal as hal; // memory layout // panic handler

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

#[defmt_test::tests]
mod tests {
    use crate::hal;
    use crate::hal::clocks::init_clocks_and_plls;
    use crate::hal::pac;
    use crate::XTAL_FREQ_HZ;
    use hal::watchdog::Watchdog;
    use rp2040_hal::gpio::{PinGroup, PinState};

    #[init]
    fn setup() -> () {
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

        let sio = hal::Sio::new(pac.SIO);

        let _pins = hal::gpio::Pins::new(
            pac.IO_BANK0,
            pac.PADS_BANK0,
            sio.gpio_bank0,
            &mut pac.RESETS,
        );
    }

    #[test]
    fn check_ie() {
        // Safety: Test cases do not run in parallel
        let pac = unsafe { pac::Peripherals::steal() };
        for id in 0..=29 {
            assert!(pac.PADS_BANK0.gpio(id).read().ie().bit_is_clear());
        }
    }

    #[test]
    fn check_ie_gets_enabled() {
        // Safety: Test cases do not run in parallel
        let pac = unsafe { pac::Peripherals::steal() };
        for id in 0..=29 {
            let pin = unsafe {
                hal::gpio::new_pin(hal::gpio::DynPinId {
                    bank: hal::gpio::DynBankId::Bank0,
                    num: id as u8,
                })
            };
            let pin = pin
                .try_into_function::<hal::gpio::FunctionSioInput>()
                .ok()
                .unwrap();
            assert!(pac.PADS_BANK0.gpio(id).read().ie().bit_is_set());
            let pin = pin
                .try_into_function::<hal::gpio::FunctionNull>()
                .ok()
                .unwrap();
            assert!(pac.PADS_BANK0.gpio(id).read().ie().bit_is_clear());
            let pin = pin
                .try_into_function::<hal::gpio::FunctionPio0>()
                .ok()
                .unwrap();
            assert!(pac.PADS_BANK0.gpio(id).read().ie().bit_is_set());
            let _pin = pin
                .try_into_function::<hal::gpio::FunctionNull>()
                .ok()
                .unwrap();
            assert!(pac.PADS_BANK0.gpio(id).read().ie().bit_is_clear());
        }
    }

    #[test]
    fn check_pin_groups() {
        // Safety: Test cases do not run in parallel
        let mut pac = unsafe { pac::Peripherals::steal() };
        let pingroup = PinGroup::new();
        let sio = hal::Sio::new(pac.SIO);
        let pins = hal::gpio::Pins::new(
            pac.IO_BANK0,
            pac.PADS_BANK0,
            sio.gpio_bank0,
            &mut pac.RESETS,
        );

        let pingroup = pingroup.add_pin(pins.gpio0.into_push_pull_output_in_state(PinState::Low));
        let pingroup = pingroup.add_pin(pins.gpio1.into_push_pull_output_in_state(PinState::Low));
        let pingroup = pingroup.add_pin(pins.gpio2.into_bus_keep_input());
        let mut pingroup = pingroup.add_pin(pins.gpio3.into_bus_keep_input());

        cortex_m::asm::delay(10);
        assert!(pingroup.read() == 0);
        pingroup.toggle();
        cortex_m::asm::delay(10);
        assert!(pingroup.read() == 0xf);
        pingroup.toggle();
        cortex_m::asm::delay(10);
        assert!(pingroup.read() == 0);
    }
}
