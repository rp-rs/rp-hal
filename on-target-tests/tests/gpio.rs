#![no_std]
#![no_main]
#![cfg(test)]

use defmt_rtt as _; // defmt transport
use defmt_test as _;
use panic_probe as _;
#[cfg(feature = "rp2040")]
use rp2040_hal as hal; // memory layout // panic handler
#[cfg(feature = "rp235x")]
use rp235x_hal as hal;

/// The linker will place this boot block at the start of our program image. We
/// need this to help the ROM bootloader get our code up and running.
/// Note: This boot block is not necessary when using a rp-hal based BSP
/// as the BSPs already perform this step.
#[cfg(feature = "rp2040")]
#[link_section = ".boot2"]
#[used]
pub static BOOT2: [u8; 256] = rp2040_boot2::BOOT_LOADER_GENERIC_03H;

/// Tell the Boot ROM about our application
#[cfg(feature = "rp235x")]
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
    use hal::gpio::{PinGroup, PinState};
    use hal::watchdog::Watchdog;

    #[init]
    fn setup() -> () {
        unsafe {
            crate::init::reset_cleanup();
        }
        let mut pac = pac::Peripherals::take().unwrap();
        #[cfg(feature = "rp2040")]
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
        let sio = hal::Sio::new(pac.SIO);
        let pins = hal::gpio::Pins::new(
            pac.IO_BANK0,
            pac.PADS_BANK0,
            sio.gpio_bank0,
            &mut pac.RESETS,
        );

        // GPIO (0 <=> 2) and (1 <=> 3) connected together
        let mut group = PinGroup::new()
            .add_pin(pins.gpio0.into_push_pull_output())
            .add_pin(pins.gpio1.into_push_pull_output())
            .add_pin(pins.gpio2.into_bus_keep_input())
            .add_pin(pins.gpio3.into_bus_keep_input());

        group.set(PinState::Low);
        cortex_m::asm::delay(10);
        assert_eq!(group.read(), 0b0000);
        group.set(PinState::High);
        cortex_m::asm::delay(10);
        assert_eq!(group.read(), 0b1111);
        group.set(PinState::Low);
        cortex_m::asm::delay(10);
        assert_eq!(group.read(), 0b0000);

        group.set(PinState::Low);
        group.toggle();
        cortex_m::asm::delay(10);
        assert_eq!(group.read(), 0b1111);
        group.toggle();
        cortex_m::asm::delay(10);
        assert_eq!(group.read(), 0b0000);
        group.toggle();
        cortex_m::asm::delay(10);
        assert_eq!(group.read(), 0b1111);

        group.set_u32(0b0001);
        cortex_m::asm::delay(10);
        assert_eq!(group.read(), 0b0101);
        group.set_u32(0b0010);
        cortex_m::asm::delay(10);
        assert_eq!(group.read(), 0b1010);
    }

    #[test]
    fn read_adc() {
        use embedded_hal_0_2::adc::OneShot;

        // Safety: Test cases do not run in parallel
        let mut pac = unsafe { pac::Peripherals::steal() };
        let mut adc = hal::Adc::new(pac.ADC, &mut pac.RESETS);
        let mut temp_sensor = hal::adc::Adc::take_temp_sensor(&mut adc).unwrap();
        let _temperature: u16 = adc.read(&mut temp_sensor).unwrap();
    }
}
