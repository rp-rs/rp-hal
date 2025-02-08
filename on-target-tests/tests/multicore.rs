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

use core::sync::atomic::Ordering;
use portable_atomic::{AtomicU32, AtomicU8};

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

static STATE: AtomicU8 = AtomicU8::new(0);
static COUNTER: AtomicU32 = AtomicU32::new(0);
const STEPS: u32 = 100000;

mod init;

#[defmt_test::tests]
mod tests {
    use crate::hal;
    use crate::hal::clocks::init_clocks_and_plls;
    use crate::hal::pac;
    use crate::XTAL_FREQ_HZ;
    use hal::watchdog::Watchdog;

    use core::sync::atomic::Ordering;
    use hal::multicore::{Multicore, Stack};

    static CORE1_STACK: Stack<4096> = Stack::new();

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

        let mut sio = hal::Sio::new(pac.SIO);

        let _pins = hal::gpio::Pins::new(
            pac.IO_BANK0,
            pac.PADS_BANK0,
            sio.gpio_bank0,
            &mut pac.RESETS,
        );

        let mut mc = Multicore::new(&mut pac.PSM, &mut pac.PPB, &mut sio.fifo);

        let cores = mc.cores();
        let core1 = &mut cores[1];
        let _test = core1.spawn(CORE1_STACK.take().unwrap(), move || super::core1_task());
    }

    #[test]
    fn check_atomics() {
        super::STATE.store(1, Ordering::Release);
        for _ in 0..super::STEPS {
            super::COUNTER.fetch_add(1, Ordering::Relaxed);
        }
        while super::STATE.load(Ordering::Acquire) != 2 {}

        let counter = super::COUNTER.load(Ordering::Acquire);
        assert_eq!(2 * super::STEPS, counter);
    }

    #[test]
    fn check_floats() {
        super::STATE.store(3, Ordering::Release);
        super::calculations();
        // if calculations fail on core1, the test case will hang here:
        while super::STATE.load(Ordering::Acquire) != 4 {}
    }
}

fn core1_task() {
    loop {
        match STATE.load(Ordering::Acquire) {
            // check_atomics
            1 => {
                for _ in 0..STEPS {
                    COUNTER.fetch_add(1, Ordering::Relaxed);
                }
                STATE.store(2, Ordering::Release);
            }
            // response to check_atomics
            2 => (),
            // check_floats
            3 => {
                calculations();
                STATE.store(4, Ordering::Release);
            }
            // response to check_floats
            4 => (),
            _ => (),
        }
    }
}

fn calculations() {
    for i in 0..STEPS {
        let m = core::hint::black_box(1.1) * (i as f32);
        let n = core::hint::black_box(1.1) * (i as f64);
        assert!((m as f64 - n).abs() < 0.01f64);
    }
}
