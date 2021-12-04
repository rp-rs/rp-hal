#![no_std]
#![no_main]

use panic_halt as _;
use rp2040_hal as hal;

#[rtic::app(device = crate::hal::pac, peripherals = true)]
mod app {

    use embedded_hal::digital::v2::OutputPin;
    use pico::{
        hal::{self, clocks::init_clocks_and_plls, pac, watchdog::Watchdog, Sio},
        XOSC_CRYSTAL_FREQ,
    };

    const SCAN_TIME_US: u32 = 1000000;

    #[shared]
    struct Shared {
        scan_timer: pac::TIMER,
        led: hal::gpio::Pin<hal::gpio::pin::bank0::Gpio25, hal::gpio::PushPullOutput>,
    }

    #[local]
    struct Local {}

    #[init]
    fn init(c: init::Context) -> (Shared, Local, init::Monotonics) {
        let mut resets = c.device.RESETS;
        let mut watchdog = Watchdog::new(c.device.WATCHDOG);
        let _clocks = init_clocks_and_plls(
            XOSC_CRYSTAL_FREQ,
            c.device.XOSC,
            c.device.CLOCKS,
            c.device.PLL_SYS,
            c.device.PLL_USB,
            &mut resets,
            &mut watchdog,
        )
        .ok()
        .unwrap();

        let sio = Sio::new(c.device.SIO);
        let pins = hal::gpio::Pins::new(
            c.device.IO_BANK0,
            c.device.PADS_BANK0,
            sio.gpio_bank0,
            &mut resets,
        );
        let mut led = pins.gpio25.into_push_pull_output();
        led.set_low().unwrap();

        let timer = c.device.TIMER;
        timer.dbgpause.write(|w| w.dbg0().set_bit());
        let current_time = timer.timelr.read().bits();
        timer
            .alarm0
            .write(|w| unsafe { w.bits(current_time + SCAN_TIME_US) });
        timer.inte.write(|w| w.alarm_0().set_bit());

        (
            Shared {
                scan_timer: timer,
                led,
            },
            Local {},
            init::Monotonics(),
        )
    }

    #[task(
        binds = TIMER_IRQ_0,
        priority = 1,
        shared = [scan_timer, led],
        local = [tog: bool = false],
    )]
    fn scan_timer_irq(mut c: scan_timer_irq::Context) {
        if *c.local.tog {
            c.shared.led.lock(|l| l.set_high().unwrap());
        } else {
            c.shared.led.lock(|l| l.set_low().unwrap());
        }
        *c.local.tog = !*c.local.tog;

        let current_time = c.shared.scan_timer.lock(|t| t.timelr.read().bits());
        c.shared.scan_timer.lock(|t| {
            t.alarm0
                .write(|w| unsafe { w.bits(current_time + SCAN_TIME_US) })
        });
        c.shared
            .scan_timer
            .lock(|t| t.intr.write(|w| w.alarm_0().set_bit()));
    }
}
