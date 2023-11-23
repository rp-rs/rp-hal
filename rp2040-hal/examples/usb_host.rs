//! USB host support example, using the keyboard driver
//!
//! This example illustrates initializing the USB host stack, and using
//! the keyboard driver.
//!
//! It also interprets pressing the NumLock key, by toggling the NumLock LED (and GPIO25)
//!
//! If a LED is connected to GPIO25, like on a Pico board, the LED should toggle with NumLock LED
//! on keyboard.
#![no_std]
#![no_main]

use panic_halt as _;

/// The linker will place this boot block at the start of our program image. We
/// need this to help the ROM bootloader get our code up and running.
/// Note: This boot block is not necessary when using a rp-hal based BSP
/// as the BSPs already perform this step.
#[link_section = ".boot2"]
#[used]
pub static BOOT2: [u8; 256] = rp2040_boot2::BOOT_LOADER_GENERIC_03H;

#[rtic::app(device = rp2040_hal::pac, dispatchers = [TIMER_IRQ_1])]
mod app {
    use embedded_hal::digital::v2::OutputPin;
    use hal::usb::host::UsbHostBus;
    use rp2040_hal as hal;
    use usbh::{
        driver::kbd::{KbdDriver, KbdEvent, KbdLed},
        driver::log::{EventMask, LogDriver},
        PollResult, UsbHost,
    };

    /// External high-speed crystal on the Raspberry Pi Pico board is 12 MHz. Adjust
    /// if your board has a different frequency
    const XTAL_FREQ_HZ: u32 = 12_000_000u32;

    #[shared]
    struct Shared {}

    #[local]
    struct Local {
        usb_host: UsbHost<UsbHostBus>,
        log_driver: LogDriver,
        kbd_driver: KbdDriver,
        pin: hal::gpio::Pin<
            hal::gpio::bank0::Gpio2,
            hal::gpio::FunctionSioOutput,
            hal::gpio::PullDown,
        >,
        led: hal::gpio::Pin<
            hal::gpio::bank0::Gpio25,
            hal::gpio::FunctionSioOutput,
            hal::gpio::PullDown,
        >,
    }

    #[init]
    fn init(mut c: init::Context) -> (Shared, Local, init::Monotonics) {
        // Soft-reset does not release the hardware spinlocks
        // Release them now to avoid a deadlock after debug or watchdog reset
        unsafe {
            hal::sio::spinlock_reset();
        }

        let mut watchdog = hal::Watchdog::new(c.device.WATCHDOG);
        let clocks = hal::clocks::init_clocks_and_plls(
            XTAL_FREQ_HZ,
            c.device.XOSC,
            c.device.CLOCKS,
            c.device.PLL_SYS,
            c.device.PLL_USB,
            &mut c.device.RESETS,
            &mut watchdog,
        )
        .ok()
        .unwrap();
        let sio = hal::Sio::new(c.device.SIO);
        let pins = hal::gpio::Pins::new(
            c.device.IO_BANK0,
            c.device.PADS_BANK0,
            sio.gpio_bank0,
            &mut c.device.RESETS,
        );

        let pin = pins.gpio2.into_push_pull_output();
        let led = pins.gpio25.into_push_pull_output();

        // Create UsbHost instance. Conceptually the `UsbHost` is not specific to any host implementation.
        let usb_host = UsbHost::new(
            // UsbHostBus here is the `usbh::bus::HostBus` implementation for the rp2040.
            UsbHostBus::new(
                c.device.USBCTRL_REGS,
                c.device.USBCTRL_DPRAM,
                clocks.usb_clock,
                &mut c.device.RESETS,
            ),
        );

        (
            Shared {
                // Initialization of shared resources go here
            },
            Local {
                // Initialization of local resources go here
                usb_host,
                kbd_driver: KbdDriver::new(),
                log_driver: LogDriver::new(EventMask::all()),
                pin,
                led,
            },
            init::Monotonics(),
        )
    }

    // Optional idle, can be removed if not needed.
    #[idle]
    fn idle(_: idle::Context) -> ! {
        loop {
            continue;
        }
    }

    #[task(binds = USBCTRL_IRQ, local = [usb_host, kbd_driver, log_driver, pin, led, num_state: bool = false])]
    fn usbctrl_irq(c: usbctrl_irq::Context) {
        _ = c.local.pin.set_high();

        match c
            .local
            .usb_host
            .poll(&mut [c.local.log_driver, c.local.kbd_driver])
        {
            PollResult::NoDevice => {
                // No device is attached (yet)
                return;
            }
            PollResult::Busy => {
                // Bus is currently busy communicating with the device
            }
            PollResult::Idle => {
                // Bus is idle, we can issue commands via the host interface
            }

            PollResult::BusError(_) => {
                // Bus error
            }

            PollResult::DiscoveryError(_) => {
                // Discovery for device failed
            }

            _ => {}
        }

        match c.local.kbd_driver.take_event() {
            None => {}
            Some(event) => {
                match event {
                    KbdEvent::DeviceAdded(dev_addr) => {
                        // Keyboard with address dev_addr added
                        c.local
                            .kbd_driver
                            .set_idle(dev_addr, 0, c.local.usb_host)
                            .ok()
                            .unwrap();
                    }
                    KbdEvent::DeviceRemoved(_) => {
                        // Keyboard with address dev_addr removed
                    }
                    KbdEvent::InputChanged(dev_addr, report) => {
                        // Input on keyboard

                        // toggle Num lock LED when NumLock key is pressed
                        if report.pressed_keys().any(|key| key == 83) {
                            *c.local.num_state = !*c.local.num_state;
                            if *c.local.num_state {
                                c.local.led.set_high().unwrap();
                            } else {
                                c.local.led.set_low().unwrap();
                            }
                            c.local
                                .kbd_driver
                                .set_led(
                                    dev_addr,
                                    KbdLed::NumLock,
                                    *c.local.num_state,
                                    c.local.usb_host,
                                )
                                .ok()
                                .unwrap();
                        }
                    }
                    _ => {}
                }
            }
        }

        _ = c.local.pin.set_low();
    }
}
