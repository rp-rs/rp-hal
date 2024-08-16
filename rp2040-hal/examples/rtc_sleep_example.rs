//! # RTC Sleep Example
//!
//! This application demonstrates use of RTC Interrupt to wake from deepsleep.
//! It is also intended as a general introduction to interrupts and RTC with RP2040.
//!
//! See the top-level `README.md` file for Copyright and license details.

#![no_std]
#![no_main]

// Ensure we halt the program on panic (if we don't mention this crate it won't
// be linked)
use panic_halt as _;

// Alias for our HAL crate
use rp2040_hal as hal;

// A shorter alias for the Peripheral Access Crate, which provides low-level
// register access and to the gpio and rtc modules.
use hal::{clocks::ClockGate, gpio, pac, rtc};

// Some traits we need
use embedded_hal::digital::StatefulOutputPin;

// Our interrupt macro
use hal::pac::interrupt;

// Some short-cuts to useful types
use core::cell::RefCell;
use critical_section::Mutex;

// Time & clock traits
use fugit::{HertzU32, RateExtU32};
use hal::Clock;

/// The linker will place this boot block at the start of our program image. We
/// need this to help the ROM bootloader get our code up and running.
/// Note: This boot block is not necessary when using a rp-hal based BSP
/// as the BSPs already perform this step.
#[link_section = ".boot2"]
#[used]
pub static BOOT2: [u8; 256] = rp2040_boot2::BOOT_LOADER_GENERIC_03H;

/// External high-speed crystal on the Raspberry Pi Pico board is 12 MHz. Adjust
/// if your board has a different frequency
const XTAL_FREQ_HZ: HertzU32 = HertzU32::Hz(12_000_000u32);

/// This how we transfer our RTC instance into the Interrupt Handler.
static GLOBAL_SHARED: Mutex<RefCell<Option<rtc::RealTimeClock>>> = Mutex::new(RefCell::new(None));

/// Entry point to our bare-metal application.
///
/// The `#[rp2040_hal::entry]` macro ensures the Cortex-M start-up code calls this function
/// as soon as all global variables and the spinlock are initialised.
///
/// The function configures the RP2040 peripherals, then toggles a GPIO pin in
/// an infinite loop. If there is an LED connected to that pin, it will blink.
#[rp2040_hal::entry]
fn main() -> ! {
    // Grab our singleton objects
    let mut pac = pac::Peripherals::take().unwrap();
    let mut core = pac::CorePeripherals::take().unwrap();

    // Configure the clocks
    let mut clocks = hal::clocks::ClocksManager::new(pac.CLOCKS);
    // First, enable and wait for xosc to be stable.
    let xosc = hal::xosc::setup_xosc_blocking(pac.XOSC, XTAL_FREQ_HZ).unwrap();

    // use xosc at 12MHz for clk_ref -> clk_sys -> clk_peri
    clocks
        .reference_clock
        .configure_clock(&xosc, XTAL_FREQ_HZ)
        .unwrap();
    clocks
        .system_clock
        .configure_clock(&clocks.reference_clock, XTAL_FREQ_HZ)
        .unwrap();
    clocks
        .peripheral_clock
        .configure_clock(&clocks.system_clock, XTAL_FREQ_HZ)
        .unwrap();
    // use xosc at 12MHz/256 for clk_rtc
    clocks.rtc_clock.configure_clock(&xosc, 46875.Hz()).unwrap();
    // Only leave the rtc's clock enabled while in deep sleep.
    let mut config = ClockGate::default();
    config.set_rtc_rtc(true);
    clocks.configure_sleep_enable(config);

    // The single-cycle I/O block controls our GPIO pins
    let sio = hal::Sio::new(pac.SIO);

    // Set the pins to their default state
    let pins = gpio::Pins::new(
        pac.IO_BANK0,
        pac.PADS_BANK0,
        sio.gpio_bank0,
        &mut pac.RESETS,
    );

    // Configure GPIO 25 as an output to drive our LED.
    let mut led = pins.gpio25.into_push_pull_output();

    // Prepare the RTC for the example using the 1/1/0 (Day/Month/Year) at 0:00:00 as the initial
    // day and time (it may not have been a Monday but it doesn't matter for this example.).
    let mut rtc = hal::rtc::RealTimeClock::new(
        pac.RTC,
        clocks.rtc_clock,
        &mut pac.RESETS,
        rtc::DateTime {
            year: 0,
            month: 1,
            day: 1,
            day_of_week: rtc::DayOfWeek::Monday,
            hour: 0,
            minute: 0,
            second: 0,
        },
    )
    .unwrap();

    // Trigger the IRQ every time a minute starts.
    rtc.schedule_alarm(rtc::DateTimeFilter::default().second(0));
    // Let the alarm trigger an interrupt in the NVIC.
    rtc.enable_interrupt();

    // Give away our rtc by moving them into the `GLOBAL_SHARED` variable.
    // We won't need to access it in the main thread again
    critical_section::with(|cs| {
        GLOBAL_SHARED.borrow(cs).replace(Some(rtc));
    });

    // Let the core enter deep-sleep while waiting on wfi
    core.SCB.set_sleepdeep();

    // Unmask the RTC IRQ so that the NVIC interrupt controller
    // will jump to the interrupt function when the interrupt occurs.
    // We do this last so that the interrupt can't go off while
    // it is in the middle of being configured
    unsafe {
        pac::NVIC::unmask(pac::Interrupt::RTC_IRQ);
    }

    loop {
        // Wait to be awaken by an interrupt
        cortex_m::asm::wfi();

        // Toggle the led
        let _ = led.toggle();
    }
}

#[allow(non_snake_case)]
#[interrupt]
fn RTC_IRQ() {
    critical_section::with(|cs| {
        // borrow the content of the Mutexed RefCell.
        let mut maybe_rtc = GLOBAL_SHARED.borrow_ref_mut(cs);

        // borrow the content of the Option
        if let Some(rtc) = maybe_rtc.as_mut() {
            // clear the interrupt flag so that it stops firing for now and can be triggered again.
            rtc.clear_interrupt();
        }
    });
}

// End of file
