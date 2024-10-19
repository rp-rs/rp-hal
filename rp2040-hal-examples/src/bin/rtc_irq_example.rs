//! # RTC IRQ Example
//!
//! This application demonstrates use of RTC Interrupts.
//! It is also intended as a general introduction to interrupts with RP2040.
//!
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
use hal::{gpio, pac, rtc};

// Some traits we need
use embedded_hal::digital::StatefulOutputPin;

// Our interrupt macro
use hal::pac::interrupt;

// Some short-cuts to useful types
use core::cell::RefCell;
use critical_section::Mutex;

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

// Pin types quickly become very long!
// We'll create some type aliases using `type` to help with that

/// This pin will be our output - it will drive an LED if you run this on a Pico
type LedPin = gpio::Pin<gpio::bank0::Gpio25, gpio::FunctionSioOutput, gpio::PullNone>;

/// Since we're always accessing the pin and the rtc together we'll store them in a tuple.
/// Giving this tuple a type alias means we won't need to use () when putting them
/// inside an Option. That will be easier to read.
type LedAndRtc = (LedPin, rtc::RealTimeClock);

/// This how we transfer our Led pin and RTC into the Interrupt Handler.
/// We'll have the option hold both using the LedAndRtc type.
/// This will make it a bit easier to unpack them later.
static GLOBAL_SHARED: Mutex<RefCell<Option<LedAndRtc>>> = Mutex::new(RefCell::new(None));

/// Entry point to our bare-metal application.
///
/// The `#[rp2040_hal::entry]` macro ensures the Cortex-M start-up code calls this function
/// as soon as all global variables and the spinlock are initialised.
///
/// The function configures the RP2040 peripherals, sets up the RTC irq then goes into sleep in an
/// infinite loop. If there is an LED connected to that pin, it will toggle very minute (1/60 Hz).
#[rp2040_hal::entry]
fn main() -> ! {
    // Grab our singleton objects
    let mut pac = pac::Peripherals::take().unwrap();

    // Set up the watchdog driver - needed by the clock setup code
    let mut watchdog = hal::Watchdog::new(pac.WATCHDOG);

    // Configure the clocks
    let clocks = hal::clocks::init_clocks_and_plls(
        XTAL_FREQ_HZ,
        pac.XOSC,
        pac.CLOCKS,
        pac.PLL_SYS,
        pac.PLL_USB,
        &mut pac.RESETS,
        &mut watchdog,
    )
    .unwrap();

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
    // we can use reconfigure() instead of into_pull_up_input()
    // since the variable we're pushing it into has that type
    let led = pins.gpio25.reconfigure();

    // Prepare the RTC for the example using the 1/1/0 (Day/Month/Year) at 0:00:00 as the initial
    // day and time (it may not have been a Monday but it doesn't matter for this example).
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
    rtc.enable_interrupt();

    // Give away our pin and rtc by moving them into the `GLOBAL_SHARED` variable.
    // We won't need to access them in the main thread again
    critical_section::with(|cs| {
        GLOBAL_SHARED.borrow(cs).replace(Some((led, rtc)));
    });

    // Unmask the RTC IRQ so that the NVIC interrupt controller
    // will jump to the interrupt function when the interrupt occurs.
    // We do this last so that the interrupt can't go off while
    // it is in the middle of being configured
    unsafe {
        pac::NVIC::unmask(pac::Interrupt::RTC_IRQ);
    }

    loop {
        // interrupts handle everything else in this example.
        cortex_m::asm::wfi();
    }
}

#[allow(non_snake_case)]
#[allow(static_mut_refs)] // See https://github.com/rust-embedded/cortex-m/pull/561
#[interrupt]
fn RTC_IRQ() {
    // The `#[interrupt]` attribute covertly converts this to `&'static mut Option<LedAndRtc>`
    static mut LED_AND_RTC: Option<LedAndRtc> = None;

    // This is one-time lazy initialisation. We steal the variables given to us
    // via `GLOBAL_SHARED`.
    if LED_AND_RTC.is_none() {
        critical_section::with(|cs| {
            *LED_AND_RTC = GLOBAL_SHARED.borrow(cs).take();
        });
    }

    // Need to check if our Option<LedAndButtonPins> contains our pins
    // LED_AND_RTC is an `&'static mut Option<LedAndRtc>` thanks to the interrupt macro's magic.
    // The pattern binding mode handles an ergonomic conversion of the match from `if let Some(led_and_rtc)`
    // to `if let Some(ref mut led_and_rtc)`.
    //
    // https://doc.rust-lang.org/reference/patterns.html#binding-modes
    if let Some(led_and_rtc) = LED_AND_RTC {
        // borrow led and rtc by *destructuring* the tuple
        // these will be of type `&mut LedPin` and `&mut RealTimeClock`, so we don't have
        // to move them back into the static after we use them
        let (led, rtc) = led_and_rtc;

        // Toggle the led
        let _ = led.toggle();

        // clear the interrupt flag so that it stops firing for now and can be triggered again.
        rtc.clear_interrupt();
    }
}

// End of file
