//! # RAM Vector Table example
//!
//! This application demonstrates how to create a new Interrupt Vector Table in RAM.
//! To demonstrate the extra utility of this, we also replace an entry in the Vector Table
//! with a new one.
//!
//! See the `Cargo.toml` file for Copyright and license details.

#![no_std]
#![no_main]

// Ensure we halt the program on panic
use panic_halt as _;

// Alias for our HAL crate
use rp235x_hal as hal;

// Some things we need
use core::cell::RefCell;
use critical_section::Mutex;
use embedded_hal::delay::DelayNs;
use embedded_hal::digital::StatefulOutputPin;
use hal::fugit::MicrosDurationU32;
use hal::timer::Alarm;

use hal::pac::interrupt;
use hal::vector_table::VectorTable;
use static_cell::ConstStaticCell;

// Memory that will hold our vector table in RAM
static RAM_VTABLE: ConstStaticCell<VectorTable> = ConstStaticCell::new(VectorTable::new());

// Give our LED and Alarm a type alias to make it easier to refer to them
type LedAndAlarm = (
    hal::gpio::Pin<hal::gpio::bank0::Gpio25, hal::gpio::FunctionSioOutput, hal::gpio::PullDown>,
    hal::timer::Alarm0<hal::timer::CopyableTimer0>,
);

// Place our LED and Alarm type in a static variable, so we can access it from interrupts
static LED_AND_ALARM: Mutex<RefCell<Option<LedAndAlarm>>> = Mutex::new(RefCell::new(None));

// Period that each of the alarms will be set for - 1 second and 300ms respectively
const SLOW_BLINK_INTERVAL_US: MicrosDurationU32 = MicrosDurationU32::secs(1);
const FAST_BLINK_INTERVAL_US: MicrosDurationU32 = MicrosDurationU32::millis(300);

/// Tell the Boot ROM about our application
#[link_section = ".start_block"]
#[used]
pub static IMAGE_DEF: hal::block::ImageDef = hal::block::ImageDef::secure_exe();

/// External high-speed crystal on the Raspberry Pi Pico 2 board is 12 MHz.
/// Adjust if your board has a different frequency
const XTAL_FREQ_HZ: u32 = 12_000_000u32;

/// Entry point to our bare-metal application.
///
/// The `#[hal::entry]` macro ensures the Cortex-M start-up code calls this function
/// as soon as all global variables and the spinlock are initialised.
///
/// The function configures the rp235x peripherals, then toggles a GPIO pin in
/// an infinite loop. If there is an LED connected to that pin, it will blink.
#[hal::entry]
fn main() -> ! {
    // Grab our singleton objects
    let mut pac = hal::pac::Peripherals::take().unwrap();

    // Set up the watchdog driver - needed by the clock setup code
    let mut watchdog = hal::Watchdog::new(pac.WATCHDOG);
    // The single-cycle I/O block controls our GPIO pins
    let sio = hal::Sio::new(pac.SIO);

    // Need to make a reference to the Peripheral Base at this scope to avoid confusing the borrow checker
    let ppb = &mut pac.PPB;
    let ram_vtable = RAM_VTABLE.take();

    // Copy the vector table that cortex_m_rt produced into the RAM vector table
    ram_vtable.init(ppb);
    // Replace the function that is called on Alarm0 interrupts with a new one
    ram_vtable.register_handler(
        hal::pac::Interrupt::TIMER0_IRQ_0 as usize,
        timer_irq0_replacement,
    );

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
    .ok()
    .unwrap();

    // Create simple delay
    let mut delay = hal::Timer::new_timer1(pac.TIMER1, &mut pac.RESETS, &clocks);

    // Set the pins to their default state
    let pins = hal::gpio::Pins::new(
        pac.IO_BANK0,
        pac.PADS_BANK0,
        sio.gpio_bank0,
        &mut pac.RESETS,
    );

    // Configure GPIO25 as an output
    let led_pin = pins.gpio25.into_push_pull_output();

    let mut timer = hal::Timer::new_timer0(pac.TIMER0, &mut pac.RESETS, &clocks);
    critical_section::with(|cs| {
        let mut alarm = timer.alarm_0().unwrap();
        // Schedule an alarm in 1 second
        let _ = alarm.schedule(SLOW_BLINK_INTERVAL_US);
        // Enable generating an interrupt on alarm
        alarm.enable_interrupt();
        // Move alarm into ALARM, so that it can be accessed from interrupts
        LED_AND_ALARM.borrow(cs).replace(Some((led_pin, alarm)));
    });
    // Unmask the timer0 IRQ so that it will generate an interrupt
    unsafe {
        cortex_m::peripheral::NVIC::unmask(hal::pac::Interrupt::TIMER0_IRQ_0);
    }

    // After 5 seconds, switch to our modified vector rable
    delay.delay_ms(5000);
    unsafe {
        critical_section::with(|_| {
            ram_vtable.activate(ppb);
        });
    }

    loop {
        // Wait for an interrupt to fire before doing any more work
        hal::arch::wfi();
    }
}

// Regular interrupt handler for Alarm0. The `interrupt` macro will perform some transformations to ensure
// that this interrupt entry ends up in the vector table.
#[interrupt]
fn TIMER0_IRQ_0() {
    critical_section::with(|cs| {
        // Temporarily take our LED_AND_ALARM
        let ledalarm = LED_AND_ALARM.borrow(cs).take();
        if let Some((mut led, mut alarm)) = ledalarm {
            // Clear the alarm interrupt or this interrupt service routine will keep firing
            alarm.clear_interrupt();
            // Schedule a new alarm after SLOW_BLINK_INTERVAL_US have passed (1 second)
            let _ = alarm.schedule(SLOW_BLINK_INTERVAL_US);
            // Blink the LED so we know we hit this interrupt
            led.toggle().unwrap();
            // Return LED_AND_ALARM into our static variable
            LED_AND_ALARM
                .borrow(cs)
                .replace_with(|_| Some((led, alarm)));
        }
    });
}

// This is the function we will use to replace TIMER_IRQ_0 in our RAM Vector Table
extern "C" fn timer_irq0_replacement() {
    critical_section::with(|cs| {
        let ledalarm = LED_AND_ALARM.borrow(cs).take();
        if let Some((mut led, mut alarm)) = ledalarm {
            // Clear the alarm interrupt or this interrupt service routine will keep firing
            alarm.clear_interrupt();
            // Schedule a new alarm after FAST_BLINK_INTERVAL_US have passed (300 milliseconds)
            let _ = alarm.schedule(FAST_BLINK_INTERVAL_US);
            led.toggle().unwrap();
            // Return LED_AND_ALARM into our static variable
            LED_AND_ALARM
                .borrow(cs)
                .replace_with(|_| Some((led, alarm)));
        }
    });
}

/// Program metadata for `picotool info`
#[link_section = ".bi_entries"]
#[used]
pub static PICOTOOL_ENTRIES: [hal::binary_info::EntryAddr; 5] = [
    hal::binary_info::rp_cargo_bin_name!(),
    hal::binary_info::rp_cargo_version!(),
    hal::binary_info::rp_program_description!(c"Interrupt Vector Table Example"),
    hal::binary_info::rp_cargo_homepage_url!(),
    hal::binary_info::rp_program_build_attribute!(),
];

// End of file
