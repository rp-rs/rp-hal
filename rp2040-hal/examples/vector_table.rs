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
use rp2040_hal as hal;

// A shorter alias for the Peripheral Access Crate
use hal::pac;

// Some traits we need
use core::cell::RefCell;
use critical_section::Mutex;
use embedded_hal::digital::v2::ToggleableOutputPin;
use fugit::MicrosDurationU32;
use pac::interrupt;
use rp2040_hal::clocks::Clock;
use rp2040_hal::timer::Alarm;
use rp2040_hal::vector_table::VectorTable;

// Memory that will hold our vector table in RAM
static mut RAM_VTABLE: VectorTable = VectorTable::new();

// Give our LED and Alarm a type alias to make it easier to refer to them
type LedAndAlarm = (
    hal::gpio::Pin<hal::gpio::bank0::Gpio25, hal::gpio::PushPullOutput>,
    hal::timer::Alarm0,
);

// Place our LED and Alarm type in a static variable, so we can access it from interrupts
static mut LED_AND_ALARM: Mutex<RefCell<Option<LedAndAlarm>>> = Mutex::new(RefCell::new(None));

// Period that each of the alarms will be set for - 1 second and 300ms respectively
const SLOW_BLINK_INTERVAL_US: MicrosDurationU32 = MicrosDurationU32::secs(1);
const FAST_BLINK_INTERVAL_US: MicrosDurationU32 = MicrosDurationU32::millis(300);

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
    let core = pac::CorePeripherals::take().unwrap();

    // Set up the watchdog driver - needed by the clock setup code
    let mut watchdog = hal::Watchdog::new(pac.WATCHDOG);
    // The single-cycle I/O block controls our GPIO pins
    let sio = hal::Sio::new(pac.SIO);

    // Need to make a reference to the Peripheral Base at this scope to avoid confusing the borrow checker
    let ppb = &mut pac.PPB;
    unsafe {
        // Copy the vector table that cortex_m_rt produced into the RAM vector table
        RAM_VTABLE.init(ppb);
        // Replace the function that is called on Alarm0 interrupts with a new one
        RAM_VTABLE.register_handler(pac::Interrupt::TIMER_IRQ_0 as usize, timer_irq0_replacement);
    }

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
    let mut delay = cortex_m::delay::Delay::new(core.SYST, clocks.system_clock.freq().to_Hz());

    // Set the pins to their default state
    let pins = hal::gpio::Pins::new(
        pac.IO_BANK0,
        pac.PADS_BANK0,
        sio.gpio_bank0,
        &mut pac.RESETS,
    );

    // Configure GPIO25 as an output
    let led_pin = pins.gpio25.into_push_pull_output();

    let mut timer = hal::Timer::new(pac.TIMER, &mut pac.RESETS);
    critical_section::with(|cs| {
        let mut alarm = timer.alarm_0().unwrap();
        // Schedule an alarm in 1 second
        let _ = alarm.schedule(SLOW_BLINK_INTERVAL_US);
        // Enable generating an interrupt on alarm
        alarm.enable_interrupt();
        // Move alarm into ALARM, so that it can be accessed from interrupts
        unsafe {
            LED_AND_ALARM.borrow(cs).replace(Some((led_pin, alarm)));
        }
    });
    // Unmask the timer0 IRQ so that it will generate an interrupt
    unsafe {
        pac::NVIC::unmask(pac::Interrupt::TIMER_IRQ_0);
    }

    // After 5 seconds, switch to our modified vector rable
    delay.delay_ms(5000);
    unsafe {
        critical_section::with(|_| {
            RAM_VTABLE.activate(ppb);
        });
    }

    loop {
        // Wait for an interrupt to fire before doing any more work
        cortex_m::asm::wfi();
    }
}

// Regular interrupt handler for Alarm0. The `interrupt` macro will perform some transformations to ensure
// that this interrupt entry ends up in the vector table.
#[interrupt]
fn TIMER_IRQ_0() {
    critical_section::with(|cs| {
        // Temporarily take our LED_AND_ALARM
        let ledalarm = unsafe { LED_AND_ALARM.borrow(cs).take() };
        if let Some((mut led, mut alarm)) = ledalarm {
            // Clear the alarm interrupt or this interrupt service routine will keep firing
            alarm.clear_interrupt();
            // Schedule a new alarm after SLOW_BLINK_INTERVAL_US have passed (1 second)
            let _ = alarm.schedule(SLOW_BLINK_INTERVAL_US);
            // Blink the LED so we know we hit this interrupt
            led.toggle().unwrap();
            // Return LED_AND_ALARM into our static variable
            unsafe {
                LED_AND_ALARM
                    .borrow(cs)
                    .replace_with(|_| Some((led, alarm)));
            }
        }
    });
}

// This is the function we will use to replace TIMER_IRQ_0 in our RAM Vector Table
extern "C" fn timer_irq0_replacement() {
    critical_section::with(|cs| {
        let ledalarm = unsafe { LED_AND_ALARM.borrow(cs).take() };
        if let Some((mut led, mut alarm)) = ledalarm {
            // Clear the alarm interrupt or this interrupt service routine will keep firing
            alarm.clear_interrupt();
            // Schedule a new alarm after FAST_BLINK_INTERVAL_US have passed (300 milliseconds)
            let _ = alarm.schedule(FAST_BLINK_INTERVAL_US);
            led.toggle().unwrap();
            // Return LED_AND_ALARM into our static variable
            unsafe {
                LED_AND_ALARM
                    .borrow(cs)
                    .replace_with(|_| Some((led, alarm)));
            }
        }
    });
}

// End of file
