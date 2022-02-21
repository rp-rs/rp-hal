//! # Pico External Interrupt Example
//!
//! Implements reading of a quadrature encoder using an external interrupt
//! to react immediately when the encoder is clicked. The interrupt handler
//! reads another pin to determine directiona and updates a count variable
//! accordingly. The click and direction are used to set the brightness
//! on the Pico board using PWM.
//!
//! The interrupt is on gpio pin 8, but can be applied equally to any gpio pin.
//!
//! See the `Cargo.toml` file for Copyright and licence details.

#![no_std]
#![no_main]

// The macro for our start-up function
use core::{cell::RefCell, ops::DerefMut};
use cortex_m_rt::entry;

use cortex_m::{
    delay::Delay,
    interrupt::{free, Mutex},
};

// If the following two lines are included, the code may run
// in debug. They are not needed if run in release, ie
// cargo run --example pico_external_interrupt --release
// use defmt::info;
// use defmt_rtt as _;

use embedded_hal::digital::v2::InputPin;
use embedded_hal::PwmPin;
use embedded_time::fixed_point::FixedPoint;

// Ensure we halt the program on panic (if we don't mention this crate it won't
// be linked)
use panic_halt as _;

// All the device definitions required in the code
use rp_pico::hal;
use rp_pico::hal::{
    clocks::{init_clocks_and_plls, Clock},
    gpio::{
        self,
        bank0::{Gpio8, Gpio9},
        Floating, Input, Pin,
    },
    pac::{self, interrupt},
    Sio, Watchdog,
};

// Define types to make it more readable passing to interrupt
type CountPinType = Pin<Gpio8, Input<Floating>>;
type DirPinType = Pin<Gpio9, Input<Floating>>;

// Variables required inside the interrupt handler have to be transfered using
// some mechanism. A mutext is used here. Mutexes are defined for the pins, because
// they are allocated in main, but used in teh interrupt handler, and a global
// variable, G_COUNT, is also defined to transfer the count into and out of the
// interrupt handler.
static G_COUNTPIN: Mutex<RefCell<Option<CountPinType>>> = Mutex::new(RefCell::new(None));
static G_DIRPIN: Mutex<RefCell<Option<DirPinType>>> = Mutex::new(RefCell::new(None));
static G_COUNT: Mutex<RefCell<Option<i32>>> = Mutex::new(RefCell::new(None));

/// Entry point to our bare-metal application.
///
/// The `#[entry]` macro ensures the Cortex-M start-up code calls this function
/// as soon as all global variables are initialised.
///
/// The function configures the RP2040 peripherals, then just goes into a
/// main loop where it checks if the count has changed. It it has, it reports
/// it using log!
///
/// Actually changing count occurs in the interrupt handler.
#[entry]
fn main() -> ! {
    // Grab our singleton objects
    let mut pac = pac::Peripherals::take().unwrap();

    // We need the core peripheral to set up a delay
    let core = pac::CorePeripherals::take().unwrap();

    let mut watchdog = Watchdog::new(pac.WATCHDOG);

    // Set up the SIO that controls all the pints
    let sio = Sio::new(pac.SIO);

    // Set up the clocks to give us access to the delay
    // function in the main loop
    let external_xtal_freq_hz = 12_000_000u32;
    let clocks = init_clocks_and_plls(
        external_xtal_freq_hz,
        pac.XOSC,
        pac.CLOCKS,
        pac.PLL_SYS,
        pac.PLL_USB,
        &mut pac.RESETS,
        &mut watchdog,
    )
    .ok()
    .unwrap();

    let mut delay = Delay::new(core.SYST, clocks.system_clock.freq().integer());
    // let mut timer = hal::Timer::new(pac.TIMER, &mut pac.RESETS);

    // Set up the pins used
    let pins = rp_pico::Pins::new(
        pac.IO_BANK0,
        pac.PADS_BANK0,
        sio.gpio_bank0,
        &mut pac.RESETS,
    );

    // Set up PWM on pico LED>. See pico_pwm_blink.rs example for more
    let mut pwm_slices = hal::pwm::Slices::new(pac.PWM, &mut pac.RESETS);
    let pwm = &mut pwm_slices.pwm4;
    pwm.set_ph_correct();
    pwm.enable();
    let channel = &mut pwm.channel_b;
    channel.output_to(pins.led);

    // (1) Set up the two specific pins used for input
    let count_pin: CountPinType = pins.gpio8.into_floating_input();
    let dir_pin: DirPinType = pins.gpio9.into_floating_input();

    // Set up the interrupt on the count_pin, with type EdgeHigh, so
    // trigger when the pin transitions from low to high.
    count_pin.set_interrupt_enabled(gpio::Interrupt::EdgeHigh, true);

    // Set up a global count variable for use in the main and interrupt routines
    let count = 0;

    // Enable the exeternal interrupt on IO BANK0
    unsafe {
        pac::NVIC::unmask(pac::Interrupt::IO_IRQ_BANK0);
    }

    // Lock the direction pin, count pin, and count value global variables
    //  while they're initialised with the values created in lines
    // from (1) onwards above
    free(|cs| {
        G_DIRPIN.borrow(cs).replace(Some(dir_pin));
        G_COUNTPIN.borrow(cs).replace(Some(count_pin));
        G_COUNT.borrow(cs).replace(Some(count));
    });
    let mut old: i32 = 0;

    // Loop at 20 Hz, roughly.
    loop {
        // Each time round the loop, access the G_COUNT global variable from
        // with a critical section, so that access is safe. Compare to a stored
        // value in old, and take action if the value has changed. Here, the code
        // sends a value to the PWM of the LED on the RP2040.
        free(|cs| {
            if let Some(count) = G_COUNT.borrow(cs).borrow_mut().deref_mut() {
                if old != *count {
                    old = *count;
                    channel.set_duty((old % 100) as u16 * 250);
                }
            };
        });
        delay.delay_ms(50);
    }
}

/// Interrupt handler for the exeternal interrupt on pin 8
#[interrupt]
fn IO_IRQ_BANK0() {
    // This interrupt routine takes over ownership of the direction pin
    // and only runs the first time the handler is called
    static mut DIRPIN: Option<DirPinType> = None;
    if DIRPIN.is_none() {
        free(|cs| {
            *DIRPIN = G_DIRPIN.borrow(cs).take();
        });
    }

    // Use free to lock the global variables while they are accessed. We
    // know the interrupt pin was triggered, so we clear the interrupt,
    // then read the direction to see whether to count up or down. Finally,
    // count is updated within the critical section.
    //
    // (This particular quadrature encode has two overlapping pulse outputs.
    // Which output goes high first depends on the direction of rotation. In
    // this code, only output is used to trigger the interrupt on the rp2020,
    // the other is checked immediately, and used to determine where to count
    // up or down. The encoder never issues the pulses too close together, which
    // might have meant the second pulse has gone back to zero before being read.
    // Make sure you check your own device datasheets)
    free(|cs| {
        if let Some(ref mut count_pin) = G_COUNTPIN.borrow(cs).borrow_mut().deref_mut() {
            count_pin.clear_interrupt(gpio::Interrupt::EdgeHigh);

            if let Some(d) = DIRPIN {
                if let Some(ref mut count) = G_COUNT.borrow(cs).borrow_mut().deref_mut() {
                    if d.is_high().unwrap() {
                        *count -= 1;
                    } else {
                        *count += 1
                    }
                }
            }
        }
    });
}
