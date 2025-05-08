//! # GPIO IRQ Example
//!
//! This application demonstrates use of GPIO Interrupts.
//! It is also intended as a general introduction to interrupts with rp235x.
//!
//! Each GPIO can be triggered on the input being high (LevelHigh), being low (LevelLow)
//! starting high and then going low (EdgeLow) or starting low and becoming high (EdgeHigh)
//!
//! In this example, we trigger on EdgeLow. Our input pin configured to be pulled to the high logic-level
//! via an internal pullup resistor. This resistor is quite weak, so you can bring the logic level back to low
//! via an external jumper wire or switch.
//! Whenever we see the edge transition, we will toggle the output on GPIO25 - this is the LED pin on a Pico.
//!
//! Note that this demo does not perform any [software debouncing](https://en.wikipedia.org/wiki/Switch#Contact_bounce).
//! You can fix that through hardware, or you could disable the button interrupt in the interrupt and re-enable it
//! some time later using one of the Alarms of the Timer peripheral - this is left as an exercise for the reader.
//!
//! It may need to be adapted to your particular board layout and/or pin assignment.
//!
//! See the `Cargo.toml` file for Copyright and license details.

#![no_std]
#![no_main]

// Ensure we halt the program on panic (if we don't mention this crate it won't
// be linked)
use panic_halt as _;

// Alias for our HAL crate
use rp235x_hal as hal;

// Some traits we need
use embedded_hal::digital::StatefulOutputPin;

// Some more helpful aliases
use core::cell::RefCell;
use critical_section::Mutex;
use hal::gpio;

/// Tell the Boot ROM about our application
#[link_section = ".start_block"]
#[used]
pub static IMAGE_DEF: hal::block::ImageDef = hal::block::ImageDef::secure_exe();

/// External high-speed crystal on the Raspberry Pi Pico 2 board is 12 MHz.
/// Adjust if your board has a different frequency
const XTAL_FREQ_HZ: u32 = 12_000_000u32;

// Pin types quickly become very long!
// We'll create some type aliases using `type` to help with that

/// This pin will be our output - it will drive an LED if you run this on a Pico
type LedPin = gpio::Pin<gpio::bank0::Gpio25, gpio::FunctionSioOutput, gpio::PullNone>;

/// This pin will be our interrupt source.
/// It will trigger an interrupt if pulled to ground (via a switch or jumper wire)
type ButtonPin = gpio::Pin<gpio::bank0::Gpio26, gpio::FunctionSioInput, gpio::PullUp>;

/// Since we're always accessing these pins together we'll store them in a tuple.
/// Giving this tuple a type alias means we won't need to use () when putting them
/// inside an Option. That will be easier to read.
type LedAndButton = (LedPin, ButtonPin);

/// This how we transfer our Led and Button pins into the Interrupt Handler.
/// We'll have the option hold both using the LedAndButton type.
/// This will make it a bit easier to unpack them later.
static GLOBAL_STATE: Mutex<RefCell<Option<LedAndButton>>> = Mutex::new(RefCell::new(None));

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

    // Configure the clocks
    let _clocks = hal::clocks::init_clocks_and_plls(
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

    // Set up the GPIO pin that will be our input
    let in_pin = pins.gpio26.reconfigure();

    // Trigger on the 'falling edge' of the input pin.
    // This will happen as the button is being pressed
    in_pin.set_interrupt_enabled(gpio::Interrupt::EdgeLow, true);

    // Give away our pins by moving them into the `GLOBAL_PINS` variable.
    // We won't need to access them in the main thread again
    critical_section::with(|cs| {
        GLOBAL_STATE.borrow(cs).replace(Some((led, in_pin)));
    });

    // Unmask the IRQ for I/O Bank 0 so that the RP2350's interrupt controller
    // (NVIC in Arm mode, or Xh3irq in RISC-V mode) will jump to the interrupt
    // function when the interrupt occurs. We do this last so that the interrupt
    // can't go off while it is in the middle of being configured
    unsafe {
        hal::arch::interrupt_unmask(hal::pac::Interrupt::IO_IRQ_BANK0);
    }

    // Enable interrupts on this core
    unsafe {
        hal::arch::interrupt_enable();
    }

    loop {
        // interrupts handle everything else in this example.
        hal::arch::wfi();
    }
}

/// This is the interrupt handler that fires when GPIO Bank 0 detects an event
/// (like an edge).
///
/// We give it an unmangled name so that it replaces the default (empty)
/// handler. These handlers are referred to by name from the Interrupt Vector
/// Table created by cortex-m-rt.
#[allow(non_snake_case)]
#[no_mangle]
fn IO_IRQ_BANK0() {
    // Enter a critical section to ensure this code cannot be concurrently
    // executed on the other core. This also protects us if the main thread
    // decides to execute this function (which it shouldn't, but we can't stop
    // them if they wanted to).
    critical_section::with(|cs| {
        // Grab a mutable reference to the global state, using the CS token as
        // proof we have turned off interrupts. Performs a run-time borrow check
        // of the RefCell to ensure no-one else is currently borrowing it (and
        // they shouldn't, because we're in a critical section right now).
        let mut maybe_state = GLOBAL_STATE.borrow_ref_mut(cs);
        // Need to check if our Option<LedAndButtonPins> contains our pins
        if let Some((led, button)) = maybe_state.as_mut() {
            // Check if the interrupt source is from the pushbutton going from high-to-low.
            // Note: this will always be true in this example, as that is the only enabled GPIO interrupt source
            if button.interrupt_status(gpio::Interrupt::EdgeLow) {
                // toggle can't fail, but the embedded-hal traits always allow for it
                // we can discard the return value by assigning it to an unnamed variable
                let _ = led.toggle();
                // Our interrupt doesn't clear itself.
                // Do that now so we don't immediately jump back to this interrupt handler.
                button.clear_interrupt(gpio::Interrupt::EdgeLow);
            }
        }
    });
}

/// Program metadata for `picotool info`
#[link_section = ".bi_entries"]
#[used]
pub static PICOTOOL_ENTRIES: [hal::binary_info::EntryAddr; 5] = [
    hal::binary_info::rp_cargo_bin_name!(),
    hal::binary_info::rp_cargo_version!(),
    hal::binary_info::rp_program_description!(c"GPIO IRQ Example"),
    hal::binary_info::rp_cargo_homepage_url!(),
    hal::binary_info::rp_program_build_attribute!(),
];

// End of file
