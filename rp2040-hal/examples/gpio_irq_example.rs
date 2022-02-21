//! # GPIO IRQ Example
//!
//! This application demonstrates use of GPIO Interrupts.
//! It is also intended as a general introduction to interrupts with RP2040.
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
//! See the `Cargo.toml` file for Copyright and licence details.

#![no_std]
#![no_main]

// The macro for our start-up function
use cortex_m_rt::entry;

// Ensure we halt the program on panic (if we don't mention this crate it won't
// be linked)
use panic_halt as _;

// Alias for our HAL crate
use rp2040_hal as hal;

// A shorter alias for the Peripheral Access Crate, which provides low-level
// register access
use hal::pac;

// Some traits we need
use embedded_hal::digital::v2::ToggleableOutputPin;

// Our interrupt macro
use hal::pac::interrupt;

// Some short-cuts to useful types
use core::cell::RefCell;
use cortex_m::interrupt::Mutex;
use rp2040_hal::gpio;

// The GPIO interrupt type we're going to generate
use rp2040_hal::gpio::Interrupt::EdgeLow;

/// The linker will place this boot block at the start of our program image. We
/// need this to help the ROM bootloader get our code up and running.
#[link_section = ".boot2"]
#[used]
pub static BOOT2: [u8; 256] = rp2040_boot2::BOOT_LOADER_W25Q080;

/// External high-speed crystal on the Raspberry Pi Pico board is 12 MHz. Adjust
/// if your board has a different frequency
const XTAL_FREQ_HZ: u32 = 12_000_000u32;

// Pin types quickly become very long!
// We'll create some type aliases using `type` to help with that

/// This pin will be our output - it will drive an LED if you run this on a Pico
type LedPin = gpio::Pin<gpio::bank0::Gpio25, gpio::PushPullOutput>;

/// This pin will be our interrupt source.
/// It will trigger an interrupt if pulled to ground (via a switch or jumper wire)
type ButtonPin = gpio::Pin<gpio::bank0::Gpio26, gpio::PullUpInput>;

/// Since we're always accessing these pins together we'll store them in a tuple.
/// Giving this tuple a type alias means we won't need to use () when putting them
/// inside an Option. That will be easier to read.
type LedAndButton = (LedPin, ButtonPin);

/// This how we transfer our Led and Button pins into the Interrupt Handler.
/// We'll have the option hold both using the LedAndButton type.
/// This will make it a bit easier to unpack them later.
static GLOBAL_PINS: Mutex<RefCell<Option<LedAndButton>>> = Mutex::new(RefCell::new(None));

/// Entry point to our bare-metal application.
///
/// The `#[entry]` macro ensures the Cortex-M start-up code calls this function
/// as soon as all global variables are initialised.
///
/// The function configures the RP2040 peripherals, then toggles a GPIO pin in
/// an infinite loop. If there is an LED connected to that pin, it will blink.
#[entry]
fn main() -> ! {
    // Grab our singleton objects
    let mut pac = pac::Peripherals::take().unwrap();

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
    .ok()
    .unwrap();

    // The single-cycle I/O block controls our GPIO pins
    let sio = hal::Sio::new(pac.SIO);

    // Set the pins to their default state
    let pins = hal::gpio::Pins::new(
        pac.IO_BANK0,
        pac.PADS_BANK0,
        sio.gpio_bank0,
        &mut pac.RESETS,
    );

    // Configure GPIO 25 as an output to drive our LED.
    // we can use into_mode() instead of into_pull_up_input()
    // since the variable we're pushing it into has that type
    let led = pins.gpio25.into_mode();

    // Set up the GPIO pin that will be our input
    let in_pin = pins.gpio26.into_mode();

    // Trigger on the 'falling edge' of the input pin.
    // This will happen as the button is being pressed
    in_pin.set_interrupt_enabled(EdgeLow, true);

    // Give away our pins by moving them into the `GLOBAL_PINS` variable.
    // We won't need to access them in the main thread again
    cortex_m::interrupt::free(|cs| {
        GLOBAL_PINS.borrow(cs).replace(Some((led, in_pin)));
    });

    // Unmask the IO_BANK0 IRQ so that the NVIC interrupt controller
    // will jump to the interrupt function when the interrupt occurs.
    // We do this last so that the interrupt can't go off while
    // it is in the middle of being configured
    unsafe {
        pac::NVIC::unmask(pac::Interrupt::IO_IRQ_BANK0);
    }

    loop {
        // interrupts handle everything else in this example.
        // if we wanted low power we could go to sleep. to
        // keep this example simple we'll just execute a `nop`.
        // the `nop` (No Operation) instruction does nothing,
        // but if we have no code here clippy would complain.
        cortex_m::asm::nop();
    }
}

#[interrupt]
fn IO_IRQ_BANK0() {
    // The `#[interrupt]` attribute covertly converts this to `&'static mut Option<LedAndButton>`
    static mut LED_AND_BUTTON: Option<LedAndButton> = None;

    // This is one-time lazy initialisation. We steal the variables given to us
    // via `GLOBAL_PINS`.
    if LED_AND_BUTTON.is_none() {
        cortex_m::interrupt::free(|cs| {
            *LED_AND_BUTTON = GLOBAL_PINS.borrow(cs).take();
        });
    }

    // Need to check if our Option<LedAndButtonPins> contains our pins
    if let Some(gpios) = LED_AND_BUTTON {
        // borrow led and button by *destructuring* the tuple
        // these will be of type `&mut LedPin` and `&mut ButtonPin`, so we don't have
        // to move them back into the static after we use them
        let (led, button) = gpios;

        // toggle can't fail, but the embedded-hal traits always allow for it
        // we can discard the return value by assigning it to an unnamed variable
        let _ = led.toggle();

        // Our interrupt doesn't clear itself.
        // Do that now so we don't immediately jump back to this interrupt handler.
        button.clear_interrupt(EdgeLow);
    }
}

// End of file
