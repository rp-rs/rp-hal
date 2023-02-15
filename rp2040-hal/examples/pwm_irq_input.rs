//! # PWM IRQ Input Example
//!
//! Read a 5V 50Hz PWM servo input signal from gpio pin 1 and turn the LED on when
//! the input signal is high ( > 1600 us duty pulse width ) and off when low ( < 1400 us ).
//!
//! This signal is commonly used with radio control model systems and small servos.
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
use rp2040_hal as hal;

// Some traits we need
use embedded_hal::digital::v2::OutputPin;
use embedded_hal::PwmPin;

// Our interrupt macro
use hal::pac::interrupt;

// A shorter alias for the Peripheral Access Crate, which provides low-level
// register access
use hal::pac;

// Shorter alias for gpio and pwm modules
use hal::gpio;
use hal::pwm;

// Some short-cuts to useful types for sharing data with the interrupt handlers
use core::cell::RefCell;
use critical_section::Mutex;

/// The linker will place this boot block at the start of our program image. We
/// need this to help the ROM bootloader get our code up and running.
/// Note: This boot block is not necessary when using a rp-hal based BSP
/// as the BSPs already perform this step.
#[link_section = ".boot2"]
#[used]
pub static BOOT2: [u8; 256] = rp2040_boot2::BOOT_LOADER_GENERIC_03H;

/// 50 Hz PWM servo signals have a pulse width between 1000 us and 2000 us with
/// 1500 us as the centre point. us is the abbreviation for micro seconds.

/// The PWM threshold value for turning off the LED in us
const LOW_US: u16 = 1475;

/// The PWM threshold value for turning on the LED in us
const HIGH_US: u16 = 1525;

/// External high-speed crystal on the Raspberry Pi Pico board is 12 MHz. Adjust
/// if your board has a different frequency
const XTAL_FREQ_HZ: u32 = 12_000_000u32;

/// Pin types quickly become very long!
/// We'll create some type aliases using `type` to help with that

/// This pin will be our output - it will drive an LED if you run this on a Pico
type LedPin = gpio::Pin<gpio::bank0::Gpio25, gpio::PushPullOutput>;

/// This pin will be our input for a 50 Hz servo PWM signal
type InputPwmPin = gpio::Pin<gpio::bank0::Gpio1, gpio::FunctionPwm>;

/// This will be our PWM Slice - it will interpret the PWM signal from the pin
type PwmSlice = pwm::Slice<pwm::Pwm0, pwm::InputHighRunning>;

/// Since we're always accessing these pins together we'll store them in a tuple.
/// Giving this tuple a type alias means we won't need to use () when putting them
/// inside an Option. That will be easier to read.
type LedInputAndPwm = (LedPin, InputPwmPin, PwmSlice);

/// This how we transfer our LED pin, input pin and PWM slice into the Interrupt Handler.
/// We'll have the option hold both using the LedAndInput type.
/// This will make it a bit easier to unpack them later.
static GLOBAL_PINS: Mutex<RefCell<Option<LedInputAndPwm>>> = Mutex::new(RefCell::new(None));

/// Entry point to our bare-metal application.
///
/// The `#[rp2040_hal::entry]` macro ensures the Cortex-M start-up code calls this function
/// as soon as all global variables and the spinlock are initialised.
///
/// The function configures the RP2040 peripherals, then fades the LED in an
/// infinite loop.
#[rp2040_hal::entry]
fn main() -> ! {
    // Grab our singleton objects
    let mut pac = pac::Peripherals::take().unwrap();

    // Set up the watchdog driver - needed by the clock setup code
    let mut watchdog = hal::Watchdog::new(pac.WATCHDOG);

    // Configure the clocks
    //
    // The default is to generate a 125 MHz system clock
    hal::clocks::init_clocks_and_plls(
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

    // Set the pins up according to their function on this particular board
    let pins = gpio::Pins::new(
        pac.IO_BANK0,
        pac.PADS_BANK0,
        sio.gpio_bank0,
        &mut pac.RESETS,
    );

    // Init PWMs
    let pwm_slices = pwm::Slices::new(pac.PWM, &mut pac.RESETS);

    // Configure PWM0 slice
    // The PWM slice clock should only run when the input is high (InputHighRunning)
    let mut pwm: pwm::Slice<_, pwm::InputHighRunning> = pwm_slices.pwm0.into_mode();

    // Divide the 125 MHz system clock by 125 to give a 1 MHz PWM slice clock (1 us per tick)
    pwm.set_div_int(125);
    pwm.enable();

    // Connect to GPI O1 as the input to channel B on PWM0
    let channel = &mut pwm.channel_b;
    let input_pin = channel.input_from(pins.gpio1);
    channel.enable();

    // Enable an interrupt whenever GPI O1 goes from high to low (the end of a pulse)
    input_pin.set_interrupt_enabled(gpio::Interrupt::EdgeLow, true);

    // Configure GPIO 25 as an output to drive our LED.
    // we can use into_mode() instead of into_pull_up_input()
    // since the variable we're pushing it into has that type
    let led = pins.gpio25.into_mode();

    // Give away our pins by moving them into the `GLOBAL_PINS` variable.
    // We won't need to access them in the main thread again
    critical_section::with(|cs| {
        GLOBAL_PINS.borrow(cs).replace(Some((led, input_pin, pwm)));
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
        cortex_m::asm::wfi();
    }
}

#[interrupt]
fn IO_IRQ_BANK0() {
    // The `#[interrupt]` attribute covertly converts this to `&'static mut Option<LedAndInput>`
    static mut LED_INPUT_AND_PWM: Option<LedInputAndPwm> = None;

    // This is one-time lazy initialisation. We steal the variables given to us
    // via `GLOBAL_PINS`.
    if LED_INPUT_AND_PWM.is_none() {
        critical_section::with(|cs| {
            *LED_INPUT_AND_PWM = GLOBAL_PINS.borrow(cs).take();
        });
    }

    // Need to check if our Option<LedInputAndPwm> contains our pins and pwm slice
    // borrow led, input and pwm by *destructuring* the tuple
    // these will be of type `&mut LedPin`, `&mut InputPwmPin` and `&mut PwmSlice`, so we
    // don't have to move them back into the static after we use them
    if let Some((led, input, pwm)) = LED_INPUT_AND_PWM {
        // Check if the interrupt source is from the input pin going from high-to-low.
        // Note: this will always be true in this example, as that is the only enabled GPIO interrupt source
        if input.interrupt_status(gpio::Interrupt::EdgeLow) {
            // Read the width of the last pulse from the PWM Slice counter
            let pulse_width_us = pwm.get_counter();

            // if the PWM signal indicates low, turn off the LED
            if pulse_width_us < LOW_US {
                // set_low can't fail, but the embedded-hal traits always allow for it
                // we can discard the Result by transforming it to an Option
                led.set_low().ok();
            }
            // if the PWM signal indicates low, turn on the LED
            else if pulse_width_us > HIGH_US {
                // set_high can't fail, but the embedded-hal traits always allow for it
                // we can discard the Result by transforming it to an Option
                led.set_high().ok();
            }

            // If the PWM signal was in the dead-zone between LOW and HIGH, don't change the LED's
            // state. The dead-zone avoids the LED flickering rapidly when receiving a signal close
            // to the mid-point, 1500 us in this case.

            // Reset the pwm counter back to 0, ready for the next pulse
            pwm.set_counter(0);

            // Our interrupt doesn't clear itself.
            // Do that now so we don't immediately jump back to this interrupt handler.
            input.clear_interrupt(gpio::Interrupt::EdgeLow);
        }
    }
}

// End of file
