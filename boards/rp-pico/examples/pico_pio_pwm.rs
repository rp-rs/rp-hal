//! # Pico PIO PWM Blink Example
//!
//! Fades the LED on a Pico board using the PIO peripheral with an pwm program.
//!
//! This will fade in the LED attached to GP25, which is the pin the Pico
//! uses for the on-board LED.
//!
//! This example uses a few advance pio tricks such as side setting pins and instruction injection.
//!
//! See the `Cargo.toml` file for Copyright and license details. Except for the pio program which is subject to a different license.

#![no_std]
#![no_main]

use defmt::info;
use defmt_rtt as _;
// The macro for our start-up function
use rp_pico::entry;

// Ensure we halt the program on panic (if we don't mention this crate it won't
// be linked)
use panic_halt as _;

// Pull in any important traits
use rp_pico::hal::prelude::*;

// A shorter alias for the Peripheral Access Crate, which provides low-level
// register access
use rp_pico::hal::pac;

// A shorter alias for the Hardware Abstraction Layer, which provides
// higher-level drivers.
use rp_pico::hal;

// Import pio crates
use hal::pio::{PIOBuilder, Running, StateMachine, Tx, ValidStateMachine, SM0};
use pio::{Instruction, InstructionOperands, OutDestination};
use pio_proc::pio_file;

/// Set pio pwm period
///
/// This uses a sneaky trick to set a second value besides the duty cycle.
/// We first write a value to the tx fifo. But instead of the normal instructions we
/// have stopped the state machine and inject our own instructions that move the written value to the ISR.
fn pio_pwm_set_period<T: ValidStateMachine>(
    sm: StateMachine<(hal::pac::PIO0, SM0), Running>,
    tx: &mut Tx<T>,
    period: u32,
) -> StateMachine<(hal::pac::PIO0, SM0), Running> {
    // To make sure the inserted instructions actually use our newly written value
    // We first busy loop to empty the queue. (Which typically should be the case)
    while !tx.is_empty() {}

    let mut sm = sm.stop();
    tx.write(period);
    sm.exec_instruction(Instruction {
        operands: InstructionOperands::PULL {
            if_empty: false,
            block: false,
        },
        delay: 0,
        side_set: None,
    });
    sm.exec_instruction(Instruction {
        operands: InstructionOperands::OUT {
            destination: OutDestination::ISR,
            bit_count: 32,
        },
        delay: 0,
        side_set: None,
    });
    sm.start()
}

/// Set pio pwm duty cycle
///
/// The value written to the TX FIFO is used directly by the normal pio program
fn pio_pwm_set_level<T: ValidStateMachine>(tx: &mut Tx<T>, level: u32) {
    // Write duty cycle to TX Fifo
    tx.write(level);
}

/// Entry point to our bare-metal application.
///
/// The `#[entry]` macro ensures the Cortex-M start-up code calls this function
/// as soon as all global variables are initialised.
///
/// The function configures the RP2040 peripherals, then fades the LED in an
/// infinite loop.
#[entry]
fn main() -> ! {
    // Grab our singleton objects
    let mut pac = pac::Peripherals::take().unwrap();
    let core = pac::CorePeripherals::take().unwrap();

    // Set up the watchdog driver - needed by the clock setup code
    let mut watchdog = hal::Watchdog::new(pac.WATCHDOG);

    // Configure the clocks
    //
    // The default is to generate a 125 MHz system clock
    let clocks = hal::clocks::init_clocks_and_plls(
        rp_pico::XOSC_CRYSTAL_FREQ,
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
    let pins = rp_pico::Pins::new(
        pac.IO_BANK0,
        pac.PADS_BANK0,
        sio.gpio_bank0,
        &mut pac.RESETS,
    );

    // The delay object lets us wait for specified amounts of time (in
    // milliseconds)
    let mut delay = cortex_m::delay::Delay::new(core.SYST, clocks.system_clock.freq().to_Hz());

    let (mut pio0, sm0, _, _, _) = pac.PIO0.split(&mut pac.RESETS);

    // Create a pio program
    let program = pio_file!("./examples/pwm.pio", select_program("pwm"),);
    let installed = pio0.install(&program.program).unwrap();

    // Set gpio25 to pio
    let _led: hal::gpio::Pin<_, hal::gpio::FunctionPio0> = pins.led.into_mode();
    let led_pin_id = 25;

    // Build the pio program and set pin both for set and side set!
    // We are running with the default divider which is 1 (max speed)
    let (mut sm, _, mut tx) = PIOBuilder::from_program(installed)
        .set_pins(led_pin_id, 1)
        .side_set_pin_base(led_pin_id)
        .build(sm0);

    // Set pio pindir for gpio25
    sm.set_pindirs([(led_pin_id, hal::pio::PinDir::Output)]);

    // Start state machine
    let sm = sm.start();

    // Set period
    pio_pwm_set_period(sm, &mut tx, u16::MAX as u32 - 1);

    // Loop forever and adjust duty cycle to make te led brighter
    let mut level = 0;
    loop {
        info!("Level = {}", level);
        pio_pwm_set_level(&mut tx, level * level);
        level = (level + 1) % 256;
        delay.delay_ms(10);
    }
}

// End of file
