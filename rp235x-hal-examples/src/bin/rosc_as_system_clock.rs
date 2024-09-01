//! # ROSC as system clock Example
//!
//! This application demonstrates how to use the ROSC as the system clock on the rp235x.
//!
//! It shows setting the frequency of the ROSC to a measured known frequency, and contains
//! helper functions to configure the ROSC drive strength to reach a desired target frequency.
//!
//! See the `Cargo.toml` file for Copyright and license details.

#![no_std]
#![no_main]

// Ensure we halt the program on panic (if we don't mention this crate it won't
// be linked)
use panic_halt as _;

// Alias for our HAL crate
use rp235x_hal as hal;

use embedded_hal::delay::DelayNs;
use embedded_hal::digital::OutputPin;
use fugit::{HertzU32, RateExtU32};
use hal::clocks::{Clock, ClockSource, ClocksManager, StoppableClock};
use hal::pac::rosc::ctrl::FREQ_RANGE_A;
use hal::pac::{CLOCKS, ROSC};
use hal::rosc::RingOscillator;

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
/// The function attempts to find appropriate settings for the RingOscillator to reach a target
/// frequency, and then logs the actual attained frequency.
///
/// The main reasons you'd want to use this is for power-saving in applications where precise
/// timings are not critical (you don't need to use USB peripherals for example).
/// Using the ROSC as the system clock allows under-clocking or over-clocking rp235x, and
/// it also can allow fast waking from a dormant state on the order of µs, which the XOSC cannot do.
///
/// A motivating application for this was a flir lepton thermal camera module which
/// makes thermal images available via SPI at a rate of around 9Hz.  Using the rp235xs ROSC, we
/// are able to clock out the thermal image via SPI and then enter dormant mode until the next vsync
/// interrupt wakes us again, saving some power.
#[hal::entry]
fn main() -> ! {
    // Set target rosc frequency to 150Mhz
    // Setting frequencies can be a matter of a bit of trial and error to see what
    // actual frequencies you can easily hit.  In practice, the lowest achieved with this method
    // is around 32Mhz, and it seems to be able to ramp up to around 230Mhz
    let desired_rosc_freq: HertzU32 = 150_000_000.Hz();
    // Grab our singleton objects
    let mut pac = hal::pac::Peripherals::take().unwrap();

    // The single-cycle I/O block controls our GPIO pins
    let sio = hal::Sio::new(pac.SIO);

    // Set the pins to their default state
    let pins = hal::gpio::Pins::new(
        pac.IO_BANK0,
        pac.PADS_BANK0,
        sio.gpio_bank0,
        &mut pac.RESETS,
    );
    // Configure GPIO25 as an output
    let mut led_pin = pins.gpio25.into_push_pull_output();
    led_pin.set_low().unwrap();

    // Setup the crystal oscillator to do accurate measurements against
    let xosc = hal::xosc::setup_xosc_blocking(pac.XOSC, XTAL_FREQ_HZ.Hz()).unwrap();

    // Find appropriate settings for the desired ring oscillator frequency.
    let measured_rosc_frequency =
        find_target_rosc_frequency(&pac.ROSC, &pac.CLOCKS, desired_rosc_freq);
    let rosc = RingOscillator::new(pac.ROSC);

    // Now initialise the ROSC with the reached frequency and set it as the system clock.
    let rosc = rosc.initialize_with_freq(measured_rosc_frequency);

    let mut clocks = ClocksManager::new(pac.CLOCKS);
    clocks
        .system_clock
        .configure_clock(&rosc, rosc.get_freq())
        .unwrap();

    clocks
        .peripheral_clock
        .configure_clock(&clocks.system_clock, clocks.system_clock.freq())
        .unwrap();
    let mut delay = hal::Timer::new_timer0(pac.TIMER0, &mut pac.RESETS, &clocks);

    // Now we can disable the crystal oscillator and run off the ring oscillator, for power savings.
    let _xosc_disabled = xosc.disable();
    // You may also wish to disable other clocks/peripherals that you don't need.
    clocks.usb_clock.disable();
    clocks.gpio_output0_clock.disable();
    clocks.gpio_output1_clock.disable();
    clocks.gpio_output2_clock.disable();
    clocks.gpio_output3_clock.disable();
    clocks.adc_clock.disable();

    // Check that desired frequency is close to the frequency speed.
    // If it is, turn the LED on.  If not, blink the LED.
    let got_to_within_1_mhz_of_target = desired_rosc_freq
        .to_kHz()
        .abs_diff(measured_rosc_frequency.to_kHz())
        < 1000;

    if got_to_within_1_mhz_of_target {
        // Now it's possible to easily take the ROSC dormant, to be woken by an external interrupt.
        led_pin.set_high().unwrap();
        loop {
            hal::arch::wfi();
        }
    } else {
        loop {
            led_pin.set_high().unwrap();
            delay.delay_ms(500);
            led_pin.set_low().unwrap();
            delay.delay_ms(500);
        }
    }
}

/// Measure the actual speed of the ROSC at the current freq_range and drive strength config
fn rosc_frequency_count_hz(clocks: &CLOCKS) -> HertzU32 {
    // Wait for the frequency counter to be ready
    while clocks.fc0_status().read().running().bit_is_set() {
        hal::arch::nop();
    }

    // Set the speed of the reference clock in kHz.
    clocks
        .fc0_ref_khz()
        .write(|w| unsafe { w.fc0_ref_khz().bits(XTAL_FREQ_HZ / 1000) });

    // Corresponds to a 1ms test time, which seems to give good enough accuracy
    clocks
        .fc0_interval()
        .write(|w| unsafe { w.fc0_interval().bits(10) });

    // We don't really care about the min/max, so these are just set to min/max values.
    clocks
        .fc0_min_khz()
        .write(|w| unsafe { w.fc0_min_khz().bits(0) });
    clocks
        .fc0_max_khz()
        .write(|w| unsafe { w.fc0_max_khz().bits(0xffffffff) });

    // To measure rosc directly we use the value 0x03.
    clocks
        .fc0_src()
        .write(|w| unsafe { w.fc0_src().bits(0x03) });

    // Wait until the measurement is ready
    while clocks.fc0_status().read().done().bit_is_clear() {
        hal::arch::nop();
    }

    let speed_hz = clocks.fc0_result().read().khz().bits() * 1000;
    speed_hz.Hz()
}

/// Resets ROSC frequency range and stages drive strength, then increases the frequency range,
/// drive strength bits, and finally divider in order to try to come close to the desired target
/// frequency, returning the final measured ROSC frequency attained.
fn find_target_rosc_frequency(
    rosc: &ROSC,
    clocks: &CLOCKS,
    target_frequency: HertzU32,
) -> HertzU32 {
    reset_rosc_operating_frequency(rosc);
    let mut div = 1;
    let mut measured_rosc_frequency;
    loop {
        measured_rosc_frequency = rosc_frequency_count_hz(clocks);
        // If it has overshot the target frequency, increase the divider and continue.
        if measured_rosc_frequency > target_frequency {
            div += 1;
            set_rosc_div(rosc, div);
        } else {
            break;
        }
    }
    loop {
        measured_rosc_frequency = rosc_frequency_count_hz(clocks);
        if measured_rosc_frequency > target_frequency {
            // And probably want to step it down a notch?
            break;
        }
        let can_increase = increase_drive_strength(rosc);
        if !can_increase {
            let can_increase_range = increase_freq_range(rosc);
            if !can_increase_range {
                break;
            }
        }
    }
    measured_rosc_frequency
}

fn set_rosc_div(rosc: &ROSC, div: u32) {
    assert!(div <= 32);
    let div = if div == 32 { 0 } else { div };
    rosc.div().write(|w| unsafe { w.bits(0xaa0 + div) });
}

fn reset_rosc_operating_frequency(rosc: &ROSC) {
    // Set divider to 1
    set_rosc_div(rosc, 1);
    rosc.ctrl().write(|w| w.freq_range().low());
    write_freq_stages(rosc, &[0, 0, 0, 0, 0, 0, 0, 0]);
}

fn read_freq_stage(rosc: &ROSC, stage: u8) -> u8 {
    match stage {
        0 => rosc.freqa().read().ds0().bits(),
        1 => rosc.freqa().read().ds1().bits(),
        2 => rosc.freqa().read().ds2().bits(),
        3 => rosc.freqa().read().ds3().bits(),
        4 => rosc.freqb().read().ds4().bits(),
        5 => rosc.freqb().read().ds5().bits(),
        6 => rosc.freqb().read().ds6().bits(),
        7 => rosc.freqb().read().ds7().bits(),
        _ => panic!("invalid frequency drive strength stage"),
    }
}

/// Increase the ROSC drive strength bits for the current freq_range
fn increase_drive_strength(rosc: &ROSC) -> bool {
    const MAX_STAGE_DRIVE: u8 = 3;
    // Assume div is 1, and freq_range is high
    let mut stages: [u8; 8] = [0; 8];
    for (stage_index, stage) in stages.iter_mut().enumerate() {
        *stage = read_freq_stage(rosc, stage_index as u8)
    }
    let num_stages_at_drive_level = match rosc.ctrl().read().freq_range().variant() {
        Some(FREQ_RANGE_A::LOW) => 8,
        Some(FREQ_RANGE_A::MEDIUM) => 6,
        Some(FREQ_RANGE_A::HIGH) => 4,
        Some(FREQ_RANGE_A::TOOHIGH) => panic!("Don't use TOOHIGH freq_range"),
        None => {
            // Start out at initial unset drive stage
            return false;
        }
    };
    let mut next_i = 0;
    for (index, x) in stages[0..num_stages_at_drive_level].windows(2).enumerate() {
        if x[1] < x[0] {
            next_i = index + 1;
            break;
        }
    }
    if stages[next_i] < MAX_STAGE_DRIVE {
        stages[next_i] += 1;
        let min = *stages[0..num_stages_at_drive_level]
            .iter()
            .min()
            .unwrap_or(&0);
        for stage in &mut stages[num_stages_at_drive_level..] {
            *stage = min;
        }
        write_freq_stages(rosc, &stages);
        true
    } else {
        false
    }
}

/// Sets the `freqa` and `freqb` ROSC drive strength stage registers.
fn write_freq_stages(rosc: &ROSC, stages: &[u8; 8]) {
    let passwd: u32 = 0x9696 << 16;
    let mut freq_a = passwd;
    let mut freq_b = passwd;
    for (stage_index, stage) in stages.iter().enumerate().take(4) {
        freq_a |= ((*stage & 0x07) as u32) << (stage_index * 4);
    }
    for (stage_index, stage) in stages.iter().enumerate().skip(4) {
        freq_b |= ((*stage & 0x07) as u32) << ((stage_index - 4) * 4);
    }
    rosc.freqa().write(|w| unsafe { w.bits(freq_a) });
    rosc.freqb().write(|w| unsafe { w.bits(freq_b) });
}

/// Increase the rosc frequency range up to the next step.
/// Returns a boolean to indicate whether the frequency was increased.
fn increase_freq_range(rosc: &ROSC) -> bool {
    match rosc.ctrl().read().freq_range().variant() {
        None => {
            // Initial unset frequency range, move to LOW frequency range
            rosc.ctrl().write(|w| w.freq_range().low());
            // Reset all the drive strength bits.
            write_freq_stages(rosc, &[0, 0, 0, 0, 0, 0, 0, 0]);
            true
        }
        Some(FREQ_RANGE_A::LOW) => {
            // Transition from LOW to MEDIUM frequency range
            rosc.ctrl().write(|w| w.freq_range().medium());
            // Reset all the drive strength bits.
            write_freq_stages(rosc, &[0, 0, 0, 0, 0, 0, 0, 0]);
            true
        }
        Some(FREQ_RANGE_A::MEDIUM) => {
            // Transition from MEDIUM to HIGH frequency range
            rosc.ctrl().write(|w| w.freq_range().high());
            // Reset all the drive strength bits.
            write_freq_stages(rosc, &[0, 0, 0, 0, 0, 0, 0, 0]);
            true
        }
        Some(FREQ_RANGE_A::HIGH) | Some(FREQ_RANGE_A::TOOHIGH) => {
            // Already in the HIGH frequency range, and can't increase
            false
        }
    }
}

/// Program metadata for `picotool info`
#[link_section = ".bi_entries"]
#[used]
pub static PICOTOOL_ENTRIES: [hal::binary_info::EntryAddr; 5] = [
    hal::binary_info::rp_cargo_bin_name!(),
    hal::binary_info::rp_cargo_version!(),
    hal::binary_info::rp_program_description!(c"Ring Oscillator Example"),
    hal::binary_info::rp_cargo_homepage_url!(),
    hal::binary_info::rp_program_build_attribute!(),
];

// End of file
