//! # Pico PWM Audio Example
//!
//! Drives GPIO0 with PWM to generate an audio signal for use with a speaker.
//!
//! Note that you will need to supply your own speaker. When hooked up to GPIO0,
//! you should hear an audible chime.
//!
//! See the `Cargo.toml` file for Copyright and license details.

#![no_std]
#![no_main]

use embedded_hal::digital::v2::OutputPin;
use hal::{
    clocks::{ClocksManager, InitError},
    pac::interrupt,
    pll::{common_configs::PLL_USB_48MHZ, PLLConfig},
    pwm::{FreeRunning, Pwm0},
    Watchdog,
};

// The macro for our start-up function
use rp_pico::entry;

// GPIO traits
use embedded_hal::PwmPin;

// Time handling traits
use embedded_time::rate::*;

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

/// Signed 8-bit raw PCM samples
///
/// If you want to create your own, use Audacity to create a recording with a
/// sample rate of 32,000 Hz and then export it as raw 8-bit signed PCM with no
/// file header.
const AUDIO: &[u8] = include_bytes!("pico_pwm_audio.raw");

/// The hardware PWM driver that is shared with the interrupt routine.
static mut PWM: Option<hal::pwm::Slice<Pwm0, FreeRunning>> = None;

// Output from vocalc.py
/// This clock rate is closest to 176,400,000 Hz, which is a multiple of 44,100 Hz.
#[allow(dead_code)]
const PLL_SYS_176MHZ: PLLConfig<Megahertz> = PLLConfig {
    vco_freq: Megahertz(528),
    refdiv: 1,
    post_div1: 3,
    post_div2: 1,
};

/// This clock rate is closest to 131,072,000 Hz, which is a multiple of 32,000 Hz (the audio sample rate).
#[allow(dead_code)]
const PLL_SYS_131MHZ: PLLConfig<Megahertz> = PLLConfig {
    vco_freq: Megahertz(1572),
    refdiv: 1,
    post_div1: 6,
    post_div2: 2,
};

/// Initialize system clocks and PLLs according to specified configs
#[allow(clippy::too_many_arguments)]
fn init_clocks_and_plls_cfg(
    xosc_crystal_freq: u32,
    xosc_dev: pac::XOSC,
    clocks_dev: pac::CLOCKS,
    pll_sys_dev: pac::PLL_SYS,
    pll_usb_dev: pac::PLL_USB,
    pll_sys_cfg: PLLConfig<Megahertz>,
    pll_usb_cfg: PLLConfig<Megahertz>,
    resets: &mut pac::RESETS,
    watchdog: &mut Watchdog,
) -> Result<ClocksManager, InitError> {
    let xosc = hal::xosc::setup_xosc_blocking(xosc_dev, xosc_crystal_freq.Hz())
        .map_err(InitError::XoscErr)?;

    // Configure watchdog tick generation to tick over every microsecond
    watchdog.enable_tick_generation((xosc_crystal_freq / 1_000_000) as u8);

    let mut clocks = ClocksManager::new(clocks_dev);

    let pll_sys = hal::pll::setup_pll_blocking(
        pll_sys_dev,
        xosc.operating_frequency().into(),
        pll_sys_cfg,
        &mut clocks,
        resets,
    )
    .map_err(InitError::PllError)?;
    let pll_usb = hal::pll::setup_pll_blocking(
        pll_usb_dev,
        xosc.operating_frequency().into(),
        pll_usb_cfg,
        &mut clocks,
        resets,
    )
    .map_err(InitError::PllError)?;

    clocks
        .init_default(&xosc, &pll_sys, &pll_usb)
        .map_err(InitError::ClockError)?;
    Ok(clocks)
}

/// Entry point to our bare-metal application.
///
/// The `#[entry]` macro ensures the Cortex-M start-up code calls this function
/// as soon as all global variables are initialised.
///
/// The function configures the RP2040 peripherals, then outputs an audio signal
/// on GPIO0 in an infinite loop.
#[entry]
fn main() -> ! {
    // Grab our singleton objects
    let mut pac = pac::Peripherals::take().unwrap();
    let core = pac::CorePeripherals::take().unwrap();

    // Set up the watchdog driver - needed by the clock setup code
    let mut watchdog = hal::Watchdog::new(pac.WATCHDOG);

    // Configure the clocks
    // Note that we choose a nonstandard system clock rate, so that we can closely
    // control the PWM cycles so that they're (close to) a multiple of the audio sample rate.
    let clocks = init_clocks_and_plls_cfg(
        rp_pico::XOSC_CRYSTAL_FREQ,
        pac.XOSC,
        pac.CLOCKS,
        pac.PLL_SYS,
        pac.PLL_USB,
        PLL_SYS_131MHZ,
        PLL_USB_48MHZ,
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
    let mut delay = cortex_m::delay::Delay::new(core.SYST, clocks.system_clock.freq().integer());

    // Init PWMs
    let pwm_slices = hal::pwm::Slices::new(pac.PWM, &mut pac.RESETS);

    // Setup the LED pin
    let mut led_pin = pins.led.into_push_pull_output();

    // Configure PWM0
    let mut pwm = pwm_slices.pwm0;
    pwm.default_config();

    // 131,000,000 Hz divided by (top * div.int).
    //
    // fPWM = fSYS / ((TOP + 1) * (CSR_PH_CORRECT + 1) * (DIV_INT + (DIV_FRAC / 16)))
    //
    // 32kHz ~= 131,000,000 / ((4096 + 1) * 1 * 1)
    pwm.set_top(4096);
    pwm.set_div_int(1);

    pwm.enable_interrupt();
    pwm.enable();

    // Output channel A on PWM0 to GPIO0
    pwm.channel_a.output_to(pins.gpio0);

    unsafe {
        // Share the PWM with our interrupt routine.
        PWM = Some(pwm);

        // Unmask the PWM_IRQ_WRAP interrupt so we start receiving events.
        pac::NVIC::unmask(pac::interrupt::PWM_IRQ_WRAP);
    }

    // N.B: Note that this would be much more efficiently implemented by using a DMA controller
    // to continuously feed audio samples to the PWM straight from memory. The hardware
    // is set up in a way where a rollover interrupt from the PWM channel can trigger a DMA
    // request for the next byte (or u16) of memory.
    // So while this is a good illustration for driving an audio signal from PWM, use DMA instead
    // for a real project.

    // Infinite loop, with LED on while audio is playing.
    loop {
        let _ = led_pin.set_high();

        for i in AUDIO {
            // Rescale from signed i8 numbers to 0..4096 (the TOP register we specified earlier)
            //
            // The PWM channel will increment an internal counter register, and if the counter is
            // above or equal to this number, the PWM will output a logic high signal.
            let i = ((*i as u16) << 4).wrapping_add(2048) & 0xFFF;

            cortex_m::interrupt::free(|_| {
                // SAFETY: Interrupt cannot currently use this while we're in a critical section.
                let channel = &mut unsafe { PWM.as_mut() }.unwrap().channel_a;
                channel.set_duty(i);
            });

            // Throttle until the PWM channel delivers us an interrupt saying it's done
            // with this cycle (the internal counter wrapped). The interrupt handler will
            // clear the interrupt and we'll send out the next sample.
            cortex_m::asm::wfi();
        }

        // Flash the LED to let the user know that the audio is looping.
        let _ = led_pin.set_low();
        delay.delay_ms(50);
    }
}

#[interrupt]
fn PWM_IRQ_WRAP() {
    // SAFETY: This is not used outside of interrupt critical sections in the main thread.
    let pwm = unsafe { PWM.as_mut() }.unwrap();

    // Clear the interrupt (so we don't immediately re-enter this routine)
    pwm.clear_interrupt();
}

// End of file
