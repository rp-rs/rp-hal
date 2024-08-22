//! # POWMAN Example
//!
//! This application demonstrates talking to the POWMAN peripheral.
//!
//! See the `Cargo.toml` file for Copyright and license details.

#![no_std]
#![no_main]

use rp235x_hal::{
    self as hal, gpio, pac,
    powman::{AotClockSource, FractionalFrequency, Powman},
    uart::{DataBits, StopBits, UartConfig, UartPeripheral},
};

use cortex_m_rt::exception;
use pac::interrupt;

// Some traits we need
use core::fmt::Write;
use hal::fugit::RateExtU32;
use hal::Clock;

/// Tell the Boot ROM about our application
#[link_section = ".start_block"]
#[used]
pub static IMAGE_DEF: hal::block::ImageDef = hal::block::ImageDef::secure_exe();

/// External high-speed crystal on the Raspberry Pi Pico 2 board is 12 MHz.
/// Adjust if your board has a different frequency
const XTAL_FREQ_HZ: u32 = 12_000_000u32;

struct GlobalUart {
    inner: critical_section::Mutex<core::cell::RefCell<Option<MyUart>>>,
}

type MyUart = UartPeripheral<
    hal::uart::Enabled,
    pac::UART0,
    (
        gpio::Pin<gpio::bank0::Gpio0, gpio::FunctionUart, gpio::PullDown>,
        gpio::Pin<gpio::bank0::Gpio1, gpio::FunctionUart, gpio::PullDown>,
    ),
>;

static GLOBAL_UART: GlobalUart = GlobalUart {
    inner: critical_section::Mutex::new(core::cell::RefCell::new(None)),
};

impl core::fmt::Write for &GlobalUart {
    fn write_str(&mut self, s: &str) -> core::fmt::Result {
        critical_section::with(|cs| {
            let mut cell = self.inner.borrow_ref_mut(cs);
            let uart = cell.as_mut().unwrap();
            uart.write_str(s)
        })
    }
}

impl GlobalUart {
    fn init(&self, uart: MyUart) {
        critical_section::with(|cs| {
            self.inner.borrow(cs).replace(Some(uart));
        });
    }
}

/// Entry point to our bare-metal application.
///
/// The `#[hal::entry]` macro ensures the Cortex-M start-up code calls this function
/// as soon as all global variables and the spinlock are initialised.
///
/// The function configures the rp235x peripherals, then writes to the UART in
/// an infinite loop.
#[hal::entry]
fn main() -> ! {
    // Grab our singleton objects
    let mut pac = pac::Peripherals::take().unwrap();
    let mut cp = cortex_m::Peripherals::take().unwrap();

    // Enable the cycle counter
    cp.DCB.enable_trace();
    cp.DWT.enable_cycle_counter();

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

    let uart_pins = (
        // UART TX (characters sent from rp235x) on pin 1 (GPIO0)
        pins.gpio0.into_function::<gpio::FunctionUart>(),
        // UART RX (characters received by rp235x) on pin 2 (GPIO1)
        pins.gpio1.into_function::<gpio::FunctionUart>(),
    );
    let uart = hal::uart::UartPeripheral::new(pac.UART0, uart_pins, &mut pac.RESETS)
        .enable(
            UartConfig::new(115200.Hz(), DataBits::Eight, None, StopBits::One),
            clocks.peripheral_clock.freq(),
        )
        .unwrap();

    GLOBAL_UART.init(uart);

    let mut powman = Powman::new(pac.POWMAN, None);

    _ = writeln!(&GLOBAL_UART, "\n\nPOWMAN Test");

    print_aot_status(&mut powman);
    _ = writeln!(&GLOBAL_UART, "AOT time: 0x{:016x}", powman.aot_get_time());

    unsafe {
        cortex_m::peripheral::NVIC::unmask(pac::Interrupt::POWMAN_IRQ_TIMER);
    }

    _ = writeln!(&GLOBAL_UART, "Starting AOT...");
    powman.aot_start();
    // we don't know what oscillator we're on, so give it time to start whatever it is
    cortex_m::asm::delay(150_000);
    print_aot_status(&mut powman);
    rollover_test(&mut powman);
    loop_test(&mut powman);
    alarm_test(&mut powman);

    let source = AotClockSource::Xosc(FractionalFrequency::from_hz(12_000_000));
    _ = writeln!(&GLOBAL_UART, "Switching AOT to {}", source);
    powman.aot_set_clock(source).expect("selecting XOSC");
    print_aot_status(&mut powman);
    loop_test(&mut powman);

    let source = AotClockSource::Lposc(FractionalFrequency::from_hz(32768));
    _ = writeln!(&GLOBAL_UART, "Switching AOT to {}", source);
    powman.aot_set_clock(source).expect("selecting LPOSC");
    print_aot_status(&mut powman);
    loop_test(&mut powman);

    _ = writeln!(&GLOBAL_UART, "Rebooting now");

    panic!("Finished!");
}

fn print_aot_status(powman: &mut Powman) {
    if powman.aot_is_running() {
        _ = writeln!(
            &GLOBAL_UART,
            "AOT is running on {}",
            powman.aot_get_clock().unwrap()
        );
    } else {
        _ = writeln!(&GLOBAL_UART, "AOT is not running.");
    }
}

/// Check we can roll-over a 32-bit boundary
fn rollover_test(powman: &mut Powman) {
    let start_loop = 0x0000_0000_FFFF_FF00u64;
    let end_loop = 0x0000_0001_0000_00FFu64;
    _ = writeln!(
        &GLOBAL_UART,
        "Setting AOT to 0x{:016x} and waiting for rollover...",
        start_loop
    );
    powman.aot_stop();
    powman.aot_set_time(start_loop);
    powman.aot_start();

    let mut last = 0;
    loop {
        let now = powman.aot_get_time();
        if now == end_loop {
            break;
        } else if now < last || now > end_loop {
            panic!("bad AOT read {}!", now);
        }
        last = now;
    }
    _ = writeln!(&GLOBAL_UART, "Rollover test complete - no glitches found",);
    _ = writeln!(&GLOBAL_UART, "Clearing AOT...");
    powman.aot_clear();
    _ = writeln!(&GLOBAL_UART, "AOT time: 0x{:016x}", powman.aot_get_time());
}

/// In this function, we see how long it takes to pass a certain number of ticks.
fn loop_test(powman: &mut Powman) {
    let start_loop = 0;
    let end_loop = 2_000; // 2 seconds
    _ = writeln!(
        &GLOBAL_UART,
        "Setting AOT to 0x{:016x} and benchmarking...",
        start_loop
    );
    powman.aot_stop();
    powman.aot_set_time(start_loop);
    powman.aot_start();

    let start_clocks = cortex_m::peripheral::DWT::cycle_count();
    loop {
        let now = powman.aot_get_time();
        if now == end_loop {
            break;
        }
    }
    let end_clocks = cortex_m::peripheral::DWT::cycle_count();
    // Compare our AOT against our CPU clock speed
    let delta_clocks = end_clocks.wrapping_sub(start_clocks) as u64;
    let delta_ticks = end_loop - start_loop;
    let cycles_per_tick = delta_clocks / delta_ticks;
    // Assume we're running at 150 MHz
    let ms_per_tick = (cycles_per_tick as f32 * 1000.0) / 150_000_000.0;
    let percent = ((ms_per_tick - 1.0) / 1.0) * 100.0;
    _ = writeln!(
        &GLOBAL_UART,
        "Loop complete ... {delta_ticks} ticks in {delta_clocks} CPU clock cycles = {cycles_per_tick} cycles/tick ~= {ms_per_tick} ms/tick ({percent:.3}%)",
    )
    ;
}

/// Test the alarm function
fn alarm_test(powman: &mut Powman) {
    let start_time = 0x1_0000;
    let alarm_time = start_time + 3000; // alarm is 3 seconds in the future
    _ = writeln!(
        &GLOBAL_UART,
        "Setting AOT for {} ms in the future...",
        alarm_time - start_time
    );

    powman.aot_stop();
    powman.aot_set_time(start_time);
    powman.aot_alarm_clear();
    powman.aot_set_alarm(alarm_time);
    powman.aot_alarm_interrupt_enable();
    powman.aot_alarm_enable();
    powman.aot_start();

    _ = writeln!(
        &GLOBAL_UART,
        "Sleeping until alarm (* = wakeup, ! = POWMAN interrupt)...",
    );
    while !powman.aot_alarm_ringing() {
        cortex_m::asm::wfe();
        _ = write!(&GLOBAL_UART, "*",);
    }

    _ = writeln!(
        &GLOBAL_UART,
        "\nAlarm fired at 0x{:016x}. Clearing alarm.",
        powman.aot_get_time()
    );

    powman.aot_alarm_clear();

    if powman.aot_alarm_ringing() {
        panic!("Alarm did not clear!");
    }

    _ = writeln!(&GLOBAL_UART, "Alarm cleared OK");
}

#[interrupt]
#[allow(non_snake_case)]
fn POWMAN_IRQ_TIMER() {
    Powman::static_aot_alarm_interrupt_disable();
    _ = write!(&GLOBAL_UART, "!");
    cortex_m::asm::sev();
}

#[exception]
unsafe fn HardFault(ef: &cortex_m_rt::ExceptionFrame) -> ! {
    let _ = writeln!(&GLOBAL_UART, "HARD FAULT:\n{:#?}", ef);

    hal::reboot::reboot(
        hal::reboot::RebootKind::BootSel {
            msd_disabled: false,
            picoboot_disabled: false,
        },
        hal::reboot::RebootArch::Normal,
    );
}

#[panic_handler]
fn panic(info: &core::panic::PanicInfo) -> ! {
    let _ = writeln!(&GLOBAL_UART, "PANIC:\n{:?}", info);

    hal::reboot::reboot(
        hal::reboot::RebootKind::BootSel {
            msd_disabled: false,
            picoboot_disabled: false,
        },
        hal::reboot::RebootArch::Normal,
    );
}

/// Program metadata for `picotool info`
#[link_section = ".bi_entries"]
#[used]
pub static PICOTOOL_ENTRIES: [hal::binary_info::EntryAddr; 5] = [
    hal::binary_info::rp_cargo_bin_name!(),
    hal::binary_info::rp_cargo_version!(),
    hal::binary_info::rp_program_description!(c"POWMAN Test Example"),
    hal::binary_info::rp_cargo_homepage_url!(),
    hal::binary_info::rp_program_build_attribute!(),
];

// End of file
