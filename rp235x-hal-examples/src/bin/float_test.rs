//! # DCP Example
//!
//! This example only works on Arm.
//!
//! This application demonstrates performing floating point operations
//! using the *Double Co-Processor*. Note that by default, DCP acceleration
//! is enabled. To see how slow software floating point operations are,
//! build this example with `--no-default-features`.
//!
//! It may need to be adapted to your particular board layout and/or pin
//! assignment.
//!
//! Typical output (with default features):
//!
//! ```text
//! Floating Point Test
//! Testing software f64 addition
//! Running 1000 loops for f64
//! Start: acc=0, arg=0.048638031340549406
//! End  : acc=48.63803134055, took 23.8 cycles per op
//! Testing DCP f64 addition
//! Running 1000 loops for f64
//! Start: acc=0, arg=0.048638031340549406
//! End  : acc=48.63803134055, took 37.9 cycles per op
//! Testing FPU f32 addition
//! Running 1000 loops for f32
//! Start: acc=0, arg=0.04863803
//! End  : acc=48.63764, took 3.5 cycles per op
//! Testing software f64 multiplication
//! Running 1000 loops for f64
//! Start: acc=0.048638031340549406, arg=1.0001
//! End  : acc=0.053753069001926, took 40.8 cycles per op
//! Testing DCP f64 multiplication
//! Running 1000 loops for f64
//! Start: acc=0.048638031340549406, arg=1.0001
//! End  : acc=0.053753069001926, took 54.7 cycles per op
//! Testing FPU f32 multiplication
//! Running 1000 loops for f32
//! Start: acc=0.04863803, arg=1.0001
//! End  : acc=0.05375396, took 3.4 cycles per op
//! Rebooting now
//! PANIC:
//! PanicInfo { payload: Any { .. }, message: Some(Finished!), location: Location { file: "rp235x-hal/examples/float_test.rs", line: 166, col: 5 }, can_unwind: true, force_no_backtrace: false }
//! ```
//!
//! Typical output (with no default features):
//!
//! ```text
//! Floating Point Test
//! Testing software f64 addition
//! Running 1000 loops for f64
//! Start: acc=0, arg=0.8188217810460334
//! End  : acc=818.8217810460395, took 151.3 cycles per op
//! Testing DCP f64 addition
//! Running 1000 loops for f64
//! Start: acc=0, arg=0.8188217810460334
//! End  : acc=818.8217810460395, took 38.0 cycles per op
//! Testing FPU f32 addition
//! Running 1000 loops for f32
//! Start: acc=0, arg=0.8188218
//! End  : acc=818.82947, took 3.4 cycles per op
//! Testing software f64 multiplication
//! Running 1000 loops for f64
//! Start: acc=0.8188217810460334, arg=1.0001
//! End  : acc=0.9049334951218081, took 123.0 cycles per op
//! Testing DCP f64 multiplication
//! Running 1000 loops for f64
//! Start: acc=0.8188217810460334, arg=1.0001
//! End  : acc=0.9049334951218081, took 55.0 cycles per op
//! Testing FPU f32 multiplication
//! Running 1000 loops for f32
//! Start: acc=0.8188218, arg=1.0001
//! End  : acc=0.9049483, took 3.4 cycles per op
//! Rebooting now
//! PANIC:
//! PanicInfo { payload: Any { .. }, message: Some(Finished!), location: Location { file: "rp235x-hal/examples/float_test.rs", line: 166, col: 5 }, can_unwind: true, force_no_backtrace: false }
//! ```
//!
//! See the `Cargo.toml` file for Copyright and license details.

#![no_std]
#![no_main]

use rp235x_hal as hal;

use hal::{
    gpio,
    uart::{DataBits, StopBits, UartConfig, UartPeripheral},
};

use cortex_m_rt::exception;

// Some things we need
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
    hal::pac::UART0,
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
    let mut pac = hal::pac::Peripherals::take().unwrap();
    let mut cp = cortex_m::Peripherals::take().unwrap();

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

    writeln!(&GLOBAL_UART, "\n\nFloating Point Test").unwrap();

    // Optionally start a DCP operation, but not finish it, to see if the save/restore fires.
    //
    // If you uncomment this, you'll observe the DCP operations take ~46 cycles longer
    // to do all the stacking/restoring of DCP state.

    // unsafe {
    //     let val0 = 0;
    //     let val1 = 0;
    //     core::arch::asm!(
    //         "mcrr p4,#1,{0},{1},c0 // WXUP {0},{1} - write {1}|{0} to xm and xe",
    //         in(reg) val0,
    //         in(reg) val1
    //     );
    // }

    cp.DCB.enable_trace();
    cp.DWT.enable_cycle_counter();

    let int_seed = get_random_seed();
    // make up a random number between 0 and 1 so the optimiser can't cheat and
    // we are forced to do the maths
    let seed = (int_seed & u128::from(u64::MAX)) as f64 / u64::MAX as f64;

    writeln!(&GLOBAL_UART, "Testing aeabi f64 addition").unwrap();
    benchmark(0.0, seed, |x, y| x + y, &cp.DWT);
    writeln!(&GLOBAL_UART, "Testing DCP f64 addition").unwrap();
    benchmark(0.0, seed, hal::dcp::dadd, &cp.DWT);
    writeln!(&GLOBAL_UART, "Testing aeabi f32 addition").unwrap();
    benchmark(0.0, seed as f32, |x, y| x + y, &cp.DWT);

    writeln!(&GLOBAL_UART, "Testing aeabi f64 multiplication").unwrap();
    benchmark(seed, 1.0001f64, |x, y| x * y, &cp.DWT);
    writeln!(&GLOBAL_UART, "Testing DCP f64 multiplication").unwrap();
    benchmark(seed, 1.0001f64, hal::dcp::dmul, &cp.DWT);
    writeln!(&GLOBAL_UART, "Testing aeabi f32 multiplication").unwrap();
    benchmark(seed as f32, 1.0001f32, |x, y| x * y, &cp.DWT);

    writeln!(&GLOBAL_UART, "Rebooting now").unwrap();

    panic!("Finished!");
}

/// Use the SysInfo API to get a boot-time generated random number.
fn get_random_seed() -> u128 {
    let mut buffer = [0u32; 5];
    let sys_info_mask = 0x0010;
    unsafe { hal::rom_data::get_sys_info(buffer.as_mut_ptr(), buffer.len(), sys_info_mask) };
    let mut result: u128 = 0;
    for word in &buffer[1..] {
        result <<= 32;
        result |= u128::from(*word);
    }
    result
}

/// Run the given operation in a loop.
///
/// Uses the DWT to calculate elapsed CPU cycles.
fn benchmark<T, F>(acc: T, arg: T, mut f: F, dwt: &cortex_m::peripheral::DWT)
where
    T: core::fmt::Display + core::marker::Copy,
    F: FnMut(T, T) -> T,
{
    const LOOPS: u16 = 1000;
    writeln!(
        &GLOBAL_UART,
        "Running {} loops for {}",
        LOOPS,
        core::any::type_name::<T>()
    )
    .unwrap();
    let mut acc: T = acc;
    writeln!(&GLOBAL_UART, "Start: acc={}, arg={}", acc, arg).unwrap();

    let start_count = dwt.cyccnt.read();
    for _ in 0..LOOPS {
        acc = f(acc, arg);
    }
    let end_count = dwt.cyccnt.read();

    let delta = end_count.wrapping_sub(start_count);
    let cycles_per_op = delta as f32 / LOOPS as f32;
    writeln!(
        &GLOBAL_UART,
        "End  : acc={acc}, took {cycles_per_op:.1} cycles per op"
    )
    .unwrap();
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
    hal::binary_info::rp_program_description!(c"Floating Point Test"),
    hal::binary_info::rp_cargo_homepage_url!(),
    hal::binary_info::rp_program_build_attribute!(),
];

// End of file
