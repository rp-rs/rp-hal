//! # Pico W Blinky Example
//!
//! Blinks the LED on a Pico W board.
//!
//! This will blink an LED attached to WL_GPIO0, which is the pin the Pico W uses for
//! the on-board LED. It is connected to the the Wifi chip so it cannot be set using RP2040 pins
//!
//! See the `Cargo.toml` file for Copyright and license details.

#![no_std]
#![no_main]

use rp_pico_w::entry;

use defmt::*;
use defmt_rtt as _;
use panic_probe as _;

use rp_pico_w::hal::pac;

use rp_pico_w::gspi::GSpi;
use rp_pico_w::hal;

use embedded_hal_1 as eh_1;

use embedded_hal::digital::v2::OutputPin;

use embassy_executor::raw::TaskPool;
use embassy_executor::Executor;
use embassy_executor::Spawner;
use embassy_net::Stack;
use embassy_time::{Duration, Timer};

/// The function configures the RP2040 peripherals, then blinks the LED in an
/// infinite loop.
#[entry]
fn main() -> ! {
    info!("start");

    // Grab our singleton objects
    let mut pac = pac::Peripherals::take().unwrap();

    // Set up the watchdog driver - needed by the clock setup code
    let mut watchdog = hal::Watchdog::new(pac.WATCHDOG);

    // Configure the clocks
    //
    // The default is to generate a 125 MHz system clock
    let _clocks = hal::clocks::init_clocks_and_plls(
        rp_pico_w::XOSC_CRYSTAL_FREQ,
        pac.XOSC,
        pac.CLOCKS,
        pac.PLL_SYS,
        pac.PLL_USB,
        &mut pac.RESETS,
        &mut watchdog,
    )
    .ok()
    .unwrap();

    let sio = hal::Sio::new(pac.SIO);

    let pins = rp_pico_w::Pins::new(
        pac.IO_BANK0,
        pac.PADS_BANK0,
        sio.gpio_bank0,
        &mut pac.RESETS,
    );

    info!("init time driver");
    let timer = hal::timer::Timer::new(pac.TIMER, &mut pac.RESETS);
    unsafe { rp_pico_w::embassy_time_driver::init(timer) };

    let mut executor = Executor::new();

    // Safety: function never returns, executor is never dropped
    let executor: &'static mut Executor = unsafe { forever_mut(&mut executor) };

    let task_pool: TaskPool<_, 10> = TaskPool::new();
    let task_pool = unsafe { forever(&task_pool) };

    let state = cyw43::State::new();
    let state = unsafe { forever(&state) };

    info!("run spawner");
    executor.run(|spawner| {
        let spawn_token = task_pool.spawn(|| run(spawner, pins, state));
        spawner.spawn(spawn_token).unwrap();
    });
}

unsafe fn forever_mut<T>(r: &'_ mut T) -> &'static mut T {
    core::mem::transmute(r)
}

unsafe fn forever<T>(r: &'_ T) -> &'static T {
    core::mem::transmute(r)
}

async fn run(spawner: Spawner, pins: rp_pico_w::Pins, state: &'static cyw43::State) {
    // These are implicitly used by the spi driver if they are in the correct mode
    let mut spi_cs: hal::gpio::dynpin::DynPin = pins.wl_cs.into();
    // TODO should be high from the beginning :-(
    spi_cs.into_readable_output();
    spi_cs.set_high().unwrap();
    spi_cs.into_push_pull_output();
    spi_cs.set_high().unwrap();

    let mut spi_clk = pins.voltage_monitor_wl_clk.into_push_pull_output();
    spi_clk.set_low().unwrap();

    let mut spi_mosi_miso: hal::gpio::dynpin::DynPin = pins.wl_d.into();
    spi_mosi_miso.into_readable_output();
    spi_mosi_miso.set_low().unwrap();
    spi_mosi_miso.into_push_pull_output();
    spi_mosi_miso.set_low().unwrap();

    let bus = GSpi::new(spi_clk, spi_mosi_miso);
    let spi = eh_1::spi::blocking::ExclusiveDevice::new(bus, spi_cs);

    let pwr = pins.wl_on.into_push_pull_output();

    #[cfg(not(feature = "fix_fw"))]
    let fw = include_bytes!("firmware/43439A0.bin");
    #[cfg(not(feature = "fix_fw"))]
    let clm = include_bytes!("firmware/43439A0_clm.bin");

    // To make flashing faster for development, you may want to flash the firmwares independently
    // at hardcoded addresses, instead of baking them into the program with `include_bytes!`:
    //     probe-rs-cli download 43439A0.bin --format bin --chip RP2040 --base-address 0x10100000
    //     probe-rs-cli download 43439A0.clm_blob --format bin --chip RP2040 --base-address 0x10140000
    #[cfg(feature = "fix_fw")]
    let fw = unsafe { core::slice::from_raw_parts(0x10100000 as *const u8, 224190) };
    #[cfg(feature = "fix_fw")]
    let clm = unsafe { core::slice::from_raw_parts(0x10140000 as *const u8, 4752) };

    use embassy_futures::yield_now;
    yield_now().await;

    info!("create cyw43 driver");
    let (mut control, runner) = cyw43::new(state, pwr, spi, fw).await;

    let task_pool: TaskPool<_, 10> = TaskPool::new();
    let task_pool = unsafe { forever(&task_pool) };
    let spawn_token = task_pool.spawn(|| runner.run());
    spawner.spawn(spawn_token).unwrap();

    info!("init net net device");
    let net_device = control.init(clm).await;
    info!("init net net device done");

    if option_env!("WIFI_NETWORK").is_some() {
        if option_env!("WIFI_PASSWORD").is_some() {
            control
                .join_wpa2(option_env!("WIFI_NETWORK").unwrap(), option_env!("WIFI_PASSWORD").unwrap())
                .await;
        } else {
            control.join_open(option_env!("WIFI_NETWORK").unwrap()).await;
        }
    } else {
        warn!("Environment variable WIFI_NETWORK not set during compilation - not joining wireless network");
    }
    let config = embassy_net::ConfigStrategy::Dhcp;
    //let config = embassy_net::ConfigStrategy::Static(embassy_net::Config {
    //    address: Ipv4Cidr::new(Ipv4Address::new(192, 168, 69, 2), 24),
    //    dns_servers: Vec::new(),
    //    gateway: Some(Ipv4Address::new(192, 168, 69, 1)),
    //});

    // Generate random seed
    let seed = 0x0123_4567_89ab_cdef; // chosen by fair dice roll. guarenteed to be random.

    let mut stack_resources = embassy_net::StackResources::<1, 2, 8>::new();
    let stack_resources = unsafe { forever_mut(&mut stack_resources) };

    // Init network stack
    let stack = Stack::new(net_device, config, stack_resources, seed);
    let stack = unsafe { forever(&stack) };

    let task_pool: TaskPool<_, 10> = TaskPool::new();
    let task_pool = unsafe { forever(&task_pool) };
    let spawn_token = task_pool.spawn(|| stack.run());
    spawner.spawn(spawn_token).unwrap();

    // Blink the LED at 1 Hz
    loop {
        info!("on");
        control.gpio_set(0, true).await;
        Timer::after(Duration::from_millis(200)).await;

        info!("off");
        control.gpio_set(0, false).await;
        Timer::after(Duration::from_millis(200)).await;
    }
}
