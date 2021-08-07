//! Print "Hello World" to UART0 in a loop
#![no_std]
#![no_main]

use cortex_m_rt::entry;
use hal::clocks::init_clocks_and_plls;
use hal::gpio::{self, Pins};
use hal::pac;
use hal::sio::Sio;
use hal::uart::UartPeripheral;
use hal::watchdog::Watchdog;
use panic_halt as _;
use rp2040_hal as hal;
use core::fmt::Write;

#[link_section = ".boot2"]
#[used]
pub static BOOT2: [u8; 256] = rp2040_boot2::BOOT_LOADER;

#[entry]
fn main() -> ! {
    let mut pac = pac::Peripherals::take().unwrap();
    let mut watchdog = Watchdog::new(pac.WATCHDOG);
    let sio = Sio::new(pac.SIO);

    // External high-speed crystal on the pico board is 12Mhz
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

    let pins = Pins::new(
        pac.IO_BANK0,
        pac.PADS_BANK0,
        sio.gpio_bank0,
        &mut pac.RESETS,
    );

    let mut uart = UartPeripheral::<_, _>::enable(
        pac.UART0,
        &mut pac.RESETS,
        hal::uart::common_configs::_9600_8_N_1,
        clocks.peripheral_clock.into(),
    )
    .unwrap();

    // UART TX (characters sent from pico) on pin 1 (GPIO0) and RX (on pin 2 (GPIO1)
    let _tx_pin = pins.gpio0.into_mode::<gpio::FunctionUart>();
    let _rx_pin = pins.gpio1.into_mode::<gpio::FunctionUart>();

    // We need a short delay here otherwise the first few characters are garbled.
    // TODO: work out why
    cortex_m::asm::delay(100_000);
    uart.write_full_blocking(b"UART example\r\n");

    let mut value = 0u32;
    loop {
        writeln!(uart, "value: {:02}\r", value).unwrap();
        cortex_m::asm::delay(10_000_000);
        value+=1
    }
}
