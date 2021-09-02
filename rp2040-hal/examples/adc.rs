//! Read ADC samples from the temperature sensor and pin and
//! output them to the UART on pins 1 and 2 at 9600 baud
#![no_std]
#![no_main]

use core::fmt::Write;
use cortex_m::prelude::_embedded_hal_adc_OneShot;
use cortex_m_rt::entry;
use hal::adc::Adc;
use hal::clocks::init_clocks_and_plls;
use hal::gpio::{self, Pins};
use hal::pac;
use hal::sio::Sio;
use hal::uart::UartPeripheral;
use hal::watchdog::Watchdog;
use panic_halt as _;
use rp2040_hal as hal;

#[link_section = ".boot2"]
#[used]
pub static BOOT2: [u8; 256] = rp2040_boot2::BOOT_LOADER;

// External high-speed crystal on the pico board is 12Mhz
// Adjust if your board has a different frequency
const XTAL_FREQ_HZ: u32 = 12_000_000u32;
const SYS_FREQ_HZ: u32 = hal::pll::common_configs::PLL_SYS_125MHZ.vco_freq.0;

#[entry]
fn main() -> ! {
    let mut pac = pac::Peripherals::take().unwrap();
    let core = pac::CorePeripherals::take().unwrap();
    let mut watchdog = Watchdog::new(pac.WATCHDOG);
    let sio = Sio::new(pac.SIO);

    // External high-speed crystal on the pico board is 12Mhz

    let clocks = init_clocks_and_plls(
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

    let mut delay = cortex_m::delay::Delay::new(core.SYST, SYS_FREQ_HZ);

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
    uart.write_full_blocking(b"ADC example\r\n");
    // Enable adc
    let mut adc = Adc::new(pac.ADC, &mut pac.RESETS);
    // Enable the temperature sense channel
    let mut temperature_sensor = adc.enable_temp_sensor();
    // Configure one of the pins as an ADC input as well.
    let mut adc_pin_0 = pins.gpio26.into_floating_input();
    loop {
        // Read the raw ADC counts from the temperature sensor channel.
        let temp_sens_adc_counts: u16 = adc.read(&mut temperature_sensor).unwrap();
        let pin_adc_counts: u16 = adc.read(&mut adc_pin_0).unwrap();
        writeln!(
            uart,
            "ADC readings: Temperature: {:02} Pin: {:02}\r\n",
            temp_sens_adc_counts, pin_adc_counts
        )
        .unwrap();
        delay.delay_ms(1000);
    }
}
