//! This test needs a connection between GPIO 4 and GPIO 7 (pins 6 and 10 an a Pico)

#![no_std]
#![no_main]
#![cfg(test)]

use crate::hal::dma::Channels;
use defmt_rtt as _; // defmt transport
use defmt_test as _;
use hal::gpio::{self, Pin};
use hal::pac::SPI0;
use hal::spi;
use panic_probe as _;
#[cfg(feature = "rp2040")]
use rp2040_hal as hal; // memory layout // panic handler
#[cfg(feature = "rp235x")]
use rp235x_hal as hal;

/// The linker will place this boot block at the start of our program image. We
/// need this to help the ROM bootloader get our code up and running.
/// Note: This boot block is not necessary when using a rp-hal based BSP
/// as the BSPs already perform this step.
#[cfg(feature = "rp2040")]
#[link_section = ".boot2"]
#[used]
pub static BOOT2: [u8; 256] = rp2040_boot2::BOOT_LOADER_GENERIC_03H;

/// Tell the Boot ROM about our application
#[cfg(feature = "rp235x")]
#[link_section = ".start_block"]
#[used]
pub static IMAGE_DEF: hal::block::ImageDef = hal::block::ImageDef::secure_exe();

/// External high-speed crystal on the Raspberry Pi Pico board is 12 MHz. Adjust
/// if your board has a different frequency
const XTAL_FREQ_HZ: u32 = 12_000_000u32;

type MISO = Pin<gpio::bank0::Gpio4, gpio::FunctionSpi, gpio::PullNone>;
type MOSI = Pin<gpio::bank0::Gpio7, gpio::FunctionSpi, gpio::PullNone>;
type SCLK = Pin<gpio::bank0::Gpio6, gpio::FunctionSpi, gpio::PullNone>;
struct State {
    channels: Option<Channels>,
    spi: Option<spi::Spi<spi::Enabled, SPI0, (MOSI, MISO, SCLK), 16>>,
}

mod testdata {
    #[allow(dead_code)]
    pub const ARRAY_U8: [u8; 10] = [1, 2, 3, 4, 5, 6, 7, 8, 9, 10];
    #[allow(dead_code)]
    pub const ARRAY_U16: [u16; 10] = [270, 271, 272, 273, 274, 275, 276, 277, 278, 279];
    #[allow(dead_code)]
    pub const ARRAY_U32: [u32; 10] = [
        65571, 65572, 65573, 65574, 65575, 65576, 65577, 65578, 65579, 65580,
    ];
    #[allow(dead_code)]
    pub const ARRAY_U64: [u64; 10] = [
        65571, 65572, 65573, 65574, 65575, 65576, 65577, 65578, 65579, 65580,
    ];
}

mod init;

#[defmt_test::tests]
mod tests {
    use crate::testdata;
    use crate::State;
    use crate::XTAL_FREQ_HZ;
    use defmt::assert_eq;
    use defmt_rtt as _;
    use fugit::RateExtU32;
    use hal::dma::{bidirectional, DMAExt};
    use hal::Clock;
    use hal::{clocks::init_clocks_and_plls, pac, watchdog::Watchdog};
    use panic_probe as _;
    #[cfg(feature = "rp2040")]
    use rp2040_hal as hal;
    #[cfg(feature = "rp235x")]
    use rp235x_hal as hal;

    #[init]
    fn setup() -> State {
        unsafe {
            crate::init::reset_cleanup();
        }
        let mut pac = pac::Peripherals::take().unwrap();
        #[cfg(feature = "rp2040")]
        let _core = pac::CorePeripherals::take().unwrap();
        let mut watchdog = Watchdog::new(pac.WATCHDOG);

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

        let dma = pac.DMA.split(&mut pac.RESETS);

        // Setup the pins.
        let sio = hal::sio::Sio::new(pac.SIO);
        let pins = hal::gpio::Pins::new(
            pac.IO_BANK0,
            pac.PADS_BANK0,
            sio.gpio_bank0,
            &mut pac.RESETS,
        );

        // These are implicitly used by the spi driver if they are in the correct mode
        let spi_sclk = pins.gpio6.reconfigure();
        let spi_mosi = pins.gpio7.reconfigure();
        let spi_miso = pins.gpio4.reconfigure();
        let spi = hal::spi::Spi::new(pac.SPI0, (spi_mosi, spi_miso, spi_sclk));

        // Exchange the uninitialised SPI driver for an initialised one
        let spi = spi.init(
            &mut pac.RESETS,
            clocks.peripheral_clock.freq(),
            16_000_000u32.Hz(),
            &embedded_hal::spi::MODE_0,
        );

        State {
            channels: Some(dma),
            spi: Some(spi),
        }
    }

    #[test]
    fn dma_spi_loopback_u16(state: &mut State) {
        if let Some(dma) = state.channels.take() {
            if let Some(spi) = state.spi.take() {
                let rx_buf = cortex_m::singleton!(: [u16; 10] = [0; 10]).unwrap();
                let tx_buf = cortex_m::singleton!(: [u16; 10] = testdata::ARRAY_U16).unwrap();

                let transfer =
                    bidirectional::Config::new((dma.ch0, dma.ch1), tx_buf, spi, rx_buf).start();
                let ((_ch0, _ch1), tx_buf, _spi, rx_buf) = transfer.wait();

                let first = tx_buf.iter();
                let second = rx_buf.iter();
                for (x, y) in first.zip(second) {
                    assert_eq!(x, y);
                }
            }
        }
    }
}
