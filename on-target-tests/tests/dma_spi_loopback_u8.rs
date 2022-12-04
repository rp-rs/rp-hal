#![no_std]
#![no_main]
#![cfg(test)]

use crate::hal::dma::Channels;
use defmt_rtt as _; // defmt transport
use defmt_test as _;
use panic_probe as _;
use rp2040_hal as hal; // memory layout // panic handler
use rp2040_hal::spi;
use rp2040_hal::pac::SPI0;
// use rp2040_hal::spi::Spi::SPI0;

/// The linker will place this boot block at the start of our program image. We
/// need this to help the ROM bootloader get our code up and running.
/// Note: This boot block is not necessary when using a rp-hal based BSP
/// as the BSPs already perform this step.
#[link_section = ".boot2"]
#[used]
pub static BOOT2: [u8; 256] = rp2040_boot2::BOOT_LOADER_GENERIC_03H;

/// External high-speed crystal on the Raspberry Pi Pico board is 12 MHz. Adjust
/// if your board has a different frequency
const XTAL_FREQ_HZ: u32 = 12_000_000u32;

struct State {
    channels: Option<Channels>,
    spi: Option<spi::Spi<spi::Enabled, SPI0, 8>>,
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

#[defmt_test::tests]
mod tests {
    use fugit::RateExtU32;
    use crate::testdata;
    use crate::State;
    use crate::XTAL_FREQ_HZ;
    use defmt::assert_eq;
    use defmt_rtt as _;
    use panic_probe as _;
    use rp2040_hal as hal;
    use rp2040_hal::Clock;
    use hal::spi::Enabled;
    use pac::SPI0;
    use hal::{clocks::init_clocks_and_plls, pac, watchdog::Watchdog};
    use rp2040_hal::dma::{SingleBufferingConfig, BidirectionalConfig, DMAExt};


    #[init]
    fn setup() -> State {
        unsafe {
            hal::sio::spinlock_reset();
        }
        let mut pac = pac::Peripherals::take().unwrap();
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
        let _spi_sclk = pins.gpio6.into_mode::<hal::gpio::FunctionSpi>();
        let _spi_mosi = pins.gpio7.into_mode::<hal::gpio::FunctionSpi>();
        let _spi_miso = pins.gpio4.into_mode::<hal::gpio::FunctionSpi>();
        let spi = hal::spi::Spi::<_, _, 8>::new(pac.SPI0);

        // Exchange the uninitialised SPI driver for an initialised one
        let spi = spi.init(
            &mut pac.RESETS,
            clocks.peripheral_clock.freq(),
            16_000_000u32.Hz(),
            &embedded_hal::spi::MODE_0,
        );

        State {
            channels: Some(dma), spi: Some(spi),
        }
    }

    #[test]
    fn dma_spi_loopback_u8(state: &mut State) {
        if let Some(dma) = state.channels.take() {
            if let Some(spi) = state.spi.take() {
                let rx_buf = cortex_m::singleton!(: [u8; 10] = [0; 10]).unwrap();
                let tx_buf = cortex_m::singleton!(: [u8; 10] = testdata::ARRAY_U8).unwrap();
    
                // let transfer = BidirectionalConfig::new((dma.ch0, dma.ch1), tx_buf, spi, rx_buf).start();
                // let ((_ch0, _ch1), tx_buf, _spi, rx_buf) = transfer.wait();

                // We can't pass the same spi peripheral to 2 dma channels, so fabricate a token to use for this
                let spi_rx: hal::spi::Spi<Enabled, SPI0, 8> = unsafe { core::mem::transmute(()) };

                // We need these to start at the same time or they'll be desync'd.
                // Having interrupts disabled seems to work from a cold boot. I suspect it is sufficiently racy that we can't trust it
                let (transfer_tx, transfer_rx) = cortex_m::interrupt::free(|_t| {
                    let transfer_rx = SingleBufferingConfig::new(dma.ch1, spi_rx, rx_buf).start();
                    let transfer_tx = SingleBufferingConfig::new(dma.ch0, tx_buf, spi).start();
                    (transfer_tx, transfer_rx)
                });

                // Wait for both DMA channels to finish
                let (_ch0, tx_buf, _spi) = transfer_tx.wait();
                let (_ch1, _spi_rx, rx_buf) = transfer_rx.wait();


                let first = tx_buf.iter();
                let second = rx_buf.iter();
                for (x, y) in first.zip(second) {
                    assert_eq!(x, y);
                }
            }
        }
    }
}
