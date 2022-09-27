#![no_std]
#![no_main]
#![cfg(test)]

use crate::hal::dma::Channels;
use defmt_rtt as _; // defmt transport
use defmt_test as _;
use panic_probe as _;
use rp2040_hal as hal; // memory layout

#[link_section = ".boot2"]
#[used]
pub static BOOT2: [u8; 256] = rp2040_boot2::BOOT_LOADER_GENERIC_03H;

/// External high-speed crystal frequency
const XTAL_FREQ_HZ: u32 = 12_000_000u32;

struct State {
    channels: Option<Channels>,
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
    use crate::testdata;
    use crate::State;
    use crate::XTAL_FREQ_HZ;
    use defmt::assert_eq;
    use defmt_rtt as _;
    use panic_probe as _;
    use rp2040_hal as hal;

    use hal::{clocks::init_clocks_and_plls, pac, watchdog::Watchdog};

    use rp2040_hal::dma::DMAExt;

    #[init]
    fn setup() -> State {
        unsafe {
            hal::sio::spinlock_reset();
        }
        let mut pac = pac::Peripherals::take().unwrap();
        let _core = pac::CorePeripherals::take().unwrap();
        let mut watchdog = Watchdog::new(pac.WATCHDOG);

        let _clocks = init_clocks_and_plls(
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

        State {
            channels: Some(dma),
        }
    }

    #[test]
    #[should_error]
    fn dma_mem2mem_u64(state: &mut State) {
        if let Some(dma) = state.channels.take() {
            let rx_buffer = cortex_m::singleton!(: [u64; 10] = [0; 10]).unwrap();
            let tx_buffer = cortex_m::singleton!(: [u64; 10] = testdata::ARRAY_U64).unwrap();
            let tx_transfer = hal::dma::SingleBufferingConfig::new(dma.ch0, tx_buffer, rx_buffer);
            let tx_started = tx_transfer.start();
            let (_ch0, tx_buffer, rx_buffer) = tx_started.wait();
            let first = tx_buffer.iter();
            let second = rx_buffer.iter();
            for (x, y) in first.zip(second) {
                assert_eq!(x, y);
            }
        }
    }
}
