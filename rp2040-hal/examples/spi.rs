//! Perform ]
#![no_std]
#![no_main]

use cortex_m::prelude::{
    _embedded_hal_blocking_spi_Transfer, _embedded_hal_blocking_spi_Write,
    _embedded_hal_spi_FullDuplex,
};
use cortex_m_rt::entry;
use embedded_hal::spi::MODE_0;
use embedded_time::rate::Extensions;
use hal::{gpio::FunctionSpi, pac, sio::Sio, spi::Spi};
use panic_halt as _;
use rp2040_hal as hal;

#[link_section = ".boot2"]
#[used]
pub static BOOT2: [u8; 256] = rp2040_boot2::BOOT_LOADER;

const SYS_HZ: u32 = 125_000_000_u32;

#[entry]
fn main() -> ! {
    let mut pac = pac::Peripherals::take().unwrap();

    let sio = Sio::new(pac.SIO);
    let pins = hal::gpio::Pins::new(
        pac.IO_BANK0,
        pac.PADS_BANK0,
        sio.gpio_bank0,
        &mut pac.RESETS,
    );

    // These are implicitly used by the spi driver if they are in the correct mode
    let _spi_sclk = pins.gpio6.into_mode::<FunctionSpi>();
    let _spi_mosi = pins.gpio7.into_mode::<FunctionSpi>();
    let _spi_miso = pins.gpio4.into_mode::<FunctionSpi>();
    let mut spi = Spi::<_, _, 8>::new(pac.SPI0).init(
        &mut pac.RESETS,
        SYS_HZ.Hz(),
        16_000_000u32.Hz(),
        &MODE_0,
    );

    // Write out 0, ignore return value
    if let Ok(..) = spi.write(&[0]) {
        // Handle success
    };

    // write 50, then check the return
    let send_success = spi.send(50);
    match send_success {
        Ok(_) => {
            // We succeeded, check the read value
            if spi.read().is_ok() {
                // Output our read value
            };
        }
        Err(_) => todo!(),
    }

    // Do a read+write at the same time.
    // Data in read_write_cache will be replaced with the read data
    let mut read_write_cache: [u8; 4] = [1, 2, 3, 4];
    let transfer_success = spi.transfer(&mut read_write_cache);
    #[allow(clippy::single_match)]
    match transfer_success {
        Ok(_) => {}  // Handle success
        Err(_) => {} // handle errors
    };

    #[allow(clippy::empty_loop)]
    loop {}
}
