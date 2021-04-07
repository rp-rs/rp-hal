//! Example of using I2C.
//! Scans available I2C devices on bus and print the result.
//! Target board: STM32F3DISCOVERY

#![no_std]
#![no_main]
use core::panic::PanicInfo;
use cortex_m_rt::entry;
// use rp2040_hal::dma::Channel;

#[entry]
fn main() -> ! {
    // let p = rp2040_pac::Peripherals::take().unwrap();
    // let ch = Channel::acquire(None, &p).unwrap_or_else(|| panic!("Fuc"));
    //ch.set_read_increment(true);
    let mut a = 3;
    loop {
        a+=1;
    }
}

#[panic_handler]
fn wtf(handler: &PanicInfo) -> !{
    loop{}
}