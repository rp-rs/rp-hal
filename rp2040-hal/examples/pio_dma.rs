//! This example shows how to read from and write to PIO using DMA.
//!
//! If a LED is connected to that pin, like on a Pico board, it will continously output "HELLO
//! WORLD" in morse code. The example also tries to read the data back. If reading the data fails,
//! the message will only be shown once, and then the LED remains dark.
//!
//! See the `Cargo.toml` file for Copyright and licence details.
#![no_std]
#![no_main]

use cortex_m::singleton;
use cortex_m_rt::entry;
use hal::dma::{DMAExt, DoubleBufferingConfig, SingleBufferingConfig};
use hal::gpio::{FunctionPio0, Pin};
use hal::pac;
use hal::pio::PIOExt;
use hal::sio::Sio;
use panic_halt as _;
use rp2040_hal as hal;

#[link_section = ".boot2"]
#[used]
pub static BOOT2: [u8; 256] = rp2040_boot2::BOOT_LOADER;

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

    // configure LED pin for Pio0.
    let _led: Pin<_, FunctionPio0> = pins.gpio25.into_mode();
    // PIN id for use inside of PIO
    let led_pin_id = 25;

    // HELLO WORLD in morse code:
    // .... . .-.. .-.. --- / .-- --- .-. .-.. -..
    let message = [
        0b10101010_00100010_11101010_00101110,
        0b10100011_10111011_10000000_10111011,
        0b10001110_11101110_00101110_10001011,
        0b10101000_11101010_00000000_00000000,
    ];

    // Define a PIO program which reads data from the TX FIFO bit by bit, configures the LED
    // according to the data, and then writes the data back to the RX FIFO.
    let program = pio_proc::pio!(
        32,
        "
.wrap_target
    out x, 1
    mov pins, x
    in x, 1 [13]
.wrap
        "
    );

    // Initialize and start PIO
    let (mut pio, sm0, _, _, _) = pac.PIO0.split(&mut pac.RESETS);
    let installed = pio.install(&program.program).unwrap();
    let div = 0f32; // as slow as possible (0 is interpreted as 65536)
    let (mut sm, rx, tx) = rp2040_hal::pio::PIOBuilder::from_program(installed)
        .out_pins(led_pin_id, 1)
        .clock_divisor(div)
        .autopull(true)
        .autopush(true)
        .build(sm0);
    // The GPIO pin needs to be configured as an output.
    sm.set_pindirs_with_mask(1 << led_pin_id, 1 << led_pin_id);
    sm.start();

    // Read and write without DMA.
    /*let mut failure = false;
    for i in 0..message.len() {
        tx.write(message[i]);
        let mut value = None;
        while value.is_none() {
            value = rx.read();
        }
        if value.unwrap() != message[i] {
            failure = true;
        }
    }
    if failure {
        // Abort.
        #[allow(clippy::empty_loop)]
        loop {}
    }*/

    let dma = pac.DMA.split(&mut pac.RESETS);

    // Transfer a single message via DMA.
    let tx_buf = singleton!(: [u32; 4] = message).unwrap();
    let rx_buf = singleton!(: [u32; 4] = [0; 4]).unwrap();
    let tx_transfer = SingleBufferingConfig::new(dma.ch0, tx_buf, tx).start();
    let rx_transfer = SingleBufferingConfig::new(dma.ch1, rx, rx_buf).start();
    let (ch0, tx_buf, tx) = tx_transfer.wait();
    let (ch1, rx, rx_buf) = rx_transfer.wait();
    for i in 0..rx_buf.len() {
        if rx_buf[i] != tx_buf[i] {
            // The data did not match, abort.
            #[allow(clippy::empty_loop)]
            loop {}
        }
    }

    // Chain some buffers together.
    let tx_buf2 = singleton!(: [u32; 4] = message).unwrap();
    let rx_buf2 = singleton!(: [u32; 4] = [0; 4]).unwrap();
    let tx_transfer = DoubleBufferingConfig::new((ch0, ch1), tx_buf, tx).start();
    let mut tx_transfer = tx_transfer.read_next(tx_buf2);
    let rx_transfer = DoubleBufferingConfig::new((dma.ch2, dma.ch3), rx, rx_buf).start();
    let mut rx_transfer = rx_transfer.write_next(rx_buf2);
    loop {
        // We simply immediately enqueue the buffers again.
        if tx_transfer.is_done() {
            let (tx_buf, next_tx_transfer) = tx_transfer.wait();
            tx_transfer = next_tx_transfer.read_next(tx_buf);
        }
        if rx_transfer.is_done() {
            let (rx_buf, next_rx_transfer) = rx_transfer.wait();
            for i in 0..rx_buf.len() {
                if rx_buf[i] != message[i] {
                    // The data did not match, abort.
                    #[allow(clippy::empty_loop)]
                    loop {}
                }
            }
            rx_transfer = next_rx_transfer.write_next(rx_buf);
        }
    }

    /*// Endless transfer from a ring buffer via DMA - note that unaligned ring buffers require three
    // (!) DMA channels, one for each buffer and one to control the other two channels.
    // TODO: The API should use two buffers, so that the transfer can then return one half of the
    // buffer while the other one is in progress of being transferred.
    let tx_buf = singleton!(: AlignedBuffer = AlignedBuffer(message)).unwrap();
    let rx_buf = singleton!(: AlignedBuffer = AlignedBuffer([0; 4])).unwrap();
    Endless::new((ch0, ch1), &tx_buf.0, tx)
        .start_ring()
        .unwrap();
    Endless::new((dma.ch2, dma.ch3), rx, &mut rx_buf.0)
        .start_ring()
        .unwrap();

    #[allow(clippy::empty_loop)]
    loop {}*/
}

/*#[repr(align(16))]
struct AlignedBuffer([u32; 4]);*/
