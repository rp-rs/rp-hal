//! # Pico SD Card Example
//!
//! Reads and writes a file from/to the SD Card that is formatted in FAT32.
//! This example uses the SPI0 device of the Raspberry Pi Pico on the
//! pins 4,5,6 and 7. If you don't use an external 3.3V power source,
//! you can connect the +3.3V output on pin 36 to the SD card.
//!
//! SD Cards up to 2TB are supported by the `embedded_sdmmc` crate.
//! I've tested this with a 64GB micro SD card.
//!
//! You need to format the card with an regular old FAT32 filesystem
//! and also make sure the first partition has the right type. This is how your
//! `fdisk` output should look like:
//!
//! ```text
//!     fdisk /dev/sdj
//!
//!     Welcome to fdisk (util-linux 2.34).
//!     Changes will remain in memory only, until you decide to write them.
//!     Be careful before using the write command.
//!
//!     Command (m for help): Disk /dev/sdj:
//!     59,49 GiB, 63864569856 bytes, 124735488 sectors
//!     Disk model: SD/MMC/MS/MSPRO
//!     Units: sectors of 1 * 512 = 512 bytes
//!     Sector size (logical/physical): 512 bytes / 512 bytes
//!     I/O size (minimum/optimal): 512 bytes / 512 bytes
//!     Disklabel type: dos
//!     Disk identifier: 0x00000000
//!
//!     Device     Boot Start       End   Sectors  Size Id Type
//!     /dev/sdj1        2048 124735487 124733440 59,5G  c W95 FAT32 (LBA)
//! ```
//!
//! The important bit here is the _Type_ with `W95 FAT32 (LBA)`, other types
//! are rejected by the `embedded_sdmmc` filesystem implementation.
//!
//! Formatting the partition can be done using `mkfs.fat`:
//!
//!     $ mkfs.fat /dev/sdj1
//!
//! In the following ASCII art the SD card is also connected to 5 strong pull up
//! resistors. I've found varying values for these, from 50kOhm, 10kOhm
//! down to 5kOhm.
//! Stronger pull up resistors will eat more amperes, but also allow faster
//! data rates.
//!
//! ```text
//!                    +3.3V
//!          Pull Ups ->||||
//!                 4x[5kOhm]
//!                     ||| \
//!  _______________    |||  \
//! |     DAT2/NC  9\---o||   \                            _|USB|_
//! | S   DAT3/CS   1|---o+----+------SS--\               |1  R 40|
//! | D   CMD/DI    2|----o----+-----MOSI-+-\             |2  P 39|
//! |     VSS1      3|-- GND   |          | |         GND-|3    38|- GND
//! | C   VDD       4|-- +3.3V |  /--SCK--+-+----SPI0 SCK-|4  P 37|
//! | A   CLK/SCK   5|---------+-/        | \----SPI0 TX--|5  I 36|- +3.3V
//! | R   VSS2      6|-- GND   |  /--MISO-+------SPI0 RX--|6  C   |
//! | D   DAT0/DO   7|---------o-/        \------SPI0 CSn-|7  O   |
//! |     DAT1/IRQ  8|-[5k]- +3.3V                        |       |
//!  """"""""""""""""                                     |       |
//!                                                       |       |
//!                                                       .........
//!                                                       |20   21|
//!                                                        """""""
//! Symbols:
//!     - (+) crossing lines, not connected
//!     - (o) connected lines
//! ```
//!
//! The example can either be used with a probe to receive debug output
//! and also the LED is used as status output. There are different blinking
//! patterns.
//!
//! For every successful stage in the example the LED will blink long once.
//! If everything is successful (9 long blink signals), the example will go
//! into a loop and either blink in a _"short long"_ or _"short short long"_ pattern.
//!
//! If there are 5 different error patterns, all with short blinking pulses:
//!
//! - **2 short blink (in a loop)**: Block device could not be aquired, either
//!   no SD card is present or some electrical problem.
//! - **3 short blink (in a loop)**: Card size could not be retrieved.
//! - **4 short blink (in a loop)**: Error getting volume/partition 0.
//! - **5 short blink (in a loop)**: Error opening root directory.
//! - **6 short blink (in a loop)**: Could not open file 'O.TST'.
//!
//! See the `Cargo.toml` file for Copyright and licence details.

#![no_std]
#![no_main]

// The macro for our start-up function
use rp_pico::entry;

// info!() and error!() macros for printing information to the debug output
use defmt::*;
use defmt_rtt as _;

// Ensure we halt the program on panic (if we don't mention this crate it won't
// be linked)
use panic_halt as _;

// Pull in any important traits
use rp_pico::hal::prelude::*;

// Embed the `Hz` function/trait:
use embedded_time::rate::*;

// A shorter alias for the Peripheral Access Crate, which provides low-level
// register access
use rp_pico::hal::pac;

// Import the SPI abstraction:
use rp_pico::hal::spi;

// Import the GPIO abstraction:
use rp_pico::hal::gpio;

// A shorter alias for the Hardware Abstraction Layer, which provides
// higher-level drivers.
use rp_pico::hal;

// Link in the embedded_sdmmc crate.
// The `SdMmcSpi` is used for block level access to the card.
// And the `Controller` gives access to the FAT filesystem functions.
use embedded_sdmmc::{Controller, SdMmcSpi, TimeSource, Timestamp, VolumeIdx};

// Get the file open mode enum:
use embedded_sdmmc::filesystem::Mode;

/// A dummy timesource, which is mostly important for creating files.
#[derive(Default)]
pub struct DummyTimesource();

impl TimeSource for DummyTimesource {
    // In theory you could use the RTC of the rp2040 here, if you had
    // any external time synchronizing device.
    fn get_timestamp(&self) -> Timestamp {
        Timestamp {
            year_since_1970: 0,
            zero_indexed_month: 0,
            zero_indexed_day: 0,
            hours: 0,
            minutes: 0,
            seconds: 0,
        }
    }
}

// Setup some blinking codes:
const BLINK_OK_LONG: [u8; 1] = [8u8];
const BLINK_OK_SHORT_LONG: [u8; 4] = [1u8, 0u8, 6u8, 0u8];
const BLINK_OK_SHORT_SHORT_LONG: [u8; 6] = [1u8, 0u8, 1u8, 0u8, 6u8, 0u8];
const BLINK_ERR_2_SHORT: [u8; 4] = [1u8, 0u8, 1u8, 0u8];
const BLINK_ERR_3_SHORT: [u8; 6] = [1u8, 0u8, 1u8, 0u8, 1u8, 0u8];
const BLINK_ERR_4_SHORT: [u8; 8] = [1u8, 0u8, 1u8, 0u8, 1u8, 0u8, 1u8, 0u8];
const BLINK_ERR_5_SHORT: [u8; 10] = [1u8, 0u8, 1u8, 0u8, 1u8, 0u8, 1u8, 0u8, 1u8, 0u8];
const BLINK_ERR_6_SHORT: [u8; 12] = [1u8, 0u8, 1u8, 0u8, 1u8, 0u8, 1u8, 0u8, 1u8, 0u8, 1u8, 0u8];

fn blink_signals(
    pin: &mut dyn embedded_hal::digital::v2::OutputPin<Error = core::convert::Infallible>,
    delay: &mut cortex_m::delay::Delay,
    sig: &[u8],
) {
    for bit in sig {
        if *bit != 0 {
            pin.set_high().unwrap();
        } else {
            pin.set_low().unwrap();
        }

        let length = if *bit > 0 { *bit } else { 1 };

        for _ in 0..length {
            delay.delay_ms(100);
        }
    }

    pin.set_low().unwrap();

    delay.delay_ms(500);
}

fn blink_signals_loop(
    pin: &mut dyn embedded_hal::digital::v2::OutputPin<Error = core::convert::Infallible>,
    delay: &mut cortex_m::delay::Delay,
    sig: &[u8],
) -> ! {
    loop {
        blink_signals(pin, delay, sig);
        delay.delay_ms(1000);
    }
}

#[entry]
fn main() -> ! {
    info!("Program start");

    // Grab our singleton objects
    let mut pac = pac::Peripherals::take().unwrap();
    let core = pac::CorePeripherals::take().unwrap();

    // Set up the watchdog driver - needed by the clock setup code
    let mut watchdog = hal::Watchdog::new(pac.WATCHDOG);

    // Configure the clocks
    //
    // The default is to generate a 125 MHz system clock
    let clocks = hal::clocks::init_clocks_and_plls(
        rp_pico::XOSC_CRYSTAL_FREQ,
        pac.XOSC,
        pac.CLOCKS,
        pac.PLL_SYS,
        pac.PLL_USB,
        &mut pac.RESETS,
        &mut watchdog,
    )
    .ok()
    .unwrap();

    // The single-cycle I/O block controls our GPIO pins
    let sio = hal::Sio::new(pac.SIO);

    // Set the pins up according to their function on this particular board
    let pins = rp_pico::Pins::new(
        pac.IO_BANK0,
        pac.PADS_BANK0,
        sio.gpio_bank0,
        &mut pac.RESETS,
    );

    // Set the LED to be an output
    let mut led_pin = pins.led.into_push_pull_output();

    // Setup a delay for the LED blink signals:
    let mut delay = cortex_m::delay::Delay::new(core.SYST, clocks.system_clock.freq().integer());

    // These are implicitly used by the spi driver if they are in the correct mode
    let _spi_sclk = pins.gpio2.into_mode::<gpio::FunctionSpi>();
    let _spi_mosi = pins.gpio3.into_mode::<gpio::FunctionSpi>();
    let _spi_miso = pins.gpio4.into_mode::<gpio::FunctionSpi>();
    let spi_cs = pins.gpio5.into_push_pull_output();

    // Create an SPI driver instance for the SPI0 device
    let spi = spi::Spi::<_, _, 8>::new(pac.SPI0);

    // Exchange the uninitialised SPI driver for an initialised one
    let spi = spi.init(
        &mut pac.RESETS,
        clocks.peripheral_clock.freq(),
        16_000_000u32.Hz(),
        &embedded_hal::spi::MODE_0,
    );

    info!("Aquire SPI SD/MMC BlockDevice...");
    let mut sdspi = SdMmcSpi::new(spi, spi_cs);

    blink_signals(&mut led_pin, &mut delay, &BLINK_OK_LONG);

    // Next we need to aquire the block device and initialize the
    // communication with the SD card.
    let block = match sdspi.acquire() {
        Ok(block) => block,
        Err(e) => {
            error!("Error retrieving card size: {}", defmt::Debug2Format(&e));
            blink_signals_loop(&mut led_pin, &mut delay, &BLINK_ERR_2_SHORT);
        }
    };

    blink_signals(&mut led_pin, &mut delay, &BLINK_OK_LONG);

    info!("Init SD card controller...");
    let mut cont = Controller::new(block, DummyTimesource::default());

    blink_signals(&mut led_pin, &mut delay, &BLINK_OK_LONG);

    info!("OK!\nCard size...");
    match cont.device().card_size_bytes() {
        Ok(size) => info!("card size is {} bytes", size),
        Err(e) => {
            error!("Error retrieving card size: {}", defmt::Debug2Format(&e));
            blink_signals_loop(&mut led_pin, &mut delay, &BLINK_ERR_3_SHORT);
        }
    }

    blink_signals(&mut led_pin, &mut delay, &BLINK_OK_LONG);

    info!("Getting Volume 0...");
    let mut volume = match cont.get_volume(VolumeIdx(0)) {
        Ok(v) => v,
        Err(e) => {
            error!("Error getting volume 0: {}", defmt::Debug2Format(&e));
            blink_signals_loop(&mut led_pin, &mut delay, &BLINK_ERR_4_SHORT);
        }
    };

    blink_signals(&mut led_pin, &mut delay, &BLINK_OK_LONG);

    // After we have the volume (partition) of the drive we got to open the
    // root directory:
    let dir = match cont.open_root_dir(&volume) {
        Ok(dir) => dir,
        Err(e) => {
            error!("Error opening root dir: {}", defmt::Debug2Format(&e));
            blink_signals_loop(&mut led_pin, &mut delay, &BLINK_ERR_5_SHORT);
        }
    };

    info!("Root directory opened!");
    blink_signals(&mut led_pin, &mut delay, &BLINK_OK_LONG);

    // This shows how to iterate through the directory and how
    // to get the file names (and print them in hope they are UTF-8 compatible):
    cont.iterate_dir(&volume, &dir, |ent| {
        info!(
            "/{}.{}",
            core::str::from_utf8(ent.name.base_name()).unwrap(),
            core::str::from_utf8(ent.name.extension()).unwrap()
        );
    })
    .unwrap();

    blink_signals(&mut led_pin, &mut delay, &BLINK_OK_LONG);

    let mut successful_read = false;

    // Next we going to read a file from the SD card:
    if let Ok(mut file) = cont.open_file_in_dir(&mut volume, &dir, "O.TST", Mode::ReadOnly) {
        let mut buf = [0u8; 32];
        let read_count = cont.read(&volume, &mut file, &mut buf).unwrap();
        cont.close_file(&volume, file).unwrap();

        if read_count >= 2 {
            info!("READ {} bytes: {}", read_count, buf);

            // If we read what we wrote before the last reset,
            // we set a flag so that the success blinking at the end
            // changes it's pattern.
            if buf[0] == 0x42 && buf[1] == 0x1E {
                successful_read = true;
            }
        }
    }

    blink_signals(&mut led_pin, &mut delay, &BLINK_OK_LONG);

    match cont.open_file_in_dir(&mut volume, &dir, "O.TST", Mode::ReadWriteCreateOrTruncate) {
        Ok(mut file) => {
            cont.write(&mut volume, &mut file, b"\x42\x1E").unwrap();
            cont.close_file(&volume, file).unwrap();
        }
        Err(e) => {
            error!("Error opening file 'O.TST': {}", defmt::Debug2Format(&e));
            blink_signals_loop(&mut led_pin, &mut delay, &BLINK_ERR_6_SHORT);
        }
    }

    cont.free();

    blink_signals(&mut led_pin, &mut delay, &BLINK_OK_LONG);

    if successful_read {
        info!("Successfully read previously written file 'O.TST'");
    } else {
        info!("Could not read file, which is ok for the first run.");
        info!("Reboot the pico!");
    }

    loop {
        if successful_read {
            blink_signals(&mut led_pin, &mut delay, &BLINK_OK_SHORT_SHORT_LONG);
        } else {
            blink_signals(&mut led_pin, &mut delay, &BLINK_OK_SHORT_LONG);
        }

        delay.delay_ms(1000);
    }
}
