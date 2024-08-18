//! # 'ROM Functions' Example
//!
//! This application demonstrates how to call functions in the rp235x's boot ROM.
//!
//! It may need to be adapted to your particular board layout and/or pin assignment.
//!
//! See the `Cargo.toml` file for Copyright and license details.

#![no_std]
#![no_main]

// Ensure we halt the program on panic (if we don't mention this crate it won't
// be linked)
use panic_halt as _;

// Alias for our HAL crate
use rp235x_hal as hal;

// Some things we need
use core::fmt::Write;
use hal::fugit::RateExtU32;
use hal::Clock;

// UART related types
use hal::uart::{DataBits, StopBits, UartConfig};

/// Tell the Boot ROM about our application
#[link_section = ".start_block"]
#[used]
pub static IMAGE_DEF: hal::block::ImageDef = hal::block::ImageDef::secure_exe();

/// External high-speed crystal on the Raspberry Pi Pico board is 12 MHz. Adjust
/// if your board has a different frequency
const XTAL_FREQ_HZ: u32 = 12_000_000u32;

/// Entry point to our bare-metal application.
///
/// The `#[hal::entry]` macro ensures the Cortex-M start-up code calls this function
/// as soon as all global variables and the spinlock are initialised.
///
/// The function configures the rp235x peripherals, then writes to the UART in
/// an infinite loop.
#[hal::entry]
fn main() -> ! {
    // Grab our singleton objects
    let mut pac = hal::pac::Peripherals::take().unwrap();

    // Set up the watchdog driver - needed by the clock setup code
    let mut watchdog = hal::Watchdog::new(pac.WATCHDOG);

    // Configure the clocks
    let clocks = hal::clocks::init_clocks_and_plls(
        XTAL_FREQ_HZ,
        pac.XOSC,
        pac.CLOCKS,
        pac.PLL_SYS,
        pac.PLL_USB,
        &mut pac.RESETS,
        &mut watchdog,
    )
    .unwrap();

    // The single-cycle I/O block controls our GPIO pins
    let sio = hal::Sio::new(pac.SIO);

    // Set the pins to their default state
    let pins = hal::gpio::Pins::new(
        pac.IO_BANK0,
        pac.PADS_BANK0,
        sio.gpio_bank0,
        &mut pac.RESETS,
    );

    let uart_pins = (
        // UART TX (characters sent from rp235x) on pin 1 (GPIO0)
        pins.gpio0.into_function::<hal::gpio::FunctionUart>(),
        // UART RX (characters received by rp235x) on pin 2 (GPIO1)
        pins.gpio1.into_function::<hal::gpio::FunctionUart>(),
    );
    let mut uart = hal::uart::UartPeripheral::new(pac.UART0, uart_pins, &mut pac.RESETS)
        .enable(
            UartConfig::new(115200.Hz(), DataBits::Eight, None, StopBits::One),
            clocks.peripheral_clock.freq(),
        )
        .unwrap();

    _ = writeln!(uart);
    _ = writeln!(uart, "rom_func example for RP2350");
    _ = writeln!(uart, "===========================");
    _ = writeln!(uart);

    _ = writeln!(uart, "CPU Secure Mode: {}", hal::rom_data::is_secure_mode());

    _ = writeln!(
        uart,
        "ROM Git Revision: 0x{:x}",
        hal::rom_data::git_revision()
    );

    _ = writeln!(
        uart,
        "Partition Table Info @ {:?}",
        hal::rom_data::partition_table_pointer()
    );

    _ = writeln!(
        uart,
        "ROM Version: A{}",
        hal::rom_data::rom_version_number()
    );

    get_otp_data(&mut uart, &mut pac.OTP_DATA);
    get_otp_data_raw(&mut uart, &mut pac.OTP_DATA_RAW);
    get_sys_info_chip_info(&mut uart);
    get_sys_info_cpu_info(&mut uart);
    get_sys_info_flash_dev_info(&mut uart);
    get_sys_info_boot_random(&mut uart);
    get_sys_info_start_block(&mut uart);
    get_partition_table_info(&mut uart);

    const WORDS_PER_SCREEN_LINE: usize = 16;
    _ = writeln!(uart, "Reading OTP:");
    _ = write!(uart, "Page Adr: ");
    for col in 0..WORDS_PER_SCREEN_LINE {
        _ = write!(uart, " {:04x}", col);
    }
    // These are the factory programmed pages
    for page in [0, 1, 62, 63] {
        for row in 0..hal::otp::NUM_ROWS_PER_PAGE {
            let index = page * hal::otp::NUM_ROWS_PER_PAGE + row;
            if row == 0 {
                _ = write!(uart, "\nP{:02} {:04x}: ", page, index);
            } else if (index % WORDS_PER_SCREEN_LINE) == 0 {
                _ = write!(uart, "\n    {:04x}: ", index);
            }
            match hal::otp::read_ecc_word(index) {
                Ok(0) => {
                    _ = write!(uart, " ----");
                }
                Ok(word) => {
                    _ = write!(uart, " {:04x}", word);
                }
                Err(hal::otp::Error::InvalidPermissions) => {
                    _ = write!(uart, " xxxx");
                }
                Err(hal::otp::Error::InvalidIndex) => {
                    _ = write!(uart, " ????");
                }
            }
        }
    }
    _ = writeln!(uart);

    _ = writeln!(uart, "Reading raw OTP:");
    _ = write!(uart, "Page Adr: ");
    for col in 0..WORDS_PER_SCREEN_LINE {
        _ = write!(uart, " {:06x}", col);
    }
    // These are the factory programmed pages
    for page in [0, 1, 62, 63] {
        for row in 0..hal::otp::NUM_ROWS_PER_PAGE {
            let index = page * hal::otp::NUM_ROWS_PER_PAGE + row;
            if row == 0 {
                _ = write!(uart, "\nP{:02} {:04x}: ", page, index);
            } else if (index % WORDS_PER_SCREEN_LINE) == 0 {
                _ = write!(uart, "\n    {:04x}: ", index);
            }
            match hal::otp::read_raw_word(index) {
                Ok(0) => {
                    _ = write!(uart, " ------");
                }
                Ok(word) => {
                    _ = write!(uart, " {:06x}", word);
                }
                Err(hal::otp::Error::InvalidPermissions) => {
                    _ = write!(uart, " xxxxxx");
                }
                Err(hal::otp::Error::InvalidIndex) => {
                    _ = write!(uart, " ??????");
                }
            }
        }
    }
    _ = writeln!(uart);

    // Do an asynchronous reset in 2000ms time, into the bootloader.
    hal::reboot::reboot(
        hal::reboot::RebootKind::BootSel {
            msd_disabled: false,
            picoboot_disabled: false,
        },
        hal::reboot::RebootArch::Normal,
    );
}

/// Read OTP using the PAC
fn get_otp_data<T>(uart: &mut T, otp_data: &mut hal::pac::OTP_DATA)
where
    T: core::fmt::Write,
{
    _ = writeln!(uart, "Reading OTP_DATA");
    let package_id = (otp_data.chipid1().read().chipid1().bits() as u32) << 16
        | otp_data.chipid0().read().chipid0().bits() as u32;
    let device_id = (otp_data.chipid3().read().chipid3().bits() as u32) << 16
        | otp_data.chipid2().read().chipid2().bits() as u32;
    _ = writeln!(uart, "\tRP2350 Package ID: {:#010x}", package_id);
    _ = writeln!(uart, "\tRP2350 Device ID : {:#010x}", device_id);
}

/// Read OTP in raw mode using the PAC
///
/// Currently this doesn't work due to SVD issues.
fn get_otp_data_raw<T>(uart: &mut T, otp_data_raw: &mut hal::pac::OTP_DATA_RAW)
where
    T: core::fmt::Write,
{
    _ = writeln!(uart, "Reading OTP_DATA_RAW");
    _ = writeln!(
        uart,
        "\tRP2350 Package ID: {:#010x} {:#010x}",
        otp_data_raw.chipid0().read().bits(),
        otp_data_raw.chipid1().read().bits()
    );
    _ = writeln!(
        uart,
        "\tRP2350 Device ID : {:#010x} {:#010x}",
        otp_data_raw.chipid2().read().bits(),
        otp_data_raw.chipid3().read().bits()
    );
}

/// Run get_sys_info with 0x0001
fn get_sys_info_chip_info<T>(uart: &mut T)
where
    T: core::fmt::Write,
{
    let mut buffer = [0u32; 16];
    let result = unsafe { hal::rom_data::get_sys_info(buffer.as_mut_ptr(), buffer.len(), 0x0001) };
    _ = writeln!(uart, "get_sys_info(CHIP_INFO/0x0001) -> {}", result);
    _ = writeln!(uart, "\tSupported Flags: {:#06x}", buffer[0]);
    _ = writeln!(uart, "\tRP2350 Package ID: {:#010x}", buffer[1]);
    _ = writeln!(uart, "\tRP2350 Device ID : {:#010x}", buffer[2]);
    _ = writeln!(uart, "\tRP2350 Wafer ID  : {:#010x}", buffer[3]);
}

/// Run get_sys_info with 0x0004
fn get_sys_info_cpu_info<T>(uart: &mut T)
where
    T: core::fmt::Write,
{
    let mut buffer = [0u32; 16];
    let result = unsafe { hal::rom_data::get_sys_info(buffer.as_mut_ptr(), buffer.len(), 0x0004) };
    _ = writeln!(uart, "get_sys_info(CPU_INFO/0x0004) -> {}", result);
    _ = writeln!(uart, "\tSupported Flags: {:#06x}", buffer[0]);
    _ = writeln!(
        uart,
        "\tCPU Architecture: {}",
        match buffer[1] {
            0 => "Arm",
            1 => "RISC-V",
            _ => "Unknown",
        }
    );
}

/// Run get_sys_info with 0x0008
fn get_sys_info_flash_dev_info<T>(uart: &mut T)
where
    T: core::fmt::Write,
{
    let mut buffer = [0u32; 16];
    let result = unsafe { hal::rom_data::get_sys_info(buffer.as_mut_ptr(), buffer.len(), 0x0008) };
    _ = writeln!(uart, "get_sys_info(FLASH_DEV_INFO/0x0008) -> {}", result);
    let size_lookup = |value| match value {
        0 => "None",
        1 => "8K",
        2 => "16K",
        3 => "32K",
        4 => "64K",
        5 => "128K",
        6 => "256K",
        7 => "512K",
        8 => "1M",
        9 => "2M",
        10 => "4M",
        11 => "8M",
        12 => "16M",
        _ => "Unknown",
    };
    _ = writeln!(uart, "\tSupported Flags: {:#06x}", buffer[0]);
    _ = writeln!(uart, "\tCS0 Size: {}", size_lookup((buffer[1] >> 8) & 15));
    _ = writeln!(uart, "\tCS1 Size: {}", size_lookup((buffer[1] >> 12) & 15));
}

/// Run get_sys_info with 0x0010
fn get_sys_info_boot_random<T>(uart: &mut T)
where
    T: core::fmt::Write,
{
    let mut buffer = [0u32; 16];
    let result = unsafe { hal::rom_data::get_sys_info(buffer.as_mut_ptr(), buffer.len(), 0x0010) };
    _ = writeln!(uart, "get_sys_info(BOOT_RANDOM/0x0010) -> {}", result);
    _ = writeln!(uart, "\tSupported Flags: {:#06x}", buffer[0]);
    let a = buffer[1];
    let b = buffer[2];
    let c = buffer[3];
    let d = buffer[4];
    _ = writeln!(uart, "\tA random number: 0x{a:08x}{b:08x}{c:08x}{d:08x}");
}

/// Run get_sys_info with 0x0040;
fn get_sys_info_start_block<T>(uart: &mut T)
where
    T: core::fmt::Write,
{
    let mut buffer = [0u32; 16];
    let result = unsafe { hal::rom_data::get_sys_info(buffer.as_mut_ptr(), buffer.len(), 0x0040) };
    _ = writeln!(uart, "get_sys_info(start_block/0x0040) -> {}", result);
    _ = writeln!(uart, "\tSupported Flags: {:#06x}", buffer[0]);
    _ = writeln!(uart, "\tBoot Info: {:08x?}", &buffer[1..result as usize]);
}

/// Run get_partition_table_info
fn get_partition_table_info<T>(uart: &mut T)
where
    T: core::fmt::Write,
{
    let mut buffer = [0u32; 16];
    let result = unsafe {
        hal::rom_data::get_partition_table_info(buffer.as_mut_ptr(), buffer.len(), 0x0001)
    };
    _ = writeln!(
        uart,
        "get_partition_table_info(PT_INFO/0x0001) -> {}",
        result
    );
    _ = writeln!(uart, "\tSupported Flags: {:#06x}", buffer[0]);
    let partition_count = buffer[1] & 0xFF;
    let has_partition_table = (buffer[1] & (1 << 8)) != 0;
    let unpart = hal::block::UnpartitionedSpace::from_raw(buffer[2], buffer[3]);
    _ = writeln!(
        uart,
        "\tNum Partitions: {} (Has Partition Table? {})",
        partition_count, has_partition_table
    );
    _ = writeln!(uart, "\tUnpartitioned Space: {}", unpart);
    for part_idx in 0..partition_count {
        let result = unsafe {
            hal::rom_data::get_partition_table_info(
                buffer.as_mut_ptr(),
                buffer.len(),
                (part_idx << 24) | 0x8010,
            )
        };
        _ = writeln!(
            uart,
            "get_partition_table_info(PARTITION_LOCATION_AND_FLAGS|SINGLE_PARTITION/0x8010) -> {}",
            result
        );
        _ = writeln!(uart, "\tSupported Flags: {:#06x}", buffer[0]);
        let part = hal::block::Partition::from_raw(buffer[1], buffer[2]);
        _ = writeln!(uart, "\tPartition {}: {}", part_idx, part);
    }
}

/// Program metadata for `picotool info`
#[link_section = ".bi_entries"]
#[used]
pub static PICOTOOL_ENTRIES: [hal::binary_info::EntryAddr; 5] = [
    hal::binary_info::rp_cargo_bin_name!(),
    hal::binary_info::rp_cargo_version!(),
    hal::binary_info::rp_program_description!(c"Tests ROM functions and reading OTP"),
    hal::binary_info::rp_cargo_homepage_url!(),
    hal::binary_info::rp_program_build_attribute!(),
];

// End of file
