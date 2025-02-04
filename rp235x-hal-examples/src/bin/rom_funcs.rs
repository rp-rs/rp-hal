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

use rp235x_hal::rom_data::sys_info_api::{BootType, CpuInfo, FlashDevInfoSize, PartitionIndex};
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

/// External high-speed crystal on the Raspberry Pi Pico 2 board is 12 MHz.
/// Adjust if your board has a different frequency
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

    // Do a reset into the bootloader.
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
    let device_id = ((otp_data.chipid1().read().chipid1().bits() as u32) << 16)
        | otp_data.chipid0().read().chipid0().bits() as u32;
    let wafer_id = ((otp_data.chipid3().read().chipid3().bits() as u32) << 16)
        | otp_data.chipid2().read().chipid2().bits() as u32;
    _ = writeln!(uart, "\tRP2350 Device ID: {:#010x}", device_id);
    _ = writeln!(uart, "\tRP2350 Wafer ID : {:#010x}", wafer_id);
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
        "\tRP2350 Device ID: {:#010x} {:#010x}",
        otp_data_raw.chipid0().read().bits(),
        otp_data_raw.chipid1().read().bits()
    );
    _ = writeln!(
        uart,
        "\tRP2350 Wafer ID : {:#010x} {:#010x}",
        otp_data_raw.chipid2().read().bits(),
        otp_data_raw.chipid3().read().bits()
    );
}

/// Run get_sys_info with 0x0001
fn get_sys_info_chip_info<T>(uart: &mut T)
where
    T: core::fmt::Write,
{
    let result = match hal::rom_data::sys_info_api::chip_info() {
        Ok(Some(result)) => result,
        Ok(None) => {
            _ = writeln!(uart, "chip_info() not supported");
            return;
        }
        Err(e) => {
            _ = writeln!(uart, "Failed to get chip info : {:?}", e);
            return;
        }
    };

    _ = writeln!(uart, "get_sys_info(CHIP_INFO/0x0001)");
    let package_type = match result.package_sel {
        0 => "QFN80",
        1 => "QFN60",
        _ => "unknown",
    };
    _ = writeln!(
        uart,
        "\tRP2350 Package  : {:#010x} ({}, but wrong on A2 stepping)",
        result.package_sel, package_type
    );
    _ = writeln!(uart, "\tRP2350 Device ID: {:#010x}", result.device_id);
    _ = writeln!(uart, "\tRP2350 Wafer ID : {:#010x}", result.wafer_id);
}

/// Run get_sys_info with 0x0004
fn get_sys_info_cpu_info<T>(uart: &mut T)
where
    T: core::fmt::Write,
{
    let result = match hal::rom_data::sys_info_api::cpu_info() {
        Ok(Some(result)) => result,
        Ok(None) => {
            _ = writeln!(uart, "cpu_info() not supported");
            return;
        }
        Err(e) => {
            _ = writeln!(uart, "Failed to get cpu info: {:?}", e);
            return;
        }
    };

    _ = writeln!(uart, "get_sys_info(CPU_INFO/0x0004)");
    _ = writeln!(
        uart,
        "\tCPU Architecture: {}",
        match result {
            CpuInfo::Arm => "Arm",
            CpuInfo::Risc => "RISC-V",
        }
    );
}

/// Run get_sys_info with 0x0008
fn get_sys_info_flash_dev_info<T>(uart: &mut T)
where
    T: core::fmt::Write,
{
    let result = match hal::rom_data::sys_info_api::flash_dev_info() {
        Ok(Some(result)) => result,
        Ok(None) => {
            _ = writeln!(uart, "flash_dev_info() not supported");
            return;
        }
        Err(e) => {
            _ = writeln!(uart, "Failed to get flash device info: {:?}", e);
            return;
        }
    };

    _ = writeln!(uart, "get_sys_info(FLASH_DEV_INFO/0x0008)");
    let size_lookup = |value| match value {
        FlashDevInfoSize::None => "None",
        FlashDevInfoSize::K8 => "8K",
        FlashDevInfoSize::K16 => "16K",
        FlashDevInfoSize::K32 => "32K",
        FlashDevInfoSize::K64 => "64K",
        FlashDevInfoSize::K128 => "128K",
        FlashDevInfoSize::K256 => "256K",
        FlashDevInfoSize::K512 => "512K",
        FlashDevInfoSize::M1 => "1M",
        FlashDevInfoSize::M2 => "2M",
        FlashDevInfoSize::M4 => "4M",
        FlashDevInfoSize::M8 => "8M",
        FlashDevInfoSize::M16 => "16M",
        FlashDevInfoSize::Unknown => "Unknown",
    };
    _ = writeln!(uart, "\tCS1 GPIO: {}", result.cs1_gpio());
    _ = writeln!(
        uart,
        "\tD8H Erase Supported: {}",
        result.d8h_erase_supported()
    );
    _ = writeln!(uart, "\tCS0 Size: {}", size_lookup(result.cs0_size()));
    _ = writeln!(uart, "\tCS1 Size: {}", size_lookup(result.cs1_size()));
}

/// Run get_sys_info with 0x0010
fn get_sys_info_boot_random<T>(uart: &mut T)
where
    T: core::fmt::Write,
{
    let result = match hal::rom_data::sys_info_api::boot_random() {
        Ok(Some(result)) => result,
        Ok(None) => {
            _ = writeln!(uart, "boot_random() not supported");
            return;
        }
        Err(e) => {
            _ = writeln!(uart, "Failed to get random boot integer: {:?}", e);
            return;
        }
    };

    _ = writeln!(uart, "get_sys_info(BOOT_RANDOM/0x0010)");
    _ = writeln!(uart, "\tA random number: 0x{:32x}", result.0);
}

/// Run get_sys_info with 0x0040;
fn get_sys_info_start_block<T>(uart: &mut T)
where
    T: core::fmt::Write,
{
    let result = match hal::rom_data::sys_info_api::boot_info() {
        Ok(Some(result)) => result,
        Ok(None) => {
            _ = writeln!(uart, "boot_info() not supported");
            return;
        }
        Err(e) => {
            _ = writeln!(uart, "Failed to get boot info: {:?}", e);
            return;
        }
    };

    _ = writeln!(uart, "get_sys_info(start_block/0x0040)");
    _ = writeln!(
        uart,
        "\tDiagnostic Partition: {}",
        match result.diagnostic_partition {
            PartitionIndex::Partition(_) => "Numbered partition",
            PartitionIndex::None => "None",
            PartitionIndex::Slot0 => "Slot 0",
            PartitionIndex::Slot1 => "Slot 1",
            PartitionIndex::Image => "Image",
            PartitionIndex::Unknown => "Unknown",
        }
    );
    _ = writeln!(
        uart,
        "\tBoot Type: {}",
        match result.boot_type {
            BootType::Normal => "Normal",
            BootType::BootSel => "bootsel",
            BootType::RamImage => "RAM image",
            BootType::FlashUpdate => "Flash update",
            BootType::PcSp => "pc_sp",
            BootType::Unknown => "Unknown",
        }
    );
    _ = writeln!(uart, "\tChained: {}", result.chained);
    _ = writeln!(uart, "\tPartition: {}", result.partition);
    _ = writeln!(uart, "\tTBYB Info: {:02x}", result.tbyb_update_info);
    _ = writeln!(uart, "\tBoot Diagnostic: {:04x}", result.boot_diagnostic);
    _ = writeln!(
        uart,
        "\tBoot Params: {:04x}, {:04x}",
        result.boot_params[0], result.boot_params[1]
    );
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
