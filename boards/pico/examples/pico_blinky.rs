//! # Pico Blinky Example
//!
//! Blinks the LED on a Pico board.
//!
//! This will blink an LED attached to GP25, which is the pin the Pico uses for
//! the on-board LED.
//!
//! See the `Cargo.toml` file for Copyright and licence details.

#![no_std]
#![no_main]

// The macro for our start-up function
use cortex_m_rt::entry;

// GPIO traits
use embedded_hal::digital::v2::OutputPin;

// Time handling traits
use embedded_time::rate::*;

// Ensure we halt the program on panic (if we don't mention this crate it won't
// be linked)
use panic_halt as _;

// Pull in any important traits
use pico::hal::prelude::*;

// A shorter alias for the Peripheral Access Crate, which provides low-level
// register access
use pico::hal::pac;

// A shorter alias for the Hardware Abstraction Layer, which provides
// higher-level drivers.
use pico::hal;

//// The linker will place this boot block at the start of our program image. We
//// need this to help the ROM bootloader get our code up and running.
#[link_section = ".boot2"]
#[used]
pub static BOOT2: [u8; 256] = rp2040_boot2::BOOT_LOADER;

pub mod binary_info {
    /// This is the 'Binary Info' header block that `picotool` looks for in your
    /// UF2 file to give you useful metadata about your program. It should be
    /// placed in the first 256 bytes of your program, so use your `memory.x` to
    /// insert a section between `.text` and `.vector_table` and put a static
    /// value of this type in that section.
    #[repr(C)]
    pub struct Header {
        /// Must be equal to Picotool::MARKER_START
        marker_start: u32,
        /// The first in our table of pointers to Entries
        entries_start: &'static EntryAddr,
        /// The last in our table of pointers to Entries
        entries_end: &'static EntryAddr,
        /// The first entry in a null-terminated RAM/Flash mapping table
        mapping_table: *const MappingTableEntry,
        /// Must be equal to Picotool::MARKER_END
        marker_end: u32,
    }

    /// This is a reference to an entry. It's like a `&dyn` ref to some type `T:
    /// Entry`, except that the run-time type information is encoded into the
    /// Entry itself in very specific way.
    #[repr(transparent)]
    pub struct EntryAddr(*const u32);

    /// Allows us to tell picotool where values are in the UF2 given their
    /// run-time address. The most obvious example is RAM variables, which must
    /// be found in the `.data` section of the UF2.
    #[repr(C)]
    pub struct MappingTableEntry {
        pub source_addr_start: *const u32,
        pub dest_addr_start: *const u32,
        pub dest_addr_end: *const u32,
    }

    /// This is the set of data types that `picotool` supports.
    #[repr(u16)]
    pub enum DataType {
        Raw = 1,
        SizedData = 2,
        BinaryInfoListZeroTerminated = 3,
        Bson = 4,
        IdAndInt = 5,
        IdAndString = 6,
        BlockDevice = 7,
        PinsWithFunction = 8,
        PinsWithName = 9,
        PinsWithNames = 10,
    }

    /// All Entries start with this common header
    #[repr(C)]
    struct EntryCommon {
        data_type: DataType,
        tag: u16,
    }

    /// An entry which contains both an ID (e.g. `ID_RP_PROGRAM_NAME`) and a pointer to a null-terminated string.
    #[repr(C)]
    pub struct EntryWithIdAndString {
        header: EntryCommon,
        id: u32,
        value: *const u8,
    }

    /// An entry which contains both an ID (e.g. `ID_RP_BINARY_END`) and an integer.
    #[repr(C)]
    pub struct EntryWithIdAndInt {
        header: EntryCommon,
        id: u32,
        value: u32,
    }

    /// All Raspberry Pi specified IDs have this tag. You can create your own
    /// for custom fields.
    pub const TAG_RASPBERRY_PI: u16 = make_tag(b'R', b'P');

    /// Used to note the program name - use with EntryWithIdAndString
    pub const ID_RP_PROGRAM_NAME: u32 = 0x02031c86;
    /// Used to note the program version - use with EntryWithIdAndString
    pub const ID_RP_PROGRAM_VERSION_STRING: u32 = 0x11a9bc3a;
    /// Used to note the program build date - use with EntryWithIdAndString
    pub const ID_RP_PROGRAM_BUILD_DATE_STRING: u32 = 0x9da22254;
    /// Used to note the size of the binary - use with EntryWithIdAndInt
    pub const ID_RP_BINARY_END: u32 = 0x68f465de;
    /// Used to note a URL for the program - use with EntryWithIdAndString
    pub const ID_RP_PROGRAM_URL: u32 = 0x1856239a;
    /// Used to note a description of the program - use with EntryWithIdAndString
    pub const ID_RP_PROGRAM_DESCRIPTION: u32 = 0xb6a07c19;
    /// Used to note some feature of the program - use with EntryWithIdAndString
    pub const ID_RP_PROGRAM_FEATURE: u32 = 0xa1f4b453;
    /// Used to note some whether this was a Debug or Release build - use with EntryWithIdAndString
    pub const ID_RP_PROGRAM_BUILD_ATTRIBUTE: u32 = 0x4275f0d3;
    /// Used to note the Pico SDK version used - use with EntryWithIdAndString
    pub const ID_RP_SDK_VERSION: u32 = 0x5360b3ab;
    /// Used to note which board this program targets - use with EntryWithIdAndString
    pub const ID_RP_PICO_BOARD: u32 = 0xb63cffbb;
    /// Used to note which `boot2` image this program uses - use with EntryWithIdAndString
    pub const ID_RP_BOOT2_NAME: u32 = 0x7f8882e1;

    impl Header {
        /// This is the `BINARY_INFO_MARKER_START` magic value from `picotool`
        const MARKER_START: u32 = 0x7188ebf2;
        /// This is the `BINARY_INFO_MARKER_END` magic value from `picotool`
        const MARKER_END: u32 = 0xe71aa390;

        /// Create a new `picotool` compatible header.
        ///
        /// * `entries_start` - the first [`EntryAddr`](binary_info::EntryAddr) in the table
        /// * `entries_end` - the last [`EntryAddr`](binary_info::EntryAddr) in the table
        /// * `mapping_table` - the RAM/Flash address mapping table
        pub const fn new(
            entries_start: &'static EntryAddr,
            entries_end: &'static EntryAddr,
            mapping_table: &'static [MappingTableEntry],
        ) -> Self {
            let mapping_table = mapping_table.as_ptr();
            Self {
                marker_start: Self::MARKER_START,
                entries_start,
                entries_end,
                mapping_table,
                marker_end: Self::MARKER_END,
            }
        }
    }

    impl EntryWithIdAndString {
        /// Get this entry's address
        pub const fn addr(&self) -> EntryAddr {
            EntryAddr(self as *const Self as *const u32)
        }
    }

    impl EntryWithIdAndInt {
        /// Get this entry's address
        pub const fn addr(&self) -> EntryAddr {
            EntryAddr(self as *const Self as *const u32)
        }
    }

    /// Create a 'Binary Info' entry containing the program name
    ///
    /// The given string must be null-terminated, so put a `\0` at the end of
    /// it.
    pub const fn program_name(name: &'static str) -> EntryWithIdAndString {
        EntryWithIdAndString {
            header: EntryCommon {
                data_type: DataType::IdAndString,
                tag: TAG_RASPBERRY_PI,
            },
            id: ID_RP_PROGRAM_NAME,
            value: name.as_ptr() as *const u8,
        }
    }

    /// Create a 'Binary Info' entry containing the program version.
    ///
    /// The given string must be null-terminated, so put a `\0` at the end of
    /// it.
    pub const fn version(name: &'static str) -> EntryWithIdAndString {
        EntryWithIdAndString {
            header: EntryCommon {
                data_type: DataType::IdAndString,
                tag: TAG_RASPBERRY_PI,
            },
            id: ID_RP_PROGRAM_VERSION_STRING,
            value: name.as_ptr() as *const u8,
        }
    }

    /// Create a 'Binary Info' entry containing a custom integer entry.
    pub const fn custom_integer(tag: u16, id: u32, value: u32) -> EntryWithIdAndInt {
        EntryWithIdAndInt {
            header: EntryCommon {
                data_type: DataType::IdAndInt,
                tag,
            },
            id,
            value,
        }
    }

    /// Create a tag from two ASCII letters.
    pub const fn make_tag(c1: u8, c2: u8) -> u16 {
        u16::from_be_bytes([c2, c1])
    }

    // We need this as rustc complains that is is unsafe to share `*const u32`
    // pointers between threads. We only allow these to be created with static
    // data, so this is OK.
    unsafe impl Sync for Header {}

    // We need this as rustc complains that is is unsafe to share `*const u8`
    // pointers between threads. We only allow these to be created with static
    // string slices, so it's OK.
    unsafe impl Sync for EntryWithIdAndString {}

    // We need this as rustc complains that is is unsafe to share `*const u32`
    // pointers between threads. We only allow these to be created with static
    // data, so this is OK.
    unsafe impl Sync for MappingTableEntry {}

    // We need this as rustc complains that is is unsafe to share `*const u32`
    // pointers between threads. We only allow these to be created with static
    // data, so this is OK.
    unsafe impl Sync for EntryAddr {}
}

extern "C" {
    static __bi_entries_start: binary_info::EntryAddr;
    static __bi_entries_end: binary_info::EntryAddr;
    static __sdata: u32;
    static __edata: u32;
    static __sidata: u32;
}

/// Picotool can find this block in our ELF file and report interesting metadata.
#[link_section = ".bi_header"]
#[used]
pub static PICOTOOL_META: binary_info::Header =
    unsafe { binary_info::Header::new(&__bi_entries_start, &__bi_entries_end, &MAPPING_TABLE) };

/// This tells picotool how to convert RAM addresses back into Flash addresses
static MAPPING_TABLE: [binary_info::MappingTableEntry; 2] = [
    // This is the entry for .data
    binary_info::MappingTableEntry {
        source_addr_start: unsafe { &__sidata },
        dest_addr_start: unsafe { &__sdata },
        dest_addr_end: unsafe { &__edata },
    },
    // This is the terminating marker
    binary_info::MappingTableEntry {
        source_addr_start: core::ptr::null(),
        dest_addr_start: core::ptr::null(),
        dest_addr_end: core::ptr::null(),
    },
];

/// This is a list of references to our table entries
#[link_section = ".bi_entries"]
#[used]
pub static PICOTOOL_ENTRIES: [binary_info::EntryAddr; 3] = [
    PROGRAM_NAME.addr(),
    PROGRAM_VERSION.addr(),
    NUMBER_OF_KITTENS.addr(),
];

/// This is the name of our program
static PROGRAM_NAME: binary_info::EntryWithIdAndString =
    binary_info::program_name(concat!("my stupid tool 2", "\0"));

/// This is the version of our program
static PROGRAM_VERSION: binary_info::EntryWithIdAndString =
    binary_info::version(concat!(env!("GIT_VERSION"), "\0"));

/// This is just some application-specific random information to test integer support
static NUMBER_OF_KITTENS: binary_info::EntryWithIdAndInt =
    binary_info::custom_integer(binary_info::make_tag(b'J', b'P'), 0x0000_0001, 0x12345678);

/// Entry point to our bare-metal application.
///
/// The `#[entry]` macro ensures the Cortex-M start-up code calls this function
/// as soon as all global variables are initialised.
///
/// The function configures the RP2040 peripherals, then blinks the LED in an
/// infinite loop.
#[entry]
fn main() -> ! {
    // Grab our singleton objects
    let mut pac = pac::Peripherals::take().unwrap();
    let core = pac::CorePeripherals::take().unwrap();

    // Set up the watchdog driver - needed by the clock setup code
    let mut watchdog = hal::watchdog::Watchdog::new(pac.WATCHDOG);

    // Configure the clocks
    //
    // The default is to generate a 125 MHz system clock
    let clocks = hal::clocks::init_clocks_and_plls(
        pico::XOSC_CRYSTAL_FREQ,
        pac.XOSC,
        pac.CLOCKS,
        pac.PLL_SYS,
        pac.PLL_USB,
        &mut pac.RESETS,
        &mut watchdog,
    )
    .ok()
    .unwrap();

    // The delay object lets us wait for specified amounts of time (in
    // milliseconds)
    let mut delay = cortex_m::delay::Delay::new(core.SYST, clocks.system_clock.freq().integer());

    // The single-cycle I/O block controls our GPIO pins
    let sio = hal::sio::Sio::new(pac.SIO);

    // Set the pins up according to their function on this particular board
    let pins = pico::Pins::new(
        pac.IO_BANK0,
        pac.PADS_BANK0,
        sio.gpio_bank0,
        &mut pac.RESETS,
    );

    // Set the LED to be an output
    let mut led_pin = pins.led.into_push_pull_output();

    // Blink the LED at 1 Hz
    loop {
        led_pin.set_high().unwrap();
        delay.delay_ms(500);
        led_pin.set_low().unwrap();
        delay.delay_ms(500);
    }
}

// End of file
