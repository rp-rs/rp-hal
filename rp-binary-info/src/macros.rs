//! Handy macros for making Binary Info entries

/// Generate a static [`StringEntry`](crate::StringEntry) containing the given
/// environment variable, and return its [`EntryAddr`](super::EntryAddr).
#[macro_export]
macro_rules! env {
    ($tag:expr, $id:expr, $env_var_name:expr) => {
        $crate::str!($tag, $id, {
            let value = concat!(env!($env_var_name), "\0");
            // # Safety
            //
            // We used `concat!` to null-terminate on the line above.
            let value_cstr =
                unsafe { core::ffi::CStr::from_bytes_with_nul_unchecked(value.as_bytes()) };
            value_cstr
        })
    };
}

/// Generate a static [`StringEntry`](crate::StringEntry) containing the given
/// string, and return its [`EntryAddr`](super::EntryAddr).
///
/// You must pass a numeric tag, a numeric ID, and `&CStr` (which is always
/// null-terminated).
///
/// # Example
///
/// ```
/// # use rp_binary_info::{EntryAddr, consts, str};
/// let entry: EntryAddr = str!(
///     consts::TAG_RASPBERRY_PI,
///     consts::ID_RP_PROGRAM_DESCRIPTION,
///     c"Your program description goes here"
/// );
/// ```
#[macro_export]
macro_rules! str {
    ($tag:expr, $id:expr, $str:expr) => {{
        static ENTRY: $crate::StringEntry = $crate::StringEntry::new($tag, $id, $str);
        ENTRY.addr()
    }};
}

/// Generate a static [`IntegerEntry`](crate::IntegerEntry) containing the given
/// integer, and return its [`EntryAddr`](super::EntryAddr).
///
/// You must pass a numeric tag, a numeric ID, and a `u32`.
///
/// # Example
///
/// ```
/// # use rp_binary_info::{EntryAddr, make_tag, int};
/// const EXAMPLE_ID: u32 = 0x12345678;
/// let entry: EntryAddr = int!(
///     make_tag(b"EX"),
///     EXAMPLE_ID,
///     1234
/// );
/// ```
#[macro_export]
macro_rules! int {
    ($tag:expr, $id:expr, $int:expr) => {{
        static ENTRY: $crate::IntegerEntry = $crate::IntegerEntry::new($tag, $id, $int);
        ENTRY.addr()
    }};
}

/// Generate a static [`PointerEntry`](crate::PointerEntry) containing the given
/// pointer, and return its [`EntryAddr`](super::EntryAddr).
///
/// You must pass a numeric tag, a numeric ID, and a pointer.
///
/// # Example
///
/// ```
/// # use rp_binary_info::{EntryAddr, make_tag, pointer};
/// static ITEM: u32 = 123;
/// const EXAMPLE_ID: u32 = 0x12345678;
/// let entry: EntryAddr = pointer!(
///     make_tag(b"EX"),
///     EXAMPLE_ID,
///     (&raw const ITEM) as *const ()
/// );
/// ```
#[macro_export]
macro_rules! pointer {
    ($tag:expr, $id:expr, $ptr:expr) => {{
        static ENTRY: $crate::PointerEntry = $crate::PointerEntry::new($tag, $id, $ptr);
        ENTRY.addr()
    }};
}

/// Concatenate a list of names for the [`PinsWithName`](crate::PinsWithName)
/// structure, returning a `&CStr`.
///
/// This macro adds a '|' character between each string and converts the result
/// to `&CStr`.
///
/// This macro is used by [`pins_with_names!`](super::pins_with_names) for the
/// names management.
///
/// # Example
///
/// ```
/// # use rp_binary_info::*;
/// # use core::ffi::CStr;
/// let concatenated: &CStr = pins_names_concat!("A", "B", "C");
/// assert_eq!(c"A|B|C", concatenated);
/// ```
#[macro_export]
macro_rules! pins_names_concat {
    // For a list of names in &[], just send them flat to the macro
    (&[$($names: expr),+]) => {
        pins_names_concat!($($names),+)
    };

    // For a single string, convert the result to &CStr
    ($names:expr) => {
        if let Ok(x) = core::ffi::CStr::from_bytes_until_nul((concat!($names, "\0")).as_bytes()) {
            x
        } else {
            panic!("Failed to convert &str to &Cstr");
        }
    };

    // For two strings, concatenate them to a single string and return its conversion
    ($names:expr, $name:expr) => {
        pins_names_concat!(concat!($names, "|" ,$name))
    };

    // For multiple strings, recursively concatenate them
    ($names:expr, $second:expr, $($more:expr),+) => {
        pins_names_concat!(concat!($names, "|", $second), $($more),+)
    };
}

/// Generate a static [`PinsWithName`](crate::PinsWithName) and return its
/// [`EntryAddr`](super::EntryAddr).
///
/// Usage: `pins_with_names!(pins: &[u32], names...)`
///
/// * `pins` is the list of pin numbers being named
///     - pin numbers must be in strictly ascending order
/// * `names` is the name(s) for the pins with either:
///     - a single `&str` to name all the pins with the same label, or
///     - a list of `&str` to individually name the pins (either as separate
///       arguments, or in a `&[&str]`).
///
/// # Example
///
/// ```
/// # use rp_binary_info::*;
/// // Pins 0 and 1 are named "UART"
/// let entry: EntryAddr = pins_with_names!(&[0, 1], "UART");
/// // Pin 0 is "RX", Pin 1 is "TX"
/// let entry: EntryAddr = pins_with_names!(&[0, 1], "RX", "TX");
/// let entry: EntryAddr = pins_with_names!(&[0, 1], &["RX", "TX"]);
/// ```
#[macro_export]
macro_rules! pins_with_names {
    ($pins:expr, &[$($names: literal),+]) => {
        $crate::pins_with_names!($pins, $($names),+)
    };

    ($pins:expr, $($names:literal),+) => {{
        static ENTRY: $crate::PinsWithName =
            $crate::PinsWithName::new($pins, ($crate::pins_names_concat!($($names),+)));
        ENTRY.addr()
    }};
}

/// Generate a static [`PinsWithFunction`](crate::PinsWithFunction) and return
/// its [`EntryAddr`](super::EntryAddr).
///
/// Usage: `pins_with_func!(pins: &[u32], func: PinFunction)`
///
/// * `pins` is the list of pins to label,
/// * `func` is the [`PinFunction`](super::PinFunction) to label the pins with.
///
/// **NOTE** Using this method, you can only assign up to 5 pins - for more pins
/// see [`pins_range_with_func!`](crate::pins_range_with_func!).
///
/// # Example
///
/// ```
/// # use rp_binary_info::*;
/// // Pins 0 and 1 are used for the UART
/// let entry: EntryAddr = pins_with_func!(&[0, 1], PinFunction::Uart);
/// ```
#[macro_export]
macro_rules! pins_with_func {
    ($pins:expr, $func: expr) => {{
        static ENTRY: $crate::PinsWithFunction = $crate::PinsWithFunction::new($pins, $func);
        ENTRY.addr()
    }};
}

/// Generate a static [`PinsWithFunction`](crate::PinsWithFunction) and return
/// its [`EntryAddr`](super::EntryAddr).
///
/// Usage: `pins_range_with_func!(low: u32, high: u32, func: PinFunction)`
///
/// * `low` and `high` are the boundaries of the pin number range `low..=high`,
/// * `func` is the function to assign to those pins.
///
/// # Example
///
/// ```
/// # use rp_binary_info::*;
/// // Pins 2, 3, 4 and 5 are used for SPI
/// let entry: EntryAddr = pins_range_with_func!(2, 5, PinFunction::Spi);
/// ```
#[macro_export]
macro_rules! pins_range_with_func {
    ($low:expr, $high:expr, $func:expr) => {{
        static ENTRY: $crate::PinsWithFunction =
            $crate::PinsWithFunction::new_range($low, $high, $func);
        ENTRY.addr()
    }};
}

/// Generate a static [`StringEntry`](crate::StringEntry) containing the program
/// name, and return its [`EntryAddr`](super::EntryAddr).
///
/// # Example
///
/// ```ignore
/// let entry: EntryAddr = rp_program_name!();
/// ```
#[macro_export]
macro_rules! rp_program_name {
    ($name:expr) => {
        $crate::str!(
            $crate::consts::TAG_RASPBERRY_PI,
            $crate::consts::ID_RP_PROGRAM_NAME,
            $name
        )
    };
}

/// Generate a static [`StringEntry`](crate::StringEntry) containing the
/// `CARGO_BIN_NAME` as the program name, and return its
/// [`EntryAddr`](super::EntryAddr).
///
/// # Example
///
/// ```ignore
/// let entry: EntryAddr = rp_cargo_bin_name!();
/// ```
#[macro_export]
macro_rules! rp_cargo_bin_name {
    () => {
        $crate::env!(
            $crate::consts::TAG_RASPBERRY_PI,
            $crate::consts::ID_RP_PROGRAM_NAME,
            "CARGO_BIN_NAME"
        )
    };
}

/// Generate a static [`StringEntry`](crate::StringEntry) containing the program
/// version, and return its [`EntryAddr`](super::EntryAddr).
///
/// # Example
///
/// ```ignore
/// let entry: EntryAddr = rp_program_version!();
/// ```
#[macro_export]
macro_rules! rp_program_version {
    ($version:expr) => {{
        $crate::str!(
            $crate::consts::TAG_RASPBERRY_PI,
            $crate::consts::ID_RP_PROGRAM_VERSION,
            $version
        )
    }};
}

/// Generate a static [`StringEntry`](crate::StringEntry) containing the
/// `CARGO_PKG_VERSION` as the program version, and return its
/// [`EntryAddr`](super::EntryAddr).
///
/// # Example
///
/// ```ignore
/// let entry: EntryAddr = rp_cargo_version!();
/// ```
#[macro_export]
macro_rules! rp_cargo_version {
    () => {
        $crate::env!(
            $crate::consts::TAG_RASPBERRY_PI,
            $crate::consts::ID_RP_PROGRAM_VERSION_STRING,
            "CARGO_PKG_VERSION"
        )
    };
}

/// Generate a static [`StringEntry`](crate::StringEntry) containing the program
/// URL, and return its [`EntryAddr`](super::EntryAddr).
///
/// # Example
///
/// ```ignore
/// let entry: EntryAddr = rp_program_url!();
/// ```
#[macro_export]
macro_rules! rp_program_url {
    ($url:expr) => {
        $crate::str!(
            $crate::consts::TAG_RASPBERRY_PI,
            $crate::consts::ID_RP_PROGRAM_URL,
            $url
        )
    };
}

/// Generate a static [`StringEntry`](crate::StringEntry) containing the
/// `CARGO_PKG_HOMEPAGE` as the program URL, and return its
/// [`EntryAddr`](super::EntryAddr).
///
/// # Example
///
/// ```ignore
/// let entry: EntryAddr = rp_cargo_homepage_url!();
/// ```
#[macro_export]
macro_rules! rp_cargo_homepage_url {
    () => {
        $crate::env!(
            $crate::consts::TAG_RASPBERRY_PI,
            $crate::consts::ID_RP_PROGRAM_URL,
            "CARGO_PKG_HOMEPAGE"
        )
    };
}

/// Generate a static [`StringEntry`](crate::StringEntry) containing the program
/// description, and return its [`EntryAddr`](super::EntryAddr).
///
/// # Example
///
/// ```ignore
/// let entry: EntryAddr = rp_program_description!();
/// ```
#[macro_export]
macro_rules! rp_program_description {
    ($description:expr) => {
        $crate::str!(
            $crate::consts::TAG_RASPBERRY_PI,
            $crate::consts::ID_RP_PROGRAM_DESCRIPTION,
            $description
        )
    };
}

/// Generate a static [`StringEntry`](crate::StringEntry) containing the
/// `CARGO_PKG_DESCRIPTION` as the program description, and return its
/// [`EntryAddr`](super::EntryAddr).
///
/// # Example
///
/// ```ignore
/// let entry: EntryAddr = rp_cargo_description!();
/// ```
#[macro_export]
macro_rules! rp_cargo_description {
    () => {
        $crate::env!(
            $crate::consts::TAG_RASPBERRY_PI,
            $crate::consts::ID_RP_PROGRAM_DESCRIPTION,
            "CARGO_PKG_DESCRIPTION"
        )
    };
}

/// Generate a static [`StringEntry`](crate::StringEntry) containing whether
/// this is a debug or a release build, and return its
/// [`EntryAddr`](super::EntryAddr).
///
/// # Example
///
/// ```ignore
/// let entry: EntryAddr = rp_program_build_attribute!();
/// ```
#[macro_export]
macro_rules! rp_program_build_attribute {
    () => {
        $crate::str!(
            $crate::consts::TAG_RASPBERRY_PI,
            $crate::consts::ID_RP_PROGRAM_BUILD_ATTRIBUTE,
            {
                if cfg!(debug_assertions) {
                    c"debug"
                } else {
                    c"release"
                }
            }
        )
    };
}

/// Generate a static [`StringEntry`](crate::StringEntry) containing the
/// specific board this program runs on, and return its
/// [`EntryAddr`](super::EntryAddr).
///
/// # Example
///
/// ```ignore
/// let entry: EntryAddr = rp_pico_board!("pico2");
/// ```
#[macro_export]
macro_rules! rp_pico_board {
    ($board:expr) => {
        $crate::str!(
            $crate::consts::TAG_RASPBERRY_PI,
            $crate::consts::ID_RP_PICO_BOARD,
            $board
        )
    };
}

/// Generate a static [`PointerEntry`](crate::PointerEntry) containing the
/// binary end address, and return its [`EntryAddr`](super::EntryAddr).
///
/// The argument should be a symbol provided by the linker script that is
/// located at the end of the binary. For example, you could place this in your
/// linker script:
///
/// ```text
/// SECTIONS {
///     .end_block : ALIGN(4)
///     {
///         __flash_binary_end = .;
///     } > FLASH
/// } INSERT AFTER .uninit;
/// ```
///
/// # Example
///
/// ```ignore
/// let entry: EntryAddr = rp_binary_end!(__flash_binary_end);
/// ```
#[macro_export]
macro_rules! rp_binary_end {
    ($ptr:ident) => {{
        $crate::pointer!(
            $crate::consts::TAG_RASPBERRY_PI,
            $crate::consts::ID_RP_BINARY_END,
            core::ptr::addr_of!($ptr).cast()
        )
    }};
}

#[cfg(test)]
mod test {
    #[test]
    fn names_concatenation_returns_single_name_if_only_one_provided() {
        assert_eq!(c"a", pins_names_concat!("a"));
        assert_eq!(c"a", pins_names_concat!(&["a"]));
    }

    #[test]
    fn names_concatenation_returns_pipe_separated_list_of_names() {
        assert_eq!(c"a|b|c", pins_names_concat!(&["a", "b", "c"]));
    }
}

// End of file
