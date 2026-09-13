//! Types for the Binary Info system

use crate::consts::TAG_RASPBERRY_PI;

/// This is the 'Binary Info' header block that `picotool` looks for in your UF2
/// file/ELF file/Pico in Bootloader Mode to give you useful metadata about your
/// program.
///
/// It should be placed in the first 4096 bytes of flash, so use your `memory.x`
/// to insert a section between `.text` and `.vector_table` and put a static
/// value of this type in that section.
#[repr(C)]
pub struct Header {
    /// Must be equal to Picotool::MARKER_START
    marker_start: u32,
    /// The first in our table of pointers to Entries
    entries_start: *const EntryAddr,
    /// The last in our table of pointers to Entries
    entries_end: *const EntryAddr,
    /// The first entry in a null-terminated RAM/Flash mapping table
    mapping_table: *const MappingTableEntry,
    /// Must be equal to Picotool::MARKER_END
    marker_end: u32,
}

impl Header {
    /// This is the `BINARY_INFO_MARKER_START` magic value from `picotool`
    const MARKER_START: u32 = 0x7188ebf2;
    /// This is the `BINARY_INFO_MARKER_END` magic value from `picotool`
    const MARKER_END: u32 = 0xe71aa390;

    /// Create a new `picotool` compatible header.
    ///
    /// * `entries_start` - the first [`EntryAddr`] in the table
    /// * `entries_end` - the last [`EntryAddr`] in the table
    /// * `mapping_table` - the RAM/Flash address mapping table
    pub const fn new(
        entries_start: *const EntryAddr,
        entries_end: *const EntryAddr,
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

// We need this as rustc complains that is is unsafe to share `*const u32`
// pointers between threads. We only allow these to be created with static
// data, so this is OK.
unsafe impl Sync for Header {}

/// This is a reference to an entry. It's like a `&dyn` ref to some type `T:
/// Entry`, except that the run-time type information is encoded into the
/// Entry itself in very specific way.
#[repr(transparent)]
pub struct EntryAddr(*const u32);

// We need this as rustc complains that is is unsafe to share `*const u32`
// pointers between threads. We only allow these to be created with static
// data, so this is OK.
unsafe impl Sync for EntryAddr {}

/// Allows us to tell picotool where values are in the UF2 given their run-time
/// address.
///
/// The most obvious example is RAM variables, which must be found in the
/// `.data` section of the UF2.
#[repr(C)]
pub struct MappingTableEntry {
    /// The start address in RAM (or wherever the address picotool finds will
    /// point)
    pub source_addr_start: *const u32,
    /// The start address in flash (or wherever the data actually lives in the
    /// ELF)
    pub dest_addr_start: *const u32,
    /// The end address in flash
    pub dest_addr_end: *const u32,
}

impl MappingTableEntry {
    /// Generate a null entry to mark the end of the list
    pub const fn null() -> MappingTableEntry {
        MappingTableEntry {
            source_addr_start: core::ptr::null(),
            dest_addr_start: core::ptr::null(),
            dest_addr_end: core::ptr::null(),
        }
    }
}

// We need this as rustc complains that is is unsafe to share `*const u32`
// pointers between threads. We only allow these to be created with static
// data, so this is OK.
unsafe impl Sync for MappingTableEntry {}

/// This is the set of data types that `picotool` supports.
#[repr(u16)]
pub enum DataType {
    /// Raw data
    Raw = 1,
    /// Data with a size
    SizedData = 2,
    /// A list of binary data
    BinaryInfoListZeroTerminated = 3,
    /// A BSON encoded blob
    Bson = 4,
    /// An Integer with an ID
    IdAndInt = 5,
    /// A string with an Id
    IdAndString = 6,
    /// A block device
    BlockDevice = 7,
    /// GPIO pins, with their function
    PinsWithFunction = 8,
    /// GPIO pins, with their name
    PinsWithName = 9,
    /// GPIO pins, with multiple names?
    PinsWithNames = 10,
}

/// All Entries start with this common header
#[repr(C)]
struct EntryCommon {
    data_type: DataType,
    tag: u16,
}

/// An entry which contains both an ID (e.g. `ID_RP_PROGRAM_NAME`) and a pointer
/// to a null-terminated string.
#[repr(C)]
pub struct StringEntry {
    header: EntryCommon,
    id: u32,
    value: *const core::ffi::c_char,
}

impl StringEntry {
    /// Create a new [`StringEntry`]
    pub const fn new(tag: u16, id: u32, value: &'static core::ffi::CStr) -> StringEntry {
        StringEntry {
            header: EntryCommon {
                data_type: DataType::IdAndString,
                tag,
            },
            id,
            value: value.as_ptr(),
        }
    }

    /// Get this entry's address
    pub const fn addr(&self) -> EntryAddr {
        EntryAddr(self as *const Self as *const u32)
    }
}

// We need this as rustc complains that is is unsafe to share `*const
// core::ffi::c_char` pointers between threads. We only allow these to be
// created with static string slices, so it's OK.
unsafe impl Sync for StringEntry {}

/// An entry which contains both an ID (e.g. `ID_RP_BINARY_END`) and an integer.
#[repr(C)]
pub struct IntegerEntry {
    header: EntryCommon,
    id: u32,
    value: u32,
}

impl IntegerEntry {
    /// Create a new [`IntegerEntry`]
    pub const fn new(tag: u16, id: u32, value: u32) -> IntegerEntry {
        IntegerEntry {
            header: EntryCommon {
                data_type: DataType::IdAndInt,
                tag,
            },
            id,
            value,
        }
    }

    /// Get this entry's address
    pub const fn addr(&self) -> EntryAddr {
        EntryAddr(self as *const Self as *const u32)
    }
}

/// An alias for IntegerEntry, taking a pointer instead of an integer
#[repr(C)]
pub struct PointerEntry {
    header: EntryCommon,
    id: u32,
    value: *const (),
}

impl PointerEntry {
    /// Create a new [`PointerEntry`]
    ///
    /// Pointers will be marked as 32-bit integers in the binary information
    /// structure, as there is no separate data type tag for pointers. This
    /// assumes that pointers are 32 bit wide, which is obviously true for
    /// rp2040/rp2350. On 64 bit architectures, it will create a binary
    /// structure that likely can't be parsed by picotool.
    pub const fn new(tag: u16, id: u32, value: *const ()) -> PointerEntry {
        PointerEntry {
            header: EntryCommon {
                data_type: DataType::IdAndInt,
                tag,
            },
            id,
            value,
        }
    }

    /// Get this entry's address
    pub const fn addr(&self) -> EntryAddr {
        EntryAddr(self as *const Self as *const u32)
    }
}

// We need this as rustc complains that is is unsafe to share `*const u32`
// pointers between threads. We only allow these to be created with static
// data, so this is OK.
unsafe impl Sync for PointerEntry {}

/// A structure for multiple pins with name info
#[repr(C)]
pub struct PinsWithName {
    header: EntryCommon,
    mask: u32,
    label: *const core::ffi::c_char,
}

impl PinsWithName {
    /// Create a new [`PinsWithName`] from a list of pins and labels
    ///
    /// * `pins` - an ordered list of pins (low to high)
    /// * `labels` - a [`&CStr`](core::ffi::CStr) to label those pins with,
    ///   either as `"LABEL"` or `"LABEL_A|LABEL_B|..."`
    ///
    /// As the pins are converted into a mask, pins have to be strictly sorted
    /// for the pins list to match the labels (if multiple labels used with
    /// [`pins_names_concat!()`](super::pins_names_concat))
    pub const fn new(pins: &[u32], labels: &'static core::ffi::CStr) -> Self {
        Self {
            header: EntryCommon {
                data_type: DataType::PinsWithName,
                tag: TAG_RASPBERRY_PI,
            },
            mask: Self::fold_pins_list_to_mask(0, pins),
            label: labels.as_ptr(),
        }
    }
    /// Get this entry's address
    pub const fn addr(&self) -> EntryAddr {
        EntryAddr(self as *const Self as *const u32)
    }

    /// Recursively folds the ordered list of pins into a 32bits mask representation
    ///
    /// * `mask` current value of the mask (should be set to 0 at initial call),
    /// * `rest` list of pins not yet integrated to the mask.
    ///
    /// This function ensures the pins are ordered.
    /// We have to use recursion as const rust `iter` is unstable.
    const fn fold_pins_list_to_mask(mask: u32, rest: &[u32]) -> u32 {
        match rest {
            [] => mask,
            [first_pin, rest_pins @ ..] => {
                let masked_pin =
                    shl_or_panic(1u32, *first_pin, "Pin number should be between 0 and 31");

                assert!(mask < masked_pin, "Pins should be in increasing order");

                Self::fold_pins_list_to_mask(mask | masked_pin, rest_pins)
            }
        }
    }
}

unsafe impl Sync for PinsWithName {}

/// A structure for multiple pins with function definition
#[repr(C)]
pub struct PinsWithFunction {
    header: EntryCommon,
    encoding: u32,
}

const BI_PINS_ENCODING_MULTI: u32 = 2;
const BI_PINS_ENCODING_RANGE: u32 = 1;

impl PinsWithFunction {
    /// Create a new [`PinsWithFunction`] for multiple pins
    ///
    /// * `pins` The list of pins to implement this function (maximum 5)
    /// * `func` The [`PinFunction`](super::PinFunction) to assign to these pins
    pub const fn new(pins: &[u32], func: crate::PinFunction) -> Self {
        assert!(
            pins.len() <= 5,
            "Individual pins with function must have at most 5 pins"
        );

        let func = shl_or_panic(func as u32, 3, "Failed to shift PinFunction");
        let base: u32 = BI_PINS_ENCODING_MULTI | func;
        if let Some(encoding) = Self::encode_pins(base, pins, 0, 0) {
            Self {
                header: EntryCommon {
                    data_type: DataType::PinsWithFunction,
                    tag: TAG_RASPBERRY_PI,
                },
                encoding,
            }
        } else {
            panic!("All pins should be between 0 and 31");
        }
    }

    /// Create a new [`PinsWithFunction`] from a pins range
    ///
    /// Apply function `func` to pins range going from `pin_low` to `pin_high`
    ///
    /// * `pin_low` and `pin_low` are the boundaries of the pin number range `pin_low..=pin_high`,
    /// * `func` [`PinFunction`](super::PinFunction) to label the pins with.
    pub const fn new_range(pin_low: u32, pin_high: u32, func: crate::PinFunction) -> Self {
        let func = shl_or_panic(func as u32, 3, "Failed to shift GpioFunction");
        let base: u32 = BI_PINS_ENCODING_RANGE
            | func
            | shl_or_panic(pin_low, 7, "Failed to shift pin_low")
            | shl_or_panic(pin_high, 12, "Failed to shift_pin_high");

        Self {
            header: EntryCommon {
                data_type: DataType::PinsWithFunction,
                tag: TAG_RASPBERRY_PI,
            },
            encoding: base,
        }
    }

    /// Get this entry's address
    pub const fn addr(&self) -> EntryAddr {
        EntryAddr(self as *const Self as *const u32)
    }

    /// Use recursion to iterate through the list of values and fold them to a single `u32` value.
    ///
    /// According to pico-sdk, the `encoding` value should look like this:
    ///
    /// - `bits[0..=2]`: Encoding format (`BI_PINS_ENCODING_RANGE` or `BI_PINS_ENCODING_MULTI`),
    /// - `bits[3..=6]`: a `PinFunction` value,
    /// - `bits[7..=11]`: first pin number,
    /// - `bits[12..=16]`: second pin number (or first pin number duplicated if only one pin),
    /// - `bits[17..=21]`: third pin number (or second pin number duplicated if only two pins),
    /// - `bits[22..=26]`: fourth pin number (or third pin number duplicated if only three pins),
    /// - `bits[27..=31]`: fifth pin number (or fourth pin number duplicated if only four pins).
    ///
    /// Note if not all pins are used, subsequent pins after the duplicated one are "unused"
    const fn encode_pins(
        initial_value: u32,
        values: &[u32],
        position: u32,
        last_value: u32,
    ) -> Option<u32> {
        match values {
            [] => {
                // We have to use checked_shl and if let pattern
                // as const rust doesn't support the `<<` operator nor the `expect` or `?`
                // operations.
                if let Some(value) = last_value.checked_shl(5) {
                    Some(initial_value | value)
                } else {
                    None
                }
            }
            [first, rest @ ..] => {
                // For more than one value, recursively calls itself.
                // We have to recurse here because in const rust `iter` is unstable.
                if let Some(value) = first.checked_shl(7 + (5 * position)) {
                    Self::encode_pins(initial_value | value, rest, position + 1, value)
                } else {
                    None
                }
            }
        }
    }
}

unsafe impl Sync for PinsWithFunction {}

/// Shifts a value left or panic if an error occurs
///
/// * `value` Value to shift left
/// * `shift_by` Amount to shift the value by
/// * `panic_message` Message to display in case of panic
///
/// This function is necessary because `value << shift_by` is unstable in
/// const rust.
const fn shl_or_panic(value: u32, shift_by: u32, panic_message: &str) -> u32 {
    if let Some(shifted) = value.checked_shl(shift_by) {
        shifted
    } else {
        panic!("{}", panic_message);
    }
}

#[cfg(test)]
mod test {

    use super::*;

    #[test]
    fn pins_with_function_generate_proper_outputs() {
        let pwf = PinsWithFunction::new([2].as_slice(), crate::PinFunction::Uart);
        assert_eq!(pwf.encoding, 2 | 2 << 3 | 2 << 7 | 2 << 12);

        let pwf = PinsWithFunction::new([2, 3].as_slice(), crate::PinFunction::Uart);
        assert_eq!(pwf.encoding, 2 | 2 << 3 | 2 << 7 | 3 << 12 | 3 << 17);

        let pwf = PinsWithFunction::new([2, 3, 4, 5, 6].as_slice(), crate::PinFunction::Uart);
        assert_eq!(
            pwf.encoding,
            2 | 2 << 3 | 2 << 7 | 3 << 12 | 4 << 17 | 5 << 22 | 6 << 27
        );
    }

    #[test]
    #[should_panic]
    fn pin_with_function_should_fail_when_more_than_5_pins() {
        PinsWithFunction::new([2, 3, 4, 5, 6, 8].as_slice(), crate::PinFunction::Uart);
    }

    #[test]
    fn pin_range_with_function_generate_proper_outputs() {
        let pwf = PinsWithFunction::new_range(0, 4, crate::PinFunction::Uart);
        assert_eq!(pwf.encoding, 1 | 2 << 3 | 4 << 12);
    }

    #[test]
    fn pins_with_name_fold_returns_the_corresponding_mask() {
        assert_eq!(PinsWithName::fold_pins_list_to_mask(0, &[1, 2, 3]), 0x0E);
    }

    #[test]
    #[should_panic]
    fn pins_with_name_fold_panics_on_unordered_list() {
        PinsWithName::fold_pins_list_to_mask(0, [3, 1].as_slice());
    }
}

// End of file
