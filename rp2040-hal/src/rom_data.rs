//! Functions and data from the RPI Bootrom.
//!
//! From the [RP2040 datasheet](https://datasheets.raspberrypi.org/rp2040/rp2040-datasheet.pdf), Section 2.8.2.1:
//!
//! > The Bootrom contains a number of public functions that provide useful
//! > RP2040 functionality that might be needed in the absence of any other code
//! > on the device, as well as highly optimized versions of certain key
//! > functionality that would otherwise have to take up space in most user
//! > binaries.

/// A bootrom function table code.
pub type RomFnTableCode = [u8; 2];

/// This function searches for (table)
type RomTableLookupFn<T> = unsafe extern "C" fn(*const u16, u32) -> T;

/// The following addresses are described at `2.8.2. Bootrom Contents`
/// Pointer to the lookup table function supplied by the rom.
const ROM_TABLE_LOOKUP_PTR: *const u16 = 0x0000_0018 as _;

/// Pointer to helper functions lookup table.
const FUNC_TABLE: *const u16 = 0x0000_0014 as _;

/// Pointer to the public data lookup table.
const DATA_TABLE: *const u16 = 0x0000_0016 as _;

/// Retrive rom content from a table using a code.
fn rom_table_lookup<T>(table: *const u16, tag: RomFnTableCode) -> T {
    unsafe {
        let rom_table_lookup_ptr: *const u32 = rom_hword_as_ptr(ROM_TABLE_LOOKUP_PTR);
        let rom_table_lookup: RomTableLookupFn<T> = core::mem::transmute(rom_table_lookup_ptr);
        rom_table_lookup(
            rom_hword_as_ptr(table) as *const u16,
            u16::from_le_bytes(tag) as u32,
        )
    }
}

/// To save space, the ROM likes to store memory pointers (which are 32-bit on
/// the Cortex-M0+) using only the bottom 16-bits. The assumption is that the
/// values they point at live in the first 64 KiB of ROM, and the ROM is mapped
/// to address `0x0000_0000` and so 16-bits are always sufficient.
///
/// This functions grabs a 16-bit value from ROM and expands it out to a full 32-bit pointer.
unsafe fn rom_hword_as_ptr(rom_address: *const u16) -> *const u32 {
    let ptr: u16 = *rom_address;
    ptr as *const u32
}

macro_rules! rom_funcs {
    (
        $(
            $(#[$outer:meta])*
            $c:literal $name:ident (
                $( $aname:ident : $aty:ty ),*
            ) -> $ret:ty ;
        )*
    ) => {
        $(
            $(#[$outer])*
            pub fn $name($( $aname:$aty ),*) -> $ret{
                let func:  extern "C" fn( $( $aty ),* ) -> $ret = rom_table_lookup(FUNC_TABLE, *$c);
                func($( $aname ),*)
            }
        )*
    }
}

macro_rules! rom_funcs_unsafe {
    (
        $(
            $(#[$outer:meta])*
            $c:literal $name:ident (
                $( $aname:ident : $aty:ty ),*
            ) -> $ret:ty ;
        )*
    ) => {
        $(
            $(#[$outer])*
            pub unsafe fn $name($( $aname:$aty ),*) -> $ret{
                let func:  extern "C" fn( $( $aty ),* ) -> $ret = rom_table_lookup(FUNC_TABLE, *$c);
                func($( $aname ),*)
            }
        )*
    }
}

rom_funcs! {
    /// Return a count of the number of 1 bits in value.
    b"P3" popcount32(value: u32) -> u32;

    /// Return the bits of value in the reverse order.
    b"R3" reverse32(value: u32) -> u32;

    /// Return the number of consecutive high order 0 bits of value. If value is zero, returns 32.
    b"L3" clz32(value: u32) -> u32;

    /// Return the number of consecutive low order 0 bits of value. If value is zero, returns 32.
    b"T3" ctz32(value: u32) -> u32;

    /// Resets the RP2040 and uses the watchdog facility to re-start in BOOTSEL mode:
    ///   * gpio_activity_pin_mask is provided to enable an 'activity light' via GPIO attached LED
    ///     for the USB Mass Storage Device:
    ///     * 0 No pins are used as per cold boot.
    ///     * Otherwise a single bit set indicating which GPIO pin should be set to output and
    ///       raised whenever there is mass storage activity from the host.
    ///  * disable_interface_mask may be used to control the exposed USB interfaces:
    ///    * 0 To enable both interfaces (as per cold boot).
    ///    * 1 To disable the USB Mass Storage Interface.
    ///    * 2 to Disable the USB PICOBOOT Interface.
    b"UB" reset_to_usb_boot(gpio_activity_pin_mask: u32, disable_interface_mask: u32) -> ();
}

rom_funcs_unsafe! {
    /// Sets n bytes start at ptr to the value c and returns ptr
    b"MS" memset(ptr: *mut u8, c: u8, n: u8) -> *mut u8;

    /// Sets n bytes start at ptr to the value c and returns ptr.
    ///
    /// Note this is a slightly more efficient variant of _memset that may only
    /// be used if ptr is word aligned.
    b"M4" memset4(ptr: *mut u32, c: u8, n: u32) -> *mut u32;

    /// Copies n bytes starting at src to dest and returns dest. The results are undefined if the
    /// regions overlap.
    b"MC" memcpy(dest: *mut u8, src: *mut u8, n: u32) -> u8;

    /// Copies n bytes starting at src to dest and returns dest. The results are undefined if the
    /// regions overlap.
    ///
    /// Note this is a slightly more efficient variant of _memcpy that may only be
    /// used if dest and src are word aligned.
    b"C4" memcpy44(dest: *mut u32, src: *mut u32, n: u32) -> *mut u8;

    /// Restore all QSPI pad controls to their default state, and connect the SSI to the QSPI pads.
    b"IF" connect_internal_flash() -> ();

    /// First set up the SSI for serial-mode operations, then issue the fixed XIP exit sequence.
    ///
    /// Note that the bootrom code uses the IO forcing logic to drive the CS pin, which must be
    /// cleared before returning the SSI to XIP mode (e.g. by a call to _flash_flush_cache). This
    /// function configures the SSI with a fixed SCK clock divisor of /6.
    b"EX" flash_exit_xip() -> ();

    /// Erase a count bytes, starting at addr (offset from start of flash). Optionally, pass a
    /// block erase command e.g. D8h block erase, and the size of the block erased by this
    /// command — this function will use the larger block erase where possible, for much higher
    /// erase speed. addr must be aligned to a 4096-byte sector, and count must be a multiple of
    /// 4096 bytes.
    b"RE" flash_range_erase(addr: u32, count: usize, block_size: u32, block_cmd: u8) -> ();

    /// Program data to a range of flash addresses starting at `addr` (and
    /// offset from the start of flash) and `count` bytes in size. The value
    /// `addr` must be aligned to a 256-byte boundary, and `count` must be a
    /// multiple of 256.
    b"RP" flash_range_program(addr: u32, data: *const u8, count: usize) -> ();

    /// Flush and enable the XIP cache. Also clears the IO forcing on QSPI CSn, so that the SSI can
    /// drive the flashchip select as normal.
    b"FC" flash_flush_cache() -> ();

    /// Configure the SSI to generate a standard 03h serial read command, with 24 address bits,
    /// upon each XIP access. This is a very slow XIP configuration, but is very widely supported.
    /// The debugger calls this function after performing a flash erase/programming operation, so
    /// that the freshly-programmed code and data is visible to the debug host, without having to
    /// know exactly what kind of flash device is connected.
    b"CX" flash_enter_cmd_xip() -> ();

    /// This is the method that is entered by core 1 on reset to wait to be launched by core 0.
    /// There are few cases where you should call this method (resetting core 1 is much better).
    /// This method does not return and should only ever be called on core 1.
    b"WV" wait_for_vector() -> !;
}

unsafe fn convert_str(s: *const u8) -> &'static str {
    let mut end = s;
    while *end != 0 {
        end = end.add(1);
    }
    let s = core::slice::from_raw_parts(s, end.offset_from(s) as usize);
    core::str::from_utf8_unchecked(s)
}

/// The Raspberry Pi Trading Ltd copyright string.
pub fn copyright_string() -> &'static str {
    let s: *const u8 = rom_table_lookup(DATA_TABLE, *b"CR");
    unsafe { convert_str(s) }
}

/// The 8 most significant hex digits of the Bootrom git revision.
pub fn git_revision() -> u32 {
    let s: *const u32 = rom_table_lookup(DATA_TABLE, *b"GR");
    unsafe { *s }
}

/// The start address of the floating point library code and data.
///
/// This and fplib_end along with the individual function pointers in
/// soft_float_table can be used to copy the floating point implementation into
/// RAM if desired.
pub fn fplib_start() -> *const u8 {
    rom_table_lookup(DATA_TABLE, *b"FS")
}

/// See Table 180 in the RP2040 datasheet for the contents of this table.
pub fn soft_float_table() -> *const usize {
    rom_table_lookup(DATA_TABLE, *b"SF")
}

/// The end address of the floating point library code and data.
pub fn fplib_end() -> *const u8 {
    rom_table_lookup(DATA_TABLE, *b"FE")
}

/// This entry is only present in the V2 bootrom. See Table 182 in the RP2040 datasheet for the contents of this table.
pub fn soft_double_table() -> *const usize {
    rom_table_lookup(DATA_TABLE, *b"SD")
}

/// ROM functions using single-precision arithmetic (i.e. 'f32' in Rust terms)
pub mod float_funcs {

    macro_rules! make_functions {
        (
            $(
                $(#[$outer:meta])*
                $offset:literal $name:ident (
                    $( $aname:ident : $aty:ty ),*
                ) -> $ret:ty;
            )*
        ) => {
            $(
                $(#[$outer])*
                pub fn $name() -> extern "C" fn( $( $aname : $aty ),* ) -> $ret {
                    let table: *const usize = $crate::rom_data::soft_float_table() as *const usize;
                    unsafe {
                        // This is the entry in the table. Our offset is given as a
                        // byte offset, but we want the table index (each pointer in
                        // the table is 4 bytes long)
                        let entry: *const usize = table.offset($offset / 4);
                        // Read the pointer from the table
                        let ptr: usize = core::ptr::read(entry);
                        // Convert the pointer we read into a function
                        core::mem::transmute_copy(&ptr)
                    }
                }
            )*
        }
    }

    make_functions! {
        /// Returns a function that will calculate `a + b`
        0x00 fadd(a: f32, b: f32) -> f32;
        /// Returns a function that will calculate `a - b`
        0x04 fsub(a: f32, b: f32) -> f32;
        /// Returns a function that will calculate `a * b`
        0x08 fmul(a: f32, b: f32) -> f32;
        /// Returns a function that will calculate `a / b`
        0x0c fdiv(a: f32, b: f32) -> f32;

        // 0x10 and 0x14 are deprecated

        /// Returns a function that will calculate `sqrt(v)` (or return -Infinity if v is negative)
        0x18 fsqrt(v: f32) -> f32;
        /// Returns a function that will convert an f32 to a signed integer,
        /// rounding towards -Infinity, and clamping the result to lie within the
        /// range `-0x80000000` to `0x7FFFFFFF`
        0x1c float_to_int(v: f32) -> i32;
        /// Returns a function that will convert an f32 to an signed fixed point
        /// integer representation where n specifies the position of the binary
        /// point in the resulting fixed point representation, e.g.
        /// `f(0.5f, 16) == 0x8000`. This method rounds towards -Infinity,
        /// and clamps the resulting integer to lie within the range `0x00000000` to
        /// `0xFFFFFFFF`
        0x20 float_to_fix(v: f32, n: i32) -> i32;
        /// Returns a function that will convert an f32 to an unsigned integer,
        /// rounding towards -Infinity, and clamping the result to lie within the
        /// range `0x00000000` to `0xFFFFFFFF`
        0x24 float_to_uint(v: f32) -> u32;
        /// Returns a function that will convert an f32 to an unsigned fixed point
        /// integer representation where n specifies the position of the binary
        /// point in the resulting fixed point representation, e.g.
        /// `f(0.5f, 16) == 0x8000`. This method rounds towards -Infinity,
        /// and clamps the resulting integer to lie within the range `0x00000000` to
        /// `0xFFFFFFFF`
        0x28 float_to_ufix(v: f32, n: i32) -> u32;
        /// Returns a function that will convert a signed integer to the nearest
        /// f32 value, rounding to even on tie
        0x2c int_to_float(v: i32) -> f32;
        /// Returns a function that will convert a signed fixed point integer
        /// representation to the nearest f32 value, rounding to even on tie. `n`
        /// specifies the position of the binary point in fixed point, so `f =
        /// nearest(v/(2^n))`
        0x30 fix_to_float(v: i32, n: i32) -> f32;
        /// Returns a function that will convert an unsigned integer to the nearest
        /// f32 value, rounding to even on tie
        0x34 uint_to_float(v: u32) -> f32;
        /// Returns a function that will convert an unsigned fixed point integer
        /// representation to the nearest f32 value, rounding to even on tie. `n`
        /// specifies the position of the binary point in fixed point, so `f =
        /// nearest(v/(2^n))`
        0x38 ufix_to_float(v: u32, n: i32) -> f32;
        /// Returns a function that will calculate the cosine of `angle`. The value
        /// of `angle` is in radians, and must be in the range `-1024` to `1024`
        0x3c fcos(angle: f32) -> f32;
        /// Returns a function that will calculate the sine of `angle`. The value of
        /// `angle` is in radians, and must be in the range `-1024` to `1024`
        0x40 fsin(angle: f32) -> f32;
        /// Returns a function that will calculate the tangent of `angle`. The value
        /// of `angle` is in radians, and must be in the range `-1024` to `1024`
        0x44 ftan(angle: f32) -> f32;

        // 0x48 is deprecated

        /// Returns a function that will calculate the exponential value of `v`,
        /// i.e. `e ** v`
        0x4c fexp(v: f32) -> f32;
        /// Returns a function that will calculate the natural logarithm of `v`. If `v <= 0` return -Infinity
        0x50 fln(v: f32) -> f32;

        // These are only on BootROM v2 or higher

        /// Returns a function that will compare two floating point numbers, returning:
        ///     • 0 if a == b
        ///     • -1 if a < b
        ///     • 1 if a > b
        0x54 fcmp(a: f32, b: f32) -> i32;
        /// Returns a function that will compute the arc tangent of `y/x` using the
        /// signs of arguments to determine the correct quadrant
        0x58 fatan2(y: f32, x: f32) -> f32;
        /// Returns a function that will convert a signed 64-bit integer to the
        /// nearest f32 value, rounding to even on tie
        0x5c int64_to_float(v: i64) -> f32;
        /// Returns a function that will convert a signed fixed point 64-bit integer
        /// representation to the nearest f32 value, rounding to even on tie. `n`
        /// specifies the position of the binary point in fixed point, so `f =
        /// nearest(v/(2^n))`
        0x60 fix64_to_float(v: i64, n: i32) -> f32;
        /// Returns a function that will convert an unsigned 64-bit integer to the
        /// nearest f32 value, rounding to even on tie
        0x64 uint64_to_float(v: u64) -> f32;
        /// Returns a function that will convert an unsigned fixed point 64-bit
        /// integer representation to the nearest f32 value, rounding to even on
        /// tie. `n` specifies the position of the binary point in fixed point, so
        /// `f = nearest(v/(2^n))`
        0x68 ufix64_to_float(v: u64, n: i32) -> f32;
        /// Convert an f32 to a signed 64-bit integer, rounding towards -Infinity,
        /// and clamping the result to lie within the range `-0x8000000000000000` to
        /// `0x7FFFFFFFFFFFFFFF`
        0x6c float_to_int64(v: f32) -> i64;
        /// Returns a function that will convert an f32 to a signed fixed point
        /// 64-bit integer representation where n specifies the position of the
        /// binary point in the resulting fixed point representation - e.g. `f(0.5f,
        /// 16) == 0x8000`. This method rounds towards -Infinity, and clamps the
        /// resulting integer to lie within the range `-0x8000000000000000` to
        /// `0x7FFFFFFFFFFFFFFF`
        0x70 float_to_fix64(v: f32, n: i32) -> f32;
        /// Returns a function that will convert an f32 to an unsigned 64-bit
        /// integer, rounding towards -Infinity, and clamping the result to lie
        /// within the range `0x0000000000000000` to `0xFFFFFFFFFFFFFFFF`
        0x74 float_to_uint64(v: f32) -> u64;
        /// Returns a function that will convert an f32 to an unsigned fixed point
        /// 64-bit integer representation where n specifies the position of the
        /// binary point in the resulting fixed point representation, e.g. `f(0.5f,
        /// 16) == 0x8000`. This method rounds towards -Infinity, and clamps the
        /// resulting integer to lie within the range `0x0000000000000000` to
        /// `0xFFFFFFFFFFFFFFFF`
        0x78 float_to_ufix64(v: f32, n: i32) -> u64;
        /// Converts an f32 to an f64.
        0x7c float_to_double(v: f32) -> f64;
    }
}

/// Functions using double-precision arithmetic (i.e. 'f64' in Rust terms)
pub mod double_funcs {

    macro_rules! make_double_funcs {
        (
            $(
                $(#[$outer:meta])*
                $offset:literal $name:ident (
                    $( $aname:ident : $aty:ty ),*
                ) -> $ret:ty;
            )*
        ) => {
            $(
                $(#[$outer])*
                pub fn $name() -> extern "C" fn( $( $aname : $aty ),* ) -> $ret {
                    let table: *const usize = $crate::rom_data::soft_double_table() as *const usize;
                    unsafe {
                        // This is the entry in the table. Our offset is given as a
                        // byte offset, but we want the table index (each pointer in
                        // the table is 4 bytes long)
                        let entry: *const usize = table.offset($offset / 4);
                        // Read the pointer from the table
                        let ptr: usize = core::ptr::read(entry);
                        // Convert the pointer we read into a function
                        core::mem::transmute_copy(&ptr)
                    }
                }
            )*
        }
    }

    make_double_funcs! {
        /// Returns a function that will calculate `a + b`
        0x00 dadd(a: f64, b: f64) -> f64;
        /// Returns a function that will calculate `a - b`
        0x04 dsub(a: f64, b: f64) -> f64;
        /// Returns a function that will calculate `a * b`
        0x08 dmul(a: f64, b: f64) -> f64;
        /// Returns a function that will calculate `a / b`
        0x0c ddiv(a: f64, b: f64) -> f64;

        // 0x10 and 0x14 are deprecated

        /// Returns a function that will calculate `sqrt(v)` (or return -Infinity if v is negative)
        0x18 dsqrt(v: f64) -> f64;
        /// Returns a function that will convert an f64 to a signed integer,
        /// rounding towards -Infinity, and clamping the result to lie within the
        /// range `-0x80000000` to `0x7FFFFFFF`
        0x1c double_to_int(v: f64) -> i32;
        /// Returns a function that will convert an f64 to an signed fixed point
        /// integer representation where n specifies the position of the binary
        /// point in the resulting fixed point representation, e.g.
        /// `f(0.5f, 16) == 0x8000`. This method rounds towards -Infinity,
        /// and clamps the resulting integer to lie within the range `0x00000000` to
        /// `0xFFFFFFFF`
        0x20 double_to_fix(v: f64, n: i32) -> i32;
        /// Returns a function that will convert an f64 to an unsigned integer,
        /// rounding towards -Infinity, and clamping the result to lie within the
        /// range `0x00000000` to `0xFFFFFFFF`
        0x24 double_to_uint(v: f64) -> u32;
        /// Returns a function that will convert an f64 to an unsigned fixed point
        /// integer representation where n specifies the position of the binary
        /// point in the resulting fixed point representation, e.g.
        /// `f(0.5f, 16) == 0x8000`. This method rounds towards -Infinity,
        /// and clamps the resulting integer to lie within the range `0x00000000` to
        /// `0xFFFFFFFF`
        0x28 double_to_ufix(v: f64, n: i32) -> u32;
        /// Returns a function that will convert a signed integer to the nearest
        /// double value, rounding to even on tie
        0x2c int_to_double(v: i32) -> f64;
        /// Returns a function that will convert a signed fixed point integer
        /// representation to the nearest double value, rounding to even on tie. `n`
        /// specifies the position of the binary point in fixed point, so `f =
        /// nearest(v/(2^n))`
        0x30 fix_to_double(v: i32, n: i32) -> f64;
        /// Returns a function that will convert an unsigned integer to the nearest
        /// double value, rounding to even on tie
        0x34 uint_to_double(v: u32) -> f64;
        /// Returns a function that will convert an unsigned fixed point integer
        /// representation to the nearest double value, rounding to even on tie. `n`
        /// specifies the position of the binary point in fixed point, so f =
        /// nearest(v/(2^n))
        0x38 ufix_to_double(v: u32, n: i32) -> f64;
        /// Returns a function that will calculate the cosine of `angle`. The value
        /// of `angle` is in radians, and must be in the range `-1024` to `1024`
        0x3c dcos(angle: f64) -> f64;
        /// Returns a function that will calculate the sine of `angle`. The value of
        /// `angle` is in radians, and must be in the range `-1024` to `1024`
        0x40 dsin(angle: f64) -> f64;
        /// Returns a function that will calculate the tangent of `angle`. The value
        /// of `angle` is in radians, and must be in the range `-1024` to `1024`
        0x44 dtan(angle: f64) -> f64;

        // 0x48 is deprecated

        /// Returns a function that will calculate the exponential value of `v`,
        /// i.e. `e ** v`
        0x4c dexp(v: f64) -> f64;
        /// Returns a function that will calculate the natural logarithm of v. If v <= 0 return -Infinity
        0x50 dln(v: f64) -> f64;

        // These are only on BootROM v2 or higher

        /// Returns a function that will compare two floating point numbers, returning:
        ///     • 0 if a == b
        ///     • -1 if a < b
        ///     • 1 if a > b
        0x54 dcmp(a: f64, b: f64) -> i32;
        /// Returns a function that will compute the arc tangent of `y/x` using the
        /// signs of arguments to determine the correct quadrant
        0x58 datan2(y: f64, x: f64) -> f64;
        /// Returns a function that will convert a signed 64-bit integer to the
        /// nearest double value, rounding to even on tie
        0x5c int64_to_double(v: i64) -> f64;
        /// Returns a function that will convert a signed fixed point 64-bit integer
        /// representation to the nearest double value, rounding to even on tie. `n`
        /// specifies the position of the binary point in fixed point, so `f =
        /// nearest(v/(2^n))`
        0x60 fix64_to_doubl(v: i64, n: i32) -> f64;
        /// Returns a function that will convert an unsigned 64-bit integer to the
        /// nearest double value, rounding to even on tie
        0x64 uint64_to_double(v: u64) -> f64;
        /// Returns a function that will convert an unsigned fixed point 64-bit
        /// integer representation to the nearest double value, rounding to even on
        /// tie. `n` specifies the position of the binary point in fixed point, so
        /// `f = nearest(v/(2^n))`
        0x68 ufix64_to_double(v: u64, n: i32) -> f64;
        /// Convert an f64 to a signed 64-bit integer, rounding towards -Infinity,
        /// and clamping the result to lie within the range `-0x8000000000000000` to
        /// `0x7FFFFFFFFFFFFFFF`
        0x6c double_to_int64(v: f64) -> i64;
        /// Returns a function that will convert an f64 to a signed fixed point
        /// 64-bit integer representation where n specifies the position of the
        /// binary point in the resulting fixed point representation - e.g. `f(0.5f,
        /// 16) == 0x8000`. This method rounds towards -Infinity, and clamps the
        /// resulting integer to lie within the range `-0x8000000000000000` to
        /// `0x7FFFFFFFFFFFFFFF`
        0x70 double_to_fix64(v: f64, n: i32) -> i64;
        /// Returns a function that will convert an f64 to an unsigned 64-bit
        /// integer, rounding towards -Infinity, and clamping the result to lie
        /// within the range `0x0000000000000000` to `0xFFFFFFFFFFFFFFFF`
        0x74 double_to_uint64(v: f64) -> u64;
        /// Returns a function that will convert an f64 to an unsigned fixed point
        /// 64-bit integer representation where n specifies the position of the
        /// binary point in the resulting fixed point representation, e.g. `f(0.5f,
        /// 16) == 0x8000`. This method rounds towards -Infinity, and clamps the
        /// resulting integer to lie within the range `0x0000000000000000` to
        /// `0xFFFFFFFFFFFFFFFF`
        0x78 double_to_ufix64(v: f64, n: i32) -> u64;
        /// Returns a function that will convert an f64 to an f32
        0x7c double_to_float(v: f64) -> f32;
    }
}
