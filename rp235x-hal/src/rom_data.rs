//! Functions and data from the RPI Bootrom.
//!
//! From [Section 5.4](https://rptl.io/rp2350-datasheet#section_bootrom) of the
//! RP2350 datasheet:
//!
//! > Whilst some ROM space is dedicated to the implementation of the boot
//! > sequence and USB/UART boot interfaces, the bootrom also contains public
//! > functions that provide useful RP2350 functionality that may be useful for
//! > any code or runtime running on the device

/// A bootrom function table code.
pub type RomFnTableCode = [u8; 2];

/// This function searches for the tag which matches the mask.
type RomTableLookupFn = unsafe extern "C" fn(code: u32, mask: u32) -> usize;

/// Pointer to the value lookup function supplied by the ROM.
///
/// This address is described at `5.5.1. Locating the API Functions`
#[cfg(all(target_arch = "arm", target_os = "none"))]
const ROM_TABLE_LOOKUP_A2: *const u16 = 0x0000_0016 as _;

/// Pointer to the value lookup function supplied by the ROM.
///
/// This address is described at `5.5.1. Locating the API Functions`
#[cfg(all(target_arch = "arm", target_os = "none"))]
const ROM_TABLE_LOOKUP_A1: *const u32 = 0x0000_0018 as _;

/// Pointer to the data lookup function supplied by the ROM.
///
/// On Arm, the same function is used to look up code and data.
#[cfg(all(target_arch = "arm", target_os = "none"))]
const ROM_DATA_LOOKUP_A2: *const u16 = ROM_TABLE_LOOKUP_A2;

/// Pointer to the data lookup function supplied by the ROM.
///
/// On Arm, the same function is used to look up code and data.
#[cfg(all(target_arch = "arm", target_os = "none"))]
const ROM_DATA_LOOKUP_A1: *const u32 = ROM_TABLE_LOOKUP_A1;

/// Pointer to the value lookup function supplied by the ROM.
///
/// This address is described at `5.5.1. Locating the API Functions`
#[cfg(not(all(target_arch = "arm", target_os = "none")))]
const ROM_TABLE_LOOKUP_A2: *const u16 = 0x0000_7DFA as _;

/// Pointer to the value lookup function supplied by the ROM.
///
/// This address is described at `5.5.1. Locating the API Functions`
#[cfg(not(all(target_arch = "arm", target_os = "none")))]
const ROM_TABLE_LOOKUP_A1: *const u32 = 0x0000_7DF8 as _;

/// Pointer to the data lookup function supplied by the ROM.
///
/// On RISC-V, a different function is used to look up data.
#[cfg(not(all(target_arch = "arm", target_os = "none")))]
const ROM_DATA_LOOKUP_A2: *const u16 = 0x0000_7DF8 as _;

/// Pointer to the data lookup function supplied by the ROM.
///
/// On RISC-V, a different function is used to look up data.
#[cfg(not(all(target_arch = "arm", target_os = "none")))]
const ROM_DATA_LOOKUP_A1: *const u32 = 0x0000_7DF4 as _;

/// Address of the version number of the ROM.
const VERSION_NUMBER: *const u8 = 0x0000_0013 as _;

#[allow(unused)]
mod rt_flags {
    pub const FUNC_RISCV: u32 = 0x0001;
    pub const FUNC_RISCV_FAR: u32 = 0x0003;
    pub const FUNC_ARM_SEC: u32 = 0x0004;
    // reserved for 32-bit pointer: 0x0008
    pub const FUNC_ARM_NONSEC: u32 = 0x0010;
    // reserved for 32-bit pointer: 0x0020
    pub const DATA: u32 = 0x0040;
    // reserved for 32-bit pointer: 0x0080
    #[cfg(all(target_arch = "arm", target_os = "none"))]
    pub const FUNC_ARM_SEC_RISCV: u32 = FUNC_ARM_SEC;
    #[cfg(not(all(target_arch = "arm", target_os = "none")))]
    pub const FUNC_ARM_SEC_RISCV: u32 = FUNC_RISCV;
}

/// Retrieve rom content from a table using a code.
pub fn rom_table_lookup(tag: RomFnTableCode, mask: u32) -> usize {
    let tag = u16::from_le_bytes(tag) as u32;
    unsafe {
        let lookup_func = if rom_version_number() == 1 {
            ROM_TABLE_LOOKUP_A1.read() as usize
        } else {
            ROM_TABLE_LOOKUP_A2.read() as usize
        };
        let lookup_func: RomTableLookupFn = core::mem::transmute(lookup_func);
        lookup_func(tag, mask)
    }
}

/// Retrieve rom data content from a table using a code.
pub fn rom_data_lookup(tag: RomFnTableCode, mask: u32) -> usize {
    let tag = u16::from_le_bytes(tag) as u32;
    unsafe {
        let lookup_func = if rom_version_number() == 1 {
            ROM_DATA_LOOKUP_A1.read() as usize
        } else {
            ROM_DATA_LOOKUP_A2.read() as usize
        };
        let lookup_func: RomTableLookupFn = core::mem::transmute(lookup_func);
        lookup_func(tag, mask)
    }
}

/// bootrom API function return codes as defined by section 5.4.3 in the rp2350 data sheet
/// See: https://datasheets.raspberrypi.com/rp2350/rp2350-datasheet.pdf
#[repr(i32)]
#[derive(Debug)]
pub enum BootRomApiErrorCode {
    /// The operation was disallowed by a security constraint
    NotPermitted = -4,
    /// One or more parameters passed to the function is outside the range of
    /// supported values; [`BootRomApiErrorCode::InvalidAddress`] and
    /// [`BootRomApiErrorCode::BadAlignment`] are more specific errors.
    InvalidArg = -5,
    /// An address argument was out-of-bounds or was determined to be an address
    /// that the caller may not access
    InvalidAddress = -10,
    /// An address passed to the function was not correctly aligned
    BadAlignment = -11,
    /// Something happened or failed to happen in the past, and consequently the
    /// request cannot currently be serviced.
    InvalidState = -12,
    /// A user-allocated buffer was too small to hold the result or working state
    /// of the function
    BufferTooSmall = -13,
    /// The call failed because another bootrom function must be called first.
    PreconditionNotMet = -14,
    /// Cached data was determined to be inconsistent with the full version of
    /// the data it was copied from
    ModifiedData = -15,
    /// The contents of a data structure are invalid
    InvalidData = -16,
    /// An attempt was made to access something that does not exist; or, a search failed
    NotFound = -17,
    /// Modification is impossible based on current state; e.g. attempted to clear
    /// an OTP bit.
    UnsupportedModification = -18,
    /// A required lock is not owned. See Section 5.4.4.
    LockRequired = -19,
    /// An unknown error
    Unknown = -1,
}

impl From<i32> for BootRomApiErrorCode {
    fn from(value: i32) -> Self {
        match value {
            -4 => Self::NotPermitted,
            -5 => Self::InvalidArg,
            -10 => Self::InvalidAddress,
            -11 => Self::BadAlignment,
            -12 => Self::InvalidState,
            -13 => Self::BufferTooSmall,
            -14 => Self::PreconditionNotMet,
            -15 => Self::ModifiedData,
            -16 => Self::InvalidData,
            -17 => Self::NotFound,
            -18 => Self::UnsupportedModification,
            -19 => Self::LockRequired,
            _ => Self::Unknown,
        }
    }
}

/// This module defines a safe api to access the `get_sys_info` bootrom function
#[allow(unused)]
pub mod sys_info_api {
    use super::BootRomApiErrorCode;

    /// Flags that the `get_sys_info`/ rom function can take
    #[repr(u32)]
    pub enum GetSysInfoFlag {
        /// The flag used to get a chip's unique identifier
        ChipInfo = 0x0001,
        /// The flag used to get the critical register's value
        Critical = 0x0002,
        /// The flag used to get the current running CPU Architecture
        CpuInfo = 0x0004,
        /// The flag used to get flash device info
        FlashDevInfo = 0x0008,
        /// The flag used to get the random 128 bit integer generated on boot
        BootRandom = 0x0010,
        // Ignore nonce for now since it can't/shouldn't be called anyway?
        // Nonce = 0x0020,
        /// The flag used to get boot diagnostic info
        BootInfo = 0x0040,
    }

    impl GetSysInfoFlag {
        /// Returns the length of the buffer needed to hold the data for the related operation returned
        /// by [`super::get_sys_info()`]. This includes the initial segment to indicate which flags
        /// were supported. The underlying enum represent a bitmask and these masks can be OR'd
        /// together, however the safe API only uses one at a time so adding sizes is not a concern.
        const fn buffer_length(&self) -> usize {
            match self {
                GetSysInfoFlag::ChipInfo => 4,
                GetSysInfoFlag::Critical
                | GetSysInfoFlag::CpuInfo
                | GetSysInfoFlag::FlashDevInfo => 2,
                GetSysInfoFlag::BootRandom | GetSysInfoFlag::BootInfo => 5,
            }
        }
    }

    /// The unqiue identifier for each chip as reported by [`chip_info`]
    pub struct ChipInfo {
        /// The value of the `CHIP_INFO_PACKAGE_SEL` register
        pub package_sel: u32,
        /// The device's id
        pub device_id: u32,
        /// The wafer's id
        pub wafer_id: u32,
    }

    impl From<[u32; 3]> for ChipInfo {
        fn from(value: [u32; 3]) -> Self {
            ChipInfo {
                package_sel: value[0],
                device_id: value[1],
                wafer_id: value[2],
            }
        }
    }

    /// The value held within the critical register as reported by [`otp_critical_register`]
    pub struct OtpCriticalReg(u32);

    impl OtpCriticalReg {
        /// Check if secure boot is enabled
        pub fn secure_boot_enabled(&self) -> bool {
            (self.0 & 0x1) == 1
        }

        /// Check if secure debug is disabled
        pub fn secure_debug_disabled(&self) -> bool {
            (self.0 & 0x2) >> 1 == 1
        }

        /// Check if debug is disabled
        pub fn debug_disabled(&self) -> bool {
            (self.0 & 0x4) >> 2 == 1
        }

        /// Check the value of `DEFAULT_ARCHSEL`
        pub fn default_arch_sel(&self) -> bool {
            (self.0 & 0x8) >> 3 == 1
        }

        /// Check if the glitch detector is enabled
        pub fn glitch_detector_enabled(&self) -> bool {
            (self.0 & 0x10) >> 4 == 1
        }

        /// Value of `GLITCH_DETECTOR_SENS
        pub fn glitch_detector_sens(&self) -> u8 {
            ((self.0 & 0x60) >> 5) as _
        }

        /// Check if ARM is disabled
        pub fn arm_disabled(&self) -> bool {
            (self.0 & 0x10000) >> 16 == 1
        }

        /// Check if Risc-V is disabled
        pub fn risc_disabled(&self) -> bool {
            (self.0 & 0x20000) >> 17 == 1
        }
    }

    impl From<[u32; 1]> for OtpCriticalReg {
        fn from(value: [u32; 1]) -> OtpCriticalReg {
            OtpCriticalReg(value[0])
        }
    }

    #[repr(u32)]
    /// CPU architectures that might be running as reported by [`cpu_info`]
    pub enum CpuInfo {
        /// Arm CPU
        Arm,
        /// Risc-V CPU
        Risc,
    }

    impl From<[u32; 1]> for CpuInfo {
        fn from(value: [u32; 1]) -> CpuInfo {
            if value[0] == 0 {
                CpuInfo::Arm
            } else {
                CpuInfo::Risc
            }
        }
    }

    /// Flash device information as reported by [`flash_dev_info`]
    pub struct FlashDevInfo(u32);

    /// A struct to represent possible byte sizes that may be reported in [`FlashDevInfo`]
    #[repr(u32)]
    pub enum FlashDevInfoSize {
        /// 0 bytes
        None,
        /// 8 KiB
        K8,
        /// 16 KiB
        K16,
        /// 32 KiB
        K32,
        /// 64 KiB
        K64,
        /// 128 KiB
        K128,
        /// 256 KiB
        K256,
        /// 512 KiB
        K512,
        /// 1 MiB
        M1,
        /// 2 MiB
        M2,
        /// 4 Mib
        M4,
        /// 8 MiB
        M8,
        /// 16 MiB
        M16,
        /// Unknown size
        Unknown,
    }

    impl From<u32> for FlashDevInfoSize {
        fn from(value: u32) -> Self {
            if value > 0xc {
                return Self::Unknown;
            }

            unsafe { core::mem::transmute::<u32, FlashDevInfoSize>(value) }
        }
    }

    impl FlashDevInfo {
        /// GPIO Number to be used for the secondary flash chip. See datasheet section 13.9
        pub fn cs1_gpio(&self) -> u8 {
            (self.0 & 0x1f) as _
        }

        /// Check if all attached devices support a block erase command with a command prefix of
        /// `D8h``
        pub fn d8h_erase_supported(&self) -> bool {
            (self.0 & 0x80) != 0
        }

        /// Flash/PSRAM size on chip select 0
        pub fn cs0_size(&self) -> FlashDevInfoSize {
            FlashDevInfoSize::from((self.0 & 0xf00) >> 8)
        }

        /// Flash/PSRAM size on chip select 1
        pub fn cs1_size(&self) -> FlashDevInfoSize {
            FlashDevInfoSize::from((self.0 & 0xf000) >> 12)
        }
    }

    impl From<[u32; 1]> for FlashDevInfo {
        fn from(value: [u32; 1]) -> FlashDevInfo {
            FlashDevInfo(value[0])
        }
    }

    /// 128 bit random integer generated per boot as reported by [`boot_random`]
    pub struct BootRandom(pub u128);

    impl From<[u32; 4]> for BootRandom {
        fn from(value: [u32; 4]) -> BootRandom {
            let mut result = 0;
            for word in value {
                result = (result << 32) | u128::from(word);
            }
            BootRandom(result)
        }
    }

    // based on https://github.com/raspberrypi/pico-sdk/blob/master/src/rp2_common/pico_bootrom/include/pico/bootrom.h
    /// Boot diagnostic info as described in 5.4 under the `get_sys_info` function
    pub struct BootInfo {
        /// Information about which partition is being diagnosed
        pub diagnostic_partition: PartitionIndex,
        /// Type of boot that occurred
        pub boot_type: BootType,
        /// Whether it was a chained boot
        pub chained: bool,
        /// What partition the boot came from
        pub partition: i8,
        // could probably make a nicer api for tbyb, but documentation is eh so im holding off for now
        /// Try Before You Buy info
        pub tbyb_update_info: u8,
        /// boot diagnostic flags for section A and section B
        pub boot_diagnostic: u32,
        /// Boot parameters 0 and 1
        pub boot_params: [u32; 2],
    }

    /// Recen boot diagnostic partition
    pub enum PartitionIndex {
        /// A partition along with its number
        Partition(u8),
        /// None
        None,
        /// Slot0
        Slot0,
        /// Slot1
        Slot1,
        /// Image
        Image,
        /// Unknown
        Unknown,
    }

    impl From<i8> for PartitionIndex {
        fn from(value: i8) -> Self {
            if !(-4..=15).contains(&value) {
                return Self::Unknown;
            }

            match value {
                -1 => Self::None,
                -2 => Self::Slot0,
                -3 => Self::Slot1,
                -4 => Self::Image,
                _ => Self::Partition(value as u8),
            }
        }
    }

    /// The type of boot that occurred
    pub enum BootType {
        /// Normal
        Normal,
        /// bootsel
        BootSel,
        /// Ram image
        RamImage,
        /// Flash update
        FlashUpdate,
        /// pc_sp
        PcSp,
        /// Unknown
        Unknown,
    }

    impl From<u8> for BootType {
        fn from(value: u8) -> Self {
            match value {
                0 => Self::Normal,
                2 => Self::BootSel,
                3 => Self::RamImage,
                4 => Self::FlashUpdate,
                8..=15 => Self::PcSp,
                _ => Self::Unknown,
            }
        }
    }

    #[repr(u16)]
    /// Diagnostic flags reported by the upper and lower words in [`BootInfo::boot_diagnostic`]
    pub enum BootDiagnosticFlags {
        /// The region was searched for a block loop
        RegionSearched = 0x0001,
        /// A block loop was found but it was invalid
        InvalidBlockLoop = 0x0002,
        /// A valid block loop was found (Blocks from a loop wholly contained within the region, and
        /// the blocks have the correct structure. Each block consists of items whose sizes sum to
        /// the size of the block)
        ValidBlockLoop = 0x0004,
        /// A valid IMAGE_DEF was found in the region. A valid IMAGE_DEF must parse correctly and must
        /// be executable
        ValidImageDef = 0x0008,
        /// Whether a partition table is present. This partition table must have a correct structure
        /// formed if [`BootDiagnosticFlags::ValidBlockLoop`] is set. If the partition table turns
        /// out to be invalid, then [`BootDiagnosticFlags::InvalidBlockLoop`] is set too (thus both
        /// [`BootDiagnosticFlags::ValidBlockLoop`] and [`BootDiagnosticFlags::InvalidBlockLoop`]
        /// will both be set)
        HasPartitionTable = 0x0010,
        /// There was a choice of partition/slot and this one was considered. The first slot/partition
        /// is chosen based on a number of factors. If the first choice fails verification, then the
        /// other choice will be considered.
        ///
        /// * the version of the PARTITION_TABLE/IMAGE_DEF present in the slot/partition respectively.
        /// * whether the slot/partition is the "update region" as per a FLASH_UPDATE reboot.
        /// * whether an IMAGE_DEF is marked as "explicit buy"
        Considered = 0x0020,
        /// This slot/partition was chosen (or was the only choice)
        Chosen = 0x0040,
        /// if a signature is required for the PARTITION_TABLE (via OTP setting), then whether the
        /// PARTITION_TABLE is signed with a key matching one of the four stored in OTP
        PartitionTableMatchingKeyForVerify = 0x0080,
        /// set if a hash value check could be performed. In the case a signature is required, this
        /// value is identical to [`BootDiagnosticFlags::PartitionTableMatchingKeyForVerify`]
        PartitionTableHashForVerify = 0x0100,
        /// whether the PARTITION_TABLE passed verification (signature/hash if present/required)
        PartitionTableVerifiedOk = 0x0200,
        /// if a signature is required for the IMAGE_DEF due to secure boot, then whether the
        /// IMAGE_DEF is signed with a key matching one of the four stored in OTP
        ImageDefMatchingKeyForVerify = 0x0400,
        /// set if a hash value check could be performed. In the case a signature is required, this
        /// value is identical to [`BootDiagnosticFlags::ImageDefMatchingKeyForVerify`]
        ImageDefHashForVerify = 0x0800,
        /// whether the PARTITION_TABLE passed verification (signature/hash if present/required) and
        /// any LOAD_MAP is valid
        ImageDefVerifiedOk = 0x1000,
        /// whether any code was copied into RAM due to a LOAD_MAP
        LoadMapEntriesLoaded = 0x2000,
        /// whether an IMAGE_DEF from this region was launched
        ImageLaunched = 0x4000,
        /// whether the IMAGE_DEF failed final checks before launching; these checks include
        ///
        /// * verification failed (if it hasn’t been verified earlier in the CONSIDERED phase).
        /// * a problem occurred setting up any rolling window.
        /// * the rollback version could not be set in OTP (if required in Secure mode)
        /// * the image was marked as Non-secure
        /// * the image was marked as "explicit buy", and this was a flash boot, but then region was
        ///   not the "flash update" region
        /// * the image has the wrong architecture, but architecture auto-switch is disabled (or the
        ///   correct architecture is disabled)
        ImageConditionFailure = 0x8000,
    }

    impl From<[u32; 4]> for BootInfo {
        fn from(value: [u32; 4]) -> Self {
            let word0 = value[0];

            BootInfo {
                diagnostic_partition: PartitionIndex::from((word0 & 0xFF) as i8),
                boot_type: BootType::from((word0 >> 8) as u8),
                chained: (word0 >> 8) & 0x80 > 0,
                partition: (word0 >> 16) as _,
                tbyb_update_info: (word0 >> 24) as _,
                boot_diagnostic: value[1],
                boot_params: [value[2], value[3]],
            }
        }
    }

    impl BootInfo {
        fn check_flag(diagnostics: u16, flag: BootDiagnosticFlags) -> bool {
            (diagnostics & flag as u16) != 0
        }

        /// Check if the diagnostic flag in section A (the lower word) is set
        pub fn check_section_a_flag(&self, flag: BootDiagnosticFlags) -> bool {
            Self::check_flag(self.boot_diagnostic as u16, flag)
        }

        /// Check if the diagnostic flag in section B (the upper word) is set
        pub fn check_section_b_flag(&self, flag: BootDiagnosticFlags) -> bool {
            Self::check_flag((self.boot_diagnostic >> 8) as u16, flag)
        }
    }

    #[macro_export]
    /// Generates a function with the following signature:
    ///
    /// ```rs
    /// pub fn $function_name() -> Result<Option<$ok_ret_type>, BootRomApiErrorCode>
    /// ```
    ///
    /// Which safely calls [`get_sys_info`](super::get_sys_info()) using the flag provided via
    /// the `flag` argument. `flag` is an expression that must resolve to a const variant of
    /// [`GetSysInfoFlag`]
    macro_rules! declare_get_sys_info_function {
        ($(#[$meta:meta])* $function_name:ident, $ok_ret_type:ty, $flag:expr) => {
            $(#[$meta])*
            pub fn $function_name() -> Result<Option<$ok_ret_type>, BootRomApiErrorCode> {
                const FLAG: GetSysInfoFlag = $flag;
                const BUFFER_LEN: usize = FLAG.buffer_length();
                let mut buffer = [0u32; FLAG.buffer_length()];
                let result =
                    unsafe { super::get_sys_info(buffer.as_mut_ptr(), buffer.len(), FLAG as u32) };

                if result < 0 {
                    return Err(BootRomApiErrorCode::from(result));
                } else if buffer[0] == 0 {
                    // The operation returned successfully but the flag wasn't supported
                    // for one reason or another
                    return Ok(None);
                }

                Ok(Some(<$ok_ret_type>::from(
                    TryInto::<[u32; BUFFER_LEN - 1]>::try_into(&buffer[1..]).unwrap(),
                )))
            }
        };
    }

    #[macro_export]
    #[cfg(all(target_arch = "arm", target_os = "none"))]
    /// Generates a function with the following signature:
    ///
    /// ```rs
    /// pub fn $function_name() -> Result<Option<$ok_ret_type>, BootRomApiErrorCode>
    /// ```
    ///
    /// Which safely calls [`get_sys_info_ns`](super::get_sys_info_ns()) using the flag provided via
    /// the `flag` argument. `flag` is an expression that must resolve to a const variant of
    /// [`GetSysInfoFlag`]
    macro_rules! declare_get_sys_info_ns_function {
        ($(#[$meta:meta])* $function_name:ident, $ok_ret_type:ty, $flag:expr) => {
            $(#[$meta])*
            pub fn $function_name() -> Result<Option<$ok_ret_type>, BootRomApiErrorCode> {
                const FLAG: GetSysInfoFlag = $flag;
                const BUFFER_LEN: usize = FLAG.buffer_length();
                let mut buffer = [0u32; FLAG.buffer_length()];
                let result =
                    unsafe { super::get_sys_info_ns(buffer.as_mut_ptr(), buffer.len(), FLAG as u32) };

                if result < 0 {
                    return Err(BootRomApiErrorCode::from(result));
                } else if buffer[0] == 0 {
                    // The operation returned successfully but the flag wasn't supported
                    // for one reason or another
                    return Ok(None);
                }

                Ok(Some(<$ok_ret_type>::from(
                    TryInto::<[u32; BUFFER_LEN - 1]>::try_into(&buffer[1..]).unwrap(),
                )))
            }
        };
    }

    declare_get_sys_info_function!(
        /// Get the unique identifier for the chip
        chip_info, ChipInfo, GetSysInfoFlag::ChipInfo
    );

    declare_get_sys_info_function!(
        /// Get the value of the OTP critical register
        otp_critical_register,
        OtpCriticalReg,
        GetSysInfoFlag::Critical
    );

    declare_get_sys_info_function!(
        /// Get the current running CPU's info
        cpu_info, CpuInfo, GetSysInfoFlag::CpuInfo
    );

    declare_get_sys_info_function!(
        /// Get flash device info in the format of OTP FLASH_DEVINFO
        flash_dev_info, FlashDevInfo, GetSysInfoFlag::FlashDevInfo
    );

    declare_get_sys_info_function!(
        /// Get a 128-bit random number generated on each boot
        boot_random, BootRandom, GetSysInfoFlag::BootRandom
    );

    declare_get_sys_info_function!(
        /// Get diagnostic boot info
        boot_info, BootInfo, GetSysInfoFlag::BootInfo
    );

    #[cfg(all(target_arch = "arm", target_os = "none"))]
    declare_get_sys_info_ns_function!(
        /// Get the unique identifier for the chip
        chip_info_ns, ChipInfo, GetSysInfoFlag::ChipInfo
    );

    #[cfg(all(target_arch = "arm", target_os = "none"))]
    declare_get_sys_info_ns_function!(
        /// Get the value of the OTP critical register
        otp_critical_register_ns,
        OtpCriticalReg,
        GetSysInfoFlag::Critical
    );

    #[cfg(all(target_arch = "arm", target_os = "none"))]
    declare_get_sys_info_ns_function!(
        /// Get the current running CPU's info
        cpu_info_ns, CpuInfo, GetSysInfoFlag::CpuInfo
    );

    #[cfg(all(target_arch = "arm", target_os = "none"))]
    declare_get_sys_info_ns_function!(
        /// Get flash device info in the format of OTP FLASH_DEVINFO
        flash_dev_info_ns, FlashDevInfo, GetSysInfoFlag::FlashDevInfo
    );

    #[cfg(all(target_arch = "arm", target_os = "none"))]
    declare_get_sys_info_ns_function!(
        /// Get a 128-bit random number generated on each boot
        boot_random_ns, BootRandom, GetSysInfoFlag::BootRandom
    );

    #[cfg(all(target_arch = "arm", target_os = "none"))]
    declare_get_sys_info_ns_function!(
        /// Get diagnostic boot info
        boot_info_ns, BootInfo, GetSysInfoFlag::BootInfo
    );
}

macro_rules! declare_rom_function {
    (
        $(#[$outer:meta])*
        fn $name:ident( $($argname:ident: $ty:ty),* ) -> $ret:ty
        $lookup:block
    ) => {
        #[doc = r"Additional access for the `"]
        #[doc = stringify!($name)]
        #[doc = r"` ROM function."]
        pub mod $name {
            /// Retrieve a function pointer.
            #[cfg(not(feature = "rom-func-cache"))]
            pub fn ptr() -> extern "C" fn( $($argname: $ty),* ) -> $ret {
                let p: usize = $lookup;
                unsafe {
                    let func : extern "C" fn( $($argname: $ty),* ) -> $ret = core::mem::transmute(p);
                    func
                }
            }

            /// Retrieve a function pointer.
            #[cfg(feature = "rom-func-cache")]
            pub fn ptr() -> extern "C" fn( $($argname: $ty),* ) -> $ret {
                use core::sync::atomic::{AtomicU16, Ordering};

                // All pointers in the ROM fit in 16 bits, so we don't need a
                // full width word to store the cached value.
                static CACHED_PTR: AtomicU16 = AtomicU16::new(0);
                // This is safe because the lookup will always resolve
                // to the same value.  So even if an interrupt or another
                // core starts at the same time, it just repeats some
                // work and eventually writes back the correct value.
                let p: usize = match CACHED_PTR.load(Ordering::Relaxed) {
                    0 => {
                        let raw: usize = $lookup;
                        CACHED_PTR.store(raw as u16, Ordering::Relaxed);
                        raw
                    },
                    val => val as usize,
                };
                unsafe {
                    let func : extern "C" fn( $($argname: $ty),* ) -> $ret = core::mem::transmute(p);
                    func
                }
            }
        }

        $(#[$outer])*
        pub extern "C" fn $name( $($argname: $ty),* ) -> $ret {
            $name::ptr()($($argname),*)
        }
    };

    (
        $(#[$outer:meta])*
        unsafe fn $name:ident( $($argname:ident: $ty:ty),* ) -> $ret:ty
        $lookup:block
    ) => {
        #[doc = r"Additional access for the `"]
        #[doc = stringify!($name)]
        #[doc = r"` ROM function."]
        pub mod $name {
            /// Retrieve a function pointer.
            #[cfg(not(feature = "rom-func-cache"))]
            pub fn ptr() -> unsafe extern "C" fn( $($argname: $ty),* ) -> $ret {
                let p: usize = $lookup;
                unsafe {
                    let func : unsafe extern "C" fn( $($argname: $ty),* ) -> $ret = core::mem::transmute(p);
                    func
                }
            }

            /// Retrieve a function pointer.
            #[cfg(feature = "rom-func-cache")]
            pub fn ptr() -> unsafe extern "C" fn( $($argname: $ty),* ) -> $ret {
                use core::sync::atomic::{AtomicU16, Ordering};

                // All pointers in the ROM fit in 16 bits, so we don't need a
                // full width word to store the cached value.
                static CACHED_PTR: AtomicU16 = AtomicU16::new(0);
                // This is safe because the lookup will always resolve
                // to the same value.  So even if an interrupt or another
                // core starts at the same time, it just repeats some
                // work and eventually writes back the correct value.
                let p: usize = match CACHED_PTR.load(Ordering::Relaxed) {
                    0 => {
                        let raw: usize = $lookup;
                        CACHED_PTR.store(raw as u16, Ordering::Relaxed);
                        raw
                    },
                    val => val as usize,
                };
                unsafe {
                    let func : unsafe extern "C" fn( $($argname: $ty),* ) -> $ret = core::mem::transmute(p);
                    func
                }
            }
        }

        $(#[$outer])*
        /// # Safety
        ///
        /// This is a low-level C function. It may be difficult to call safely from
        /// Rust. If in doubt, check the rp235x datasheet for details and do your own
        /// safety evaluation.
        pub unsafe extern "C" fn $name( $($argname: $ty),* ) -> $ret {
            $name::ptr()($($argname),*)
        }
    };
}

// **************** 5.5.7 Low-level Flash Commands ****************

declare_rom_function! {
    /// Restore all QSPI pad controls to their default state, and connect the
    /// QMI peripheral to the QSPI pads.
    ///
    /// Supported architectures: ARM-S, RISC-V
    unsafe fn connect_internal_flash() -> () {
        crate::rom_data::rom_table_lookup(*b"IF", crate::rom_data::rt_flags::FUNC_ARM_SEC_RISCV)
    }
}

declare_rom_function! {
    /// Initialise the QMI for serial operations (direct mode)
    ///
    /// Also initialise a basic XIP mode, where the QMI will perform 03h serial
    /// read commands at low speed (CLKDIV=12) in response to XIP reads.
    ///
    /// Then, issue a sequence to the QSPI device on chip select 0, designed to
    /// return it from continuous read mode ("XIP mode") and/or QPI mode to a
    /// state where it will accept serial commands. This is necessary after
    /// system reset to restore the QSPI device to a known state, because
    /// resetting RP2350 does not reset attached QSPI devices. It is also
    /// necessary when user code, having already performed some
    /// continuous-read-mode or QPI-mode accesses, wishes to return the QSPI
    /// device to a state where it will accept the serial erase and programming
    /// commands issued by the bootrom’s flash access functions.
    ///
    /// If a GPIO for the secondary chip select is configured via FLASH_DEVINFO,
    /// then the XIP exit sequence is also issued to chip select 1.
    ///
    /// The QSPI device should be accessible for XIP reads after calling this
    /// function; the name flash_exit_xip refers to returning the QSPI device
    /// from its XIP state to a serial command state.
    ///
    /// Supported architectures: ARM-S, RISC-V
    unsafe fn flash_exit_xip() -> () {
        crate::rom_data::rom_table_lookup(*b"EX", crate::rom_data::rt_flags::FUNC_ARM_SEC_RISCV)
    }
}

declare_rom_function! {
    /// Erase count bytes, starting at addr (offset from start of flash).
    ///
    /// Optionally, pass a block erase command e.g. D8h block erase, and the
    /// size of the block erased by this command — this function will use the
    /// larger block erase where possible, for much higher erase speed. addr
    /// must be aligned to a 4096-byte sector, and count must be a multiple of
    /// 4096 bytes.
    ///
    /// This is a low-level flash API, and no validation of the arguments is
    /// performed. See flash_op() for a higher-level API which checks alignment,
    /// flash bounds and partition permissions, and can transparently apply a
    /// runtime-to-storage address translation.
    ///
    /// The QSPI device must be in a serial command state before calling this
    /// API, which can be achieved by calling connect_internal_flash() followed
    /// by flash_exit_xip(). After the erase, the flash cache should be flushed
    /// via flash_flush_cache() to ensure the modified flash data is visible to
    /// cached XIP accesses.
    ///
    /// Finally, the original XIP mode should be restored by copying the saved
    /// XIP setup function from bootram into SRAM, and executing it: the bootrom
    /// provides a default function which restores the flash mode/clkdiv
    /// discovered during flash scanning, and user programs can override this
    /// with their own XIP setup function.
    ///
    /// For the duration of the erase operation, QMI is in direct mode (Section
    /// 12.14.5) and attempting to access XIP from DMA, the debugger or the
    /// other core will return a bus fault. XIP becomes accessible again once
    /// the function returns.
    ///
    /// Supported architectures: ARM-S, RISC-V
    unsafe fn flash_range_erase(addr: u32, count: usize, block_size: u32, block_cmd: u8) -> () {
        crate::rom_data::rom_table_lookup(*b"RE", crate::rom_data::rt_flags::FUNC_ARM_SEC_RISCV)
    }
}

declare_rom_function! {
    /// Program data to a range of flash storage addresses starting at addr
    /// (offset from the start of flash) and count bytes in size.
    ///
    /// `addr` must be aligned to a 256-byte boundary, and count must be a
    /// multiple of 256.
    ///
    /// This is a low-level flash API, and no validation of the arguments is
    /// performed. See flash_op() for a higher-level API which checks alignment,
    /// flash bounds and partition permissions, and can transparently apply a
    /// runtime-to-storage address translation.
    ///
    /// The QSPI device must be in a serial command state before calling this
    /// API — see notes on flash_range_erase().
    ///
    /// Supported architectures: ARM-S, RISC-V
    unsafe fn flash_range_program(addr: u32, data: *const u8, count: usize) -> () {
        crate::rom_data::rom_table_lookup(*b"RP", crate::rom_data::rt_flags::FUNC_ARM_SEC_RISCV)
    }
}

declare_rom_function! {
    /// Flush the entire XIP cache, by issuing an invalidate by set/way
    /// maintenance operation to every cache line (Section 4.4.1).
    ///
    /// This ensures that flash program/erase operations are visible to
    /// subsequent cached XIP reads.
    ///
    /// Note that this unpins pinned cache lines, which may interfere with
    /// cache-as-SRAM use of the XIP cache.
    ///
    /// No other operations are performed.
    ///
    /// Supported architectures: ARM-S, RISC-V
    unsafe fn flash_flush_cache() -> () {
        crate::rom_data::rom_table_lookup(*b"FC", crate::rom_data::rt_flags::FUNC_ARM_SEC_RISCV)
    }
}

declare_rom_function! {
    /// Configure the QMI to generate a standard 03h serial read command, with
    /// 24 address bits, upon each XIP access.
    ///
    /// This is a slow XIP configuration, but is widely supported. CLKDIV is set
    /// to 12. The debugger may call this function to ensure that flash is
    /// readable following a program/erase operation.
    ///
    /// Note that the same setup is performed by flash_exit_xip(), and the
    /// RP2350 flash program/erase functions do not leave XIP in an inaccessible
    /// state, so calls to this function are largely redundant. It is provided
    /// for compatibility with RP2040.
    ///
    /// Supported architectures: ARM-S, RISC-V
    unsafe fn flash_enter_cmd_xip() -> () {
        crate::rom_data::rom_table_lookup(*b"CX", crate::rom_data::rt_flags::FUNC_ARM_SEC_RISCV)
    }
}

declare_rom_function! {
    /// Configure QMI for one of a small menu of XIP read modes supported by the
    /// bootrom.
    ///
    /// This mode is configured for both memory windows (both chip
    /// selects), and the clock divisor is also applied to direct mode.
    ///
    /// The available modes are:
    ///
    /// * 0: `03h` serial read: serial address, serial data, no wait cycles
    /// * 1: `0Bh` serial read: serial address, serial data, 8 wait cycles
    /// * 2: `BBh` dual-IO read: dual address, dual data, 4 wait cycles
    ///   (including MODE bits, which are driven to 0)
    /// * 3: `EBh` quad-IO read: quad address, quad data, 6 wait cycles
    ///   (including MODE bits, which are driven to 0)
    ///
    /// The XIP write command/format are not configured by this function. When
    /// booting from flash, the bootrom tries each of these modes in turn, from
    /// 3 down to 0. The first mode that is found to work is remembered, and a
    /// default XIP setup function is written into bootram that calls this
    /// function (flash_select_xip_read_mode) with the parameters discovered
    /// during flash scanning. This can be called at any time to restore the
    /// flash parameters discovered during flash boot.
    ///
    /// All XIP modes configured by the bootrom have an 8-bit serial command
    /// prefix, so that the flash can remain in a serial command state, meaning
    /// XIP accesses can be mixed more freely with program/erase serial
    /// operations. This has a performance penalty, so users can perform their
    /// own flash setup after flash boot using continuous read mode or QPI mode
    /// to avoid or alleviate the command prefix cost.
    ///
    /// Supported architectures: ARM-S, RISC-V
    unsafe fn flash_select_xip_read_mode(bootrom_xip_mode: u8, clkdiv: u8) -> () {
        crate::rom_data::rom_table_lookup(*b"XM", crate::rom_data::rt_flags::FUNC_ARM_SEC_RISCV)
    }
}

declare_rom_function! {
    /// Restore the QMI address translation registers, ATRANS0 through ATRANS7,
    /// to their reset state.
    ///
    /// This makes the runtime- to-storage address map an identity map, i.e. the
    /// mapped and unmapped address are equal, and the entire space is fully mapped.
    ///
    /// See [Section 12.14.4](https://rptl.io/rp2350-datasheet#section_bootrom) of the RP2350
    /// datasheet.
    ///
    /// Supported architectures: ARM-S, RISC-V
    unsafe fn flash_reset_address_trans() -> () {
        crate::rom_data::rom_table_lookup(*b"RA", crate::rom_data::rt_flags::FUNC_ARM_SEC_RISCV)
    }
}

// **************** High-level Flash Commands ****************

declare_rom_function! {
    /// Applies the address translation currently configured by QMI address
    /// translation registers, ATRANS0 through ATRANS7.
    ///
    /// See [Section 12.14.4](https://rptl.io/rp2350-datasheet#section_bootrom) of the RP2350
    /// datasheet.
    ///
    /// Translating an address outside of the XIP runtime address window, or
    /// beyond the bounds of an ATRANSx_SIZE field, returns
    /// BOOTROM_ERROR_INVALID_ADDRESS, which is not a valid flash storage
    /// address. Otherwise, return the storage address which QMI would access
    /// when presented with the runtime address addr. This is effectively a
    /// virtual-to-physical address translation for QMI.
    ///
    /// Supported architectures: ARM-S, RISC-V
    unsafe fn flash_runtime_to_storage_addr(addr: u32) -> i32 {
        crate::rom_data::rom_table_lookup(*b"FA", crate::rom_data::rt_flags::FUNC_ARM_SEC_RISCV)
    }
}

declare_rom_function! {
    /// Non-secure version of [flash_runtime_to_storage_addr()]
    ///
    /// Supported architectures: ARM-NS
    #[cfg(all(target_arch = "arm", target_os = "none"))]
    unsafe fn flash_runtime_to_storage_addr_ns(addr: u32) -> i32 {
        crate::rom_data::rom_table_lookup(*b"FA", crate::rom_data::rt_flags::FUNC_ARM_NONSEC)
    }
}

declare_rom_function! {
    /// Perform a flash read, erase, or program operation.
    ///
    /// Erase operations must be sector-aligned (4096 bytes) and sector-
    /// multiple-sized, and program operations must be page-aligned (256 bytes)
    /// and page-multiple-sized; misaligned erase and program operations will
    /// return BOOTROM_ERROR_BAD_ALIGNMENT. The operation — erase, read, program
    /// — is selected by the CFLASH_OP_BITS bitfield of the flags argument.
    ///
    /// See datasheet section 5.5.8.2 for more details.
    ///
    /// Supported architectures: ARM-S, RISC-V
    unsafe fn flash_op(flags: u32, addr: u32, size_bytes: u32, buffer: *mut u8) -> i32 {
        crate::rom_data::rom_table_lookup(*b"FO", crate::rom_data::rt_flags::FUNC_ARM_SEC_RISCV)
    }
}

declare_rom_function! {
    /// Non-secure version of [flash_op()]
    ///
    /// Supported architectures: ARM-NS
    #[cfg(all(target_arch = "arm", target_os = "none"))]
    unsafe fn flash_op_ns(flags: u32, addr: u32, size_bytes: u32, buffer: *mut u8) -> i32 {
        crate::rom_data::rom_table_lookup(*b"FO", crate::rom_data::rt_flags::FUNC_ARM_NONSEC)
    }
}

// **************** Security Related Functions ****************

declare_rom_function! {
    /// Allow or disallow the specific NS API (note all NS APIs default to
    /// disabled).
    ///
    /// See datasheet section 5.5.9.1 for more details.
    ///
    /// Supported architectures: ARM-S
    #[cfg(all(target_arch = "arm", target_os = "none"))]
    unsafe fn set_ns_api_permission(ns_api_num: u32, allowed: u8) -> i32 {
        crate::rom_data::rom_table_lookup(*b"SP", crate::rom_data::rt_flags::FUNC_ARM_SEC)
    }
}

declare_rom_function! {
    /// Utility method that can be used by secure ARM code to validate a buffer
    /// passed to it from Non-secure code.
    ///
    /// See datasheet section 5.5.9.2 for more details.
    ///
    /// Supported architectures: ARM-S, RISC-V
    unsafe fn validate_ns_buffer() -> () {
        crate::rom_data::rom_table_lookup(*b"VB", crate::rom_data::rt_flags::FUNC_ARM_SEC_RISCV)
    }
}

// **************** Miscellaneous Functions ****************

declare_rom_function! {
    /// Resets the RP2350 and uses the watchdog facility to restart.
    ///
    /// See datasheet section 5.5.10.1 for more details.
    ///
    /// Supported architectures: ARM-S, RISC-V
    fn reboot(flags: u32, delay_ms: u32, p0: u32, p1: u32) -> i32 {
        crate::rom_data::rom_table_lookup(*b"RB", crate::rom_data::rt_flags::FUNC_ARM_SEC_RISCV)
    }
}

declare_rom_function! {
    /// Non-secure version of [reboot()]
    ///
    /// Supported architectures: ARM-NS
    #[cfg(all(target_arch = "arm", target_os = "none"))]
    fn reboot_ns(flags: u32, delay_ms: u32, p0: u32, p1: u32) -> i32 {
        crate::rom_data::rom_table_lookup(*b"RB", crate::rom_data::rt_flags::FUNC_ARM_NONSEC)
    }
}

declare_rom_function! {
    /// Resets internal bootrom state.
    ///
    /// See datasheet section 5.5.10.2 for more details.
    ///
    /// Supported architectures: ARM-S, RISC-V
    unsafe fn bootrom_state_reset(flags: u32) -> () {
        crate::rom_data::rom_table_lookup(*b"SR", crate::rom_data::rt_flags::FUNC_ARM_SEC_RISCV)
    }
}

declare_rom_function! {
    /// Set a boot ROM callback.
    ///
    /// The only supported callback_number is 0 which sets the callback used for
    /// the secure_call API.
    ///
    /// See datasheet section 5.5.10.3 for more details.
    ///
    /// Supported architectures: ARM-S, RISC-V
    unsafe fn set_rom_callback(callback_number: i32, callback_fn: *const ()) -> i32 {
        crate::rom_data::rom_table_lookup(*b"RC", crate::rom_data::rt_flags::FUNC_ARM_SEC_RISCV)
    }
}

// **************** System Information Functions ****************

declare_rom_function! {
    /// Fills a buffer with various system information.
    ///
    /// Note that this API is also used to return information over the PICOBOOT
    /// interface.
    ///
    /// See datasheet section 5.5.11.1 for more details.
    ///
    /// Supported architectures: ARM-S, RISC-V
    unsafe fn get_sys_info(out_buffer: *mut u32, out_buffer_word_size: usize, flags: u32) -> i32 {
        crate::rom_data::rom_table_lookup(*b"GS", crate::rom_data::rt_flags::FUNC_ARM_SEC_RISCV)
    }
}

declare_rom_function! {
    /// Non-secure version of [get_sys_info()]
    ///
    /// Supported architectures: ARM-NS
    #[cfg(all(target_arch = "arm", target_os = "none"))]
    unsafe fn get_sys_info_ns(out_buffer: *mut u32, out_buffer_word_size: usize, flags: u32) -> i32 {
        crate::rom_data::rom_table_lookup(*b"GS", crate::rom_data::rt_flags::FUNC_ARM_NONSEC)
    }
}

declare_rom_function! {
    /// Fills a buffer with information from the partition table.
    ///
    /// Note that this API is also used to return information over the PICOBOOT
    /// interface.
    ///
    /// See datasheet section 5.5.11.2 for more details.
    ///
    /// Supported architectures: ARM-S, RISC-V
    unsafe fn get_partition_table_info(out_buffer: *mut u32, out_buffer_word_size: usize, flags_and_partition: u32) -> i32 {
        crate::rom_data::rom_table_lookup(*b"GP", crate::rom_data::rt_flags::FUNC_ARM_SEC_RISCV)
    }
}

declare_rom_function! {
    /// Non-secure version of [get_partition_table_info()]
    ///
    /// Supported architectures: ARM-NS
    #[cfg(all(target_arch = "arm", target_os = "none"))]
    unsafe fn get_partition_table_info_ns(out_buffer: *mut u32, out_buffer_word_size: usize, flags_and_partition: u32) -> i32 {
        crate::rom_data::rom_table_lookup(*b"GP", crate::rom_data::rt_flags::FUNC_ARM_NONSEC)
    }
}

declare_rom_function! {
    /// Loads the current partition table from flash, if present.
    ///
    /// See datasheet section 5.5.11.3 for more details.
    ///
    /// Supported architectures: ARM-S, RISC-V
    unsafe fn load_partition_table(workarea_base: *mut u8, workarea_size: usize, force_reload: bool) -> i32 {
        crate::rom_data::rom_table_lookup(*b"LP", crate::rom_data::rt_flags::FUNC_ARM_SEC_RISCV)
    }
}

declare_rom_function! {
    /// Writes data from a buffer into OTP, or reads data from OTP into a buffer.
    ///
    /// See datasheet section 5.5.11.4 for more details.
    ///
    /// Supported architectures: ARM-S, RISC-V
    unsafe fn otp_access(buf: *mut u8, buf_len: usize, row_and_flags: u32) -> i32 {
        crate::rom_data::rom_table_lookup(*b"OA", crate::rom_data::rt_flags::FUNC_ARM_SEC_RISCV)
    }
}

declare_rom_function! {
    /// Non-secure version of [otp_access()]
    ///
    /// Supported architectures: ARM-NS
    #[cfg(all(target_arch = "arm", target_os = "none"))]
    unsafe fn otp_access_ns(buf: *mut u8, buf_len: usize, row_and_flags: u32) -> i32 {
        crate::rom_data::rom_table_lookup(*b"OA", crate::rom_data::rt_flags::FUNC_ARM_NONSEC)
    }
}

// **************** Boot Related Functions ****************

declare_rom_function! {
    /// Determines which of the partitions has the "better" IMAGE_DEF. In the
    /// case of executable images, this is the one that would be booted.
    ///
    /// See datasheet section 5.5.12.1 for more details.
    ///
    /// Supported architectures: ARM-S, RISC-V
    unsafe fn pick_ab_parition(workarea_base: *mut u8, workarea_size: usize, partition_a_num: u32) -> i32 {
        crate::rom_data::rom_table_lookup(*b"AB", crate::rom_data::rt_flags::FUNC_ARM_SEC_RISCV)
    }
}

declare_rom_function! {
    /// Searches a memory region for a launchable image, and executes it if
    /// possible.
    ///
    /// See datasheet section 5.5.12.2 for more details.
    ///
    /// Supported architectures: ARM-S, RISC-V
    unsafe fn chain_image(workarea_base: *mut u8, workarea_size: usize, region_base: i32, region_size: u32) -> i32 {
        crate::rom_data::rom_table_lookup(*b"CI", crate::rom_data::rt_flags::FUNC_ARM_SEC_RISCV)
    }
}

declare_rom_function! {
    /// Perform an "explicit" buy of an executable launched via an IMAGE_DEF
    /// which was "explicit buy" flagged.
    ///
    /// See datasheet section 5.5.12.3 for more details.
    ///
    /// Supported architectures: ARM-S, RISC-V
    unsafe fn explicit_buy(buffer: *mut u8, buffer_size: u32) -> i32 {
        crate::rom_data::rom_table_lookup(*b"EB", crate::rom_data::rt_flags::FUNC_ARM_SEC_RISCV)
    }
}

declare_rom_function! {
    /// Not yet documented.
    ///
    /// See datasheet section 5.5.12.4 for more details.
    ///
    /// Supported architectures: ARM-S, RISC-V
    unsafe fn get_uf2_target_partition(workarea_base: *mut u8, workarea_size: usize, family_id: u32, partition_out: *mut u32) -> i32 {
        crate::rom_data::rom_table_lookup(*b"GU", crate::rom_data::rt_flags::FUNC_ARM_SEC_RISCV)
    }
}

declare_rom_function! {
    /// Returns: The index of the B partition of partition A if a partition
    /// table is present and loaded, and there is a partition A with a B
    /// partition; otherwise returns BOOTROM_ERROR_NOT_FOUND.
    ///
    /// See datasheet section 5.5.12.5 for more details.
    ///
    /// Supported architectures: ARM-S, RISC-V
    unsafe fn get_b_partition(partition_a: u32) -> i32 {
        crate::rom_data::rom_table_lookup(*b"GB", crate::rom_data::rt_flags::FUNC_ARM_SEC_RISCV)
    }
}

// **************** Non-secure-specific Functions ****************

// NB: The "secure_call" function should be here, but it doesn't have a fixed
// function signature as it is designed to let you bounce into any secure
// function from non-secure mode.

// **************** RISC-V Functions ****************

declare_rom_function! {
    /// Set stack for RISC-V bootrom functions to use.
    ///
    /// See datasheet section 5.5.14.1 for more details.
    ///
    /// Supported architectures: RISC-V
    #[cfg(not(all(target_arch = "arm", target_os = "none")))]
    unsafe fn set_bootrom_stack(base_size: *mut u32) -> i32 {
        crate::rom_data::rom_table_lookup(*b"SS", crate::rom_data::rt_flags::FUNC_RISCV)
    }
}

/// The version number of the rom.
pub fn rom_version_number() -> u8 {
    unsafe { *VERSION_NUMBER }
}

/// The 8 most significant hex digits of the Bootrom git revision.
pub fn git_revision() -> u32 {
    let ptr = rom_data_lookup(*b"GR", rt_flags::DATA) as *const u32;
    unsafe { ptr.read() }
}

/// A pointer to the resident partition table info.
///
/// The resident partition table is the subset of the full partition table that
/// is kept in memory, and used for flash permissions.
pub fn partition_table_pointer() -> *const u32 {
    let ptr = rom_data_lookup(*b"PT", rt_flags::DATA) as *const *const u32;
    unsafe { ptr.read() }
}

/// Determine if we are in secure mode
///
/// Returns `true` if we are in secure mode and `false` if we are in non-secure
/// mode.
#[cfg(all(target_arch = "arm", target_os = "none"))]
pub fn is_secure_mode() -> bool {
    // Look at the start of ROM, which is always readable
    #[allow(clippy::zero_ptr)]
    let rom_base: *mut u32 = 0x0000_0000 as *mut u32;
    // Use the 'tt' instruction to check the permissions for that address
    let tt = cortex_m::asm::tt(rom_base);
    // Is the secure bit set? => secure mode
    (tt & (1 << 22)) != 0
}

/// Determine if we are in secure mode
///
/// Always returns `false` on RISC-V as it is impossible to determine if
/// you are in Machine Mode or User Mode by design.
#[cfg(not(all(target_arch = "arm", target_os = "none")))]
pub fn is_secure_mode() -> bool {
    false
}
