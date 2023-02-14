//! Single Cycle Input and Output (SIO)
//!
//! To be able to partition parts of the SIO block to other modules:
//!
//! ```no_run
//! use rp2040_hal::{gpio::Pins, pac, sio::Sio};
//!
//! let mut peripherals = pac::Peripherals::take().unwrap();
//! let sio = Sio::new(peripherals.SIO);
//! ```
//!
//! And then for example
//!
//! ```no_run
//! # use rp2040_hal::{gpio::Pins, pac, sio::Sio};
//! # let mut peripherals = pac::Peripherals::take().unwrap();
//! # let sio = Sio::new(peripherals.SIO);
//! let pins = Pins::new(peripherals.IO_BANK0, peripherals.PADS_BANK0, sio.gpio_bank0, &mut peripherals.RESETS);
//! ```

use crate::typelevel::Sealed;

use super::*;
use core::convert::Infallible;

/// Marker struct for ownership of SIO gpio bank0
pub struct SioGpioBank0 {
    _private: (),
}

/// Marker struct for ownership of SIO FIFO
pub struct SioFifo {
    _private: (),
}

/// Marker struct for ownership of SIO gpio qspi
pub struct SioGpioQspi {
    _private: (),
}

/// Marker struct for ownership of divide/modulo module
pub struct HwDivider {
    _private: (),
}

/// Result of divide/modulo operation
pub struct DivResult<T> {
    /// The quotient of divide/modulo operation
    pub quotient: T,
    /// The remainder of divide/modulo operation
    pub remainder: T,
}

/// Struct containing ownership markers for managing ownership of the SIO registers.
pub struct Sio {
    _sio: pac::SIO,
    /// GPIO Bank 0 registers
    pub gpio_bank0: SioGpioBank0,
    /// GPIO QSPI registers
    pub gpio_qspi: SioGpioQspi,
    /// 8-cycle hardware divide/modulo module
    pub hwdivider: HwDivider,
    /// Inter-core FIFO
    pub fifo: SioFifo,
    /// Interpolator 0
    pub interp0: Interp0,
    /// Interpolator 1
    pub interp1: Interp1,
}

impl Sio {
    /// Create `Sio` from the PAC.
    pub fn new(sio: pac::SIO) -> Self {
        Self {
            _sio: sio,
            gpio_bank0: SioGpioBank0 { _private: () },
            gpio_qspi: SioGpioQspi { _private: () },
            fifo: SioFifo { _private: () },
            hwdivider: HwDivider { _private: () },
            interp0: Interp0 {
                lane0: Interp0Lane0 { _private: () },
                lane1: Interp0Lane1 { _private: () },
            },
            interp1: Interp1 {
                lane0: Interp1Lane0 { _private: () },
                lane1: Interp1Lane1 { _private: () },
            },
        }
    }

    /// Returns whether we are running on Core 0 (`0`) or Core 1 (`1`).
    pub fn core() -> u8 {
        // Safety: it is always safe to read this read-only register
        unsafe { (*pac::SIO::ptr()).cpuid.read().bits() as u8 }
    }
}

impl SioFifo {
    /// Check if the inter-core FIFO has valid data for reading.
    ///
    /// Returning `true` means there is valid data, `false` means it is empty
    /// and you must not read from it.
    pub fn is_read_ready(&mut self) -> bool {
        let sio = unsafe { &(*pac::SIO::ptr()) };
        sio.fifo_st.read().vld().bit_is_set()
    }

    /// Check if the inter-core FIFO is ready to receive data.
    ///
    /// Returning `true` means there is room, `false` means it is full and you
    /// must not write to it.
    pub fn is_write_ready(&mut self) -> bool {
        let sio = unsafe { &(*pac::SIO::ptr()) };
        sio.fifo_st.read().rdy().bit_is_set()
    }

    /// Return the FIFO status, as an integer.
    pub fn status(&self) -> u32 {
        let sio = unsafe { &(*pac::SIO::ptr()) };
        sio.fifo_st.read().bits()
    }

    /// Write to the inter-core FIFO.
    ///
    /// You must ensure the FIFO has space by calling `is_write_ready`
    pub fn write(&mut self, value: u32) {
        let sio = unsafe { &(*pac::SIO::ptr()) };
        sio.fifo_wr.write(|w| unsafe { w.bits(value) });
        // Fire off an event to the other core.
        // This is required as the other core may be `wfe` (waiting for event)
        cortex_m::asm::sev();
    }

    /// Read from the inter-core FIFO.
    ///
    /// Will return `Some(data)`, or `None` if the FIFO is empty.
    pub fn read(&mut self) -> Option<u32> {
        if self.is_read_ready() {
            let sio = unsafe { &(*pac::SIO::ptr()) };
            Some(sio.fifo_rd.read().bits())
        } else {
            None
        }
    }

    /// Read from the FIFO until it is empty, throwing the contents away.
    pub fn drain(&mut self) {
        while self.read().is_some() {
            // Retry until FIFO empty
        }
    }

    /// Push to the FIFO, spinning if there's no space.
    pub fn write_blocking(&mut self, value: u32) {
        // We busy-wait for the FIFO to have some space
        while !self.is_write_ready() {
            cortex_m::asm::nop();
        }

        // Write the value to the FIFO - the other core will now be able to
        // pop it off its end of the FIFO.
        self.write(value);

        // Fire off an event to the other core
        cortex_m::asm::sev();
    }

    /// Pop from the FIFO, spinning if there's currently no data.
    pub fn read_blocking(&mut self) -> u32 {
        // Keep trying until FIFO has data
        loop {
            // Have we got something?
            if let Some(data) = self.read() {
                // Yes, return it right away
                return data;
            } else {
                // No, so sleep the CPU. We expect the sending core to `sev`
                // on write.
                cortex_m::asm::wfe();
            }
        }
    }
}

// This takes advantage of how AAPCS defines a 64-bit return on 32-bit registers
// by packing it into r0[0:31] and r1[32:63].  So all we need to do is put
// the remainder in the high order 32 bits of a 64 bit result.   We can also
// alias the division operators to these for a similar reason r0 is the
// result either way and r1 a scratch register, so the caller can't assume it
// retains the argument value.
#[cfg(target_arch = "arm")]
core::arch::global_asm!(
    ".macro hwdivider_head",
    "ldr    r2, =(0xd0000000)", // SIO_BASE
    // Check the DIRTY state of the divider by shifting it into the C
    // status bit.
    "ldr    r3, [r2, #0x078]", // DIV_CSR
    "lsrs   r3, #2",           // DIRTY = 1, so shift 2 down
    // We only need to save the state when DIRTY, otherwise we can just do the
    // division directly.
    "bcs    2f",
    "1:",
    // Do the actual division now, we're either not DIRTY, or we've saved the
    // state and branched back here so it's safe now.
    ".endm",
    ".macro hwdivider_tail",
    // 8 cycle delay to wait for the result.  Each branch takes two cycles
    // and fits into a 2-byte Thumb instruction, so this is smaller than
    // 8 NOPs.
    "b      3f",
    "3: b   3f",
    "3: b   3f",
    "3: b   3f",
    "3:",
    // Read the quotient last, since that's what clears the dirty flag.
    "ldr    r1, [r2, #0x074]", // DIV_REMAINDER
    "ldr    r0, [r2, #0x070]", // DIV_QUOTIENT
    // Either return to the caller or back to the state restore.
    "bx     lr",
    "2:",
    // Since we can't save the signed-ness of the calculation, we have to make
    // sure that there's at least an 8 cycle delay before we read the result.
    // The push takes 5 cycles, and we've already spent at least 7 checking
    // the DIRTY state to get here.
    "push   {{r4-r6, lr}}",
    // Read the quotient last, since that's what clears the dirty flag.
    "ldr    r3, [r2, #0x060]", // DIV_UDIVIDEND
    "ldr    r4, [r2, #0x064]", // DIV_UDIVISOR
    "ldr    r5, [r2, #0x074]", // DIV_REMAINDER
    "ldr    r6, [r2, #0x070]", // DIV_QUOTIENT
    // If we get interrupted here (before a write sets the DIRTY flag) it's
    // fine, since we have the full state, so the interruptor doesn't have to
    // restore it.  Once the write happens and the DIRTY flag is set, the
    // interruptor becomes responsible for restoring our state.
    "bl     1b",
    // If we are interrupted here, then the interruptor will start an incorrect
    // calculation using a wrong divisor, but we'll restore the divisor and
    // result ourselves correctly. This sets DIRTY, so any interruptor will
    // save the state.
    "str    r3, [r2, #0x060]", // DIV_UDIVIDEND
    // If we are interrupted here, the the interruptor may start the
    // calculation using incorrectly signed inputs, but we'll restore the
    // result ourselves. This sets DIRTY, so any interruptor will save the
    // state.
    "str    r4, [r2, #0x064]", // DIV_UDIVISOR
    // If we are interrupted here, the interruptor will have restored
    // everything but the quotient may be wrongly signed.  If the calculation
    // started by the above writes is still ongoing it is stopped, so it won't
    // replace the result we're restoring.  DIRTY and READY set, but only
    // DIRTY matters to make the interruptor save the state.
    "str    r5, [r2, #0x074]", // DIV_REMAINDER
    // State fully restored after the quotient write.  This sets both DIRTY
    // and READY, so whatever we may have interrupted can read the result.
    "str    r6, [r2, #0x070]", // DIV_QUOTIENT
    "pop    {{r4-r6, pc}}",
    ".endm",
);

macro_rules! division_function {
    (
        $name:ident $($intrinsic:ident)* ( $argty:ty ) {
            $($begin:literal),+
        }
    ) => {
        #[cfg(all(target_arch = "arm", not(feature = "disable-intrinsics")))]
        core::arch::global_asm!(
            // Mangle the name slightly, since this is a global symbol.
            concat!(".global _rphal_", stringify!($name)),
            concat!(".type _rphal_", stringify!($name), ", %function"),
            ".align 2",
            concat!("_rphal_", stringify!($name), ":"),
            $(
                concat!(".global ", stringify!($intrinsic)),
                concat!(".type ", stringify!($intrinsic), ", %function"),
                concat!(stringify!($intrinsic), ":"),
            )*

            "hwdivider_head",
            $($begin),+ ,
            "hwdivider_tail",
        );

        #[cfg(all(target_arch = "arm", feature = "disable-intrinsics"))]
        core::arch::global_asm!(
            // Mangle the name slightly, since this is a global symbol.
            concat!(".global _rphal_", stringify!($name)),
            concat!(".type _rphal_", stringify!($name), ", %function"),
            ".align 2",
            concat!("_rphal_", stringify!($name), ":"),

            "hwdivider_head",
            $($begin),+ ,
            "hwdivider_tail",
        );

        #[cfg(target_arch = "arm")]
        extern "aapcs" {
            // Connect a local name to global symbol above through FFI.
            #[link_name = concat!("_rphal_", stringify!($name)) ]
            fn $name(n: $argty, d: $argty) -> u64;
        }

        #[cfg(not(target_arch = "arm"))]
        #[allow(unused_variables)]
        unsafe fn $name(n: $argty, d: $argty) -> u64 { 0 }
    };
}

division_function! {
    unsigned_divmod __aeabi_uidivmod __aeabi_uidiv ( u32 ) {
        "str    r0, [r2, #0x060]", // DIV_UDIVIDEND
        "str    r1, [r2, #0x064]"  // DIV_UDIVISOR
    }
}

division_function! {
    signed_divmod __aeabi_idivmod __aeabi_idiv ( i32 ) {
        "str    r0, [r2, #0x068]", // DIV_SDIVIDEND
        "str    r1, [r2, #0x06c]"  // DIV_SDIVISOR
    }
}

fn divider_unsigned(n: u32, d: u32) -> DivResult<u32> {
    let packed = unsafe { unsigned_divmod(n, d) };
    DivResult {
        quotient: packed as u32,
        remainder: (packed >> 32) as u32,
    }
}

fn divider_signed(n: i32, d: i32) -> DivResult<i32> {
    let packed = unsafe { signed_divmod(n, d) };
    // Double casts to avoid sign extension
    DivResult {
        quotient: packed as u32 as i32,
        remainder: (packed >> 32) as u32 as i32,
    }
}

impl HwDivider {
    /// Perform hardware unsigned divide/modulo operation
    pub fn unsigned(&self, dividend: u32, divisor: u32) -> DivResult<u32> {
        divider_unsigned(dividend, divisor)
    }

    /// Perform hardware signed divide/modulo operation
    pub fn signed(&self, dividend: i32, divisor: i32) -> DivResult<i32> {
        divider_signed(dividend, divisor)
    }
}

intrinsics! {
    extern "C" fn __udivsi3(n: u32, d: u32) -> u32 {
        divider_unsigned(n, d).quotient
    }

    extern "C" fn __umodsi3(n: u32, d: u32) -> u32 {
        divider_unsigned(n, d).remainder
    }

    extern "C" fn __udivmodsi4(n: u32, d: u32, rem: Option<&mut u32>) -> u32 {
        let quo_rem = divider_unsigned(n, d);
        if let Some(rem) = rem {
            *rem = quo_rem.remainder;
        }
        quo_rem.quotient
    }

    extern "C" fn __divsi3(n: i32, d: i32) -> i32 {
        divider_signed(n, d).quotient
    }

    extern "C" fn __modsi3(n: i32, d: i32) -> i32 {
        divider_signed(n, d).remainder
    }

    extern "C" fn __divmodsi4(n: i32, d: i32, rem: &mut i32) -> i32 {
        let quo_rem = divider_signed(n, d);
        *rem = quo_rem.remainder;
        quo_rem.quotient
    }
}

/// This type is just used to limit us to Spinlocks `0..=31`
pub trait SpinlockValid: Sealed {}

/// Hardware based spinlock.
///
/// You can claim this lock by calling either [`claim`], [`try_claim`] or
/// [`claim_async`]. These spin-locks are hardware backed, so if you lock
/// e.g. `Spinlock<6>`, then any other part of your application using
/// `Spinlock<6>` will contend for the same lock, without them needing to
/// share a reference or otherwise communicate with each other.
///
/// When the obtained spinlock goes out of scope, it is automatically unlocked.
///
///
/// ```no_run
/// use rp2040_hal::sio::Spinlock0;
/// static mut SOME_GLOBAL_VAR: u32 = 0;
///
/// /// This function is safe to call from two different cores, but is not safe
/// /// to call from an interrupt routine!
/// fn update_global_var() {
///     // Do not say `let _ = ` here - it will immediately unlock!
///     let _lock = Spinlock0::claim();
///     // Do your thing here that Core 0 and Core 1 might want to do at the
///     // same time, like update this global variable:
///     unsafe { SOME_GLOBAL_VAR += 1 };
///     // The lock is dropped here.
/// }
/// ```
///
/// **Warning**: These spinlocks are not re-entrant, meaning that the
///   following code will cause a deadlock:
///
/// ```no_run
/// use rp2040_hal::sio::Spinlock0;
/// let lock_1 = Spinlock0::claim();
/// let lock_2 = Spinlock0::claim(); // deadlock here
/// ```
///
/// **Note:** The `critical-section` implementation uses Spinlock 31.
///
/// [`claim`]: #method.claim
/// [`try_claim`]: #method.try_claim
/// [`claim_async`]: #method.claim_asyncs
pub struct Spinlock<const N: usize>(core::marker::PhantomData<()>)
where
    Spinlock<N>: SpinlockValid;

impl<const N: usize> Spinlock<N>
where
    Spinlock<N>: SpinlockValid,
{
    /// Try to claim the spinlock. Will return `Some(Self)` if the lock is obtained, and `None` if the lock is
    /// already in use somewhere else.
    pub fn try_claim() -> Option<Self> {
        // Safety: We're only reading from this register
        let sio = unsafe { &*pac::SIO::ptr() };
        let lock = sio.spinlock[N].read().bits();
        if lock > 0 {
            Some(Self(core::marker::PhantomData))
        } else {
            None
        }
    }

    /// Claim the spinlock, will block the current thread until the lock is available.
    ///
    /// Note that calling this multiple times in a row will cause a deadlock
    pub fn claim() -> Self {
        loop {
            if let Some(result) = Self::try_claim() {
                break result;
            }
        }
    }

    /// Try to claim the spinlock. Will return `WouldBlock` until the spinlock is available.
    pub fn claim_async() -> nb::Result<Self, Infallible> {
        Self::try_claim().ok_or(nb::Error::WouldBlock)
    }

    /// Clear a locked spin-lock.
    ///
    /// # Safety
    ///
    /// Only call this function if you hold the spin-lock.
    pub unsafe fn release() {
        let sio = &*pac::SIO::ptr();
        // Write (any value): release the lock
        sio.spinlock[N].write_with_zero(|b| b.bits(1));
    }
}

impl<const N: usize> Drop for Spinlock<N>
where
    Spinlock<N>: SpinlockValid,
{
    fn drop(&mut self) {
        // This is safe because we own the object, and hence hold the lock.
        unsafe { Self::release() }
    }
}

macro_rules! spinlock {
    ($first:expr, $($rest:tt),+) => {
        spinlock!($first);
        spinlock!($($rest),+);
    };
    ($id:expr) => {
        $crate::paste::paste! {
            /// Spinlock number $id
            pub type [<Spinlock $id>] = Spinlock<$id>;
            impl SpinlockValid for Spinlock<$id> {}
            impl Sealed for Spinlock<$id> {}
        }
    };
}
spinlock!(
    0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15, 16, 17, 18, 19, 20, 21, 22, 23, 24, 25,
    26, 27, 28, 29, 30
);

/// Spinlock number 31 - used by critical section implementation
#[cfg(feature = "critical-section-impl")]
pub(crate) type Spinlock31 = Spinlock<31>;

/// Spinlock number 31 - only public if critical-section-impl is not enabled
#[cfg(not(feature = "critical-section-impl"))]
pub type Spinlock31 = Spinlock<31>;

impl SpinlockValid for Spinlock<31> {}
impl Sealed for Spinlock<31> {}

/// Returns the current state of the spinlocks. Each index corresponds to the associated spinlock, e.g. if index `5` is set to `true`, it means that [`Spinlock5`] is currently locked.
///
/// Note that spinlocks can be claimed or released at any point, so this function cannot guarantee the spinlock is actually available right after calling this function. This function is mainly intended for debugging.
pub fn spinlock_state() -> [bool; 32] {
    // Safety: we're only reading from a register
    let sio = unsafe { &*pac::SIO::ptr() };
    // A bitmap containing the state of all 32 spinlocks (1=locked).
    let register = sio.spinlock_st.read().bits();
    let mut result = [false; 32];
    #[allow(clippy::needless_range_loop)]
    for i in 0..32 {
        result[i] = (register & (1 << i)) > 0;
    }
    result
}

/// Free all spinlocks, regardless of their current status
///
/// RP2040 does not release all spinlocks on reset.
/// The C SDK clears these all during entry, and so do we if you call hal::entry!
/// But if someone is using the default cortex-m entry they risk hitting deadlocks so provide *something* to help out
///
/// # Safety
/// Where possible, you should use the hal::entry macro attribute on main instead of this.
/// You should call this as soon as possible after reset - preferably as the first entry in fn main(), before *ANY* use of spinlocks, atomics, or critical_section
pub unsafe fn spinlock_reset() {
    // Using raw pointers to avoid taking peripherals accidently at startup
    const SIO_BASE: u32 = 0xd0000000;
    const SPINLOCK0_PTR: *mut u32 = (SIO_BASE + 0x100) as *mut u32;
    const SPINLOCK_COUNT: usize = 32;
    for i in 0..SPINLOCK_COUNT {
        SPINLOCK0_PTR.wrapping_add(i).write_volatile(1);
    }
}

/// Configuration struct for one lane of the interpolator
pub struct LaneCtrl {
    /// Bit 22 - Only present on INTERP1 on each core. If CLAMP mode is enabled:  
    /// - LANE0 result is shifted and masked ACCUM0, clamped by a lower bound of  
    /// BASE0 and an upper bound of BASE1.  
    /// - Signedness of these comparisons is determined by LANE0_CTRL_SIGNED
    pub clamp: bool,
    /// Bit 21 - Only present on INTERP0 on each core. If BLEND mode is enabled:
    /// - LANE1 result is a linear interpolation between BASE0 and BASE1, controlled
    /// by the 8 LSBs of lane 1 shift and mask value (a fractional number between
    /// 0 and 255/256ths)
    /// - LANE0 result does not have BASE0 added (yields only
    /// the 8 LSBs of lane 1 shift+mask value)
    /// - FULL result does not have lane 1 shift+mask value added (BASE2 + lane 0 shift+mask)
    /// LANE1 SIGNED flag controls whether the interpolation is signed or unsigned.
    pub blend: bool,
    /// Bits 19:20 - ORed into bits 29:28 of the lane result presented to the processor on the bus.  
    /// No effect on the internal 32-bit datapath. Handy for using a lane to generate sequence  
    /// of pointers into flash or SRAM.
    pub force_msb: u8,
    /// Bit 18 - If 1, mask + shift is bypassed for LANE0 result. This does not affect FULL result.
    pub add_raw: bool,
    /// Bit 17 - If 1, feed the opposite lane's result into this lane's accumulator on POP.
    pub cross_result: bool,
    /// Bit 16 - If 1, feed the opposite lane's accumulator into this lane's shift + mask hardware.  
    /// Takes effect even if ADD_RAW is set (the CROSS_INPUT mux is before the shift+mask bypass)
    pub cross_input: bool,
    /// Bit 15 - If SIGNED is set, the shifted and masked accumulator value is sign-extended to 32 bits  
    /// before adding to BASE0, and LANE0 PEEK/POP appear extended to 32 bits when read by processor.
    pub signed: bool,
    /// Bits 10:14 - The most-significant bit allowed to pass by the mask (inclusive)  
    /// Setting MSB < LSB may cause chip to turn inside-out
    pub mask_msb: u8,
    /// Bits 5:9 - The least-significant bit allowed to pass by the mask (inclusive)
    pub mask_lsb: u8,
    /// Bits 0:4 - Logical right-shift applied to accumulator before masking
    pub shift: u8,
}

impl LaneCtrl {
    /// Default configuration. Normal operation, unsigned, mask keeps all bits, no shift.
    pub const fn new() -> Self {
        Self {
            clamp: false,
            blend: false,
            force_msb: 0,
            add_raw: false,
            cross_result: false,
            cross_input: false,
            signed: false,
            mask_msb: 31,
            mask_lsb: 0,
            shift: 0,
        }
    }

    /// encode the configuration to be loaded in the ctrl register of one lane of an interpolator
    pub const fn encode(&self) -> u32 {
        assert!(!(self.blend && self.clamp));
        assert!(self.force_msb < 0b100);
        assert!(self.mask_msb < 0b100000);
        assert!(self.mask_lsb < 0b100000);
        assert!(self.mask_msb >= self.mask_lsb);
        assert!(self.shift < 0b100000);
        ((self.clamp as u32) << 22)
            | ((self.blend as u32) << 21)
            | ((self.force_msb as u32) << 19)
            | ((self.add_raw as u32) << 18)
            | ((self.cross_result as u32) << 17)
            | ((self.cross_input as u32) << 16)
            | ((self.signed as u32) << 15)
            | ((self.mask_msb as u32) << 10)
            | ((self.mask_lsb as u32) << 5)
            | (self.shift as u32)
    }
}

///Trait representing the functionnality of a single lane of an interpolator.
pub trait Lane: Sealed {
    ///Read the lane result, and simultaneously write lane results to both accumulators.
    fn pop(&mut self) -> u32;
    ///Read the lane result without altering any internal state
    fn peek(&self) -> u32;
    ///Write a value to the accumulator
    fn set_accum(&mut self, v: u32);
    ///Read the value from the accumulator
    fn get_accum(&self) -> u32;
    ///Write a value to the base register
    fn set_base(&mut self, v: u32);
    ///Read the value from the base register
    fn get_base(&self) -> u32;
    ///Write to the control register
    fn set_ctrl(&mut self, v: u32);
    ///Read from the control register
    fn get_ctrl(&self) -> u32;
    ///Add the value to the accumulator register
    fn add_accum(&mut self, v: u32);
    ///Read the raw shift and mask value (BASE register not added)
    fn read_raw(&self) -> u32;
}

///Trait representing the functionnality of an interpolator.
/// ```no_run
/// use rp2040_hal::sio::{Sio,LaneCtrl,Lane};
/// use rp2040_hal::pac;
/// let mut peripherals = pac::Peripherals::take().unwrap();
/// let mut sio = Sio::new(peripherals.SIO);
///
/// // by having the configuration const, the validity is checked during compilation.
/// const config: u32 = LaneCtrl {
///     mask_msb: 4, // Most significant bit of the mask is bit 4
///                  // By default the least significant bit is bit 0
///                  // this will keep only the 5 least significant bits.
///                  // this is equivalent to %32
///     ..LaneCtrl::new()
/// }.encode();
/// sio.interp0.get_lane0().set_ctrl(config);
/// sio.interp0.get_lane0().set_accum(0);
/// sio.interp0.get_lane0().set_base(1); // will increment the value by 1 on each call to pop
///
/// sio.interp0.get_lane0().peek(); // returns 1
/// sio.interp0.get_lane0().pop();  // returns 1
/// sio.interp0.get_lane0().pop();  // returns 2
/// sio.interp0.get_lane0().pop();  // returns 3
/// ```
pub trait Interp: Sealed {
    ///Read the interpolator result (Result 2 in the datasheet), and simultaneously write lane results to both accumulators.
    fn pop(&mut self) -> u32;
    ///Read the interpolator result (Result 2 in the datasheet) without altering any internal state
    fn peek(&self) -> u32;
    ///Write to the interpolator Base register (Base2 in the datasheet)
    fn set_base(&mut self, v: u32);
    ///Read the interpolator Base register (Base2 in the datasheet)
    fn get_base(&self) -> u32;
    ///Write the lower 16 bits to BASE0 and the upper bits to BASE1 simultaneously. Each half is sign-extended to 32 bits if that lane's SIGNED flag is set
    fn set_base_1and0(&mut self, v: u32);
}

macro_rules! interpolators {
    (
        $($interp:ident : ( $( [ $lane:ident,$lane_id:expr ] ),+ ) ),+
    ) => {
        $crate::paste::paste! {


                $(
                    $(
                        #[doc = "The lane " $lane_id " of " $interp]
                        pub struct [<$interp $lane>]{
                            _private: (),
                        }
                        impl Lane for [<$interp $lane>]{
                            fn pop(&mut self) ->u32{
                                let sio = unsafe { &*pac::SIO::ptr() };
                                sio.[<$interp:lower _pop_ $lane:lower>].read().bits()
                            }
                            fn peek(&self) ->u32{
                                let sio = unsafe { &*pac::SIO::ptr() };
                                sio.[<$interp:lower _peek_ $lane:lower>].read().bits()
                            }
                            fn set_accum(&mut self,v:u32){
                                let sio = unsafe { &*pac::SIO::ptr() };
                                sio.[<$interp:lower _accum $lane_id>].write(|w| unsafe { w.bits(v) });
                            }
                            fn get_accum(&self)->u32{
                                let sio = unsafe { &*pac::SIO::ptr() };
                                sio.[<$interp:lower _accum $lane_id>].read().bits()
                            }
                            fn set_base(&mut self, v:u32){
                                let sio = unsafe { &*pac::SIO::ptr() };
                                sio.[<$interp:lower _base $lane_id>].write(|w| unsafe { w.bits(v) });
                            }
                            fn get_base(&self)->u32{
                                let sio = unsafe { &*pac::SIO::ptr() };
                                sio.[<$interp:lower _base $lane_id>].read().bits()
                            }
                            fn set_ctrl(&mut self, v:u32){
                                let sio = unsafe { &*pac::SIO::ptr() };
                                sio.[<$interp:lower _ctrl_lane $lane_id>].write(|w| unsafe { w.bits(v) });
                            }
                            fn get_ctrl(&self)->u32{
                                let sio = unsafe { &*pac::SIO::ptr() };
                                sio.[<$interp:lower _ctrl_lane $lane_id>].read().bits()
                            }
                            fn add_accum(&mut self, v:u32){
                                let sio = unsafe { &*pac::SIO::ptr() };
                                sio.[<$interp:lower _accum $lane_id _add>].write(|w| unsafe { w.bits(v) });
                            }
                            fn read_raw(&self)->u32{
                                let sio = unsafe { &*pac::SIO::ptr() };
                                sio.[<$interp:lower _accum $lane_id _add>].read().bits()
                            }
                        }
                        impl Sealed for [<$interp $lane>] {}
                    )+
                    #[doc = "Interpolator " $interp]
                    pub struct $interp {
                        $(
                            [<$lane:lower>]: [<$interp $lane>],
                        )+
                    }
                    impl $interp{
                        $(
                            /// Lane accessor function
                            pub fn [<get_ $lane:lower>](&mut self)->&mut [<$interp $lane>]{
                                &mut self.[<$lane:lower>]
                            }
                        )+
                    }
                    impl Interp for $interp{
                        fn pop(&mut self) ->u32{
                            let sio = unsafe { &*pac::SIO::ptr() };
                            sio.[<$interp:lower _pop_full>].read().bits()
                        }
                        fn peek(&self) ->u32{
                            let sio = unsafe { &*pac::SIO::ptr() };
                            sio.[<$interp:lower _peek_full>].read().bits()
                        }
                        fn set_base(&mut self, v:u32){
                            let sio = unsafe { &*pac::SIO::ptr() };
                            sio.[<$interp:lower _base2>].write(|w| unsafe { w.bits(v)});
                        }
                        fn get_base(&self)->u32{
                            let sio = unsafe { &*pac::SIO::ptr() };
                            sio.[<$interp:lower _base2>].read().bits()
                        }
                        fn set_base_1and0(&mut self, v:u32){
                            let sio = unsafe { &*pac::SIO::ptr() };
                            sio.[<$interp:lower _base_1and0>].write(|w| unsafe { w.bits(v)});
                        }
                    }
                    impl Sealed for $interp {}
                )+
            }
        }
    }

interpolators!(
    Interp0 : ([Lane0,0],[Lane1,1]),
    Interp1 : ([Lane0,0],[Lane1,1])
);
