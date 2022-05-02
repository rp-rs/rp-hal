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
    // we can hand out other things here, for example:
    // interp0
    // interp1
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
        self.write(value as u32);

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
pub trait SpinlockValid {}

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

/// Spinlock number 0
pub type Spinlock0 = Spinlock<0>;

impl SpinlockValid for Spinlock<0> {}

/// Spinlock number 1
pub type Spinlock1 = Spinlock<1>;

impl SpinlockValid for Spinlock<1> {}

/// Spinlock number 2
pub type Spinlock2 = Spinlock<2>;

impl SpinlockValid for Spinlock<2> {}

/// Spinlock number 3
pub type Spinlock3 = Spinlock<3>;

impl SpinlockValid for Spinlock<3> {}

/// Spinlock number 4
pub type Spinlock4 = Spinlock<4>;

impl SpinlockValid for Spinlock<4> {}

/// Spinlock number 5
pub type Spinlock5 = Spinlock<5>;

impl SpinlockValid for Spinlock<5> {}

/// Spinlock number 6
pub type Spinlock6 = Spinlock<6>;

impl SpinlockValid for Spinlock<6> {}

/// Spinlock number 7
pub type Spinlock7 = Spinlock<7>;

impl SpinlockValid for Spinlock<7> {}

/// Spinlock number 8
pub type Spinlock8 = Spinlock<8>;

impl SpinlockValid for Spinlock<8> {}

/// Spinlock number 9
pub type Spinlock9 = Spinlock<9>;

impl SpinlockValid for Spinlock<9> {}

/// Spinlock number 10
pub type Spinlock10 = Spinlock<10>;

impl SpinlockValid for Spinlock<10> {}

/// Spinlock number 11
pub type Spinlock11 = Spinlock<11>;

impl SpinlockValid for Spinlock<11> {}

/// Spinlock number 12
pub type Spinlock12 = Spinlock<12>;

impl SpinlockValid for Spinlock<12> {}

/// Spinlock number 13
pub type Spinlock13 = Spinlock<13>;

impl SpinlockValid for Spinlock<13> {}

/// Spinlock number 14
pub type Spinlock14 = Spinlock<14>;

impl SpinlockValid for Spinlock<14> {}

/// Spinlock number 15
pub type Spinlock15 = Spinlock<15>;

impl SpinlockValid for Spinlock<15> {}

/// Spinlock number 16
pub type Spinlock16 = Spinlock<16>;

impl SpinlockValid for Spinlock<16> {}

/// Spinlock number 17
pub type Spinlock17 = Spinlock<17>;

impl SpinlockValid for Spinlock<17> {}

/// Spinlock number 18
pub type Spinlock18 = Spinlock<18>;

impl SpinlockValid for Spinlock<18> {}

/// Spinlock number 19
pub type Spinlock19 = Spinlock<19>;

impl SpinlockValid for Spinlock<19> {}

/// Spinlock number 20
pub type Spinlock20 = Spinlock<20>;

impl SpinlockValid for Spinlock<20> {}

/// Spinlock number 21
pub type Spinlock21 = Spinlock<21>;

impl SpinlockValid for Spinlock<21> {}

/// Spinlock number 22
pub type Spinlock22 = Spinlock<22>;

impl SpinlockValid for Spinlock<22> {}

/// Spinlock number 23
pub type Spinlock23 = Spinlock<23>;

impl SpinlockValid for Spinlock<23> {}

/// Spinlock number 24
pub type Spinlock24 = Spinlock<24>;

impl SpinlockValid for Spinlock<24> {}

/// Spinlock number 25
pub type Spinlock25 = Spinlock<25>;

impl SpinlockValid for Spinlock<25> {}

/// Spinlock number 26
pub type Spinlock26 = Spinlock<26>;

impl SpinlockValid for Spinlock<26> {}

/// Spinlock number 27
pub type Spinlock27 = Spinlock<27>;

impl SpinlockValid for Spinlock<27> {}

/// Spinlock number 28
pub type Spinlock28 = Spinlock<28>;

impl SpinlockValid for Spinlock<28> {}

/// Spinlock number 29
pub type Spinlock29 = Spinlock<29>;

impl SpinlockValid for Spinlock<29> {}

/// Spinlock number 30
pub type Spinlock30 = Spinlock<30>;

impl SpinlockValid for Spinlock<30> {}

/// Spinlock number 31 - used by critical section implementation
pub(crate) type Spinlock31 = Spinlock<31>;

impl SpinlockValid for Spinlock<31> {}

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
