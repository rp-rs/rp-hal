//! Single Cycle Input and Output (SIO)
//!
//! To be able to partition parts of the SIO block to other modules:
//!
//! ```no_run
//! use rp235x_hal::{self as hal, gpio::Pins, sio::Sio};
//!
//! let mut peripherals = hal::pac::Peripherals::take().unwrap();
//! let sio = Sio::new(peripherals.SIO);
//! ```
//!
//! And then for example
//!
//! ```no_run
//! # use rp235x_hal::{self as hal, gpio::Pins, sio::Sio};
//! # let mut peripherals = hal::pac::Peripherals::take().unwrap();
//! # let sio = Sio::new(peripherals.SIO);
//! let pins = Pins::new(
//!     peripherals.IO_BANK0,
//!     peripherals.PADS_BANK0,
//!     sio.gpio_bank0,
//!     &mut peripherals.RESETS,
//! );
//! ```

use crate::typelevel::Sealed;

use super::*;
use core::convert::Infallible;

/// Id of the core.
#[repr(u8)]
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum CoreId {
    #[allow(missing_docs)]
    Core0 = 0,
    #[allow(missing_docs)]
    Core1 = 1,
}

/// Marker struct for ownership of SIO gpio bank0
#[derive(Debug)]
pub struct SioGpioBank0 {
    _private: (),
}

/// Marker struct for ownership of SIO FIFO
#[derive(Debug)]
pub struct SioFifo {
    _private: (),
}

/// Marker struct for ownership of SIO gpio qspi
#[derive(Debug)]
pub struct SioGpioQspi {
    _private: (),
}

/// Struct containing ownership markers for managing ownership of the SIO registers.
pub struct Sio {
    _sio: pac::SIO,
    /// GPIO Bank 0 registers
    pub gpio_bank0: SioGpioBank0,
    /// GPIO QSPI registers
    pub gpio_qspi: SioGpioQspi,
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

    /// Reads the whole bank0 at once.
    pub fn read_bank0() -> u32 {
        unsafe { (*pac::SIO::PTR).gpio_in().read().bits() }
    }

    /// Returns whether we are running on Core 0 (`0`) or Core 1 (`1`).
    pub fn core() -> CoreId {
        // Safety: it is always safe to read this read-only register
        match unsafe { (*pac::SIO::ptr()).cpuid().read().bits() as u8 } {
            0 => CoreId::Core0,
            1 => CoreId::Core1,
            _ => unreachable!("This MCU only has 2 cores."),
        }
    }
}

impl SioFifo {
    /// Check if the inter-core FIFO has valid data for reading.
    ///
    /// Returning `true` means there is valid data, `false` means it is empty
    /// and you must not read from it.
    pub fn is_read_ready(&mut self) -> bool {
        let sio = unsafe { &(*pac::SIO::ptr()) };
        sio.fifo_st().read().vld().bit_is_set()
    }

    /// Check if the inter-core FIFO is ready to receive data.
    ///
    /// Returning `true` means there is room, `false` means it is full and you
    /// must not write to it.
    pub fn is_write_ready(&mut self) -> bool {
        let sio = unsafe { &(*pac::SIO::ptr()) };
        sio.fifo_st().read().rdy().bit_is_set()
    }

    /// Return the FIFO status, as an integer.
    pub fn status(&self) -> u32 {
        let sio = unsafe { &(*pac::SIO::ptr()) };
        sio.fifo_st().read().bits()
    }

    /// Write to the inter-core FIFO.
    ///
    /// You must ensure the FIFO has space by calling `is_write_ready`
    pub fn write(&mut self, value: u32) {
        let sio = unsafe { &(*pac::SIO::ptr()) };
        sio.fifo_wr().write(|w| unsafe { w.bits(value) });
        // Fire off an event to the other core.
        // This is required as the other core may be `wfe` (waiting for event)
        crate::arch::sev();
    }

    /// Read from the inter-core FIFO.
    ///
    /// Will return `Some(data)`, or `None` if the FIFO is empty.
    pub fn read(&mut self) -> Option<u32> {
        if self.is_read_ready() {
            let sio = unsafe { &(*pac::SIO::ptr()) };
            Some(sio.fifo_rd().read().bits())
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
            crate::arch::nop();
        }

        // Write the value to the FIFO - the other core will now be able to
        // pop it off its end of the FIFO.
        self.write(value);

        // Fire off an event to the other core
        crate::arch::sev();
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
                crate::arch::wfe();
            }
        }
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
/// use rp235x_hal::sio::Spinlock0;
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
/// use rp235x_hal::sio::Spinlock0;
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
        let lock = sio.spinlock(N).read().bits();
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
        sio.spinlock(N).write_with_zero(|b| b.bits(1));
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
    let register = sio.spinlock_st().read().bits();
    let mut result = [false; 32];
    #[allow(clippy::needless_range_loop)]
    for i in 0..32 {
        result[i] = (register & (1 << i)) > 0;
    }
    result
}

/// Free all spinlocks, regardless of their current status
///
/// rp235x does not release all spinlocks on reset.
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
    ///   BASE0 and an upper bound of BASE1.  
    /// - Signedness of these comparisons is determined by LANE0_CTRL_SIGNED
    pub clamp: bool,
    /// Bit 21 - Only present on INTERP0 on each core. If BLEND mode is enabled:
    ///
    /// - LANE1 result is a linear interpolation between BASE0 and BASE1, controlled
    ///   by the 8 LSBs of lane 1 shift and mask value (a fractional number between
    ///   0 and 255/256ths)
    /// - LANE0 result does not have BASE0 added (yields only
    ///   the 8 LSBs of lane 1 shift+mask value)
    /// - FULL result does not have lane 1 shift+mask value added (BASE2 + lane 0 shift+mask)
    ///
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

impl Default for LaneCtrl {
    fn default() -> Self {
        Self::new()
    }
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

///Trait representing the functionality of a single lane of an interpolator.
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

///Trait representing the functionality of an interpolator.
/// ```no_run
/// use rp235x_hal::{
///     self as hal,
///     sio::{Lane, LaneCtrl, Sio},
/// };
/// let mut peripherals = hal::pac::Peripherals::take().unwrap();
/// let mut sio = Sio::new(peripherals.SIO);
///
/// // by having the configuration const, the validity is checked during compilation.
/// const config: u32 = LaneCtrl {
///     mask_msb: 4, // Most significant bit of the mask is bit 4
///     // By default the least significant bit is bit 0
///     // this will keep only the 5 least significant bits.
///     // this is equivalent to %32
///     ..LaneCtrl::new()
/// }
/// .encode();
/// sio.interp0.get_lane0().set_ctrl(config);
/// sio.interp0.get_lane0().set_accum(0);
/// sio.interp0.get_lane0().set_base(1); // will increment the value by 1 on each call to pop
///
/// sio.interp0.get_lane0().peek(); // returns 1
/// sio.interp0.get_lane0().pop(); // returns 1
/// sio.interp0.get_lane0().pop(); // returns 2
/// sio.interp0.get_lane0().pop(); // returns 3
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
                                sio.[<$interp:lower _pop_ $lane:lower>]().read().bits()
                            }
                            fn peek(&self) ->u32{
                                let sio = unsafe { &*pac::SIO::ptr() };
                                sio.[<$interp:lower _peek_ $lane:lower>]().read().bits()
                            }
                            fn set_accum(&mut self,v:u32){
                                let sio = unsafe { &*pac::SIO::ptr() };
                                sio.[<$interp:lower _accum $lane_id>]().write(|w| unsafe { w.bits(v) });
                            }
                            fn get_accum(&self)->u32{
                                let sio = unsafe { &*pac::SIO::ptr() };
                                sio.[<$interp:lower _accum $lane_id>]().read().bits()
                            }
                            fn set_base(&mut self, v:u32){
                                let sio = unsafe { &*pac::SIO::ptr() };
                                sio.[<$interp:lower _base $lane_id>]().write(|w| unsafe { w.bits(v) });
                            }
                            fn get_base(&self)->u32{
                                let sio = unsafe { &*pac::SIO::ptr() };
                                sio.[<$interp:lower _base $lane_id>]().read().bits()
                            }
                            fn set_ctrl(&mut self, v:u32){
                                let sio = unsafe { &*pac::SIO::ptr() };
                                sio.[<$interp:lower _ctrl_lane $lane_id>]().write(|w| unsafe { w.bits(v) });
                            }
                            fn get_ctrl(&self)->u32{
                                let sio = unsafe { &*pac::SIO::ptr() };
                                sio.[<$interp:lower _ctrl_lane $lane_id>]().read().bits()
                            }
                            fn add_accum(&mut self, v:u32){
                                let sio = unsafe { &*pac::SIO::ptr() };
                                sio.[<$interp:lower _accum $lane_id _add>]().write(|w| unsafe { w.bits(v) });
                            }
                            fn read_raw(&self)->u32{
                                let sio = unsafe { &*pac::SIO::ptr() };
                                sio.[<$interp:lower _accum $lane_id _add>]().read().bits()
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
                            sio.[<$interp:lower _pop_full>]().read().bits()
                        }
                        fn peek(&self) ->u32{
                            let sio = unsafe { &*pac::SIO::ptr() };
                            sio.[<$interp:lower _peek_full>]().read().bits()
                        }
                        fn set_base(&mut self, v:u32){
                            let sio = unsafe { &*pac::SIO::ptr() };
                            sio.[<$interp:lower _base2>]().write(|w| unsafe { w.bits(v)});
                        }
                        fn get_base(&self)->u32{
                            let sio = unsafe { &*pac::SIO::ptr() };
                            sio.[<$interp:lower _base2>]().read().bits()
                        }
                        fn set_base_1and0(&mut self, v:u32){
                            let sio = unsafe { &*pac::SIO::ptr() };
                            sio.[<$interp:lower _base_1and0>]().write(|w| unsafe { w.bits(v)});
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
