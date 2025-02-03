//! Pin Groups
//!
//! Lets you set multiple GPIOs simultaneously.

use embedded_hal::digital::PinState;
use frunk::{hlist::Plucker, HCons, HNil};

use crate::typelevel::Sealed;

use super::{
    pin::pin_sealed::TypeLevelPinId, AnyPin, FunctionSio, FunctionSioInput, FunctionSioOutput, Pin,
    PinId, PullType, SioConfig,
};

/// Generate a read mask for a pin list.
pub trait ReadPinHList: Sealed {
    /// Generate a mask for a pin list.
    fn read_mask(&self) -> u32;
}
impl ReadPinHList for HNil {
    fn read_mask(&self) -> u32 {
        0
    }
}
impl<H: AnyPin, T: ReadPinHList> ReadPinHList for HCons<H, T> {
    fn read_mask(&self) -> u32 {
        (1 << self.head.borrow().id().num) | self.tail.read_mask()
    }
}

/// Generate a write mask for a pin list.
pub trait WritePinHList: Sealed {
    /// Generate a mask for a pin list.
    fn write_mask(&self) -> u32;
}
impl WritePinHList for HNil {
    fn write_mask(&self) -> u32 {
        0
    }
}
impl<P: PinId, M: PullType, T: WritePinHList> WritePinHList
    for HCons<Pin<P, FunctionSioInput, M>, T>
{
    fn write_mask(&self) -> u32 {
        // This is an input pin, so don't include it in write_mask
        self.tail.write_mask()
    }
}
impl<P: PinId, M: PullType, T: WritePinHList> WritePinHList
    for HCons<Pin<P, FunctionSioOutput, M>, T>
{
    fn write_mask(&self) -> u32 {
        (1 << self.head.id().num) | self.tail.write_mask()
    }
}

/// A group of pins to be controlled together and guaranty single cycle control of several pins.
///
/// ```no_run
/// # macro_rules! defmt { ($($a:tt)*) => {}}
/// use rp235x_hal::{
///     self as hal,
///     gpio::{bank0::Gpio12, Pin, PinGroup, PinState, Pins},
///     sio::Sio,
/// };
///
/// let mut peripherals = hal::pac::Peripherals::take().unwrap();
/// let sio = Sio::new(peripherals.SIO);
/// let pins = Pins::new(
///     peripherals.IO_BANK0,
///     peripherals.PADS_BANK0,
///     sio.gpio_bank0,
///     &mut peripherals.RESETS,
/// );
///
/// let group = PinGroup::new();
/// let group = group.add_pin(pins.gpio0.into_pull_up_input());
/// let mut group = group.add_pin(pins.gpio4.into_push_pull_output_in_state(PinState::High));
///
/// defmt!("Group's state is: {}", group.read());
/// group.toggle();
/// defmt!("Group's state is: {}", group.read());
/// ```
pub struct PinGroup<T = HNil>(T);
impl PinGroup<HNil> {
    /// Creates an empty pin group.
    pub fn new() -> Self {
        PinGroup(HNil)
    }

    /// Add a pin to the group.
    pub fn add_pin<P, C>(self, pin: P) -> PinGroup<HCons<P, HNil>>
    where
        C: SioConfig,
        P: AnyPin<Function = FunctionSio<C>>,
        P::Id: TypeLevelPinId,
    {
        PinGroup(HCons {
            head: pin,
            tail: self.0,
        })
    }
}
impl<T, H> PinGroup<HCons<H, T>>
where
    H::Id: TypeLevelPinId,
    H: AnyPin,
{
    /// Add a pin to the group.
    pub fn add_pin<C, P>(self, pin: P) -> PinGroup<HCons<P, HCons<H, T>>>
    where
        C: SioConfig,
        P: AnyPin<Function = FunctionSio<C>>,
        P::Id: TypeLevelPinId<Bank = <H::Id as TypeLevelPinId>::Bank>,
    {
        PinGroup(HCons {
            head: pin,
            tail: self.0,
        })
    }

    /// Pluck a pin from the group.
    #[allow(clippy::type_complexity)]
    pub fn remove_pin<P, Index>(
        self,
    ) -> (P, PinGroup<<HCons<H, T> as Plucker<P, Index>>::Remainder>)
    where
        HCons<H, T>: Plucker<P, Index>,
    {
        let (p, rest): (P, _) = self.0.pluck();
        (p, PinGroup(rest))
    }
}
impl<H, T> PinGroup<HCons<H, T>>
where
    HCons<H, T>: ReadPinHList + WritePinHList,
    H: AnyPin,
{
    /// Read the whole group at once.
    ///
    /// The returned value is a bit field where each pin populates its own index. Therefore, there
    /// might be "holes" in the value. Unoccupied bits will always read as 0.
    ///
    /// For example, if the group contains Gpio1 and Gpio3, a read may yield:
    /// ```text
    /// 0b0000_0000__0000_0000__0000_0000__0000_1010
    ///                          This is Gpio3  ↑↑↑
    ///                      Gpio2 is not used   ||
    ///                          This is Gpio1    |
    /// ```
    pub fn read(&self) -> u32 {
        let mask = self.0.read_mask();
        crate::sio::Sio::read_bank0() & mask
    }

    /// Write this set of pins all at the same time.
    ///
    /// This only affects output pins. Input pins in the
    /// set are ignored.
    pub fn set(&mut self, state: PinState) {
        use super::pin::pin_sealed::PinIdOps;
        let mask = self.0.write_mask();
        let head_id = self.0.head.borrow().id();
        if state == PinState::Low {
            head_id.sio_out_clr().write(|w| unsafe { w.bits(mask) });
        } else {
            head_id.sio_out_set().write(|w| unsafe { w.bits(mask) });
        }
    }

    /// Set this set of pins to the state given in a single operation.
    ///
    /// The state passed in must be a mask where each bit corresponds to a gpio.
    ///
    /// For example, if the group contains Gpio1 and Gpio3, a read may yield:
    /// ```text
    /// 0b0000_0000__0000_0000__0000_0000__0000_1010
    ///                          This is Gpio3  ↑↑↑
    ///                      Gpio2 is not used   ||
    ///                          This is Gpio1    |
    /// ```
    ///
    /// State corresponding to bins not in this group are ignored.
    pub fn set_u32(&mut self, state: u32) {
        use super::pin::pin_sealed::PinIdOps;
        let mask = self.0.write_mask();
        let state_masked = mask & state;
        let head_id = self.0.head.borrow().id();
        // UNSAFE: this register is 32bit wide and all bits are valid.
        // The value set is masked
        head_id.sio_out().modify(|r, w| unsafe {
            // clear all bit part of this group
            let cleared = r.bits() & !mask;
            // set bits according to state
            w.bits(cleared | state_masked)
        });
    }

    /// Toggles this set of pins all at the same time.
    ///
    /// This only affects output pins. Input pins in the
    /// set are ignored.
    pub fn toggle(&mut self) {
        use super::pin::pin_sealed::PinIdOps;
        let mask = self.0.write_mask();
        self.0
            .head
            .borrow()
            .id()
            .sio_out_xor()
            .write(|w| unsafe { w.bits(mask) });
    }
}
impl Default for PinGroup<HNil> {
    fn default() -> Self {
        Self::new()
    }
}
