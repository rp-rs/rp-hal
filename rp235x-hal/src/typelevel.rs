//! Module supporting type-level programming
//!
//! This is heavily inspired by the work in [`atsamd-rs`](https://github.com/atsamd-rs/atsamd). Please refer to the
//! [documentation](https://docs.rs/atsamd-hal/0.15.1/atsamd_hal/typelevel/index.html)
//! over there for more details.

mod private {
    /// Super trait used to mark traits with an exhaustive set of
    /// implementations
    pub trait Sealed {}
}

use core::borrow::{Borrow, BorrowMut};

pub(crate) use private::Sealed;

impl<A: Sealed, B: Sealed> Sealed for (A, B) {}
impl<A: Sealed, B: Sealed, C: Sealed> Sealed for (A, B, C) {}
impl<A: Sealed, B: Sealed, C: Sealed, D: Sealed> Sealed for (A, B, C, D) {}

impl Sealed for frunk::HNil {}
impl<H: Sealed, T: Sealed> Sealed for frunk::HCons<H, T> {}

/// Marker trait for type identity
///
/// This trait is used as part of the [`AnyKind`] trait pattern. It represents
/// the concept of type identity, because all implementors have
/// `<Self as Is>::Type == Self`. When used as a trait bound with a specific
/// type, it guarantees that the corresponding type parameter is exactly the
/// specific type. Stated differently, it guarantees that `T == Specific` in
/// the following example.
///
/// ```text
/// where T: Is<Type = Specific>
/// ```
///
/// Moreover, the super traits guarantee that any instance of or reference to a
/// type `T` can be converted into the `Specific` type.
///
/// ```
/// # use rp235x_hal::typelevel::Is;
/// # struct Specific;
/// fn example<T>(mut any: T)
/// where
///     T: Is<Type = Specific>,
/// {
///     let specific_mut: &mut Specific = any.borrow_mut();
///     let specific_ref: &Specific = any.borrow();
///     let specific: Specific = any.into();
/// }
/// ```
///
/// [`AnyKind`]: https://docs.rs/atsamd-hal/0.15.1/atsamd_hal/typelevel/index.html#anykind-trait-pattern
pub trait Is
where
    Self: Sealed,
    Self: From<IsType<Self>>,
    Self: Into<IsType<Self>>,
    Self: Borrow<IsType<Self>>,
    Self: BorrowMut<IsType<Self>>,
{
    #[allow(missing_docs)]
    type Type;
}

/// Type alias for [`Is::Type`]
pub type IsType<T> = <T as Is>::Type;

impl<T> Is for T
where
    T: Sealed + Borrow<T> + BorrowMut<T>,
{
    type Type = T;
}

// =====================
// Type level option
// =====================

/// Type-level `enum` for Option.
pub trait OptionT: Sealed {
    /// Is this Some or None ?
    const IS_SOME: bool;
}

/// Type-level variant for `OptionT`
pub struct OptionTNone;
impl Sealed for OptionTNone {}
impl OptionT for OptionTNone {
    const IS_SOME: bool = false;
}

/// Type-level variant for `OptionT`
pub struct OptionTSome<T>(pub T);
impl<T> Sealed for OptionTSome<T> {}
impl<T> OptionT for OptionTSome<T> {
    const IS_SOME: bool = true;
}
