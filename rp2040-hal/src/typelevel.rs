//! Module supporting type-level programming
//!
//! Copied from [atsamd-hal](https://github.com/atsamd-rs/atsamd).
//!
//! # Introduction
//!
//! Embedded software is often difficult to debug, so there is a strong
//! motivation to catch as many bugs as possible at compile-time. However, the
//! performance requirements of embedded software also make it difficult to
//! justify changes that impose additional overhead in terms of size or speed.
//! Ideally, we would like to add as many compile-time checks as possible, while
//! also producing the fewest possible assembly instructions.
//!
//! The Rust type system can help accomplish this goal. By expressing software
//! constraints within the type system, developers can enforce invariants at
//! compile-time.
//!
//! Sometimes this is done using Rust macros. However, that approach can produce
//! code that is difficult to read and understand. Moreover, macro-generated
//! code can only be extended by *more* macros, which further spreads the
//! problem. In `atsamd-hal` specifically, issue
//! [#214](https://github.com/atsamd-rs/atsamd/issues/214) discussed the extent
//! to which macros were once used in the repository.
//!
//! Alternatively, many of the same goals can be accomplished with the Rust
//! type & trait system directly, which is quite powerful. In fact, it is
//! [turing complete](https://sdleffler.github.io/RustTypeSystemTuringComplete/).
//! By expressing our invariants entirely within the type system, we can encode
//! the desired compile-time checks in a form that is easier to read, understand
//! and document.
//!
//! This module documents some of the type-level programming techniques used
//! throughout this HAL, and it contains a few items used to implement them.
//!
//! ## Contents
//!
//! - [Basics of type-level programming](#basics-of-type-level-programming)
//!     - [Type-level enums](#type-level-enums)
//!     - [Type classes](#type-classes)
//!     - [Type-level containers](#type-level-containers)
//!     - [Type-level functions](#type-level-functions)
//! - [`OptionalKind` trait pattern](#optionalkind-trait-pattern)
//! - [`AnyKind` trait pattern](#anykind-trait-pattern)
//!     - [Defining an `AnyKind` trait](#defining-an-anykind-trait)
//!     - [Using an `AnyKind` trait](#using-an-anykind-trait)
//!
//! # Basics of type-level programming
//!
//! Type-level programming aims to execute a form of compile-time computation.
//! But to perform such computation, we need to map our traditional notions of
//! programming to the Rust type system.
//!
//! In normal Rust, individual values are grouped or categorized into types. For
//! example, `0`, `1`, `2`, etc. are all members of the `usize` type. Similarly,
//! `Enum::A` and `Enum::B` are members of the `Enum` type, defined as
//!
//! ```ignore
//! enum Enum { A, B }
//! ```
//!
//! We use composite types and containers to create more complex data
//! structures, and we use functions to map between values.
//!
//! All of these concepts can also be expressed within the Rust type system.
//! However, in this case, types are grouped and categorized into traits. For
//! instance, the [`typenum`](https://docs.rs/typenum/1.13.0/typenum/index.html)
//! crate provides the types `U0`, `U1`, `U2`, etc., which are all members of
//! the `Unsigned` trait. Similarly, the following sections will illustrate how
//! to define type-level enums, containers and functions.
//!
//! ## Type-level enums
//!
//! Type-level enums are one of the foundational concepts of type-level
//! programming used in this HAL.
//!
//! At the value-level, a typical Rust enum represents some set of variants that
//! can be assigned to a particular variable. Similarly, a type-level enum
//! represents some set of types that can be assigned to a particular type
//! parameter.
//!
//! To lift an enum from the value level to the type level, you typically map
//! the enum variants to types and the enum itself to a trait. For instance, the
//! value-level enum
//!
//! ```ignore
//! enum Enum {
//!     A,
//!     B,
//! }
//! ```
//!
//! would be mapped to the type level like so.
//!
//! ```ignore
//! trait Enum {}
//!
//! enum A {}
//! enum B {}
//!
//! impl Enum for A {}
//! impl Enum for B {}
//! ```
//!
//! At the value level, the variants `A` and `B` are grouped by the `Enum` type,
//! while at the type level, the types `A` and `B` are grouped by the `Enum`
//! trait.
//!
//! ## Type classes
//!
//! At the value-level, a type restricts the possible values that can be taken
//! by some free variable. While at the type-level, a trait bound restricts the
//! possible types that can be taken by some free type parameter. In effect,
//! trait bounds can be used to create a kind of meta-type, or type class. The
//! type-level enums in the previous section represent the most primitive
//! application of the concept, but type classes can take other forms. The
//! `OptionalKind` and `AnyKind` trait patterns discussed below are more
//! advanced applications of the same concept.
//!
//! ## Type-level containers
//!
//! To represent more complex relationships, we need a way to form composite
//! data structures at the type level.
//!
//! At the value level, a container holds an instance of a particular type. The
//! exact value of that instance is usually not known to the author, it is only
//! known at run-time.
//!
//! At the type level, we don't have the same notion of "run-time", but we do
//! have two different notions of "compile-time" that form a similar
//! relationship. There is compile time for the HAL authors, and there is a
//! separate compile-time for the HAL users. We want to create a type-level
//! container where the exact type is not known at author-time, but it is known
//! at user-time.
//!
//! For example, take the following, value-level container struct. It contains
//! two fields, `a` and `b`, of different types, `EnumOne` and `EnumTwo`.
//!
//! ```ignore
//! struct Container {
//!     a: EnumOne,
//!     b: EnumTwo,
//! }
//! ```
//!
//! We can create an instance of this container with specific values.
//!
//! ```ignore
//! let x = Container { a: EnumOne::VariantX, b: EnumTwo::VariantY };
//! ```
//!
//! Next, suppose we had already translated `EnumOne` and `EnumTwo` to the type
//! level using the technique in the previous section. If we wanted to create a
//! similar, composite data structure at the type level, we could use type
//! parameters in place of struct fields to represent the unknown types.
//!
//! ```ignore
//! struct Container<A, B>
//! where
//!     A: EnumOne,
//!     B: EnumTwo,
//! {
//!     a: PhantomData<A>,
//!     b: PhantomData<B>,
//! }
//! ```
//!
//! And we could create an instance of this container with specific types.
//!
//! ```ignore
//! type X = Container<VariantX, VariantY>;
//! ```
//!
//! You might notice the use of `PhantomData` in the definition of the
//! type-level container. Because it is geared more toward value-level
//! programming, Rust requires all type parameters actually be used by the
//! corresponding type. However, we don't need to "store" a type in the same way
//! we store values. The compiler is responsible for tracking the concrete type
//! for each type parameter. But the language still requires us to act as if we
//! used each type parameter. `PhantomData` is the solution here, because it
//! lets us make use of the type parameters without actually storing any values.
//!
//! Separately, `PhantomData` also allows us to create "instances" of types that
//! normally can't be instantiated, like empty enums. For example, instances of
//! `Enum` below can never exist directly.
//!
//! ```ignore
//! enum Enum {}
//! ```
//!
//! But instances of `PhantomData<Enum>` are perfectly valid. In this way,
//! library authors can create types that only exist at the type level, which
//! can sometimes simplify a design.
//!
//! ## Type-level functions
//!
//! To perform type-level computations, we need some way to map or transform
//! types into other types.
//!
//! At the value level, functions and methods map values of the input types to
//! values of the output types. The same can be accomplished at the type level
//! using traits and associated types. Type-level functions are implemented as
//! traits, where the implementing type and any type parameters are the inputs,
//! and associated types are the outputs.
//!
//! For example, consider the value level `not` method below.
//!
//! ```ignore
//! enum Bool {
//!     False,
//!     True,
//! }
//!
//! impl Bool {
//!     fn not(self) -> Self {
//!         use Bool::*;
//!         match self {
//!             True => False,
//!             False => True,
//!         }
//!     }
//! }
//! ```
//!
//! We can translate this example to the type level like so.
//!
//! ```ignore
//! trait Bool {}
//!
//! enum True {}
//! enum False {}
//!
//! impl Bool for True {}
//! impl Bool for False {}
//!
//! trait Not: Bool {
//!     type Result: Bool;
//! }
//!
//! impl Not for True {
//!     type Result = False;
//! }
//!
//! impl Not for False {
//!     type Result = True;
//! }
//! ```
//!
//! We can use the `Not` trait bound to transform one type to another. For
//! instance, we can create a container that accepts one type parameter but
//! stores a different one.
//!
//! ```ignore
//! struct Container<B: Not> {
//!     not: PhantomData<B::Result>;
//! }
//! ```
//!
//! Alternatively, we could redefine the trait and declar a corresponding type
//! alias as
//!
//! ```ignore
//! trait NotFunction: Bool {
//!     type Result: Bool;
//! }
//!
//! type Not<B> = <B as NotFunction>::Result;
//! ```
//!
//! Doing so would allow us to us reframe the last example as
//!
//! ```ignore
//! struct Container<B: NotFunction> {
//!     not: PhantomData<Not<B>>;
//! }
//! ```
//!
//! Type-level functions can be more complicated than this example, but they
//! ultimately represent a mapping from a set of input types (the implementing
//! type and any type parameters) to a set of output types (the associated
//! types).
//!
//! # `OptionalKind` trait pattern
//!
//! As mentioned above, traits can be used to define a kind of meta-type or type
//! class, essentially forming a set of valid types for a given type parameter.
//! They also represent the concept of types lifted from the value level to the
//! type level.
//!
//! What if we want to define a type class representing either a set of useful
//! types or some useless, null type? Essentially, how do we take the notion of
//! an [`Option`] type and raise it to the type level?
//!
//! Suppose we have some existing type class, defined by the `Class` trait, that
//! we want to make optional. We can define a new type class that includes all
//! instances of `Class` as well as some null type. For the latter we use
//! [`NoneT`], defined in this module.
//!
//! ```ignore
//! trait OptionalClass {}
//!
//! impl OptionalClass for NoneT {}
//! impl<C: Class> OptionalClass for C {}
//! ```
//!
//! We can use this new type class to store an optional instance of a `Class`
//! type in a struct.
//!
//! ```ignore
//! struct Container<C: OptionalClass> {
//!     class: PhantomData<C>,
//! }
//! ```
//!
//! And we can restrict some of its methods to only operate on instances with a
//! valid `Class`.
//!
//! ```ignore
//! impl<C: Class> Container<C> {
//!     fn method(self) { ... }
//! }
//! ```
//!
//! Although it is not strictly necessary, we can also introduce a new type
//! class to differentiate the bare usage of `Class` from instances of some
//! `Class` where an `OptionalClass` is accepted.
//!
//! ```ignore
//! trait SomeClass: OptionalClass + Class {}
//!
//! impl<C: Class> SomeClass for C {}
//! ```
//!
//! This new trait doesn't add any new information, but it can still help
//! readers understand that a particular type parameter is restricted to an
//! instances of `Class` when an `OptionalClass` could be accepted.
//!
//! Note that when `Class` and `OptionalClass` contain associated types, name
//! clashes may occur when using `SomeClass` as a trait bound. This can be
//! avoided by removing the `OptionalClass` super trait from `SomeClass`.
//! Ultimately, it is redundant anyway, because any implementer of `Class` also
//! implements `OptionalClass`.
//!
//! # `AnyKind` trait pattern
//!
//! The `AnyKind` trait pattern allows you to encapsulate types with multiple
//! type parameters and represent them with only a single type parameter. It
//! lets you introduce a layer of abstraction, which can simplify interfaces and
//! make them more readable. But most of all, it does so without sacrificing any
//! of our normal, type-level abilities.
//!
//! ## Defining an `AnyKind` trait
//!
//! Suppose you had a composite, type-level data structure. For example, the
//! GPIO `Pin` struct contains instances of two type-level enums, a `PinId` and
//! a `PinMode`. It looks something like this.
//!
//! ```ignore
//! struct Pin<I: PinId, M: PinMode> {
//!     // ...
//! }
//! ```
//!
//! Rust does not provide any way to speak about a `Pin` generally. Any mention
//! of the `Pin` type must also include its type parameters, i.e. `Pin<I, M>`.
//! This is not a deal-breaker, but it is less than ideal for type-level
//! programming. It would be nice if there were a way to succinctly refer to any
//! `Pin`, regardless of its type parameters.
//!
//! We've seen above that we can use traits to form a type class. What if we
//! were to introduce a new trait to label all instances of `Pin`? It would look
//! something like this.
//!
//! ```ignore
//! trait AnyPin {}
//!
//! impl<I: PinId, M: PinMode> AnyPin for Pin<I, M> {}
//! ```
//!
//! Now, instead of refering to `Pin<I, M>`, we can refer to instances of the
//! `AnyPin` type class.
//!
//! ```ignore
//! fn example<P: AnyPin>(pin: P) { ... }
//! ```
//!
//! Unfortunately, while this is more ergonomic, it is not very useful. As
//! authors of the code, we know that `AnyPin` is only implemented for `Pin`
//! types. But the compiler doesn't know that. Traits in Rust are open, so the
//! compiler must consider that `AnyPin` could be implemented for other types.
//!
//! As a consequence, the compiler knows very little about the type `P` in the
//! function above. In fact, because the `AnyPin` trait is completely empty, the
//! compiler knows *absolutely nothing* about the type `P`.
//!
//! Is there a way to make the `AnyPin` trait more useful? We can see from the
//! current implementation that we are throwing away information.
//!
//! ```ignore
//! impl<I: PinId, M: PinMode> AnyPin for Pin<I, M> {}
//! ```
//!
//! The implementation of `AnyPin` is identical for every `Pin`, regardless of
//! the type parameters `I` and `M`, which erases that information. Instead, we
//! could choose to save that information in the form of associated types.
//!
//! Let's redesign the `AnyPin` trait to record the `PinId` and `PinMode`.
//!
//! ```ignore
//! trait AnyPin {
//!     type Id: PinId;
//!     type Mode: PinMode;
//! }
//!
//! impl<I: PinId, M: PinMode> AnyPin for Pin<I, M> {
//!     type Id = I;
//!     type Mode = M;
//! }
//! ```
//!
//! This is better. When `P` implements `AnyPin`, we can at least recover the
//! corresponding `PinId` and `PinMode` types. However, `AnyPin` still doesn't
//! include any trait methods nor any super traits, so the compiler won't allow
//! us to do anything useful with an instances of `P`.
//!
//! We need some way to tell the compiler that when `P` implements `AnyPin`,
//! it is equivalent to saying `P` is exactly `Pin<P::Id, P::Mode>`.
//! Essentially, we want to take a generic type parameter `P` and treat it as if
//! it were an instance of a specific `Pin` type.
//!
//! We can start by defining a trait alias to recover the specific `Pin` type.
//!
//! ```ignore
//! type SpecificPin<P> = Pin<<P as AnyPin>::Id, <P as AnyPin>::Mode>;
//! ```
//!
//! With this new definition, we can rephrase our statement above. We need some
//! way to tell the compiler that when `P` implements `AnyPin`,
//! `P == SpecificPin<P>`. There's no way to do that exactly, but we can come
//! close with some useful trait bounds: [`From`], [`Into`], [`AsRef`] and
//! [`AsMut`].
//!
//! ```ignore
//! trait AnyPin
//! where
//!     Self: From<SpecificPin<Self>>,
//!     Self: Into<SpecificPin<Self>>,
//!     Self: AsRef<SpecificPin<Self>>,
//!     Self: AsMut<SpecificPin<Self>>,
//! {
//!     type Id: PinId;
//!     type Mode: PinMode;
//! }
//! ```
//!
//! Now we've given the compiler some useful information. When a type implements
//! `AnyPin`, it can be converted from and into instances of `Pin`. And
//! references to types that implement `AnyPin` can be converted into references
//! to `Pin`s.
//!
//! ```ignore
//! fn example<P: AnyPin>(mut any_pin: P) {
//!     // None of the type annotations here are necessary
//!     // Everything can be inferred
//!     // Remember that SpecificPin<P> is Pin<P::Id, P::Mode>
//!     let pin_mut: &mut SpecificPin<P> = any_pin.as_mut();
//!     let pin_ref: &SpecificPin<P> = any_pin.as_ref();
//!     let pin: SpecificPin<P> = any_pin.into();
//! }
//! ```
//!
//! Finally, to simplify this pattern, we can gather all of the super trait
//! bounds into a single, reusable trait.
//!
//! ```ignore
//! trait Is
//! where
//!     Self: From<IsType<Self>>,
//!     Self: Into<IsType<Self>>,
//!     Self: AsRef<IsType<Self>>,
//!     Self: AsMut<IsType<Self>>,
//! {
//!     type Type;
//! }
//!
//! type IsType<T> = <T as Is>::Type;
//!
//! impl<T: AsRef<T> + AsMut<T>> Is for T {
//!     type Type = T;
//! }
//! ```
//!
//! And we can rewrite our `AnyPin` trait as
//!
//! ```ignore
//! trait AnyPin: Is<Type = SpecificPin<Self>> {
//!     type Id: PinId;
//!     type Mode: PinMode;
//! }
//! ```
//!
//! ## Using an `AnyKind` trait
//!
//! If a type takes multiple type parameters, storing it within a container
//! requires repeating all of the corresponding type parameters. For instance,
//! imagine a container that stores two completely generic `Pin` types.
//!
//! ```ignore
//! struct TwoPins<I1, I2, M1, M2>
//! where
//!     I1: PinId,
//!     I2: PinId,
//!     M1: PinMode,
//!     M2: PinMode,
//! {
//!     pin1: Pin<I1, M1>,
//!     pin2: Pin<I2, M2>,
//! }
//! ```
//!
//! This struct has already ballooned to four type parameters, without even
//! doing much useful work. Given its heavy use of type parameters, this
//! limitation can make type-level programming tedious, cumbersome and
//! error-prone.
//!
//! Instead, we can use the `AnyKind` trait pattern to encapsulate each `Pin`
//! with a single type parameter.
//!
//! ```ignore
//! struct TwoPins<P1, P2>
//! where
//!     P1: AnyPin,
//!     P2: AnyPin,
//! {
//!     pin1: P1,
//!     pin2: P2,
//! }
//! ```
//!
//! The result is far more readable and generally more comprehensible. Moreover,
//! although we no longer have direct access to the `PinId` and `PinMode` type
//! parameters, we haven't actually lost any expressive power.
//!
//! In the first version of `TwoPins`, suppose we wanted to implement a method
//! for pins in `FloatingInput` mode while simultaneously restricting the
//! possible `PinId`s based on some type class. The result might look like
//! this.
//!
//! ```ignore
//! impl<I1, I2> for TwoPins<I1, I2, FloatingInput, FloatingInput>
//! where
//!     I1: PinId + Class,
//!     I2: PinId + Class,
//! {
//!     fn method(&self) {
//!         // ...
//!     }
//! }
//! ```
//!
//! The same method could be expressed with the `AnyPin` approach like so
//!
//! ```ignore
//! impl<P1, P2> for TwoPins<P1, P2>
//! where
//!     P1: AnyPin<Mode = FloatingInput>,
//!     P2: AnyPin<Mode = FloatingInput>,
//!     P1::Id: Class,
//!     P2::Id: Class,
//! {
//!     fn method(&self) {
//!         // ...
//!     }
//! }
//! ```
//!
//! This example demonstrates the simultaneous readability and expressive power
//! of the `AnyKind` pattern.
//!
//! However, remember that when working with a type `P` that implements
//! `AnyPin`, the compiler can only use what it knows about the `AnyPin` trait.
//! But all of the functionality for GPIO pins is defined on the `Pin` type. To
//! make use of a generic type `P` implementing `AnyPin`, you must first convert
//! it to its corresponding `SpecificPin` using [`Into`], [`AsRef`] or
//! [`AsMut`]. And, in some instances, you may also need to convert back to the
//! type `P`.
//!
//! Suppose you wanted to store a completely generic `Pin` within a struct.
//!
//! ```ignore
//! pub struct Example<P: AnyPin> {
//!     pin: P,
//! }
//! ```
//!
//! Next, suppose you want to create a method that would take the `Pin` out of
//! the struct, perform some operations in different `PinMode`s, and put it back
//! into the struct before returning. The `elided` method below shows such an
//! example. However, it can be a bit tricky to follow all of the type
//! conversions here. For clarity, the `expanded` method shows the same behavior
//! with each transformation given its proper type annotation.
//!
//! ```ignore
//! impl<P: AnyPin> Example<P> {
//!     pub fn elided(mut self) -> Self {
//!         let pin = self.pin.into();
//!         let mut pin = pin.into_push_pull_output();
//!         pin.set_high().ok();
//!         let pin = pin.into_floating_input();
//!         let _bit = pin.is_low().unwrap();
//!         let pin = pin.into_mode();
//!         self.pin = pin.into();
//!         self
//!     }
//!     pub fn expanded(mut self) -> Self {
//!         let pin: SpecificPin<P> = self.pin.into();
//!         let mut pin: Pin<P::Id, PushPullOutput> = pin.into_push_pull_output();
//!         pin.set_high().ok();
//!         let pin: Pin<P::Id, FloatingInput> = pin.into_floating_input();
//!         let _bit = pin.is_low().unwrap();
//!         let pin: SpecificPin<P> = pin.into_mode::<P::Mode>();
//!         self.pin = pin.into();
//!         self
//!     }
//! }
//! ```
//!
//! Notice that it is not enough to simply put back the correct `SpecificPin`.
//! Even though the `SpecificPin` implements
//! `AnyPin<Id = P::Id, Mode = P::Mode>` the compiler doesn't understand that
//! `SpecificPin<P> == P` for all `P`. As far as the compiler is concerned,
//! there could be several different types that implement
//! `AnyPin<Id = P::Id, Mode = P::Mode>`. Instead, the compiler requires that
//! you put back an instance of `P` exactly. The final use of [`Into`] is key
//! here. It transforms the `SpecificPin` back into `P` itself.

mod private {
    /// Super trait used to mark traits with an exhaustive set of
    /// implementations
    pub trait Sealed {}
}

pub(crate) use private::Sealed;

/// Type-level version of the [None] variant
#[derive(Default)]
pub struct NoneT;
impl Sealed for NoneT {}

/// Marker trait for type identity
///
/// This trait is used as part of the [`AnyKind`] trait pattern. It represents
/// the concept of type identity, because all implementors have
/// `<Self as Is>::Type == Self`. When used as a trait bound with a specific
/// type, it guarantees that the corresponding type parameter is exactly the
/// specific type. Stated differently, it guarantees that `T == Specific` in
/// the following example.
///
/// ```ignore
/// where T: Is<Type = Specific>
/// ```
///
/// Moreover, the super traits guarantee that any instance of or reference to a
/// type `T` can be converted into the `Specific` type.
///
/// ```ignore
/// fn example<T>(mut any: T)
/// where
///     T: Is<Type = Specific>,
/// {
///     let specific_mut: &mut Specific = any.as_mut();
///     let specific_ref: &Specific = any.as_ref();
///     let specific: Specific = any.into();
/// }
/// ```
///
/// [`AnyKind`]: #anykind-trait-pattern
pub trait Is
where
    Self: Sealed,
    Self: From<IsType<Self>>,
    Self: Into<IsType<Self>>,
    Self: AsRef<IsType<Self>>,
    Self: AsMut<IsType<Self>>,
{
    #[allow(missing_docs)]
    type Type;
}

/// Type alias for [`Is::Type`]
pub type IsType<T> = <T as Is>::Type;

impl<T> Is for T
where
    T: Sealed + AsRef<T> + AsMut<T>,
{
    type Type = T;
}
