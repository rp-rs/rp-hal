#![no_std]

#[cfg(feature = "rp2040")]
pub use rp2040_pac as pac;

#[cfg(feature = "rp235x")]
pub use rp235x_pac as pac;

#[cfg(any(feature = "rp2040", feature = "rp235x"))]
pub use pac::Peripherals;

pub mod arch;
pub mod critical_section_impl;
pub mod sio;
pub mod typelevel;

#[doc(hidden)]
pub use paste;

pub use sio::Sio;
