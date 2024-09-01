//! Common HAL code
//!
//! This library contains types and functions which are shared between the
//! RP2040 HAL and the RP235x HAL.
//!
//! You shouldn't include anything here which requires either the `cortex-m`
//! crate, or a PAC.

#![no_std]

pub mod uart;
