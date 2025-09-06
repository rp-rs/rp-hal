//! Common HAL code
//!
//! This library contains types and functions which are shared between
//! rp2040-hal and the rp235x-hal.
//!
//! You shouldn't include anything here which requires either the `cortex-m`
//! crate, or a PAC.

#![no_std]

pub mod uart;
