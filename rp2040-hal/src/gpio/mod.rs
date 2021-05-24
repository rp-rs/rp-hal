//! General Purpose Input and Output (GPIO)
//!
//! See [`pin`](self::pin) for documentation.

// Based heavily on and in some places copied from `atsamd-hal` gpio::v2
pub mod pin;
pub use pin::*;

pub mod dynpin;
pub use dynpin::*;

mod reg;

#[derive(Clone, Copy, Eq, PartialEq, Debug)]
/// The amount of current that a pin can drive when used as an output
pub enum OutputDriveStrength {
    /// 2 mA
    TwoMilliAmps,
    /// 4 mA
    FourMilliAmps,
    /// 8 mA
    EightMilliAmps,
    /// 12 mA
    TwelveMilliAmps,
}

#[derive(Clone, Copy, Eq, PartialEq, Debug)]
/// The slew rate of a pin when used as an output
pub enum OutputSlewRate {
    /// Slew slow
    Slow,
    /// Slew fast
    Fast,
}
