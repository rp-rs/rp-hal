//! Defines a wrapper for the GPIO pins that can route external clocks into the RP2040.
//!
//! See [2.15.2.3. External Clocks](https://datasheets.raspberrypi.org/rp2040/rp2040-datasheet.pdf) for more details.
//! Or see [examples/gpin.rs](https://github.com/rp-rs/rp-hal/tree/main/rp2040-hal-examples/src/bin/gpin.rs) for a practical example

use fugit::HertzU32;

use crate::{
    gpio::{
        bank0::{Gpio20, Gpio22},
        FunctionClock, Pin, PullNone, PullType,
    },
    typelevel::Sealed,
};

macro_rules! gpin {
    ($id:ident, $pin:ident) => {
        /// A gpin pin: a pin that can be used as a clock input.
        pub struct $id<M = PullNone>
        where
            M: PullType,
        {
            pin: Pin<$pin, FunctionClock, M>,
            frequency: HertzU32,
        }

        impl<M: PullType> $id<M> {
            #[doc = concat!("Creates a new ", stringify!($id), " given the input pin.")]
            pub fn new(pin: Pin<$pin, FunctionClock, M>, frequency: HertzU32) -> Self {
                Self { pin, frequency }
            }

            /// Set the frequency of the externally applied clock signal.
            /// This frequency is used when computing clock dividers.
            pub fn set_frequency(mut self, frequency: HertzU32) -> Self {
                self.frequency = frequency;
                self
            }

            /// Retrieve frequency
            pub fn frequency(&self) -> HertzU32 {
                self.frequency
            }

            #[doc = concat!("Release the underlying device and ", stringify!($pin), ".")]
            pub fn free(self) -> Pin<$pin, FunctionClock, M> {
                self.pin
            }
        }

        impl<M: PullType> Sealed for $id<M> {}
    };
}

gpin!(GpIn0, Gpio20);
gpin!(GpIn1, Gpio22);
