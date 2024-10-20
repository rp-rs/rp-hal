use paste::paste;

pub(crate) mod pull_sealed {
    use super::DynPullType;

    pub trait PullType {
        fn from(pm: DynPullType) -> Self;
        fn as_dyn(&self) -> DynPullType;
    }
}
/// Type-level `enum` for pull resistor types.
pub trait PullType: pull_sealed::PullType {}

#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
/// Value-level `enum` for pull resistor types.
pub enum DynPullType {
    /// No pull enabled.
    None,
    /// Enable pull up.
    Up,
    /// Enable pull down.
    Down,
    /// This enables bus-keep mode.
    ///
    /// This is not documented in the datasheet but described in the
    /// [c-sdk](https://github.com/raspberrypi/pico-sdk/blob/e7267f99febc70486923e17a8210088af058c915/src/rp2_common/hardware_gpio/gpio.c#L53)
    /// as:
    ///
    /// > […]  on rp235x, setting both pulls enables a "bus keep" function,
    /// > i.e. weak pull to whatever is current high/low state of GPIO.
    BusKeep,
}

impl PullType for DynPullType {}
impl pull_sealed::PullType for DynPullType {
    fn from(pm: DynPullType) -> Self {
        pm
    }

    fn as_dyn(&self) -> DynPullType {
        *self
    }
}

macro_rules! pin_pull_type {
    ($($pull_type:ident),*) => {
        $(paste! {
            /// Type-level `variant` of [`PullType`].
            #[derive(Debug)]
            pub struct [<Pull $pull_type>](pub(super) ());
            impl PullType for [<Pull $pull_type>] {}
            impl pull_sealed::PullType for [<Pull $pull_type>] {
                fn from(_pm: DynPullType) -> Self {
                    Self(())
                }
                fn as_dyn(&self) -> DynPullType {
                    DynPullType::[<$pull_type>]
                }
            }
        })*
    };
}
pin_pull_type!(None, Up, Down, BusKeep);
