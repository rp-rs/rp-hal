use paste::paste;

pub(crate) mod pull_sealed {
    use super::DynPullType;

    pub trait PullType {
        fn from(pm: DynPullType) -> Self;
        fn as_dyn(&self) -> DynPullType;
    }
}
/// Type-level `enum` for pull resitor types.
pub trait PullType: pull_sealed::PullType {}

#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
/// Value-level `enum` for pull resistor types.
pub enum DynPullType {
    #[allow(missing_docs)]
    None,
    #[allow(missing_docs)]
    Up,
    #[allow(missing_docs)]
    Down,
    /// This enables both pull resistor. This setup is described as bus-keep in the RP2040
    /// datasheet.
    Both,
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
pin_pull_type!(None, Up, Down, Both);
