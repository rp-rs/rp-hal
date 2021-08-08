//! Semi-internal enums mostly used in typelevel magic

/// Value-level `struct` representing slice IDs
#[derive(PartialEq, Clone, Copy)]
pub struct DynSliceId {
    /// Slice id
    pub num: u8,
}

/// Slice modes
#[derive(PartialEq, Clone, Copy)]
pub enum DynSliceMode {
    /// Count continuously whenever the slice is enabled
    FreeRunning,
    /// Count continuously when a high level is detected on the B pin
    InputHighRunning,
    /// Count once with each rising edge detected on the B pin
    CountRisingEdge,
    /// Count once with each falling edge detected on the B pin
    CountFallingEdge,
}

/// Channel ids
#[derive(PartialEq, Clone, Copy)]
pub enum DynChannelId {
    /// Channel A
    A,
    /// Channel B
    B,
}
