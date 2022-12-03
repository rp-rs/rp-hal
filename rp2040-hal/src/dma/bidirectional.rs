use super::{single_channel::SingleChannel, Pace, ReadTarget, WriteTarget};

/// DMA configuration for sending and receiving data simultaneously
pub struct BidirectionalConfig<CH1, CH2, FROM, BIDI, TO>
where
    CH1: SingleChannel,
    CH2: SingleChannel,
    FROM: ReadTarget,
    BIDI: ReadTarget + WriteTarget,
    TO: WriteTarget,
{
    ch: (CH1, CH2),
    from: FROM,
    bidi: BIDI,
    to: TO,
    from_pace: Pace,
    to_pace: Pace,
}

impl<CH1, CH2, FROM, BIDI, TO, WORD> BidirectionalConfig<CH1, CH2, FROM, BIDI, TO>
where
    CH1: SingleChannel,
    CH2: SingleChannel,
    FROM: ReadTarget<ReceivedWord = WORD>,
    BIDI: ReadTarget<ReceivedWord = WORD> + WriteTarget<TransmittedWord = WORD>,
    TO: WriteTarget<TransmittedWord = WORD>,
{
    /// Create a DMA configuration for sending and receiving data simultaneously
    pub fn new(
        ch: (CH1, CH2),
        from: FROM,
        bidi: BIDI,
        to: TO,
    ) -> BidirectionalConfig<CH1, CH2, FROM, BIDI, TO> {
        BidirectionalConfig {
            ch,
            from,
            bidi,
            to,
            from_pace: Pace::PreferSink,
            to_pace: Pace::PreferSink,
        }
    }

    /// Set the transfer pacing for the DMA transfer from the source
    pub fn from_pace(&mut self, pace: Pace) {
        self.from_pace = pace;
    }

    /// Set the transfer pacing for the DMA transfer to the target
    pub fn to_pace(&mut self, pace: Pace) {
        self.to_pace = pace;
    }

    /// Start the DMA transfer
    pub fn start(self) -> Bidirectional<CH1, CH2, FROM, BIDI, TO> {
        // TODO
        let _ch = self.ch;
        let _from = self.from;
        let _bidi = self.bidi;
        let _to = self.to;
        panic!("Not yet implemented.");
    }
}

/// Instance of a bidirectional DMA transfer
pub struct Bidirectional<CH1, CH2, FROM, BIDI, TO>
where
    CH1: SingleChannel,
    CH2: SingleChannel,
    FROM: ReadTarget,
    BIDI: ReadTarget + WriteTarget,
    TO: WriteTarget,
{
    ch: (CH1, CH2),
    from: FROM,
    bidi: BIDI,
    to: TO,
}

impl<CH1, CH2, FROM, BIDI, TO, WORD> Bidirectional<CH1, CH2, FROM, BIDI, TO>
where
    CH1: SingleChannel,
    CH2: SingleChannel,
    FROM: ReadTarget<ReceivedWord = WORD>,
    BIDI: ReadTarget<ReceivedWord = WORD> + WriteTarget<TransmittedWord = WORD>,
    TO: WriteTarget<TransmittedWord = WORD>,
{
    /// Block until transfer is complete
    pub fn wait(self) -> ((CH1, CH2), FROM, BIDI, TO) {
        // TODO
        // Reading self.value to satisfy clippy. Remove once implemented
        let _ch = self.ch;
        let _from = self.from;
        let _bidi = self.bidi;
        let _to = self.to;
        panic!("Not yet implemented.");
    }
}
