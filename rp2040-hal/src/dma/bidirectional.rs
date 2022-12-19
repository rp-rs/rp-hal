//! Bidirectional DMA transfers

use core::sync::atomic::{compiler_fence, Ordering};

use super::{
    single_channel::{ChannelConfig, SingleChannel},
    Pace, ReadTarget, WriteTarget,
};

/// DMA configuration for sending and receiving data simultaneously
pub struct Config<CH1, CH2, FROM, BIDI, TO>
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

impl<CH1, CH2, FROM, BIDI, TO, WORD> Config<CH1, CH2, FROM, BIDI, TO>
where
    CH1: SingleChannel,
    CH2: SingleChannel,
    FROM: ReadTarget<ReceivedWord = WORD>,
    BIDI: ReadTarget<ReceivedWord = WORD> + WriteTarget<TransmittedWord = WORD>,
    TO: WriteTarget<TransmittedWord = WORD>,
{
    /// Create a DMA configuration for sending and receiving data simultaneously
    pub fn new(ch: (CH1, CH2), from: FROM, bidi: BIDI, to: TO) -> Config<CH1, CH2, FROM, BIDI, TO> {
        Config {
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
    pub fn start(mut self) -> Transfer<CH1, CH2, FROM, BIDI, TO> {
        cortex_m::asm::dsb();
        compiler_fence(Ordering::SeqCst);

        // Configure the DMA channel and start it.
        self.ch
            .0
            .config(&self.from, &mut self.bidi, self.from_pace, None, false);
        self.ch
            .1
            .config(&self.bidi, &mut self.to, self.to_pace, None, false);
        self.ch.0.start_both(&mut self.ch.1);

        Transfer {
            ch: self.ch,
            from: self.from,
            bidi: self.bidi,
            to: self.to,
        }
    }
}

/// Instance of a bidirectional DMA transfer
pub struct Transfer<CH1, CH2, FROM, BIDI, TO>
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

impl<CH1, CH2, FROM, BIDI, TO, WORD> Transfer<CH1, CH2, FROM, BIDI, TO>
where
    CH1: SingleChannel,
    CH2: SingleChannel,
    FROM: ReadTarget<ReceivedWord = WORD>,
    BIDI: ReadTarget<ReceivedWord = WORD> + WriteTarget<TransmittedWord = WORD>,
    TO: WriteTarget<TransmittedWord = WORD>,
{
    /// Check if an interrupt is pending for either channel and clear the corresponding pending bit
    pub fn check_irq0(&mut self) -> bool {
        let a = self.ch.0.check_irq0();
        let b = self.ch.1.check_irq1();
        a | b
    }

    /// Check if an interrupt is pending for either channel and clear the corresponding pending bit
    pub fn check_irq1(&mut self) -> bool {
        let a = self.ch.0.check_irq1();
        let b = self.ch.1.check_irq1();
        a | b
    }

    /// Check if the transfer is completed
    pub fn is_done(&self) -> bool {
        let a = self.ch.1.ch().ch_ctrl_trig.read().busy().bit_is_set();
        let b = self.ch.0.ch().ch_ctrl_trig.read().busy().bit_is_set();
        !(a | b)
    }

    /// Block until transfer is complete
    pub fn wait(self) -> ((CH1, CH2), FROM, BIDI, TO) {
        while !self.is_done() {}

        // Make sure that memory contents reflect what the user intended.
        cortex_m::asm::dsb();
        compiler_fence(Ordering::SeqCst);

        // TODO: Use a tuple type?
        ((self.ch.0, self.ch.1), self.from, self.bidi, self.to)
    }
}
