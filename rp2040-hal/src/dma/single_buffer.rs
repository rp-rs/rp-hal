//! Single-buffered or peripheral-peripheral DMA Transfers

use core::sync::atomic::{compiler_fence, Ordering};

use super::{
    single_channel::ChannelConfig, single_channel::SingleChannel, Pace, ReadTarget, WriteTarget,
};

/// Configuration for single-buffered DMA transfer
pub struct Config<CH: SingleChannel, FROM: ReadTarget, TO: WriteTarget> {
    ch: CH,
    from: FROM,
    to: TO,
    pace: Pace,
    bswap: bool,
}

impl<CH, FROM, TO, WORD> Config<CH, FROM, TO>
where
    CH: SingleChannel,
    FROM: ReadTarget<ReceivedWord = WORD>,
    TO: WriteTarget<TransmittedWord = WORD>,
{
    /// Create a new configuration for single-buffered DMA transfer
    pub fn new(ch: CH, from: FROM, to: TO) -> Config<CH, FROM, TO> {
        Config {
            ch,
            from,
            to,
            pace: Pace::PreferSource,
            bswap: false,
        }
    }

    /// Sets the (preferred) pace for the DMA transfers.
    ///
    /// Usually, the code will automatically configure the correct pace, but
    /// peripheral-to-peripheral transfers require the user to manually select whether the source
    /// or the sink shall be queried for the pace signal.
    pub fn pace(&mut self, pace: Pace) {
        self.pace = pace;
    }

    /// Enable/disable byteswapping for the DMA transfers, default value is false.
    ///
    /// For byte data, this has no effect. For halfword data, the two bytes of
    /// each halfword are swapped. For word data, the four bytes of each word
    /// are swapped to reverse order.
    ///
    /// This is a convenient way to change the (half-)words' byte endianness on the fly.
    pub fn bswap(&mut self, bswap: bool) {
        self.bswap = bswap;
    }

    /// Start the DMA transfer
    pub fn start(mut self) -> Transfer<CH, FROM, TO> {
        // TODO: Do we want to call any callbacks to configure source/sink?

        // Make sure that memory contents reflect what the user intended.
        // TODO: How much of the following is necessary?
        cortex_m::asm::dsb();
        compiler_fence(Ordering::SeqCst);

        // Configure the DMA channel and start it.
        self.ch
            .config(&self.from, &mut self.to, self.pace, self.bswap, None, true);

        Transfer {
            ch: self.ch,
            from: self.from,
            to: self.to,
        }
    }
}

// TODO: Drop for most of these structs
/// Instance of a single-buffered DMA transfer
pub struct Transfer<CH: SingleChannel, FROM: ReadTarget, TO: WriteTarget> {
    ch: CH,
    from: FROM,
    to: TO,
}

impl<CH, FROM, TO, WORD> Transfer<CH, FROM, TO>
where
    CH: SingleChannel,
    FROM: ReadTarget<ReceivedWord = WORD>,
    TO: WriteTarget<TransmittedWord = WORD>,
{
    /// Check if an interrupt is pending for this channel and clear the corresponding pending bit
    pub fn check_irq0(&mut self) -> bool {
        self.ch.check_irq0()
    }

    /// Check if an interrupt is pending for this channel and clear the corresponding pending bit
    pub fn check_irq1(&mut self) -> bool {
        self.ch.check_irq1()
    }

    /// Check if the transfer has completed.
    pub fn is_done(&self) -> bool {
        !self.ch.ch().ch_ctrl_trig().read().busy().bit_is_set()
    }

    /// Block until the transfer is complete, returning the channel and targets
    pub fn wait(self) -> (CH, FROM, TO) {
        while !self.is_done() {}

        // Make sure that memory contents reflect what the user intended.
        cortex_m::asm::dsb();
        compiler_fence(Ordering::SeqCst);

        (self.ch, self.from, self.to)
    }

    /// Aborts the current transfer, returning the channel and targets
    pub fn abort(mut self) -> (CH, FROM, TO) {
        let irq0_was_enabled = self.ch.is_enabled_irq0();
        let irq1_was_enabled = self.ch.is_enabled_irq1();
        self.ch.disable_irq0();
        self.ch.disable_irq1();

        let chan_abort = unsafe { &*crate::pac::DMA::ptr() }.chan_abort();
        let abort_mask = (1 << self.ch.id()) as u16;

        chan_abort.write(|w| unsafe { w.chan_abort().bits(abort_mask) });

        while chan_abort.read().chan_abort().bits() != 0 {}

        while !self.is_done() {}

        self.ch.check_irq0();
        self.ch.check_irq1();

        if irq0_was_enabled {
            self.ch.enable_irq0();
        }

        if irq1_was_enabled {
            self.ch.enable_irq1();
        }

        // Make sure that memory contents reflect what the user intended.
        cortex_m::asm::dsb();
        compiler_fence(Ordering::SeqCst);

        (self.ch, self.from, self.to)
    }
}
