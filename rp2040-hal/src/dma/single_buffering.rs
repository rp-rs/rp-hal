use core::sync::atomic::{compiler_fence, Ordering};

use super::{
    single_channel::ChannelConfig, single_channel::SingleChannel, Pace, ReadTarget, WriteTarget,
};

/// Configuration for single-buffered DMA transfer
pub struct SingleBufferingConfig<CH: SingleChannel, FROM: ReadTarget, TO: WriteTarget> {
    ch: CH,
    from: FROM,
    to: TO,
    pace: Pace,
}

impl<CH, FROM, TO, WORD> SingleBufferingConfig<CH, FROM, TO>
where
    CH: SingleChannel,
    FROM: ReadTarget<ReceivedWord = WORD>,
    TO: WriteTarget<TransmittedWord = WORD>,
{
    /// Create a new configuration for single-buffered DMA transfer
    pub fn new(ch: CH, from: FROM, to: TO) -> SingleBufferingConfig<CH, FROM, TO> {
        SingleBufferingConfig {
            ch,
            from,
            to,
            pace: Pace::PreferSource,
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

    /// Start the DMA transfer
    pub fn start(mut self) -> SingleBuffering<CH, FROM, TO> {
        // TODO: Do we want to call any callbacks to configure source/sink?

        // Make sure that memory contents reflect what the user intended.
        // TODO: How much of the following is necessary?
        cortex_m::asm::dsb();
        compiler_fence(Ordering::SeqCst);

        // Configure the DMA channel and start it.
        self.ch
            .config(&self.from, &mut self.to, self.pace, None, true);

        SingleBuffering {
            ch: self.ch,
            from: self.from,
            to: self.to,
        }
    }
}

// TODO: Drop for most of these structs
/// Instance of a single-buffered DMA transfer
pub struct SingleBuffering<CH: SingleChannel, FROM: ReadTarget, TO: WriteTarget> {
    ch: CH,
    from: FROM,
    to: TO,
}

impl<CH, FROM, TO, WORD> SingleBuffering<CH, FROM, TO>
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
        !self.ch.ch().ch_ctrl_trig.read().busy().bit_is_set()
    }

    /// Block until the transfer is complete, returning the channel and targets
    pub fn wait(self) -> (CH, FROM, TO) {
        while !self.is_done() {}

        // Make sure that memory contents reflect what the user intended.
        cortex_m::asm::dsb();
        compiler_fence(Ordering::SeqCst);

        (self.ch, self.from, self.to)
    }
}

/* impl<WORD, CH, FROM, TO> SingleBufferingConfig<CH, FROM, TO>
where
    CH: ChannelPair,
    FROM: ReadTarget<ReceivedWord = WORD>,
    TO: WriteTarget<TransmittedWord = WORD>,
{
    pub fn start_ring(mut self) -> Result<RingTransfer<CH, FROM, TO>, DMAError> {
        // TODO: Fix the code for src_count/dest_count == 1.
        let (src, src_count) = self.from.rx_address_count();
        let src_incr = self.from.rx_increment();
        let dest_incr = self.to.tx_increment();
        let (dest, dest_count) = self.to.tx_address_count();
        let transfer_size = mem::size_of::<WORD>() as u32;

        // Buffers with increment need to be aligned.
        if src_incr && !Self::check_alignment(src, src_count * transfer_size) {
            return Err(DMAError::Alignment);
        }
        if dest_incr && !Self::check_alignment(dest, dest_count * transfer_size) {
            return Err(DMAError::Alignment);
        }
        // Memory-to-memory copies are not supported.
        if src_incr && dest_incr {
            return Err(DMAError::IllegalConfig);
        }
        // Additional sanity checks - we have to completely transfer all memory buffers.
        if src_incr && src_count > dest_count {
            return Err(DMAError::IllegalConfig);
        }
        if dest_incr && dest_count > src_count {
            return Err(DMAError::IllegalConfig);
        }

        const TREQ_UNPACED: u8 = 0x3f;
        let treq = match self.pace {
            Pace::PreferSource => FROM::rx_treq().or(TO::tx_treq()).unwrap_or(TREQ_UNPACED),
            Pace::PreferSink => TO::tx_treq().or(FROM::rx_treq()).unwrap_or(TREQ_UNPACED),
        };

        // We split buffers in two halves for the two DMA channels.
        let (src1, src2) = if src_incr {
            (src, src + src_count * transfer_size)
        } else {
            (src, src)
        };
        let (dest1, dest2, wrap_dest) = if src_incr {
            (dest, dest + dest_count * transfer_size, true)
        } else {
            (dest, dest, false)
        };
        let len = u32::min(src_count, dest_count);
        let ring_size = if src_incr || dest_incr {
            (len * transfer_size / 2).trailing_zeros() as u8
        } else {
            0
        };
        // Configure the second DMA channel - we do not write any trigger register as we do not
        // want to immediately start the transfer.
        self.ch
            .ch2()
            .ch_read_addr
            .write(|w| unsafe { w.bits(src2) });
        self.ch
            .ch2()
            .ch_trans_count
            .write(|w| unsafe { w.bits(len / 2) });
        self.ch
            .ch2()
            .ch_write_addr
            .write(|w| unsafe { w.bits(dest2) });
        self.ch.ch2().ch_al1_ctrl.write(|w| unsafe {
            w.data_size()
                .bits(mem::size_of::<WORD>() as u8 >> 1)
                .incr_read()
                .bit(src_incr)
                .incr_write()
                .bit(dest_incr)
                .treq_sel()
                .bits(treq)
                .en()
                .bit(true)
                .chain_to()
                .bits(self.ch.id())
                .ring_sel()
                .bit(wrap_dest)
                .ring_size()
                .bits(ring_size)
        });
        // Configure the first DMA channel - the access to CTRL_TRIG starts the transfer.
        self.ch.ch().ch_read_addr.write(|w| unsafe { w.bits(src1) });
        self.ch
            .ch()
            .ch_trans_count
            .write(|w| unsafe { w.bits(len / 2) });
        self.ch
            .ch()
            .ch_write_addr
            .write(|w| unsafe { w.bits(dest1) });
        self.ch.ch().ch_ctrl_trig.write(|w| unsafe {
            w.data_size()
                .bits(mem::size_of::<WORD>() as u8 >> 1)
                .incr_read()
                .bit(src_incr)
                .incr_write()
                .bit(dest_incr)
                .treq_sel()
                .bits(treq)
                .en()
                .bit(true)
                .chain_to()
                .bits(self.ch.id2())
                .ring_sel()
                .bit(wrap_dest)
                .ring_size()
                .bits(ring_size)
        });

        Ok(RingTransfer {
            ch: self.ch,
            from: self.from,
            to: self.to,
        })
    }

    fn check_alignment(addr: u32, len: u32) -> bool {
        if !len.is_power_of_two() {
            return false;
        }
        if (addr & (len - 1)) != 0 {
            return false;
        }
        true
    }
}

*/

/*

SingleBuffered<CH, RX, TX> {
    config(...) -> SingleBufferedConfig;
    is_done() -> bool
    wait() -> (CH1, RX, TX)
}

*/
