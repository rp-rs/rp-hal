use rp2040_pac::DMA;

use super::{Channel, ChannelIndex, Pace, ReadTarget, WriteTarget};
use crate::{
    atomic_register_access::{write_bitmask_clear, write_bitmask_set},
    dma::ChannelRegs,
    typelevel::Sealed,
};
use core::mem;

/// Trait which implements low-level functionality for transfers using a single DMA channel.
pub trait SingleChannel: Sealed {
    /// Returns the registers associated with this DMA channel.
    ///
    /// In the case of channel pairs, this returns the first channel.
    fn ch(&self) -> &rp2040_pac::dma::CH;
    /// Returns the index of the DMA channel.
    fn id(&self) -> u8;

    /// Enables the DMA_IRQ_0 signal for this channel.
    fn listen_irq0(&mut self) {
        // Safety: We only use the atomic alias of the register.
        unsafe {
            write_bitmask_set((*DMA::ptr()).inte0.as_ptr(), 1 << self.id());
        }
    }

    /// Disables the DMA_IRQ_0 signal for this channel.
    fn unlisten_irq0(&mut self) {
        // Safety: We only use the atomic alias of the register.
        unsafe {
            write_bitmask_clear((*DMA::ptr()).inte0.as_ptr(), 1 << self.id());
        }
    }

    /// Check if an interrupt is pending for this channel and clear the corresponding pending bit
    fn check_irq0(&mut self) -> bool {
        // Safety: The following is race-free as we only ever clear the bit for this channel.
        // Nobody else modifies that bit.
        unsafe {
            let status = (*DMA::ptr()).ints0.read().bits();
            if (status & (1 << self.id())) != 0 {
                // Clear the interrupt.
                (*DMA::ptr()).ints0.write(|w| w.bits(1 << self.id()));
                true
            } else {
                false
            }
        }
    }

    /// Enables the DMA_IRQ_1 signal for this channel.
    fn listen_irq1(&mut self) {
        // Safety: We only use the atomic alias of the register.
        unsafe {
            write_bitmask_set((*DMA::ptr()).inte1.as_ptr(), 1 << self.id());
        }
    }

    /// Disables the DMA_IRQ_1 signal for this channel.
    fn unlisten_irq1(&mut self) {
        // Safety: We only use the atomic alias of the register.
        unsafe {
            write_bitmask_clear((*DMA::ptr()).inte1.as_ptr(), 1 << self.id());
        }
    }

    /// Check if an interrupt is pending for this channel and clear the corresponding pending bit
    fn check_irq1(&mut self) -> bool {
        // Safety: The following is race-free as we only ever clear the bit for this channel.
        // Nobody else modifies that bit.
        unsafe {
            let status = (*DMA::ptr()).ints1.read().bits();
            if (status & (1 << self.id())) != 0 {
                // Clear the interrupt.
                (*DMA::ptr()).ints1.write(|w| w.bits(1 << self.id()));
                true
            } else {
                false
            }
        }
    }
}

/// Trait which implements low-level functionality for transfers requiring two DMA channels.
///
/// Anything that requires more than a single buffer exactly once requires two channels to be
/// combined.
pub trait ChannelPair: SingleChannel + Sealed {
    /// Returns the registers associated with the second DMA channel associated with this channel
    /// pair.
    fn ch2(&self) -> &rp2040_pac::dma::CH;
    /// Returns the index of the second DMA channel.
    fn id2(&self) -> u8;
}

impl<CH: ChannelIndex> SingleChannel for Channel<CH> {
    fn ch(&self) -> &rp2040_pac::dma::CH {
        self.regs()
    }

    fn id(&self) -> u8 {
        CH::id()
    }
}

impl<CH: ChannelIndex> Sealed for Channel<CH> {}

impl<CH1: ChannelIndex, CH2: ChannelIndex> SingleChannel for (Channel<CH1>, Channel<CH2>) {
    fn ch(&self) -> &rp2040_pac::dma::CH {
        self.0.regs()
    }

    fn id(&self) -> u8 {
        CH1::id()
    }
}

impl<CH1: ChannelIndex, CH2: ChannelIndex> Sealed for (Channel<CH1>, Channel<CH2>) {}

pub(crate) trait ChannelConfig {
    fn config<WORD, FROM, TO>(
        &mut self,
        from: &FROM,
        to: &mut TO,
        pace: Pace,
        chain_to: Option<u8>,
        start: bool,
    ) where
        FROM: ReadTarget<ReceivedWord = WORD>,
        TO: WriteTarget<TransmittedWord = WORD>;

    fn set_chain_to_enabled<CH: SingleChannel>(&mut self, other: &mut CH);
    fn start(&mut self);
    fn start_both<CH: SingleChannel>(&mut self, other: &mut CH);
}

impl<CH: SingleChannel> ChannelConfig for CH {
    fn config<WORD, FROM, TO>(
        &mut self,
        from: &FROM,
        to: &mut TO,
        pace: Pace,
        chain_to: Option<u8>,
        start: bool,
    ) where
        FROM: ReadTarget<ReceivedWord = WORD>,
        TO: WriteTarget<TransmittedWord = WORD>,
    {
        // Configure the DMA channel.
        assert!(
            mem::size_of::<WORD>() != 8,
            "DMA does not support transferring 64bit data"
        );
        let (src, src_count) = from.rx_address_count();
        let src_incr = from.rx_increment();
        let (dest, dest_count) = to.tx_address_count();
        let dest_incr = to.tx_increment();
        const TREQ_UNPACED: u8 = 0x3f;
        let treq = match pace {
            Pace::PreferSource => FROM::rx_treq().or_else(TO::tx_treq).unwrap_or(TREQ_UNPACED),
            Pace::PreferSink => TO::tx_treq().or_else(FROM::rx_treq).unwrap_or(TREQ_UNPACED),
        };
        let len = u32::min(src_count, dest_count);
        self.ch().ch_al1_ctrl.write(|w| unsafe {
            w.data_size().bits(mem::size_of::<WORD>() as u8 >> 1);
            w.incr_read().bit(src_incr);
            w.incr_write().bit(dest_incr);
            w.treq_sel().bits(treq);
            w.chain_to().bits(chain_to.unwrap_or_else(|| self.id()));
            w.en().bit(true);
            w
        });
        self.ch().ch_read_addr.write(|w| unsafe { w.bits(src) });
        self.ch().ch_trans_count.write(|w| unsafe { w.bits(len) });
        if start {
            self.ch()
                .ch_al2_write_addr_trig
                .write(|w| unsafe { w.bits(dest) });
        } else {
            self.ch().ch_write_addr.write(|w| unsafe { w.bits(dest) });
        }
    }

    fn set_chain_to_enabled<CH2: SingleChannel>(&mut self, other: &mut CH2) {
        // We temporarily pause the channel when setting CHAIN_TO, to prevent any race condition
        // that could occur, as we need to check afterwards whether the channel was successfully
        // chained to this channel or whether this channel was already completed. If we did not
        // pause this channel, we could get into a situation where both channels completed in quick
        // succession, yet we did not notice, as the situation is not distinguishable from one
        // where the second channel was not started at all.

        self.ch().ch_al1_ctrl.modify(|_, w| unsafe {
            w.chain_to().bits(other.id());
            w.en().clear_bit();
            w
        });
        if self.ch().ch_al1_ctrl.read().busy().bit_is_set() {
            // This channel is still active, so just continue.
            self.ch().ch_al1_ctrl.modify(|_, w| w.en().set_bit());
        } else {
            // This channel has already finished, so just start the other channel directly.
            other.start();
        }
    }

    fn start(&mut self) {
        // Safety: The write does not interfere with any other writes, it only affects this
        // channel.
        unsafe { &*rp2040_pac::DMA::ptr() }
            .multi_chan_trigger
            .write(|w| unsafe { w.bits(1 << self.id()) });
    }

    fn start_both<CH2: SingleChannel>(&mut self, other: &mut CH2) {
        // Safety: The write does not interfere with any other writes, it only affects this
        // channel and other (which we have an exclusive borrow of).
        let channel_flags = 1 << self.id() | 1 << other.id();
        unsafe { &*rp2040_pac::DMA::ptr() }
            .multi_chan_trigger
            .write(|w| unsafe { w.bits(channel_flags) });
    }
}
