//! Direct memory acces (DMA).
//!
//! The DMA unit of the RP2040 seems very simplistic at first when compared to other MCUs. For
//! example, the individual DMA channels do not support chaining multiple buffers. However, within
//! certain limits, the DMA engine supports a wide range of transfer and buffer types, often by
//! combining multiple DMA channels:
//!
//! * Simple RX/TX transfers filling a single buffer or transferring data from one peripheral to
//!   another.
//! * RX/TX transfers that use multiple chained buffers: These transfers require two channels to
//!   be combined, where the first DMA channel configures the second DMA channel. An example for
//!   this transfer type can be found in the datasheet.
//! * Repeated transfers from/to a set of buffers: By allocating one channel per buffer and
//!   chaining the channels together, continuous transfers to a set of ring buffers can be
//!   achieved. Note, however, that the MCU manually needs to reconfigure the DMA units unless the
//!   buffer addresses and sizes are aligned, in which case the ring buffer functionality of the
//!   DMA engine can be used. Even then, however, at least two DMA channels are required as a
//!   channel cannot be chained to itself.
//!
//! This API tries to provide three types of buffers: Single buffers, double-buffered transfers
//! where the user can specify the next buffer while the previous is being transferred, and
//! automatic continous ring buffers consisting of two aligned buffers being read or written
//! alternatingly.

use crate::resets::SubsystemReset;
use core::marker::PhantomData;
use core::mem;
use core::sync::atomic::{compiler_fence, Ordering};
use embedded_dma::{StaticReadBuffer, StaticWriteBuffer};
use rp2040_pac::DMA;

/// DMA unit.
pub trait DMAExt {
    /// Splits the DMA unit into its individual channels.
    fn split(self, resets: &mut pac::RESETS) -> Channels;
}

/// DMA channel.
pub struct Channel<CH: ChannelIndex> {
    _phantom: PhantomData<CH>,
}

/// DMA channel identifier.
pub trait ChannelIndex {
    /// Numerical index of the DMA channel (0..11).
    fn id() -> u8;
}

macro_rules! channels {
    (
        $($CHX:ident: ($chX:ident, $x:expr),)+
    ) => {
        impl DMAExt for DMA {
            fn split(self, resets: &mut pac::RESETS) -> Channels {
                self.reset_bring_up(resets);

                Channels {
                    $(
                        $chX: Channel {
                            _phantom: PhantomData,
                        },
                    )+
                }
            }
        }

        /// Set of DMA channels.
        pub struct Channels {
            $(
                /// DMA channel.
                pub $chX: Channel<$CHX>,
            )+
        }
        $(
            /// DMA channel identifier.
            pub struct $CHX;
            impl ChannelIndex for $CHX {
                fn id() -> u8 {
                    $x
                }
            }
        )+
    }
}

channels! {
    CH0: (ch0, 0),
    CH1: (ch1, 1),
    CH2: (ch2, 2),
    CH3: (ch3, 3),
    CH4: (ch4, 4),
    CH5: (ch5, 5),
    CH6: (ch6, 6),
    CH7: (ch7, 7),
    CH8: (ch8, 8),
    CH9: (ch9, 9),
    CH10:(ch10, 10),
    CH11:(ch11, 11),
}

trait ChannelRegs {
    unsafe fn ptr() -> *const rp2040_pac::dma::CH;
    fn regs(&self) -> &rp2040_pac::dma::CH;
}

impl<CH: ChannelIndex> ChannelRegs for Channel<CH> {
    unsafe fn ptr() -> *const rp2040_pac::dma::CH {
        &(*rp2040_pac::DMA::ptr()).ch[CH::id() as usize] as *const _
    }

    fn regs(&self) -> &rp2040_pac::dma::CH {
        unsafe { &*Self::ptr() }
    }
}

/// Trait which implements low-level functionality for transfers using a single DMA channel.
pub trait SingleChannel {
    /// Returns the registers associated with this DMA channel.
    ///
    /// In the case of channel pairs, this returns the first channel.
    fn ch(&self) -> &rp2040_pac::dma::CH;
    fn id() -> u8;
}

// TODO: ChannelPair is not actually required, we can just implement SingleChannel on a tuple.

/// Trait which implements low-level functionality for transfers requiring two DMA channels.
///
/// Anything that requires more than a single buffer exactly once requires two channels to be
/// combined.
pub trait ChannelPair: SingleChannel {
    /// Returns the registers associated with the second DMA channel associated with this channel
    /// pair.
    fn ch2(&self) -> &rp2040_pac::dma::CH;
    fn id2() -> u8;
}

impl<CH: ChannelIndex> SingleChannel for Channel<CH> {
    fn ch(&self) -> &rp2040_pac::dma::CH {
        self.regs()
    }

    fn id() -> u8 {
        CH::id()
    }
}

impl<CH1: ChannelIndex, CH2: ChannelIndex> SingleChannel for (Channel<CH1>, Channel<CH2>) {
    fn ch(&self) -> &rp2040_pac::dma::CH {
        self.0.regs()
    }

    fn id() -> u8 {
        CH1::id()
    }
}

impl<CH1: ChannelIndex, CH2: ChannelIndex> ChannelPair for (Channel<CH1>, Channel<CH2>) {
    fn ch2(&self) -> &rp2040_pac::dma::CH {
        self.1.regs()
    }

    fn id2() -> u8 {
        CH2::id()
    }
}

/// Trait which is implemented by anything that can be read via DMA.
pub trait ReadTarget {
    /// Type which is transferred in a single DMA transfer.
    type ReceivedWord;

    /// Returns the DREQ number for this data source (`None` for memory buffers).
    fn rx_treq() -> Option<u8>;

    /// Returns the address and the maximum number of words that can be transferred from this data
    /// source in a single DMA operation.
    ///
    /// For peripherals, the cound should likely be u32::MAX. If a data source implements
    /// EndlessReadTarget, it is suitable for infinite transfers from or to ring buffers. Note that
    /// ring buffers designated for endless transfers, but with a finite buffer size, should return
    /// the size of their individual buffers here.
    ///
    /// # Safety
    ///
    /// This function has the same safety guarantees as `StaticReadBuffer::static_read_buffer`.
    fn rx_address_count(&self) -> (u32, u32);

    /// Returns whether the address shall be incremented after each transfer.
    fn rx_increment(&self) -> bool;
}

/// Marker which signals that `rx_address_count()` can be called multiple times.
///
/// The DMA code will never call `rx_address_count()` to request more than two buffers to configure
/// two DMA channels. In the case of peripherals, the function can always return the same values.
pub trait EndlessReadTarget: ReadTarget {}

impl<B: StaticReadBuffer> ReadTarget for B {
    type ReceivedWord = <B as StaticReadBuffer>::Word;

    fn rx_treq() -> Option<u8> {
        None
    }

    fn rx_address_count(&self) -> (u32, u32) {
        // Safety: We only call the function once per buffer.
        let (ptr, len) = unsafe { self.static_read_buffer() };
        (ptr as u32, len as u32)
    }

    fn rx_increment(&self) -> bool {
        true
    }
}

/// Trait which is implemented by anything that can be written via DMA.
pub trait WriteTarget {
    /// Type which is transferred in a single DMA transfer.
    type TransmittedWord;

    /// Returns the DREQ number for this data sink (`None` for memory buffers).
    fn tx_treq() -> Option<u8>;

    /// Returns the address and the maximum number of words that can be transferred from this data
    /// source in a single DMA operation.
    ///
    /// See `ReadTarget::rx_address_count` for a complete description of the semantics of this
    /// function.
    fn tx_address_count(&mut self) -> (u32, u32);

    /// Returns whether the address shall be incremented after each transfer.
    fn tx_increment(&self) -> bool;
}

/// Marker which signals that `tx_address_count()` can be called multiple times.
///
/// The DMA code will never call `tx_address_count()` to request more than two buffers to configure
/// two DMA channels. In the case of peripherals, the function can always return the same values.
pub trait EndlessWriteTarget: WriteTarget {}

impl<B: StaticWriteBuffer> WriteTarget for B {
    type TransmittedWord = <B as StaticWriteBuffer>::Word;

    fn tx_treq() -> Option<u8> {
        None
    }

    fn tx_address_count(&mut self) -> (u32, u32) {
        // Safety: We only call the function once per buffer.
        let (ptr, len) = unsafe { self.static_write_buffer() };
        (ptr as u32, len as u32)
    }

    fn tx_increment(&self) -> bool {
        true
    }
}

/*/// Ring buffer consisting of two aligned memory regions.
///
/// We require buffers to have a power-of-two size and to be aligned to their size, as otherwise
/// the RP2040 would require an additional DMA channel as well as an additional aligned buffer to
/// implement operations on the ring buffer.
///
/// The two buffers do not have to have the same size.
pub struct ReadRingBuffer<TYPE, BUF1, BUF2> {
    buf1: BUF1,
    buf2: BUF2,
    _phantom: PhantomData<TYPE>,
}

impl<TYPE, BUF1, BUF2> ReadRingBuffer<TYPE, BUF1, BUF2>
where
    BUF1: StaticReadBuffer<Word = TYPE>,
    BUF2: StaticReadBuffer<Word = TYPE>,
{
    /// Creates a new ring buffer from the two buffers, checking their size and alignment.
    pub fn new(buf1: BUF1, buf2: BUF2) -> Result<Self, DMAError> {
        // We check the alignment of the buffers before creating
        // Safety: We do not actually access the pointers returned by the function. According to
        // the documentation, later calls are required to return identical values.
        let (ptr1, len1) = unsafe { buf1.static_read_buffer() };
        let (ptr2, len2) = unsafe { buf2.static_read_buffer() };
        let len1 = len1 * mem::size_of::<TYPE>();
        let len2 = len2 * mem::size_of::<TYPE>();
        if !len1.is_power_of_two() || !len2.is_power_of_two() {
            return Err(DMAError::Alignment);
        }
        if (ptr1 as usize & (len1 - 1)) != 0 {
            return Err(DMAError::Alignment);
        }
        if (ptr2 as usize & (len2 - 1)) != 0 {
            return Err(DMAError::Alignment);
        }

        Ok(Self {
            buf1,
            buf2,
            _phantom: PhantomData,
        })
    }
}

impl<TYPE, BUF1, BUF2> ReadTarget for ReadRingBuffer<TYPE, BUF1, BUF2>
where
    BUF1: StaticReadBuffer<Word = TYPE>,
    BUF2: StaticReadBuffer<Word = TYPE>,
{
    type ReceivedWord = TYPE;

    fn rx_treq() -> Option<u8> {
        None
    }

    fn rx_address_count(&self) -> (u32, u32) {
        // TODO
        (0, 0)
    }

    fn rx_increment(&self) -> bool {
        true
    }
}

impl<TYPE, BUF1, BUF2> EndlessReadTarget for ReadRingBuffer<TYPE, BUF1, BUF2>
where
    BUF1: StaticReadBuffer<Word = TYPE>,
    BUF2: StaticReadBuffer<Word = TYPE>,
{
}

pub struct WriteRingBuffer<TYPE, BUF1, BUF2> {
    buf1: BUF1,
    buf2: BUF2,
    _phantom: PhantomData<TYPE>,
}

impl<TYPE, BUF1, BUF2> WriteRingBuffer<TYPE, BUF1, BUF2>
where
    BUF1: StaticWriteBuffer<Word = TYPE>,
    BUF2: StaticWriteBuffer<Word = TYPE>,
{
    pub fn new(buf1: BUF1, buf2: BUF2) -> Result<Self, DMAError> {
        // TODO: Check alignment.
        Ok(Self {
            buf1,
            buf2,
            _phantom: PhantomData,
        })
    }
}

impl<TYPE, BUF1, BUF2> WriteTarget for WriteRingBuffer<TYPE, BUF1, BUF2>
where
    BUF1: StaticWriteBuffer<Word = TYPE>,
    BUF2: StaticWriteBuffer<Word = TYPE>,
{
    type TransmittedWord = TYPE;

    fn tx_treq() -> Option<u8> {
        None
    }

    fn tx_address_count(&mut self) -> (u32, u32) {
        // TODO
        (0, 0)
    }

    fn tx_increment(&self) -> bool {
        true
    }
}

impl<TYPE, BUF1, BUF2> EndlessWriteTarget for WriteRingBuffer<TYPE, BUF1, BUF2>
where
    BUF1: StaticWriteBuffer<Word = TYPE>,
    BUF2: StaticWriteBuffer<Word = TYPE>,
{
}*/

// TODO: Drop for Transfer and TransferConfig?

pub struct TransferConfig<CH: SingleChannel, FROM: ReadTarget, TO: WriteTarget> {
    ch: CH,
    from: FROM,
    to: TO,
    pace: Pace,
}

impl<WORD, CH, FROM, TO> TransferConfig<CH, FROM, TO>
where
    CH: SingleChannel,
    FROM: ReadTarget<ReceivedWord = WORD>,
    TO: WriteTarget<TransmittedWord = WORD>,
{
    pub fn new(ch: CH, from: FROM, to: TO) -> TransferConfig<CH, FROM, TO> {
        TransferConfig {
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

    pub fn start(mut self) -> Transfer<CH, FROM, TO> {
        // TODO: Do we want to call any callbacks to configure source/sink?

        // Make sure that memory contents reflect what the user intended.
        // TODO: How much of the following is necessary?
        cortex_m::asm::dsb();
        compiler_fence(Ordering::SeqCst);

        // Configure the DMA channel.
        let (src, src_count) = self.from.rx_address_count();
        let src_incr = self.from.rx_increment();
        let (dest, dest_count) = self.to.tx_address_count();
        let dest_incr = self.to.tx_increment();
        const TREQ_UNPACED: u8 = 0x3f;
        let treq = match self.pace {
            Pace::PreferSource => FROM::rx_treq().or(TO::tx_treq()).unwrap_or(TREQ_UNPACED),
            Pace::PreferDest => TO::tx_treq().or(FROM::rx_treq()).unwrap_or(TREQ_UNPACED),
        };
        let len = u32::min(src_count, dest_count);
        self.ch.ch().ch_read_addr.write(|w| unsafe { w.bits(src) });
        self.ch
            .ch()
            .ch_trans_count
            .write(|w| unsafe { w.bits(len) });
        self.ch
            .ch()
            .ch_write_addr
            .write(|w| unsafe { w.bits(dest) });
        // The following access starts the transfer.
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
        });

        Transfer {
            ch: self.ch,
            from: self.from,
            to: self.to,
        }
    }
}

impl<WORD, CH, FROM, TO> TransferConfig<CH, FROM, TO>
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
            Pace::PreferDest => TO::tx_treq().or(FROM::rx_treq()).unwrap_or(TREQ_UNPACED),
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
                .bits(CH::id())
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
                .bits(CH::id2())
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
    /// Returns whether the transfer has completed.
    pub fn is_done(&self) -> bool {
        !self.ch.ch().ch_ctrl_trig.read().busy().bit_is_set()
    }

    pub fn wait(self) -> (CH, FROM, TO) {
        while !self.is_done() {}

        // Make sure that memory contents reflect what the user intended.
        cortex_m::asm::dsb();
        compiler_fence(Ordering::SeqCst);

        (self.ch, self.from, self.to)
    }
}

pub struct RingTransfer<CH: ChannelPair, FROM: ReadTarget, TO: WriteTarget> {
    ch: CH,
    from: FROM,
    to: TO,
}

impl<CH, FROM, TO> RingTransfer<CH, FROM, TO>
where
    CH: ChannelPair,
    FROM: ReadTarget,
    TO: WriteTarget,
{
    // TODO
}

pub enum Pace {
    PreferSource,
    PreferDest,
    // TODO: Timers?
}

#[derive(Debug)]
pub enum DMAError {
    Alignment,
    IllegalConfig,
}
