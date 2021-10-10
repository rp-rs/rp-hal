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
    /// Returns the index of the DMA channel.
    fn id(&self) -> u8;

    /// Enables the DMA_IRQ_0 signal for this channel.
    fn listen_irq0(&mut self) {
        const ATOMIC_SET_OFFSET: usize = 0x2000;
        // Safety: We only use the atomic alias of the register.
        unsafe {
            (&*DMA::ptr())
                .inte0
                .as_ptr()
                .add(ATOMIC_SET_OFFSET / 4)
                .write_volatile(1 << self.id());
        }
    }

    /// Disables the DMA_IRQ_0 signal for this channel.
    fn unlisten_irq0(&mut self) {
        const ATOMIC_CLEAR_OFFSET: usize = 0x3000;
        // Safety: We only use the atomic alias of the register.
        unsafe {
            (&*DMA::ptr())
                .inte1
                .as_ptr()
                .add(ATOMIC_CLEAR_OFFSET / 4)
                .write_volatile(1 << self.id());
        }
    }

    /// Checks whether an interrupt is pending for this channel and clears the corresponding IRQ
    /// bit.
    fn check_irq0(&mut self) -> bool {
        // Safety: The following is race-free as we only ever clear the bit for this channel.
        // Nobody else modifies that bit.
        unsafe {
            let status = (&*DMA::ptr()).ints0.read().bits();
            if (status & (1 << self.id())) != 0 {
                // Clear the interrupt.
                (&*DMA::ptr()).ints0.write(|w| w.bits(1 << self.id()));
                true
            } else {
                false
            }
        }
    }

    /// Enables the DMA_IRQ_0 signal for this channel.
    fn listen_irq1(&mut self) {
        const ATOMIC_SET_OFFSET: usize = 0x2000;
        // Safety: We only use the atomic alias of the register.
        unsafe {
            (&*DMA::ptr())
                .inte1
                .as_ptr()
                .add(ATOMIC_SET_OFFSET / 4)
                .write_volatile(1 << self.id());
        }
    }

    /// Disables the DMA_IRQ_1 signal for this channel.
    fn unlisten_irq1(&mut self) {
        const ATOMIC_CLEAR_OFFSET: usize = 0x3000;
        // Safety: We only use the atomic alias of the register.
        unsafe {
            (&*DMA::ptr())
                .inte1
                .as_ptr()
                .add(ATOMIC_CLEAR_OFFSET / 4)
                .write_volatile(1 << self.id());
        }
    }

    /// Checks whether an interrupt is pending for this channel and clears the corresponding IRQ
    /// bit.
    fn check_irq1(&mut self) -> bool {
        // Safety: The following is race-free as we only ever clear the bit for this channel.
        // Nobody else modifies that bit.
        unsafe {
            let status = (&*DMA::ptr()).ints1.read().bits();
            if (status & (1 << self.id())) != 0 {
                // Clear the interrupt.
                (&*DMA::ptr()).ints1.write(|w| w.bits(1 << self.id()));
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
pub trait ChannelPair: SingleChannel {
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

impl<CH1: ChannelIndex, CH2: ChannelIndex> SingleChannel for (Channel<CH1>, Channel<CH2>) {
    fn ch(&self) -> &rp2040_pac::dma::CH {
        self.0.regs()
    }

    fn id(&self) -> u8 {
        CH1::id()
    }
}

trait ChannelConfig {
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
        let (src, src_count) = from.rx_address_count();
        let src_incr = from.rx_increment();
        let (dest, dest_count) = to.tx_address_count();
        let dest_incr = to.tx_increment();
        const TREQ_UNPACED: u8 = 0x3f;
        let treq = match pace {
            Pace::PreferSource => FROM::rx_treq().or(TO::tx_treq()).unwrap_or(TREQ_UNPACED),
            Pace::PreferSink => TO::tx_treq().or(FROM::rx_treq()).unwrap_or(TREQ_UNPACED),
        };
        let len = u32::min(src_count, dest_count);
        self.ch().ch_al1_ctrl.write(|w| unsafe {
            w.data_size()
                .bits(mem::size_of::<WORD>() as u8 >> 1)
                .incr_read()
                .bit(src_incr)
                .incr_write()
                .bit(dest_incr)
                .treq_sel()
                .bits(treq)
                .chain_to()
                .bits(chain_to.unwrap_or(self.id()))
                .en()
                .bit(true)
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

        self.ch()
            .ch_al1_ctrl
            .modify(|_, w| unsafe { w.chain_to().bits(other.id()).en().clear_bit() });
        if self.ch().ch_al1_ctrl.read().busy().bit_is_set() {
            // This channel is still active, so just continue.
            self.ch()
                .ch_al1_ctrl
                .modify(|_, w| unsafe { w.en().set_bit() });
            return;
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
    /// For peripherals, the count should likely be u32::MAX. If a data source implements
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

/*SingleBuffered<CH, RX, TX> {
    config(...) -> SingleBufferedConfig;
    is_done() -> bool
    wait() -> (CH1, RX, TX)
}

DoubleBuffered<(CH1, CH2), RX, TX, ()> {
    config(...) -> SingleBufferedConfig;
    is_done() -> bool
    read_next(BUF) -> DoubleReadBuffer<CH, RX, TX, BUF>
    write_next(BUF) -> DoubleWriteBuffer<CH, RX, TX, BUF>
    wait() -> ((CH1, CH2), RX, TX)
}
DoubleBuffered<CH, RX, TX, ReadNext<RX2>> {
    is_done() -> bool
    wait() -> (DoubleBuffered<CH, RX2, TX>, RX)
}
DoubleBuffered<CH, RX, TX, WriteNext<TX2>> {
    is_done() -> bool
    wait() -> (DoubleBuffered<CH, RX2, TX>, RX)
}

Bidirectional<(CH1, CH2), FROM, BIDI, TO> {
    config(...) -> BidirectionalConfig;
    from_is_done() -> bool
    to_is_done() -> bool
    is_done() -> bool
    from_wait() -> (CH1, FROM, SingleBuffered<CH2, BIDI, TO>)
    to_wait() -> (CH2, SingleBuffered<CH2, FROM, BIDI>, TO)
    wait() -> (CH1, CH2, FROM, BIDI, TO)
}

BidiDoubleBuffered<(CH1, CH2), FROM, BIDI, TO, (), ()> {
    config(...) -> BidirectionalConfig;
    from_is_done() -> bool
    to_is_done() -> bool
    is_done() -> bool
    wait() -> (CH1, CH2, FROM, BIDI, TO)
}

Endless<(CH1, CH2), RX, TX> {
    stop() -> bool
}*/

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
    pub fn check_irq0(&mut self) -> bool {
        self.ch.check_irq0()
    }

    pub fn check_irq1(&mut self) -> bool {
        self.ch.check_irq1()
    }

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

pub struct DoubleBufferingConfig<
    CH1: SingleChannel,
    CH2: SingleChannel,
    FROM: ReadTarget,
    TO: WriteTarget,
> {
    ch: (CH1, CH2),
    from: FROM,
    to: TO,
    pace: Pace,
}

impl<CH1, CH2, FROM, TO, WORD> DoubleBufferingConfig<CH1, CH2, FROM, TO>
where
    CH1: SingleChannel,
    CH2: SingleChannel,
    FROM: ReadTarget<ReceivedWord = WORD>,
    TO: WriteTarget<TransmittedWord = WORD>,
{
    pub fn new(ch: (CH1, CH2), from: FROM, to: TO) -> DoubleBufferingConfig<CH1, CH2, FROM, TO> {
        DoubleBufferingConfig {
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

    pub fn start(mut self) -> DoubleBuffering<CH1, CH2, FROM, TO, ()> {
        // TODO: Do we want to call any callbacks to configure source/sink?

        // Make sure that memory contents reflect what the user intended.
        // TODO: How much of the following is necessary?
        cortex_m::asm::dsb();
        compiler_fence(Ordering::SeqCst);

        // Configure the DMA channel and start it.
        self.ch
            .0
            .config(&self.from, &mut self.to, self.pace, None, true);

        DoubleBuffering {
            ch: self.ch,
            from: self.from,
            to: self.to,
            pace: self.pace,
            state: (),
            second_ch: false,
        }
    }
}

pub struct ReadNext<BUF: ReadTarget>(BUF);
pub struct WriteNext<BUF: WriteTarget>(BUF);

pub struct DoubleBuffering<CH1, CH2, FROM, TO, STATE>
where
    CH1: SingleChannel,
    CH2: SingleChannel,
    FROM: ReadTarget,
    TO: WriteTarget,
{
    ch: (CH1, CH2),
    from: FROM,
    to: TO,
    pace: Pace,
    state: STATE,
    second_ch: bool,
}

impl<CH1, CH2, FROM, TO, WORD, STATE> DoubleBuffering<CH1, CH2, FROM, TO, STATE>
where
    CH1: SingleChannel,
    CH2: SingleChannel,
    FROM: ReadTarget<ReceivedWord = WORD>,
    TO: WriteTarget<TransmittedWord = WORD>,
{
    pub fn is_done(&self) -> bool {
        if self.second_ch {
            !self.ch.1.ch().ch_ctrl_trig.read().busy().bit_is_set()
        } else {
            !self.ch.0.ch().ch_ctrl_trig.read().busy().bit_is_set()
        }
    }
}

impl<CH1, CH2, FROM, TO, WORD> DoubleBuffering<CH1, CH2, FROM, TO, ()>
where
    CH1: SingleChannel,
    CH2: SingleChannel,
    FROM: ReadTarget<ReceivedWord = WORD>,
    TO: WriteTarget<TransmittedWord = WORD> + EndlessWriteTarget,
{
    pub fn wait(self) -> (CH1, CH2, FROM, TO) {
        while !self.is_done() {}

        // Make sure that memory contents reflect what the user intended.
        cortex_m::asm::dsb();
        compiler_fence(Ordering::SeqCst);

        // TODO: Use a tuple type?
        (self.ch.0, self.ch.1, self.from, self.to)
    }
}

impl<CH1, CH2, FROM, TO, WORD> DoubleBuffering<CH1, CH2, FROM, TO, ()>
where
    CH1: SingleChannel,
    CH2: SingleChannel,
    FROM: ReadTarget<ReceivedWord = WORD>,
    TO: WriteTarget<TransmittedWord = WORD> + EndlessWriteTarget,
{
    pub fn read_next<BUF: ReadTarget<ReceivedWord = WORD>>(
        mut self,
        buf: BUF,
    ) -> DoubleBuffering<CH1, CH2, FROM, TO, ReadNext<BUF>> {
        // Make sure that memory contents reflect what the user intended.
        // TODO: How much of the following is necessary?
        cortex_m::asm::dsb();
        compiler_fence(Ordering::SeqCst);

        // Configure the _other_ DMA channel, but do not start it yet.
        if self.second_ch {
            self.ch.0.config(&buf, &mut self.to, self.pace, None, false);
        } else {
            self.ch.1.config(&buf, &mut self.to, self.pace, None, false);
        }

        // Chain the first channel to the second.
        if self.second_ch {
            self.ch.1.set_chain_to_enabled(&mut self.ch.0);
        } else {
            self.ch.0.set_chain_to_enabled(&mut self.ch.1);
        }

        DoubleBuffering {
            ch: self.ch,
            from: self.from,
            to: self.to,
            pace: self.pace,
            state: ReadNext(buf),
            second_ch: self.second_ch,
        }
    }
}

impl<CH1, CH2, FROM, TO, WORD> DoubleBuffering<CH1, CH2, FROM, TO, ()>
where
    CH1: SingleChannel,
    CH2: SingleChannel,
    FROM: ReadTarget<ReceivedWord = WORD> + EndlessReadTarget,
    TO: WriteTarget<TransmittedWord = WORD>,
{
    pub fn write_next<BUF: WriteTarget<TransmittedWord = WORD>>(
        mut self,
        mut buf: BUF,
    ) -> DoubleBuffering<CH1, CH2, FROM, TO, WriteNext<BUF>> {
        // Make sure that memory contents reflect what the user intended.
        // TODO: How much of the following is necessary?
        cortex_m::asm::dsb();
        compiler_fence(Ordering::SeqCst);

        // Configure the _other_ DMA channel, but do not start it yet.
        if self.second_ch {
            self.ch
                .0
                .config(&self.from, &mut buf, self.pace, None, false);
        } else {
            self.ch
                .1
                .config(&self.from, &mut buf, self.pace, None, false);
        }

        // Chain the first channel to the second.
        if self.second_ch {
            self.ch.1.set_chain_to_enabled(&mut self.ch.0);
        } else {
            self.ch.0.set_chain_to_enabled(&mut self.ch.1);
        }

        DoubleBuffering {
            ch: self.ch,
            from: self.from,
            to: self.to,
            pace: self.pace,
            state: WriteNext(buf),
            second_ch: self.second_ch,
        }
    }
}

impl<CH1, CH2, FROM, TO, NEXT, WORD> DoubleBuffering<CH1, CH2, FROM, TO, ReadNext<NEXT>>
where
    CH1: SingleChannel,
    CH2: SingleChannel,
    FROM: ReadTarget<ReceivedWord = WORD>,
    TO: WriteTarget<TransmittedWord = WORD> + EndlessWriteTarget,
    NEXT: ReadTarget<ReceivedWord = WORD>,
{
    pub fn wait(self) -> (FROM, DoubleBuffering<CH1, CH2, NEXT, TO, ()>) {
        while !self.is_done() {}

        // Make sure that memory contents reflect what the user intended.
        cortex_m::asm::dsb();
        compiler_fence(Ordering::SeqCst);

        // Invert second_ch as now the other channel is the "active" channel.
        (
            self.from,
            DoubleBuffering {
                ch: self.ch,
                from: self.state.0,
                to: self.to,
                pace: self.pace,
                state: (),
                second_ch: !self.second_ch,
            },
        )
    }
}

impl<CH1, CH2, FROM, TO, NEXT, WORD> DoubleBuffering<CH1, CH2, FROM, TO, WriteNext<NEXT>>
where
    CH1: SingleChannel,
    CH2: SingleChannel,
    FROM: ReadTarget<ReceivedWord = WORD> + EndlessReadTarget,
    TO: WriteTarget<TransmittedWord = WORD>,
    NEXT: WriteTarget<TransmittedWord = WORD>,
{
    pub fn wait(self) -> (TO, DoubleBuffering<CH1, CH2, FROM, NEXT, ()>) {
        while !self.is_done() {}

        // Make sure that memory contents reflect what the user intended.
        cortex_m::asm::dsb();
        compiler_fence(Ordering::SeqCst);

        // Invert second_ch as now the other channel is the "active" channel.
        (
            self.to,
            DoubleBuffering {
                ch: self.ch,
                from: self.from,
                to: self.state.0,
                pace: self.pace,
                state: (),
                second_ch: !self.second_ch,
            },
        )
    }
}

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

    pub fn from_pace(&mut self, pace: Pace) {
        self.from_pace = pace;
    }

    pub fn to_pace(&mut self, pace: Pace) {
        self.to_pace = pace;
    }

    pub fn start(mut self) -> Bidirectional<CH1, CH2, FROM, BIDI, TO> {
        // TODO
        panic!("Not yet implemented.");
    }
}

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
    pub fn wait(self) -> ((CH1, CH2), FROM, BIDI, TO) {
        // TODO
        panic!("Not yet implemented.");
    }
}

/*impl<WORD, CH, FROM, TO> SingleBufferingConfig<CH, FROM, TO>
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
}*/

/// Pacing for DMA transfers.
///
/// Generally, while memory-to-memory DMA transfers can operate at maximum possible throughput,
/// transfers involving peripherals commonly have to wait for data to be available or for available
/// space in write queues. This type defines whether the sink or the source shall pace the transfer
/// for peripheral-to-peripheral transfers.
#[derive(Clone, Copy)]
pub enum Pace {
    /// The DREQ signal from the source is used, if available. If not, the sink's DREQ signal is
    /// used.
    PreferSource,
    /// The DREQ signal from the sink is used, if available. If not, the source's DREQ signal is
    /// used.
    PreferSink,
    // TODO: Timers?
}

/// Error during DMA configuration.
#[derive(Debug)]
pub enum DMAError {
    /// Buffers were not aligned to their size even though they needed to be.
    Alignment,
    /// An illegal configuration (i.e., buffer sizes not suitable for a memory-to-memory transfer)
    /// was specified.
    IllegalConfig,
}



/// The DREQ value for PIO0's TX FIFO 0
pub const DREQ_PIO0_TX0: u8 = 0;
/// The DREQ value for PIO0's TX FIFO 1
pub const DREQ_PIO0_TX1: u8 = 1;
/// The DREQ value for PIO0's TX FIFO 2
pub const DREQ_PIO0_TX2: u8 = 2;
/// The DREQ value for PIO0's TX FIFO 3
pub const DREQ_PIO0_TX3: u8 = 3;
/// The DREQ value for PIO0's RX FIFO 0
pub const DREQ_PIO0_RX0: u8 = 4;
/// The DREQ value for PIO0's RX FIFO 1
pub const DREQ_PIO0_RX1: u8 = 5;
/// The DREQ value for PIO0's RX FIFO 2
pub const DREQ_PIO0_RX2: u8 = 6;
/// The DREQ value for PIO0's RX FIFO 3
pub const DREQ_PIO0_RX3: u8 = 7;
/// The DREQ value for PIO1's TX FIFO 0
pub const DREQ_PIO1_TX0: u8 = 8;
/// The DREQ value for PIO1's TX FIFO 1
pub const DREQ_PIO1_TX1: u8 = 9;
/// The DREQ value for PIO1's TX FIFO 2
pub const DREQ_PIO1_TX2: u8 = 10;
/// The DREQ value for PIO1's TX FIFO 3
pub const DREQ_PIO1_TX3: u8 = 11;
/// The DREQ value for PIO1's RX FIFO 0
pub const DREQ_PIO1_RX0: u8 = 12;
/// The DREQ value for PIO1's RX FIFO 1
pub const DREQ_PIO1_RX1: u8 = 13;
/// The DREQ value for PIO1's RX FIFO 2
pub const DREQ_PIO1_RX2: u8 = 14;
/// The DREQ value for PIO1's RX FIFO 3
pub const DREQ_PIO1_RX3: u8 = 15;
/// The DREQ value for SPI0's TX FIFO
pub const DREQ_SPI0_TX: u8 = 16;
/// The DREQ value for SPI0's RX FIFO
pub const DREQ_SPI0_RX: u8 = 17;
/// The DREQ value for SPI1's TX FIFO
pub const DREQ_SPI1_TX: u8 = 18;
/// The DREQ value for SPI1's RX FIFO
pub const DREQ_SPI1_RX: u8 = 19;
/// The DREQ value for UART0's TX FIFO
pub const DREQ_UART0_TX: u8 = 20;
/// The DREQ value for UART0's RX FIFO
pub const DREQ_UART0_RX: u8 = 21;
/// The DREQ value for UART1's TX FIFO
pub const DREQ_UART1_TX: u8 = 22;
/// The DREQ value for UART1's RX FIFO
pub const DREQ_UART1_RX: u8 = 23;
/// The DREQ value for PWM Counter 0's Wrap Value
pub const DREQ_PWM_WRAP0: u8 = 24;
/// The DREQ value for PWM Counter 1's Wrap Value
pub const DREQ_PWM_WRAP1: u8 = 25;
/// The DREQ value for PWM Counter 2's Wrap Value
pub const DREQ_PWM_WRAP2: u8 = 26;
/// The DREQ value for PWM Counter 3's Wrap Value
pub const DREQ_PWM_WRAP3: u8 = 27;
/// The DREQ value for PWM Counter 4's Wrap Value
pub const DREQ_PWM_WRAP4: u8 = 28;
/// The DREQ value for PWM Counter 5's Wrap Value
pub const DREQ_PWM_WRAP5: u8 = 29;
/// The DREQ value for PWM Counter 6's Wrap Value
pub const DREQ_PWM_WRAP6: u8 = 30;
/// The DREQ value for PWM Counter 7's Wrap Value
pub const DREQ_PWM_WRAP7: u8 = 31;
/// The DREQ value for I2C0's TX FIFO
pub const DREQ_I2C0_TX: u8 = 32;
/// The DREQ value for I2C0's RX FIFO
pub const DREQ_I2C0_RX: u8 = 33;
/// The DREQ value for I2C1's TX FIFO
pub const DREQ_I2C1_TX: u8 = 34;
/// The DREQ value for I2C1's RX FIFO
pub const DREQ_I2C1_RX: u8 = 35;
/// The DREQ value for the ADC
pub const DREQ_ADC: u8 = 36;
/// The DREQ value for the XIP Streaming FIFO
pub const DREQ_XIP_STREAM: u8 = 37;
/// The DREQ value for the XIP SSI TX FIFO
pub const DREQ_XIP_SSITX: u8 = 38;
/// The DREQ value for the XIP SSI RX FIFO
pub const DREQ_XIP_SSIRX: u8 = 39;
