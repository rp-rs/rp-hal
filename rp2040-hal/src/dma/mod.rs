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
use embedded_dma::{ReadBuffer, WriteBuffer};
use rp2040_pac::DMA;

// Bring in our submodules
mod bidirectional;
mod double_buffering;
mod single_buffering;
mod single_channel;

// Export these types for easier use by external code
pub use crate::dma::bidirectional::BidirectionalConfig;
pub use crate::dma::double_buffering::{DoubleBuffering, DoubleBufferingConfig, ReadNext};
pub use crate::dma::single_buffering::{SingleBuffering, SingleBufferingConfig};
pub use crate::dma::single_channel::SingleChannel;

/// DMA unit.
pub trait DMAExt {
    /// Splits the DMA unit into its individual channels.
    fn split(self, resets: &mut pac::RESETS) -> Channels;
    /// Splits the DMA unit into its individual channels with runtime ownership
    fn dyn_split(self, resets: &mut pac::RESETS) -> DynChannels;
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

            fn dyn_split(self, resets: &mut pac::RESETS) -> DynChannels{
                self.reset_bring_up(resets);

                DynChannels {
                    $(
                        $chX: Some(Channel {
                            _phantom: PhantomData,
                        }),
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

        /// Set of DMA channels.
        pub struct DynChannels {
            $(
                /// DMA channel.
                pub $chX: Option<Channel<$CHX>>,
            )+
        }
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

impl<B: ReadBuffer> ReadTarget for B {
    type ReceivedWord = <B as ReadBuffer>::Word;

    fn rx_treq() -> Option<u8> {
        None
    }

    fn rx_address_count(&self) -> (u32, u32) {
        // Safety: We only call the function once per buffer.
        let (ptr, len) = unsafe { self.read_buffer() };
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

impl<B: WriteBuffer> WriteTarget for B {
    type TransmittedWord = <B as WriteBuffer>::Word;

    fn tx_treq() -> Option<u8> {
        None
    }

    fn tx_address_count(&mut self) -> (u32, u32) {
        // Safety: We only call the function once per buffer.
        let (ptr, len) = unsafe { self.write_buffer() };
        (ptr as u32, len as u32)
    }

    fn tx_increment(&self) -> bool {
        true
    }
}

/*

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
