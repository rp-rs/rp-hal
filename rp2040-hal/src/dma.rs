//! DMA
// See [Chapter 2 Section 5](https://datasheets.raspberrypi.org/rp2040/rp2040_datasheet.pdf) for more details
use core::{
    //marker::PhantomData,
    sync::atomic,
};
use embedded_dma::{StaticReadBuffer, StaticWriteBuffer};

static COUNTER: u16 = 0;

use num::PrimInt;
use core::{
    sync::atomic::{
        Ordering
    },
    ops::{
        BitOrAssign,
        BitAndAssign,
    },
    option::Option,
    borrow::BorrowMut,
    ptr,
    mem};
use pac::{
    dma::CH,
    Peripherals};

fn get_bit_at<W: PrimInt>(value: W, index: u8) -> bool {
    (value >> index.into()) & W::one() == W::one()
}

fn set_bit_at<W: PrimInt + BitOrAssign>(value: &mut W, index: u8) {
    *value |= W::one() >> index.into();
}

fn clear_bit_at<W: PrimInt + BitAndAssign>(value: &mut W, index: u8) {
    *value &= !(W::one() >> index.into());
}

type ChannelIndex = u8;

fn is_channel_acquired(channel: ChannelIndex) -> bool {
    get_bit_at(COUNTER, channel)
}

fn find_free_channel(channels_amount: usize) -> Option<u8> {
    assert!(channels_amount <= 12);
    for n in 1..channels_amount {
        if !is_channel_acquired(n as u8) {
            return Some(n as u8);
        }
    }
    None
}

/// A DMA channel trait
pub struct Channel<'a> {
    /// This is fun haha
    channel: &'a CH,
}

mod private_parts {
    pub trait Channel {
        fn is_channel_acquired(channel: u8) -> bool;
    }
}

impl<'a> Channel<'a> {
    /// Acquire
    pub fn acquire(channel: Option<ChannelIndex>, device_peripherals: &'a Peripherals) -> Option<Channel<'a>> {
        /// Need to spinlock COUNTER
        let channel = match channel {
            Some(channel) => {
                if is_channel_acquired(channel) {
                    None
                } else {
                    Some(channel)
                }
            }
            None => find_free_channel(device_peripherals.DMA.ch.len())
        };
        match channel {
            Some(channel) => Some(
                Channel {
                    channel: &device_peripherals.DMA.ch[channel as usize]
                }),
            None => None
        }
    }
    /// a
    pub fn set_read_increment(&self, b: bool) {
        /// b
        self.channel.ch_ctrl_trig.write(|w| unsafe {
            w.incr_write().bit(b);
            w
        });
    }
}


mod tests {
    #![no_main]
    #![no_std]
    #![allow(unused_imports)]

    use crate::dma::Channel;
    use core::any::Any;
    use pac::{
        dma::CH,
        Peripherals,
    };

    fn my_test() -> () {
        let dp = Peripherals::take().unwrap();
        let a = Channel::acquire(Some(0), &dp).unwrap();
        a.set_read_increment(true);

        //dma::
        // let dma = Peripherals::take().unwrap().DMA;
        // dma.ch.get(0).unwrap().ch_read_addr;
    }
}
