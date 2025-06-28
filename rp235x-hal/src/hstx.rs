//! High-speed transmitter (HSTX)
//!
//! See [Section 12.3](https://rptl.io/rp2350-datasheet#section_hstx) for more details.
//!
//! ## Usage
//!
//! ```no_run
//! use rp235x_hal::{
//!     self as hal,
//!     hstx::{Hstx, HstxPins, HstxBitConfig},
//!     Sio,
//! };
//!
//! let mut peripherals = hal::pac::Peripherals::take().unwrap();
//! let sio = Sio::new(peripherals.SIO);
//! let pins = hal::gpio::Pins::new(
//!     peripherals.IO_BANK0,
//!     peripherals.PADS_BANK0,
//!     sio.gpio_bank0,
//!     &mut peripherals.RESETS,
//! );
//!
//! let bit0p = pins.gpio12.into_function();
//! let bit3p = pins.gpio15.into_function();
//!
//! let hstx_pins = HstxPins::new().add_bit0_pin(bit0p).add_bit3_pin(bit3p);
//!
//! let mut hstx = Hstx::new(peripherals.HSTX_CTRL, peripherals.HSTX_FIFO, hstx_pins,
//!                          &mut peripherals.RESETS);
//! hstx.configure_clk(5, 0);
//! hstx.configure_shifter(2, 5);
//! hstx.configure_bit0(HstxBitConfig::Clk { inv: false });
//! hstx.configure_bit3(HstxBitConfig::Shift { sel_p:0, sel_n: 1, inv: false });
//!
//! let mut hstx = hstx.enable();
//!
//! loop {
//!     hstx.fifo_write(0xaabbccdd);
//! }
//! ```

use crate::{
    dma::{EndlessWriteTarget, WriteTarget},
    pac::{dma::ch::ch_ctrl_trig::TREQ_SEL_A, HSTX_CTRL, HSTX_FIFO, RESETS},
    resets::SubsystemReset,
    typelevel::{Is, OptionTSome, Sealed},
};
use core::marker::PhantomData;

mod pins;
pub use pins::*;
/// State of HSTX
pub trait State: Sealed {}

/// HSTX is disabled, FIFO is not being drained
pub struct Disabled {
    __private: (),
}

/// HSTX is enabled, data from FIFO is being consumed
pub struct Enabled {
    __private: (),
}

impl State for Disabled {}
impl Sealed for Disabled {}
impl State for Enabled {}
impl Sealed for Enabled {}

/// HSTX peripheral
pub struct Hstx<S: State, P: ValidHstxPinout> {
    ctrl: HSTX_CTRL,
    fifo: HSTX_FIFO,
    pins: P,
    state: PhantomData<S>,
}

impl<P: ValidHstxPinout> Hstx<Disabled, P> {
    /// Instantiate and reset HSTX peripheral
    pub fn new(
        ctrl: HSTX_CTRL,
        fifo: HSTX_FIFO,
        pins: P,
        resets: &mut RESETS,
    ) -> Hstx<Disabled, P> {
        ctrl.reset_bring_down(resets);
        ctrl.reset_bring_up(resets);
        Hstx {
            ctrl,
            fifo,
            pins,
            state: PhantomData,
        }
    }

    /// Free HSTX peripheral and get registers and pins back
    pub fn free(self) -> (HSTX_CTRL, HSTX_FIFO, P) {
        (self.ctrl, self.fifo, self.pins)
    }

    /// Configure clock generator
    pub fn configure_clk(&mut self, div: u8, phase: u8) {
        if phase >= 2 * div {
            panic!("phase should be strictly less than double the div");
        }

        self.ctrl
            .csr()
            .modify(|_, w| unsafe { w.clkdiv().bits(div).clkphase().bits(phase) });
    }

    /// Configure output shifter
    pub fn configure_shifter(&mut self, shift: u8, n_shifts: u8) {
        self.ctrl
            .csr()
            .modify(|_, w| unsafe { w.shift().bits(shift).n_shifts().bits(n_shifts) });
    }

    /// Configure expand shifter
    pub fn configure_expand_shifter(
        &mut self,
        enc_shift: u8,
        enc_n_shifts: u8,
        raw_shift: u8,
        raw_n_shifts: u8,
    ) {
        self.ctrl.expand_shift().write(|w| unsafe {
            w.enc_shift()
                .bits(enc_shift)
                .enc_n_shifts()
                .bits(enc_n_shifts)
                .raw_shift()
                .bits(raw_shift)
                .raw_n_shifts()
                .bits(raw_n_shifts)
        });
    }

    /// Configure TMDS expander
    pub fn configure_tmds(
        &mut self,
        l0_rot: u8,
        l0_nbits: u8,
        l1_rot: u8,
        l1_nbits: u8,
        l2_rot: u8,
        l2_nbits: u8,
    ) {
        self.ctrl.expand_tmds().write(|w| unsafe {
            w.l0_rot()
                .bits(l0_rot)
                .l0_nbits()
                .bits(l0_nbits)
                .l1_rot()
                .bits(l1_rot)
                .l1_nbits()
                .bits(l1_nbits)
                .l2_rot()
                .bits(l2_rot)
                .l2_nbits()
                .bits(l2_nbits)
        });
    }

    /// Enable expander
    pub fn enable_expander(&mut self) {
        self.ctrl.csr().modify(|_, w| w.expand_en().set_bit());
    }

    /// Disable expander
    pub fn disable_expander(&mut self) {
        self.ctrl.csr().modify(|_, w| w.expand_en().clear_bit());
    }

    /// Enable the output and start shifting data from the FIFO
    pub fn enable(self) -> Hstx<Enabled, P> {
        self.ctrl.csr().modify(|_, w| w.en().set_bit());

        Hstx {
            ctrl: self.ctrl,
            fifo: self.fifo,
            pins: self.pins,
            state: PhantomData,
        }
    }
}

impl<P: ValidHstxPinout> Hstx<Enabled, P> {
    /// Disable the output, stop shifting data from the FIFO
    pub fn disable(self) -> Hstx<Disabled, P> {
        self.ctrl.csr().modify(|_, w| w.en().clear_bit());

        Hstx {
            ctrl: self.ctrl,
            fifo: self.fifo,
            pins: self.pins,
            state: PhantomData,
        }
    }

    /// Blocking write to FIFO
    pub fn fifo_write(&mut self, word: u32) {
        while self.fifo_is_full() {}

        self.fifo.fifo().write(|w| unsafe { w.bits(word) });
    }

    /// Check if fifo is empty
    pub fn fifo_is_empty(&self) -> bool {
        self.fifo.stat().read().empty().bit_is_set()
    }

    /// Check if fifo is full
    pub fn fifo_is_full(&self) -> bool {
        self.fifo.stat().read().full().bit_is_set()
    }

    /// Check if fifo was written while full
    pub fn fifo_is_wof(&self) -> bool {
        self.fifo.stat().read().wof().bit_is_set()
    }

    /// Clear "written while full" bit
    pub fn clear_fifo_wof(&mut self) {
        self.fifo.stat().write(|w| w.wof().clear_bit_by_one());
    }
}

// Safety: This only writes to the TX fifo, so it doesn't
// interact with rust-managed memory.
unsafe impl<P: ValidHstxPinout> WriteTarget for Hstx<Enabled, P> {
    type TransmittedWord = u32;

    fn tx_treq() -> Option<u8> {
        Some(TREQ_SEL_A::HSTX.into())
    }

    fn tx_address_count(&mut self) -> (u32, u32) {
        (self.fifo.fifo().as_ptr() as u32, u32::MAX)
    }

    fn tx_increment(&self) -> bool {
        false
    }
}

impl<P: ValidHstxPinout> EndlessWriteTarget for Hstx<Enabled, P> {}

/// Configuration of HSTX bit
pub enum HstxBitConfig {
    /// Configure a bit as a clock output
    Clk {
        /// Invert the output
        inv: bool,
    },

    /// Configure a bit to output a bit from the output shifter
    Shift {
        /// Shift register bit for the first half of HSTX clock cycle
        sel_p: u8,

        /// Shift register bit for the second half of HSTX clock cycle
        sel_n: u8,

        /// Invert the output
        inv: bool,
    },
}

macro_rules! configure_bits_hstx {
    ( $( $bit:expr ),* ) => {
        paste::paste!{
            $(
                impl<P: ValidHstxPinout, B> Hstx<Disabled, P>
                    where P::[<Bit $bit>]: Is<Type = OptionTSome<B>>,
                          B: [<ValidHstxBit $bit Pin>] {
                    #[doc = "Configure output bit " $bit]
                    pub fn [<configure_bit$bit>](&mut self, config: HstxBitConfig) {
                        match config {
                            HstxBitConfig::Clk{inv: inv} => {
                                self.ctrl.[<bit $bit>]().write(|w| {
                                    w.clk().set_bit();
                                    if inv {
                                        w.inv().set_bit();
                                    }
                                    w
                                })
                            },
                            HstxBitConfig::Shift{sel_p: sel_p, sel_n: sel_n, inv: inv} => {
                                self.ctrl.[<bit $bit>]().write(|w| unsafe {
                                    w.sel_p().bits(sel_p)
                                    .sel_n().bits(sel_n);

                                    if inv {
                                        w.inv().set_bit();
                                    }

                                    w
                                })
                            }
                        }
                    }
                }
            )*
        }
    }
}

configure_bits_hstx!(0, 1, 2, 3, 4, 5, 6, 7);
