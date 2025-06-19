//! High-speed transmitter (HSTX)
//!
//! See [Section 12.3](https://rptl.io/rp2350-datasheet#section_hstx) for more details.
//!

use crate::{
    dma::{EndlessWriteTarget, WriteTarget},
    gpio::{bank0::*, AnyPin, FunctionHstx},
    pac::{dma::ch::ch_ctrl_trig::TREQ_SEL_A, HSTX_CTRL, HSTX_FIFO, RESETS},
    resets::SubsystemReset,
    typelevel::{Is, OptionT, OptionTNone, OptionTSome, Sealed},
};
use core::marker::PhantomData;

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

macro_rules! hstx_pins {
    ( $( $bit:expr => $pin:expr ),* ) => {
        paste::paste!{
            $(
                #[doc = "Indicates a valid bit " $bit " pin for HSTX"]
                pub trait [<ValidHstxBit $bit Pin>] : Sealed {}

                impl<T> [<ValidHstxBit $bit Pin>] for T
                where
                    T: AnyPin<Function = FunctionHstx, Id = [<Gpio $pin>]>
                {
                }

                #[doc = "Indicates a valid optional bit " $bit " pin for HSTX"]
                pub trait [<ValidOptionHstxBit $bit>]: OptionT {}

                impl [<ValidOptionHstxBit $bit>] for OptionTNone {}
                impl<T> [<ValidOptionHstxBit $bit>] for OptionTSome<T>
                where
                    T: [<ValidHstxBit $bit Pin>],
                {
                }

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

/// Declares a valid HSTX pinout
pub trait ValidHstxPinout: Sealed {
    #[allow(missing_docs)]
    type Bit0: ValidOptionHstxBit0;
    #[allow(missing_docs)]
    type Bit1: ValidOptionHstxBit1;
    #[allow(missing_docs)]
    type Bit2: ValidOptionHstxBit2;
    #[allow(missing_docs)]
    type Bit3: ValidOptionHstxBit3;
    #[allow(missing_docs)]
    type Bit4: ValidOptionHstxBit4;
    #[allow(missing_docs)]
    type Bit5: ValidOptionHstxBit5;
    #[allow(missing_docs)]
    type Bit6: ValidOptionHstxBit6;
    #[allow(missing_docs)]
    type Bit7: ValidOptionHstxBit7;
}

/// Set of valid HSTX pins
pub struct HstxPins<Opt0, Opt1, Opt2, Opt3, Opt4, Opt5, Opt6, Opt7>
where
    Opt0: ValidOptionHstxBit0,
    Opt1: ValidOptionHstxBit1,
    Opt2: ValidOptionHstxBit2,
    Opt3: ValidOptionHstxBit3,
    Opt4: ValidOptionHstxBit4,
    Opt5: ValidOptionHstxBit5,
    Opt6: ValidOptionHstxBit6,
    Opt7: ValidOptionHstxBit7,
{
    /// Optional bit 0 pin
    pub bit0: Opt0,
    /// Optional bit 1 pin
    pub bit1: Opt1,
    /// Optional bit 2 pin
    pub bit2: Opt2,
    /// Optional bit 3 pin
    pub bit3: Opt3,
    /// Optional bit 4 pin
    pub bit4: Opt4,
    /// Optional bit 5 pin
    pub bit5: Opt5,
    /// Optional bit 6 pin
    pub bit6: Opt6,
    /// Optional bit 7 pin
    pub bit7: Opt7,
}

impl<
        Opt0: ValidOptionHstxBit0,
        Opt1: ValidOptionHstxBit1,
        Opt2: ValidOptionHstxBit2,
        Opt3: ValidOptionHstxBit3,
        Opt4: ValidOptionHstxBit4,
        Opt5: ValidOptionHstxBit5,
        Opt6: ValidOptionHstxBit6,
        Opt7: ValidOptionHstxBit7,
    > Sealed for HstxPins<Opt0, Opt1, Opt2, Opt3, Opt4, Opt5, Opt6, Opt7>
{
}

impl<
        Opt0: ValidOptionHstxBit0,
        Opt1: ValidOptionHstxBit1,
        Opt2: ValidOptionHstxBit2,
        Opt3: ValidOptionHstxBit3,
        Opt4: ValidOptionHstxBit4,
        Opt5: ValidOptionHstxBit5,
        Opt6: ValidOptionHstxBit6,
        Opt7: ValidOptionHstxBit7,
    > ValidHstxPinout for HstxPins<Opt0, Opt1, Opt2, Opt3, Opt4, Opt5, Opt6, Opt7>
{
    type Bit0 = Opt0;
    type Bit1 = Opt1;
    type Bit2 = Opt2;
    type Bit3 = Opt3;
    type Bit4 = Opt4;
    type Bit5 = Opt5;
    type Bit6 = Opt6;
    type Bit7 = Opt7;
}

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

hstx_pins!(0 => 12, 1 => 13, 2 => 14, 3 => 15, 4 => 16, 5 => 17, 6 => 18, 7 => 19);

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
