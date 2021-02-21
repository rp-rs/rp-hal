//! The clocks block, containing all clock generators

use pac::CLOCKS;
use sharing::SharedClocksBlock;

/// Split up of the clocks block
///
/// This allows to use the different clock generators and devices independently.
pub struct SplitClocks {
    /// Clock output to GPIO 0
    pub gp_out0: ClkGPOut0,

    /// Clock output to GPIO 1
    pub gp_out1: ClkGPOut1,

    /// Clock output to GPIO 2
    pub gp_out2: ClkGPOut2,

    /// Clock output to GPIO 3
    pub gp_out3: ClkGPOut3,

    /// Clock output to UART and SPI
    pub peri: ClkPeri,

    /// Clock output to USB
    pub usb: ClkUsb,

    /// Clock output to ADC
    pub adc: ClkAdc,

    /// Clock output to RTC
    pub rtc: ClkRtc,

    /// Frequency counter (see Type description for details)
    pub frq_cnt: FrequencyCounter,

    /// Resus (see Type description for details)
    pub resus: Resus,

    /// Clock gates (see Type description for details)
    pub gates: ClkGates,

    /// Register block handle (see Type description for details)
    pub _dev: ClockHandle,
}

impl SplitClocks {
    /// Split the clocks block into its parts
    pub fn split(mut dev: CLOCKS) -> Self {
        let share = SharedClocksBlock::new(&mut dev);
        SplitClocks {
            gp_out0: ClkGPOut0 {
                share: share.copy(),
            },
            gp_out1: ClkGPOut1 {
                share: share.copy(),
            },
            gp_out2: ClkGPOut2 {
                share: share.copy(),
            },
            gp_out3: ClkGPOut3 {
                share: share.copy(),
            },
            peri: ClkPeri {
                share: share.copy(),
            },
            usb: ClkUsb {
                share: share.copy(),
            },
            adc: ClkAdc {
                share: share.copy(),
            },
            rtc: ClkRtc {
                share: share.copy(),
            },
            frq_cnt: FrequencyCounter {
                share: share.copy(),
            },
            resus: Resus {
                share: share.copy(),
            },
            gates: ClkGates {
                share: share.copy(),
            },
            _dev: ClockHandle { _private: dev },
        }
    }

    /// Return the PAC register block
    ///
    /// Since this function can only be called with a complete set of parts it should be save to
    /// get the register block back.
    pub fn free(self) -> CLOCKS {
        self._dev._private
    }
}

/// Clock gates used to enable or disable peripherals. See section 2.15.3.5 of the datasheet.
#[allow(dead_code)] // ToDo: Implement clock gating
pub struct ClkGates {
    share: SharedClocksBlock,
}

/// See section 2.15.4 of the datasheet
#[allow(dead_code)] // ToDo: Implement frequency counting
pub struct FrequencyCounter {
    share: SharedClocksBlock,
}

/// See section 2.15.5 of the datasheet
#[allow(dead_code)] // ToDo: Implement resus device
pub struct Resus {
    share: SharedClocksBlock,
}

/// Helper struct to hold the register block so it could be freed again. Does not give any
/// access to anything, since that should only be possible via the different structs like
/// `ClkGates`, `ClkSys` or `FrequencyCounter`.
pub struct ClockHandle {
    _private: CLOCKS,
}

/// A generator that can be switched only glitching
///
/// Provides the method provided in the datasheet to switch glitchless even with these generators.
pub trait GlitchingClockSwitch<Src> {
    /// Disable the clock source cleanly
    fn disable(&mut self);

    /// Enable the clock source cleanly
    fn enable(&mut self);

    /// Select the AUX source (glitching allowed)
    fn set_aux_src(&mut self, src: Src);

    /// Switch the clock according to the datasheet, avoiding glitches
    fn switch_glitchless(&mut self, src: Src) {
        self.disable();

        // ToDo: Wait 2 cycles

        self.set_aux_src(src);
        self.enable();

        // ToDo: Wait 2 cycles
    }
}

/// Parameters for a fractional clock divisor. See datasheet section 2.15.3.3 for details.
pub struct FractionalDivisor {
    int: u32,
    frac: u8,
}

impl FractionalDivisor {
    /// Create a new divisor, making sure the integral part fits into the register.
    pub const fn new(int: u32, frac: u8) -> Option<Self> {
        if int > 0x00_ff_ff_ff {
            return None;
        }

        Some(FractionalDivisor { int, frac })
    }
}

/// A clock source that can be divided with a fractional value
pub trait FractionalDiv {
    /// Get current divider value
    fn div(&self) -> FractionalDivisor;
    /// Set divider
    fn set_div(&mut self, div: FractionalDivisor);
}

/// A clock source with an integer divider
pub trait IntDiv {
    /// Get current divider value
    fn div(&self) -> u8;
    /// Set divider
    fn set_div(&mut self, div: u8);
}

/// A clock generator that has duty cycle correction. See section 2.15.3.4 for details
pub trait DutyCycleCorrect {
    /// Returns `true` if the correction is active.
    fn dc50(&self) -> bool;
    /// Set the correction (`true` => enabled)
    fn set_dc50(&mut self, correct: bool);
}

macro_rules! clock_generator {
    ($name:ident, $ctrl:ident) => {
        /// Clock generator for $name
        ///
        /// ToDo: Find a good way to add doc comments for all the generators
        pub struct $name {
            share: SharedClocksBlock,
        }

        impl GlitchingClockSwitch<$crate::pac::clocks::$ctrl::AUXSRC_A> for $name {
            fn disable(&mut self) {
                unsafe { &self.share.get().$ctrl }.modify(|_, w| w.enable().clear_bit());
            }

            fn enable(&mut self) {
                unsafe { &self.share.get().$ctrl }.modify(|_, w| w.enable().set_bit());
            }

            fn set_aux_src(&mut self, src: $crate::pac::clocks::$ctrl::AUXSRC_A) {
                unsafe { &self.share.get().$ctrl }.modify(|_, w| w.auxsrc().variant(src));
            }
        }
    };
}

macro_rules! dc_correct {
    ($name:ident, $ctrl:ident) => {
        impl DutyCycleCorrect for $name {
            fn dc50(&self) -> bool {
                unsafe { &self.share.get().$ctrl }
                    .read()
                    .dc50()
                    .bit_is_set()
            }

            fn set_dc50(&mut self, correct: bool) {
                unsafe { &self.share.get().$ctrl }.modify(|_, w| w.dc50().bit(correct));
            }
        }
    };
}

macro_rules! frac_div {
    ($name:ident, $div:ident) => {
        impl FractionalDiv for $name {
            fn div(&self) -> FractionalDivisor {
                let reg = unsafe { &self.share.get().$div }.read();
                FractionalDivisor {
                    int: reg.int().bits(),
                    frac: reg.frac().bits(),
                }
            }

            fn set_div(&mut self, div: FractionalDivisor) {
                unsafe { &self.share.get() }
                    .$div
                    .write(|w| unsafe { w.int().bits(div.int).frac().bits(div.frac) });
            }
        }
    };
}

macro_rules! int_div {
    ($name:ident, $div:ident) => {
        impl IntDiv for $name {
            fn div(&self) -> u8 {
                unsafe { &self.share.get().$div }.read().int().bits()
            }

            fn set_div(&mut self, div: u8) {
                unsafe { &self.share.get() }
                    .$div
                    .write(|w| unsafe { w.int().bits(div) });
            }
        }
    };
}

macro_rules! gp_out {
    ($name:ident, $ctrl:ident, $div:ident) => {
        clock_generator! {$name, $ctrl}
        dc_correct! {$name, $ctrl}
        frac_div! {$name, $div}
    };
}

gp_out! {ClkGPOut0, clk_gpout0_ctrl, clk_gpout0_div}
gp_out! {ClkGPOut1, clk_gpout1_ctrl, clk_gpout1_div}
gp_out! {ClkGPOut2, clk_gpout2_ctrl, clk_gpout2_div}
gp_out! {ClkGPOut3, clk_gpout3_ctrl, clk_gpout3_div}

clock_generator! {ClkPeri, clk_peri_ctrl}

clock_generator! {ClkUsb, clk_usb_ctrl}
int_div! {ClkUsb, clk_usb_div}

clock_generator! {ClkAdc, clk_adc_ctrl}
int_div! {ClkAdc, clk_adc_div}

clock_generator! {ClkRtc, clk_rtc_ctrl}
frac_div! {ClkRtc, clk_rtc_div}

mod sharing {
    use pac::clocks::RegisterBlock;
    use pac::CLOCKS;

    pub struct SharedClocksBlock {
        _private: (),
    }

    impl SharedClocksBlock {
        pub fn new(_dev: &mut CLOCKS) -> SharedClocksBlock {
            SharedClocksBlock { _private: () }
        }

        /// Safety: Each of the clock generators may only use its matching registers (CLK_*_(CTRL|DIV|SELECTED))
        pub unsafe fn get(&self) -> &RegisterBlock {
            &*CLOCKS::ptr()
        }

        pub fn copy(&self) -> Self {
            SharedClocksBlock { _private: () }
        }
    }
}
