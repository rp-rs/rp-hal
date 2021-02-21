use pac::CLOCKS;
use sharing::SharedClocksBlock;

pub struct SplitClocks {
    pub gp_out0: ClkGPOut0,
    pub gp_out1: ClkGPOut1,
    pub gp_out2: ClkGPOut2,
    pub gp_out3: ClkGPOut3,
    pub peri: ClkPeri,
    pub usb: ClkUsb,
    pub adc: ClkAdc,
    pub rtc: ClkRtc,
    pub frq_cnt: FrequencyCounter,
    pub resus: Resus,
    pub gates: ClkGates,
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
    pub fn free(self) -> CLOCKS {
        // Safety: free() can only be called if all parts have been placed back thus we can release
        //         the device
        self._dev._private
    }
}

pub struct ClkGates {
    share: SharedClocksBlock,
}

pub struct FrequencyCounter {
    share: SharedClocksBlock,
}

pub struct Resus {
    share: SharedClocksBlock,
}

pub struct ClockHandle {
    _private: CLOCKS,
}

pub trait GlitchingClockSwitch<Src> {
    fn disable(&mut self);
    fn enable(&mut self);
    fn set_aux_src(&mut self, src: Src);

    fn switch_glitchless(&mut self, src: Src) {
        self.disable();

        // ToDo: Wait 2 cycles

        self.set_aux_src(src);
        self.enable();

        // ToDo: Wait 2 cycles
    }
}

pub struct FractionalDivisor {
    int: u32,
    frac: u8,
}

impl FractionalDivisor {
    pub fn new(int: u32, frac: u8) -> Option<Self> {
        if int > 0x00_ff_ff_ff {
            return None;
        }

        Some(FractionalDivisor { int, frac })
    }
}

pub trait FractionalDiv {
    fn div(&self) -> FractionalDivisor;
    fn set_div(&mut self, div: FractionalDivisor);
}

pub trait IntDiv {
    fn div(&self) -> u8;
    fn set_div(&mut self, div: u8);
}

pub trait DutyCycleCorrect {
    fn dc50(&self) -> bool;
    fn set_dc50(&mut self, correct: bool);
}

macro_rules! clock_generator {
    ($name:ident, $ctrl:ident) => {
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
