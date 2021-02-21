//! Clock related parts

mod unstable;

pub mod pll;
pub mod xosc;

use embedded_time::rate::{Extensions, Hertz, Megahertz};

/// A list of the configured and frozen clock rates
pub struct ClockRates {
    clk_ref: Megahertz,
    clk_sys: Megahertz,
    clk_peri: Megahertz,
    clk_usb: Megahertz,
    clk_adc: Megahertz,
    clk_rtc: Hertz,
}

impl ClockRates {
    /// The clock rate for `clk_ref`
    pub fn clk_ref(&self) -> Megahertz {
        self.clk_ref
    }

    /// The clock rate for `clk_sys`
    pub fn clk_sys(&self) -> Megahertz {
        self.clk_sys
    }

    /// The clock rate for `clk_peri`
    pub fn clk_peri(&self) -> Megahertz {
        self.clk_peri
    }

    /// The clock rate for `clk_usb`
    pub fn clk_usb(&self) -> Megahertz {
        self.clk_usb
    }

    /// The clock rate for `clk_adc`
    pub fn clk_adc(&self) -> Megahertz {
        self.clk_adc
    }

    /// The clock rate for `clk_rtc`
    pub fn clk_rtc(&self) -> Hertz {
        self.clk_rtc
    }
}

/// Initialize all clocks to their nominal rates, according to datasheet section 2.15.3.1
pub fn init_nominal(
    xosc: pac::XOSC,
    pll_sys: pac::PLL_SYS,
    pll_usb: pac::PLL_USB,
    clocks: pac::CLOCKS,
    xosc_rate: embedded_time::rate::Generic<u32>,
) -> ClockRates {
    let nominal = ClockRates {
        clk_ref: 12.MHz(),
        clk_sys: 125.MHz(),
        clk_peri: 125.MHz(),
        clk_usb: 48.MHz(),
        clk_adc: 48.MHz(),
        clk_rtc: 46875.Hz(),
    };

    // Start and wait for oscillator
    let xosc = xosc::XOsc::new_stable_blocking(xosc, xosc_rate)
        .unwrap()
        .leak();

    // Start both PLLs
    let mut pll_sys = pll::Pll::new(pll_sys, xosc, nominal.clk_sys.into())
        .unwrap()
        .enable();
    let mut pll_usb = pll::Pll::new(pll_usb, xosc, nominal.clk_usb.into())
        .unwrap()
        .enable();

    // Wait for both PLLs to be stable
    let sys_token = nb::block!(pll_sys.await_stable()).unwrap();
    let usb_token = nb::block!(pll_usb.await_stable()).unwrap();

    let _pll_sys = pll_sys.stable(sys_token).leak();
    let _pll_usb = pll_usb.stable(usb_token).leak();

    // ToDo: Replace the code from here down with methods from `self::unstable`

    // Switch clk_ref over to xosc
    clocks.clk_ref_ctrl.write(|w| w.src().xosc_clksrc());

    // ToDo: figure out which selected bit should be set, so we can wait for it
    // loop {
    //     clocks.clk_ref_selected.read().bits()
    // }

    // Switch clk_sys to pll
    clocks.clk_sys_ctrl.write(|w| w.src().clksrc_clk_sys_aux());
    // ToDo: wait for selected to have the correct bit set
    clocks
        .clk_sys_ctrl
        .modify(|_, w| w.auxsrc().clksrc_pll_sys().src().clksrc_clk_sys_aux());

    // Disable peri, usb, adc and rtc clocks so we can switch them without glitching
    clocks.clk_peri_ctrl.write(|w| w.enable().clear_bit());
    clocks.clk_usb_ctrl.write(|w| w.enable().clear_bit());
    clocks.clk_adc_ctrl.write(|w| w.enable().clear_bit());
    clocks.clk_rtc_ctrl.write(|w| w.enable().clear_bit());

    // ToDo: Wait 2 clocks of the slowest clock

    clocks
        .clk_peri_ctrl
        .modify(|_, w| w.auxsrc().clksrc_pll_sys().enable().set_bit());

    clocks
        .clk_usb_ctrl
        .modify(|_, w| w.auxsrc().clksrc_pll_usb().enable().set_bit());
    clocks.clk_usb_div.reset();

    clocks
        .clk_adc_ctrl
        .modify(|_, w| w.auxsrc().clksrc_pll_usb().enable().set_bit());
    clocks.clk_adc_div.reset();

    clocks
        .clk_rtc_ctrl
        .modify(|_, w| w.auxsrc().xosc_clksrc().enable().set_bit());
    clocks
        .clk_rtc_div
        .write(|w| unsafe { w.frac().bits(0).int().bits(256) });

    nominal
}
