//! Clocks (CLOCKS)
// See [Chapter 2 Section 15](https://datasheets.raspberrypi.org/rp2040/rp2040_datasheet.pdf) for more details

use crate::pac::*;

#[macro_use]
mod macros;

#[derive(Copy, Clone)]
/// Provides refs to the CLOCKS block.
pub struct ShareableClocks {
    _internal: (),
}

impl ShareableClocks {
    fn new(_clocks: &mut CLOCKS) -> Self {
        ShareableClocks { _internal: () }
    }

    unsafe fn get(&self) -> &clocks::RegisterBlock {
        &*CLOCKS::ptr()
    }
}

/// Abstraction layer providing Clock Management.
pub struct ClocksManager {
    clocks: CLOCKS,
    shared_clocks: ShareableClocks,
}
impl ClocksManager {
    /// Exchanges CLOCKS block against Self.
    pub fn new(mut clocks_block: CLOCKS) -> Self {
        let shared_clocks = ShareableClocks::new(&mut clocks_block);
        ClocksManager {
            clocks: clocks_block,
            shared_clocks,
        }
    }

    /// Releases the CLOCKS block
    pub fn free(self) -> CLOCKS {
        self.clocks
    }

    /// Getter for the GPIO Output 0 Clock.
    pub fn gpio_output0_clock(&self) -> GpioOutput0Clock {
        GpioOutput0Clock {
            shared_dev: self.shared_clocks,
        }
    }

    /// Getter for the GPIO Output 1 Clock.
    pub fn gpio_output1_clock(&self) -> GpioOutput1Clock {
        GpioOutput1Clock {
            shared_dev: self.shared_clocks,
        }
    }

    /// Getter for the GPIO Output 2 Clock.
    pub fn gpio_output2_clock(&self) -> GpioOutput2Clock {
        GpioOutput2Clock {
            shared_dev: self.shared_clocks,
        }
    }

    /// Getter for the GPIO Output 3 Clock.
    pub fn gpio_output3_clock(&self) -> GpioOutput3Clock {
        GpioOutput3Clock {
            shared_dev: self.shared_clocks,
        }
    }

    /// Getter for the Reference Clock.
    pub fn ref_clock(&self) -> ReferenceClock {
        ReferenceClock {
            shared_dev: self.shared_clocks,
        }
    }

    /// Getter for the System Clock
    pub fn sys_clock(&self) -> SystemClock {
        SystemClock {
            shared_dev: self.shared_clocks,
        }
    }

    /// Getter for the PeripheralClock
    pub fn peripheral_clock(&self) -> PeripheralClock {
        PeripheralClock {
            shared_dev: self.shared_clocks,
        }
    }

    /// Getter for the Usb Clock
    pub fn usb_clock(&self) -> UsbClock {
        UsbClock {
            shared_dev: self.shared_clocks,
        }
    }

    /// Getter for the Adc Clock
    pub fn adc_clock(&self) -> AdcClock {
        AdcClock {
            shared_dev: self.shared_clocks,
        }
    }

    /// Getter for the Rtc Clock
    pub fn rtc_clock(&self) -> RtcClock {
        RtcClock {
            shared_dev: self.shared_clocks,
        }
    }
}

/// For clocks with an integer divider.
pub trait IntegerDivision {
    /// Set integer divider value.
    fn set_int_div(&mut self, div: usize);
}

/// For clocks with a fraction divider.
pub trait FractionDivision {
    /// Set fraction divider value.
    fn set_frac_div(&mut self, div: usize);
}

/// For clocks that can have XOSC as source.
pub trait XOSCClockSource {
    /// Set XOSC as a source.
    fn set_xosc_src(&mut self);
}
/// For clocks that can have ROSC as source.
pub trait ROSCClockSource {
    /// set ROSC as a source.
    fn set_rosc_src(&mut self);
}
/// For clocks that can have ... itself (?) as a source (is that the "glitchless mux" ?)
pub trait SelfAuxClockSource {
    /// Set ...
    fn set_self_aux_src(&mut self);
}
/// For clocks that can have the Reference Clock as source.
pub trait ClockREFClockSource {
    /// Set Reference Clock as
    fn set_clkref_src(&mut self);
}
/// For clocks that can have the System Clock as an auxilliary source.
pub trait ClockSYSClockAuxSource {
    /// Set System Clock as source.
    fn set_clksys_auxsrc(&mut self);
}

/// For clocks that can have the USB Clock as an auxilliary source.
pub trait ClockUSBClockAuxSource {
    /// Set USB Clock as source.
    fn set_clkusb_auxsrc(&mut self);
}

/// For clocks that can have the ADC Clock as an auxilliary source.
pub trait ClockADCClockAuxSource {
    /// Set ADC Clock as source.
    fn set_clkadc_auxsrc(&mut self);
}

/// For clocks that can have the RTC Clock as an auxilliary source.
pub trait ClockRTCClockAuxSource {
    /// Set RTC Clock as source.
    fn set_clkrtc_auxsrc(&mut self);
}

/// For clocks that can have the Reference Clock as an auxilliary source.
pub trait ClockRefClockAuxSource {
    /// Set Reference Clock as source.
    fn set_clkref_auxsrc(&mut self);
}
/// For clocks that can have XOSC as an auxilliary source.
pub trait XOSCClockAuxSource {
    /// Set XOSC as auxilliary source.
    fn set_xosc_auxsrc(&mut self);
}
/// For clocks that can have ROSC as an auxilliary source.
pub trait ROSCClockAuxSource {
    /// Set ROSC as auxilliary source.
    fn set_rosc_auxsrc(&mut self);
}
/// For clocks that can have ROSC_PH as an auxilliary source.
pub trait ROSCPHClockAuxSource {
    /// Set ROSC_PH as auxilliary source.
    fn set_rosc_ph_auxsrc(&mut self);
}
/// For clocks that can have PLL_USB as an auxilliary source.
pub trait PLLUSBClockAuxSource {
    /// Set PLL_USB as auxilliary source.
    fn set_pll_usb_auxsrc(&mut self);
}
/// For clocks that can have PLL_SYS as an auxilliary source.
pub trait PLLSYSClockAuxSource {
    /// Set PLL_SYS as auxilliary source.
    fn set_pll_sys_auxsrc(&mut self);
}
/// For clocks that can have gpin0 as an auxilliary source.
pub trait Gpin0ClockAuxSource {
    /// Set clock to be received from gpin0 (auxilliary)
    fn set_gpin0_auxsrc(&mut self);
}
/// For clocks that can have gpin1 as an auxilliary source.
pub trait Gpin1ClockAuxSource {
    /// Set clock to be received from gpin1
    fn set_gpin1_auxsrc(&mut self);
}

/// For clocks having a generator.
pub trait ClockGenerator {
    /// Enables the clock.
    fn enable(&mut self);

    /// Disables the clock.
    fn disable(&mut self);

    /// Kills the clock.
    fn kill(&mut self);
}

/// GPIO Output 0 Clock
pub struct GpioOutput0Clock {
    shared_dev: ShareableClocks,
}

clock_generator!(GpioOutput0Clock, clk_gpout0_ctrl);

// Clock aux sources
pll_sys_auxsource!(GpioOutput0Clock, clk_gpout0_ctrl);
gpin0_auxsource!(GpioOutput0Clock, clk_gpout0_ctrl);
gpin1_auxsource!(GpioOutput0Clock, clk_gpout0_ctrl);
pll_usb_auxsource!(GpioOutput0Clock, clk_gpout0_ctrl);
rosc_auxsource!(GpioOutput0Clock, clk_gpout0_ctrl);
xosc_auxsource!(GpioOutput0Clock, clk_gpout0_ctrl);
clocksys_auxsource!(GpioOutput0Clock, clk_gpout0_ctrl);
clockusb_auxsource!(GpioOutput0Clock, clk_gpout0_ctrl);
clockadc_auxsource!(GpioOutput0Clock, clk_gpout0_ctrl);
clockrtc_auxsource!(GpioOutput0Clock, clk_gpout0_ctrl);
clockref_auxsource!(GpioOutput0Clock, clk_gpout0_ctrl);

int_division!(GpioOutput0Clock, clk_gpout0_div, u32);
frac_division!(GpioOutput0Clock, clk_gpout0_div, u8);

/// GPIO Output 1 Clock
pub struct GpioOutput1Clock {
    shared_dev: ShareableClocks,
}

clock_generator!(GpioOutput1Clock, clk_gpout1_ctrl);

// Clock aux sources
pll_sys_auxsource!(GpioOutput1Clock, clk_gpout1_ctrl);
gpin0_auxsource!(GpioOutput1Clock, clk_gpout1_ctrl);
gpin1_auxsource!(GpioOutput1Clock, clk_gpout1_ctrl);
pll_usb_auxsource!(GpioOutput1Clock, clk_gpout1_ctrl);
rosc_auxsource!(GpioOutput1Clock, clk_gpout1_ctrl);
xosc_auxsource!(GpioOutput1Clock, clk_gpout1_ctrl);
clocksys_auxsource!(GpioOutput1Clock, clk_gpout1_ctrl);
clockusb_auxsource!(GpioOutput1Clock, clk_gpout1_ctrl);
clockadc_auxsource!(GpioOutput1Clock, clk_gpout1_ctrl);
clockrtc_auxsource!(GpioOutput1Clock, clk_gpout1_ctrl);
clockref_auxsource!(GpioOutput1Clock, clk_gpout1_ctrl);

int_division!(GpioOutput1Clock, clk_gpout1_div, u32);
frac_division!(GpioOutput1Clock, clk_gpout1_div, u8);

/// GPIO Output 2 Clock
pub struct GpioOutput2Clock {
    shared_dev: ShareableClocks,
}

clock_generator!(GpioOutput2Clock, clk_gpout2_ctrl);

// Clock aux sources
pll_sys_auxsource!(GpioOutput2Clock, clk_gpout2_ctrl);
gpin0_auxsource!(GpioOutput2Clock, clk_gpout2_ctrl);
gpin1_auxsource!(GpioOutput2Clock, clk_gpout2_ctrl);
pll_usb_auxsource!(GpioOutput2Clock, clk_gpout2_ctrl);
rosc_ph_auxsource!(GpioOutput2Clock, clk_gpout2_ctrl);
xosc_auxsource!(GpioOutput2Clock, clk_gpout2_ctrl);
clocksys_auxsource!(GpioOutput2Clock, clk_gpout2_ctrl);
clockusb_auxsource!(GpioOutput2Clock, clk_gpout2_ctrl);
clockadc_auxsource!(GpioOutput2Clock, clk_gpout2_ctrl);
clockrtc_auxsource!(GpioOutput2Clock, clk_gpout2_ctrl);
clockref_auxsource!(GpioOutput2Clock, clk_gpout2_ctrl);

int_division!(GpioOutput2Clock, clk_gpout2_div, u32);
frac_division!(GpioOutput2Clock, clk_gpout2_div, u8);

/// GPIO Output 3 Clock
pub struct GpioOutput3Clock {
    shared_dev: ShareableClocks,
}

clock_generator!(GpioOutput3Clock, clk_gpout3_ctrl);

// Clock aux sources
pll_sys_auxsource!(GpioOutput3Clock, clk_gpout3_ctrl);
gpin0_auxsource!(GpioOutput3Clock, clk_gpout3_ctrl);
gpin1_auxsource!(GpioOutput3Clock, clk_gpout3_ctrl);
pll_usb_auxsource!(GpioOutput3Clock, clk_gpout3_ctrl);
rosc_ph_auxsource!(GpioOutput3Clock, clk_gpout3_ctrl);
xosc_auxsource!(GpioOutput3Clock, clk_gpout3_ctrl);
clocksys_auxsource!(GpioOutput3Clock, clk_gpout3_ctrl);
clockusb_auxsource!(GpioOutput3Clock, clk_gpout3_ctrl);
clockadc_auxsource!(GpioOutput3Clock, clk_gpout3_ctrl);
clockrtc_auxsource!(GpioOutput3Clock, clk_gpout3_ctrl);
clockref_auxsource!(GpioOutput3Clock, clk_gpout3_ctrl);

int_division!(GpioOutput3Clock, clk_gpout3_div, u32);
frac_division!(GpioOutput3Clock, clk_gpout3_div, u8);

/// Reference Clock
pub struct ReferenceClock {
    shared_dev: ShareableClocks,
}
rosc_source!(ReferenceClock, clk_ref_ctrl);
selfaux_source!(ReferenceClock, clk_ref_ctrl, clksrc_clk_ref_aux);
xosc_source!(ReferenceClock, clk_ref_ctrl);

// Clock aux sources
pll_usb_auxsource!(ReferenceClock, clk_ref_ctrl);
gpin0_auxsource!(ReferenceClock, clk_ref_ctrl);
gpin1_auxsource!(ReferenceClock, clk_ref_ctrl);

int_division!(ReferenceClock, clk_ref_div, u8);

/// System Clock
pub struct SystemClock {
    shared_dev: ShareableClocks,
}
impl SystemClock {
    /// WIP - Helper function to reset source (blocking)
    pub fn reset_source_await(&self) {
        let shared_dev = unsafe { self.shared_dev.get() };

        shared_dev.clk_sys_ctrl.write(|w| {
            w.src().clear_bit();
            w
        });

        self.await_select(0x1);
    }

    /// WIP - Helper function to select new source (blocking)
    pub fn await_select(&self, clock: u8) {
        let shared_dev = unsafe { self.shared_dev.get() };

        while shared_dev.clk_sys_selected.read().bits() != clock as u32 {
            cortex_m::asm::nop();
        }
    }
}

// Clock glitchless sources
clockref_source!(SystemClock, clk_sys_ctrl);
selfaux_source!(SystemClock, clk_sys_ctrl, clksrc_clk_sys_aux);

// Clock aux sources
pll_sys_auxsource!(SystemClock, clk_sys_ctrl);
pll_usb_auxsource!(SystemClock, clk_sys_ctrl);
rosc_auxsource!(SystemClock, clk_sys_ctrl);
xosc_auxsource!(SystemClock, clk_sys_ctrl);
gpin0_auxsource!(SystemClock, clk_sys_ctrl);
gpin1_auxsource!(SystemClock, clk_sys_ctrl);

int_division!(SystemClock, clk_sys_div, u32);
frac_division!(SystemClock, clk_sys_div, u8);

/// Peripheral Clock
pub struct PeripheralClock {
    shared_dev: ShareableClocks,
}
clock_generator!(PeripheralClock, clk_peri_ctrl);

// Clock aux sources
clocksys_auxsource!(PeripheralClock, clk_peri_ctrl);
pll_sys_auxsource!(PeripheralClock, clk_peri_ctrl);
pll_usb_auxsource!(PeripheralClock, clk_peri_ctrl);
rosc_ph_auxsource!(PeripheralClock, clk_peri_ctrl);
xosc_auxsource!(PeripheralClock, clk_peri_ctrl);
gpin0_auxsource!(PeripheralClock, clk_peri_ctrl);
gpin1_auxsource!(PeripheralClock, clk_peri_ctrl);

/// USB Clock
pub struct UsbClock {
    shared_dev: ShareableClocks,
}
clock_generator!(UsbClock, clk_usb_ctrl);

// Clock aux sources
pll_usb_auxsource!(UsbClock, clk_usb_ctrl);
pll_sys_auxsource!(UsbClock, clk_usb_ctrl);
rosc_ph_auxsource!(UsbClock, clk_usb_ctrl);
xosc_auxsource!(UsbClock, clk_usb_ctrl);
gpin0_auxsource!(UsbClock, clk_usb_ctrl);
gpin1_auxsource!(UsbClock, clk_usb_ctrl);

int_division!(UsbClock, clk_usb_div, u8);

/// Adc Clock
pub struct AdcClock {
    shared_dev: ShareableClocks,
}
clock_generator!(AdcClock, clk_adc_ctrl);

// Clock aux sources
pll_usb_auxsource!(AdcClock, clk_adc_ctrl);
pll_sys_auxsource!(AdcClock, clk_adc_ctrl);
rosc_ph_auxsource!(AdcClock, clk_adc_ctrl);
xosc_auxsource!(AdcClock, clk_adc_ctrl);
gpin0_auxsource!(AdcClock, clk_adc_ctrl);
gpin1_auxsource!(AdcClock, clk_adc_ctrl);

int_division!(AdcClock, clk_adc_div, u8);

/// RTC Clock
pub struct RtcClock {
    shared_dev: ShareableClocks,
}
clock_generator!(RtcClock, clk_rtc_ctrl);

// Clock aux sources
pll_usb_auxsource!(RtcClock, clk_rtc_ctrl);
pll_sys_auxsource!(RtcClock, clk_rtc_ctrl);
rosc_ph_auxsource!(RtcClock, clk_rtc_ctrl);
xosc_auxsource!(RtcClock, clk_rtc_ctrl);
gpin0_auxsource!(RtcClock, clk_rtc_ctrl);
gpin1_auxsource!(RtcClock, clk_rtc_ctrl);

int_division!(RtcClock, clk_rtc_div, u32);
frac_division!(RtcClock, clk_rtc_div, u8);
