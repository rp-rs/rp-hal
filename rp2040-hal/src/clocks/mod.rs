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

/// Reference Clock
pub struct ReferenceClock {
    shared_dev: ShareableClocks,
}
xosc_source!(ReferenceClock, clk_ref_ctrl);
rosc_source!(ReferenceClock, clk_ref_ctrl);
selfaux_source!(ReferenceClock, clk_ref_ctrl, clksrc_clk_ref_aux);
gpin0_auxsource!(ReferenceClock, clk_ref_ctrl);
gpin1_auxsource!(ReferenceClock, clk_ref_ctrl);
pll_usb_auxsource!(ReferenceClock, clk_ref_ctrl);
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

selfaux_source!(SystemClock, clk_sys_ctrl, clksrc_clk_sys_aux);
clockref_source!(SystemClock, clk_sys_ctrl);
gpin0_auxsource!(SystemClock, clk_sys_ctrl);
gpin1_auxsource!(SystemClock, clk_sys_ctrl);
pll_usb_auxsource!(SystemClock, clk_sys_ctrl);
pll_sys_auxsource!(SystemClock, clk_sys_ctrl);
xosc_auxsource!(SystemClock, clk_sys_ctrl);
rosc_auxsource!(SystemClock, clk_sys_ctrl);
int_division!(SystemClock, clk_sys_div, u32);
frac_division!(SystemClock, clk_sys_div, u8);

/// Peripheral Clock
pub struct PeripheralClock {
    shared_dev: ShareableClocks,
}
gpin0_auxsource!(PeripheralClock, clk_peri_ctrl);
gpin1_auxsource!(PeripheralClock, clk_peri_ctrl);
pll_usb_auxsource!(PeripheralClock, clk_peri_ctrl);
pll_sys_auxsource!(PeripheralClock, clk_peri_ctrl);
xosc_auxsource!(PeripheralClock, clk_peri_ctrl);
rosc_ph_auxsource!(PeripheralClock, clk_peri_ctrl);
clocksys_auxsource!(PeripheralClock, clk_peri_ctrl);
clock_generator!(PeripheralClock, clk_peri_ctrl);
