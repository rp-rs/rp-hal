//! Subsystem Resets
//!
//! See [Chapter 7](https://rptl.io/rp2350-datasheet#section_resets) for more details.

mod private {
    pub trait SubsystemReset {
        fn reset_bring_up(&self, resets: &mut crate::pac::RESETS);
        fn reset_bring_down(&self, resets: &mut crate::pac::RESETS);
    }
}

pub(crate) use private::SubsystemReset;

macro_rules! generate_reset {
    ($MODULE:ident, $module:ident) => {
        impl SubsystemReset for $crate::pac::$MODULE {
            fn reset_bring_up(&self, resets: &mut $crate::pac::RESETS) {
                resets.reset().modify(|_, w| w.$module().clear_bit());
                while resets.reset_done().read().$module().bit_is_clear() {}
            }
            fn reset_bring_down(&self, resets: &mut $crate::pac::RESETS) {
                resets.reset().modify(|_, w| w.$module().set_bit());
            }
        }
    };
}

// In datasheet order
generate_reset!(USB, usbctrl);
generate_reset!(UART1, uart1);
generate_reset!(UART0, uart0);
generate_reset!(TIMER0, timer0);
generate_reset!(TIMER1, timer1);
generate_reset!(TBMAN, tbman);
generate_reset!(SYSINFO, sysinfo);
generate_reset!(SYSCFG, syscfg);
generate_reset!(SPI1, spi1);
generate_reset!(SPI0, spi0);
generate_reset!(HSTX_CTRL, hstx);
generate_reset!(PWM, pwm);
generate_reset!(PLL_USB, pll_usb);
generate_reset!(PLL_SYS, pll_sys);
generate_reset!(PIO1, pio1);
generate_reset!(PIO0, pio0);
generate_reset!(PADS_QSPI, pads_qspi);
generate_reset!(PADS_BANK0, pads_bank0);
//generate_reset!(JTAG,jtag); // This doesn't seem to have an item in the pac
generate_reset!(IO_QSPI, io_qspi);
generate_reset!(IO_BANK0, io_bank0);
generate_reset!(I2C1, i2c1);
generate_reset!(I2C0, i2c0);
generate_reset!(DMA, dma);
generate_reset!(BUSCTRL, busctrl);
generate_reset!(ADC, adc);
