//! Subsystem Resets
// See [Chapter 2 Section 14](https://datasheets.raspberrypi.org/rp2040/rp2040_datasheet.pdf) for more details
use rp2040_pac as pac;

mod private {
    pub trait SubsystemReset {
        fn reset_bring_up(&self, resets: &mut pac::RESETS);
        fn reset_bring_down(&self, resets: &mut pac::RESETS);
    }
}

pub(crate) use private::SubsystemReset;

macro_rules! generate_reset {
    ($MODULE:ident, $module:ident) => {
        impl SubsystemReset for pac::$MODULE {
            fn reset_bring_up(&self, resets: &mut pac::RESETS) {
                resets.reset.modify(|_, w| w.$module().clear_bit());
                while resets.reset_done.read().$module().bit_is_clear() {}
            }
            fn reset_bring_down(&self, resets: &mut pac::RESETS) {
                resets.reset.modify(|_, w| w.$module().set_bit());
            }
        }
    };
}

// In datasheet order
generate_reset!(USBCTRL_REGS, usbctrl);
generate_reset!(UART1, uart1);
generate_reset!(UART0, uart0);
generate_reset!(TIMER, timer);
generate_reset!(TBMAN, tbman);
generate_reset!(SYSINFO, sysinfo);
generate_reset!(SYSCFG, syscfg);
generate_reset!(SPI1, spi1);
generate_reset!(SPI0, spi0);
generate_reset!(RTC, rtc);
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
