use crate::{
    clocks::ClocksManager,
    gpio::Pins,
    pll::{
        common_configs::{PLL_SYS_125MHZ, PLL_USB_48MHZ},
        setup_pll_blocking, Locked, PhaseLockedLoop,
    },
    sio::Sio,
    xosc::{setup_xosc_blocking, CrystalOscillator, Stable},
};
use embedded_time::rate::Extensions;
use pac::{
    ADC, BUSCTRL, DMA, I2C0, I2C1, IO_QSPI, PADS_QSPI, PIO0, PIO1, PLL_SYS, PLL_USB, PPB, PSM, PWM,
    RESETS, ROSC, RTC, SPI0, SPI1, SYSCFG, SYSINFO, TBMAN, TIMER, UART0, UART1, USBCTRL_DPRAM,
    USBCTRL_REGS, VREG_AND_CHIP_RESET, WATCHDOG, XIP_CTRL, XIP_SSI,
};

const XOSC_HZ: u32 = 12_000_000_u32;

#[allow(non_snake_case)]
pub struct Pac {
    #[doc = "ADC"]
    pub ADC: ADC,
    #[doc = "BUSCTRL"]
    pub BUSCTRL: BUSCTRL,
    #[doc = "DMA"]
    pub DMA: DMA,
    #[doc = "I2C0"]
    pub I2C0: I2C0,
    #[doc = "I2C1"]
    pub I2C1: I2C1,
    #[doc = "IO_QSPI"]
    pub IO_QSPI: IO_QSPI,
    #[doc = "PADS_QSPI"]
    pub PADS_QSPI: PADS_QSPI,
    #[doc = "PIO0"]
    pub PIO0: PIO0,
    #[doc = "PIO1"]
    pub PIO1: PIO1,
    #[doc = "PPB"]
    pub PPB: PPB,
    #[doc = "PSM"]
    pub PSM: PSM,
    #[doc = "PWM"]
    pub PWM: PWM,
    #[doc = "RESETS"]
    pub RESETS: RESETS,
    #[doc = "ROSC"]
    pub ROSC: ROSC,
    #[doc = "RTC"]
    pub RTC: RTC,
    #[doc = "SPI0"]
    pub SPI0: SPI0,
    #[doc = "SPI1"]
    pub SPI1: SPI1,
    #[doc = "SYSCFG"]
    pub SYSCFG: SYSCFG,
    #[doc = "SYSINFO"]
    pub SYSINFO: SYSINFO,
    #[doc = "TBMAN"]
    pub TBMAN: TBMAN,
    #[doc = "TIMER"]
    pub TIMER: TIMER,
    #[doc = "UART0"]
    pub UART0: UART0,
    #[doc = "UART1"]
    pub UART1: UART1,
    #[doc = "USBCTRL_DPRAM"]
    pub USBCTRL_DPRAM: USBCTRL_DPRAM,
    #[doc = "USBCTRL_REGS"]
    pub USBCTRL_REGS: USBCTRL_REGS,
    #[doc = "VREG_AND_CHIP_RESET"]
    pub VREG_AND_CHIP_RESET: VREG_AND_CHIP_RESET,
    #[doc = "WATCHDOG"]
    pub WATCHDOG: WATCHDOG,
    #[doc = "XIP_CTRL"]
    pub XIP_CTRL: XIP_CTRL,
    #[doc = "XIP_SSI"]
    pub XIP_SSI: XIP_SSI,
}

/// Rp2040 Main interface
pub struct Rp2040 {
    pub pac: Pac,
    pub clocks: ClocksManager,
    pub xosc: CrystalOscillator<Stable>,
    pub pll_sys: PhaseLockedLoop<Locked, PLL_SYS>,
    pub pll_usb: PhaseLockedLoop<Locked, PLL_USB>,
    pub pins: Pins,
}

impl Rp2040 {
    /// Create an rp2040 instance
    pub fn take() -> Option<Self> {
        let mut pac = pac::Peripherals::take()?;

        let mut clocks = ClocksManager::new(pac.CLOCKS);

        let xosc = setup_xosc_blocking(pac.XOSC, XOSC_HZ.Hz()).ok().unwrap();

        let pll_sys = setup_pll_blocking(
            pac.PLL_SYS,
            XOSC_HZ.Hz().into(),
            PLL_SYS_125MHZ,
            &mut clocks,
            &mut pac.RESETS,
        )
        .ok()?;

        let pll_usb = setup_pll_blocking(
            pac.PLL_USB,
            XOSC_HZ.Hz().into(),
            PLL_USB_48MHZ,
            &mut clocks,
            &mut pac.RESETS,
        )
        .ok()?;

        clocks.init_default(&xosc, &pll_sys, &pll_usb);

        let sio = Sio::new(pac.SIO);
        let pins = Pins::new(
            pac.IO_BANK0,
            pac.PADS_BANK0,
            sio.gpio_bank0,
            &mut pac.RESETS,
        );

        Some(Self {
            pac: Pac {
                ADC: pac.ADC,
                BUSCTRL: pac.BUSCTRL,
                DMA: pac.DMA,
                I2C0: pac.I2C0,
                I2C1: pac.I2C1,
                IO_QSPI: pac.IO_QSPI,
                PADS_QSPI: pac.PADS_QSPI,
                PIO0: pac.PIO0,
                PIO1: pac.PIO1,
                PPB: pac.PPB,
                PSM: pac.PSM,
                PWM: pac.PWM,
                RESETS: pac.RESETS,
                ROSC: pac.ROSC,
                RTC: pac.RTC,
                SPI0: pac.SPI0,
                SPI1: pac.SPI1,
                SYSCFG: pac.SYSCFG,
                SYSINFO: pac.SYSINFO,
                TBMAN: pac.TBMAN,
                TIMER: pac.TIMER,
                UART0: pac.UART0,
                UART1: pac.UART1,
                USBCTRL_DPRAM: pac.USBCTRL_DPRAM,
                USBCTRL_REGS: pac.USBCTRL_REGS,
                VREG_AND_CHIP_RESET: pac.VREG_AND_CHIP_RESET,
                WATCHDOG: pac.WATCHDOG,
                XIP_CTRL: pac.XIP_CTRL,
                XIP_SSI: pac.XIP_SSI,
            },
            clocks,
            xosc,
            pll_sys,
            pll_usb,
            pins,
        })
    }
}
