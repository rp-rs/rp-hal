//! Clocks (CLOCKS)
//!
//!
//!
//! ## Usage simple
//! ```no_run
//! use rp235x_hal::{self as hal, clocks::init_clocks_and_plls, watchdog::Watchdog};
//!
//! let mut peripherals = hal::pac::Peripherals::take().unwrap();
//! let mut watchdog = Watchdog::new(peripherals.WATCHDOG);
//! const XOSC_CRYSTAL_FREQ: u32 = 12_000_000; // Typically found in BSP crates
//! let mut clocks = init_clocks_and_plls(
//!     XOSC_CRYSTAL_FREQ,
//!     peripherals.XOSC,
//!     peripherals.CLOCKS,
//!     peripherals.PLL_SYS,
//!     peripherals.PLL_USB,
//!     &mut peripherals.RESETS,
//!     &mut watchdog,
//! )
//! .ok()
//! .unwrap();
//! ```
//!
//! ## Usage extended
//! ```no_run
//! use fugit::RateExtU32;
//! use rp235x_hal::{clocks::{Clock, ClocksManager, ClockSource, InitError}, gpio::Pins, self as hal, pll::{common_configs::{PLL_SYS_150MHZ, PLL_USB_48MHZ}, setup_pll_blocking}, Sio, watchdog::Watchdog, xosc::setup_xosc_blocking};
//!
//! # fn func() -> Result<(), InitError> {
//! let mut peripherals = hal::pac::Peripherals::take().unwrap();
//! let mut watchdog = Watchdog::new(peripherals.WATCHDOG);
//! const XOSC_CRYSTAL_FREQ: u32 = 12_000_000; // Typically found in BSP crates
//!
//! // Enable the xosc
//! let xosc = setup_xosc_blocking(peripherals.XOSC, XOSC_CRYSTAL_FREQ.Hz()).map_err(InitError::XoscErr)?;
//!
//! // Start tick in watchdog
//! watchdog.enable_tick_generation((XOSC_CRYSTAL_FREQ / 1_000_000) as u16);
//!
//! let mut clocks = ClocksManager::new(peripherals.CLOCKS);
//!
//! // Configure PLLs
//! //                   REF     FBDIV VCO            POSTDIV
//! // PLL SYS: 12 / 1 = 12MHz * 125 = 1500MHZ / 5 / 2 = 150MHz
//! // PLL USB: 12 / 1 = 12MHz * 40  = 480 MHz / 5 / 2 =  48MHz
//! let pll_sys = setup_pll_blocking(peripherals.PLL_SYS, xosc.operating_frequency().into(), PLL_SYS_150MHZ, &mut clocks, &mut peripherals.RESETS).map_err(InitError::PllError)?;
//! let pll_usb = setup_pll_blocking(peripherals.PLL_USB, xosc.operating_frequency().into(), PLL_USB_48MHZ, &mut clocks, &mut peripherals.RESETS).map_err(InitError::PllError)?;
//!
//! // Configure clocks
//! // CLK_REF = XOSC (12MHz) / 1 = 12MHz
//! clocks.reference_clock.configure_clock(&xosc, xosc.get_freq()).map_err(InitError::ClockError)?;
//!
//! // CLK SYS = PLL SYS (150MHz) / 1 = 150MHz
//! clocks.system_clock.configure_clock(&pll_sys, pll_sys.get_freq()).map_err(InitError::ClockError)?;
//!
//! // CLK USB = PLL USB (48MHz) / 1 = 48MHz
//! clocks.usb_clock.configure_clock(&pll_usb, pll_usb.get_freq()).map_err(InitError::ClockError)?;
//!
//! // CLK ADC = PLL USB (48MHZ) / 1 = 48MHz
//! clocks.adc_clock.configure_clock(&pll_usb, pll_usb.get_freq()).map_err(InitError::ClockError)?;
//!
//! // CLK HSTX = PLL SYS (150MHz) / 1 = 150MHz
//! clocks.hstx_clock.configure_clock(&pll_sys, pll_sys.get_freq()).map_err(InitError::ClockError)?;
//!
//! // CLK PERI = clk_sys. Used as reference clock for Peripherals. No dividers so just select and enable
//! // Normally choose clk_sys or clk_usb
//! clocks.peripheral_clock.configure_clock(&clocks.system_clock, clocks.system_clock.freq()).map_err(InitError::ClockError)?;
//! # Ok(())
//! # }
//! ```
//!
//! See [Chapter 8](https://rptl.io/rp2350-datasheet#section_clocks) for more details.

use core::{convert::Infallible, marker::PhantomData};
use fugit::{HertzU32, RateExtU32};

use crate::{
    pac::{self, CLOCKS, PLL_SYS, PLL_USB, RESETS, XOSC},
    pll::{
        common_configs::{PLL_SYS_150MHZ, PLL_USB_48MHZ},
        setup_pll_blocking, Error as PllError, Locked, PhaseLockedLoop,
    },
    typelevel::Sealed,
    watchdog::Watchdog,
    xosc::{setup_xosc_blocking, CrystalOscillator, Error as XoscError, Stable},
};

#[macro_use]
mod macros;
mod clock_sources;

use clock_sources::PllSys;

use self::clock_sources::{GPin0, GPin1, LpOsc, PllUsb, Rosc, Xosc};

bitfield::bitfield! {
    /// Bit field mapping clock enable bits.
    #[cfg_attr(feature = "defmt", derive(defmt::Format))]
    #[derive(Default)]
    pub struct ClockGate(u64);
    /// Clock gate to the clock controller.
    pub sys_clock, set_sys_clock: 0;
    /// Clock gate the hardware access control registers
    pub sys_accessctrl, set_sys_accessctrl: 1;
    /// Clock gate to the adc analog logic.
    pub adc_adc, set_adc_adc: 2;
    /// Clock gate to the adc peripheral.
    pub sys_adc, set_sys_adc: 3;
    /// Clock gate the Boot RAM
    pub sys_bootram, set_sys_bootram: 4;
    /// Clock gate the memory bus controller.
    pub sys_busctrl, set_sys_busctrl: 5;
    /// Clock gate the memory bus fabric.
    pub sys_busfabric, set_sys_busfabric: 6;
    /// Clock gate the dma controller.
    pub sys_dma, set_sys_dma: 7;
    /// Clock gate the Glitch Detector peripheral
    pub sys_glitch_detector, set_sys_glitch_detector: 8;
    /// Clock gate the High-Speed TX peripheral
    pub hstx, set_hstx: 9;
    /// Clock gate the High-Speed TX peripheral
    pub sys_hstx, set_sys_hstx: 10;
    /// Clock gate I2C0.
    pub sys_i2c0, set_sys_i2c0: 11;
    /// Clock gate I2C1.
    pub sys_i2c1, set_sys_i2c1: 12;
    /// Clock gate the IO controller.
    pub sys_io, set_sys_io: 13;
    ///  Clock gate the JTAG peripheral.
    pub sys_jtag, set_sys_jtag: 14;
    /// Clock gate the OTP (one-time programmable memory) peripheral
    pub ref_otp, set_ref_otp: 15;
    /// Clock gate the OTP (one-time programmable memory) peripheral
    pub sys_otp, set_sys_otp: 16;
    /// Clock gate pad controller.
    pub sys_pads, set_sys_pads: 17;
    /// Clock gate PIO0 peripheral.
    pub sys_pio0, set_sys_pio0: 18;
    /// Clock gate PIO1 peripheral.
    pub sys_pio1, set_sys_pio1: 19;
    /// Clock gate PIO2 peripheral.
    pub sys_pio2, set_sys_pio2: 20;
    /// Clock gate the system PLL.
    pub sys_pll_sys, set_sys_pll_sys: 21;
    /// Clock gate the USB PLL.
    pub sys_pll_usb, set_sys_pll_usb: 22;
    /// Clock gate the Power Manager
    pub ref_powman, set_ref_powman: 23;
    /// Clock gate the Power Manager
    pub sys_powman, set_sys_powman: 24;
    /// Clock gate PWM peripheral.
    pub sys_pwm, set_sys_pwm: 25;
    /// Clock gate the reset controller.
    pub sys_resets, set_sys_resets: 26;
    /// Clock gate the ROM.
    pub sys_rom, set_sys_rom: 27;
    /// Clock gate the ROSC controller (not the rosc itself).
    pub sys_rosc, set_sys_rosc: 28;
    /// Clock gate the Power State Machine
    pub sys_psm, set_sys_psm: 29;
    /// Clock gate the SHA256 peripheral
    pub sys_sha256, set_sys_sha256: 30;
    /// Clock gate the SIO controller.
    pub sys_sio, set_sys_sio: 31;

    /// Clock gate SPI0's baud generation.
    pub peri_spi0, set_peri_spi0: 32;
    /// Clock gate SPI0's controller..
    pub sys_spi0, set_sys_spi0: 33;
    /// Clock gate SPI1's baud generation.
    pub peri_spi1, set_peri_spi1: 34;
    /// Clock gate SPI1's controller..
    pub sys_spi1, set_sys_spi1: 35;
    /// Clock gate SRAM0.
    pub sys_sram0, set_sys_sram0: 36;
    /// Clock gate SRAM1.
    pub sys_sram1, set_sys_sram1: 37;
    /// Clock gate SRAM2.
    pub sys_sram2, set_sys_sram2: 38;
    /// Clock gate SRAM3.
    pub sys_sram3, set_sys_sram3: 39;
    /// Clock gate SRAM4.
    pub sys_sram4, set_sys_sram4: 40;
    /// Clock gate SRAM5.
    pub sys_sram5, set_sys_sram5: 41;
    /// Clock gate SRAM6
    pub sys_sram6, set_sys_sram6: 42;
    /// Clock gate SRAM7
    pub sys_sram7, set_sys_sram7: 43;
    /// Clock gate SRAM8
    pub sys_sram8, set_sys_sram8: 44;
    /// Clock gate SRAM9
    pub sys_sram9, set_sys_sram9: 45;
    /// Clock gate the system configuration controller.
    pub sys_syscfg, set_sys_syscfg: 46;
    /// Clock gate the system information peripheral.
    pub sys_sysinfo, set_sys_sysinfo: 47;
    /// Clock gate the test bench manager.
    pub sys_tbman, set_sys_tbman: 48;
    /// Clock gate the tick generator
    pub ref_ticks, set_ref_ticks: 49;
    /// Clock gate the tick generator
    pub sys_ticks, set_sys_ticks: 50;
    /// Clock gate the Timer 0 peripheral.
    pub sys_timer0, set_sys_timer0: 51;
    /// Clock gate the Timer 1 peripheral.
    pub sys_timer1, set_sys_timer1: 52;
    /// Clock gate the TrustZone Random Number Generator
    pub sys_trng, set_sys_trng: 53;
    /// Clock gate UART0's baud generation.
    pub peri_uart0, set_peri_uart0: 54;
    /// Clock gate UART0's controller.
    pub sys_uart0, set_sys_uart0: 55;
    /// Clock gate UART1's baud generation.
    pub peri_uart1, set_peri_uart1: 56;
    /// Clock gate UART1's controller.
    pub sys_uart1, set_sys_uart1: 57;
    /// Clock gate the USB controller.
    pub sys_usbctrl, set_sys_usbctrl: 58;
    /// Clock gate the USB logic.
    pub usb, set_usb: 59;
    /// Clock gate the Watchdog controller.
    pub sys_watchdog, set_sys_watchdog: 60;
    /// .Clock gate the XIP controller.
    pub sys_xip, set_sys_xip: 61;
    /// Clock gate the XOSC controller (not xosc itself).
    pub sys_xosc, set_sys_xosc: 62;
}

impl core::fmt::Debug for ClockGate {
    fn fmt(&self, f: &mut core::fmt::Formatter<'_>) -> core::fmt::Result {
        f.debug_struct("ClockGate")
            .field("sys_clock", &self.sys_clock())
            .field("sys_accessctrl", &self.sys_accessctrl())
            .field("adc_adc", &self.adc_adc())
            .field("sys_adc", &self.sys_adc())
            .field("sys_bootram", &self.sys_bootram())
            .field("sys_busctrl", &self.sys_busctrl())
            .field("sys_busfabric", &self.sys_busfabric())
            .field("sys_dma", &self.sys_dma())
            .field("sys_glitch_detector", &self.sys_glitch_detector())
            .field("hstx", &self.hstx())
            .field("sys_hstx", &self.sys_hstx())
            .field("sys_i2c0", &self.sys_i2c0())
            .field("sys_i2c1", &self.sys_i2c1())
            .field("sys_io", &self.sys_io())
            .field("sys_jtag", &self.sys_jtag())
            .field("ref_otp", &self.ref_otp())
            .field("sys_otp", &self.sys_otp())
            .field("sys_pads", &self.sys_pads())
            .field("sys_pio0", &self.sys_pio0())
            .field("sys_pio1", &self.sys_pio1())
            .field("sys_pio2", &self.sys_pio2())
            .field("sys_pll_sys", &self.sys_pll_sys())
            .field("sys_pll_usb", &self.sys_pll_usb())
            .field("ref_powman", &self.ref_powman())
            .field("sys_powman", &self.sys_powman())
            .field("sys_pwm", &self.sys_pwm())
            .field("sys_resets", &self.sys_resets())
            .field("sys_rom", &self.sys_rom())
            .field("sys_rosc", &self.sys_rosc())
            .field("sys_psm", &self.sys_psm())
            .field("sys_sha256", &self.sys_sha256())
            .field("sys_sio", &self.sys_sio())
            .field("peri_spi0", &self.peri_spi0())
            .field("sys_spi0", &self.sys_spi0())
            .field("peri_spi1", &self.peri_spi1())
            .field("sys_spi1", &self.sys_spi1())
            .field("sys_sram0", &self.sys_sram0())
            .field("sys_sram1", &self.sys_sram1())
            .field("sys_sram2", &self.sys_sram2())
            .field("sys_sram3", &self.sys_sram3())
            .field("sys_sram4", &self.sys_sram4())
            .field("sys_sram5", &self.sys_sram5())
            .field("sys_sram6", &self.sys_sram6())
            .field("sys_sram7", &self.sys_sram7())
            .field("sys_sram8", &self.sys_sram8())
            .field("sys_sram9", &self.sys_sram9())
            .field("sys_syscfg", &self.sys_syscfg())
            .field("sys_sysinfo", &self.sys_sysinfo())
            .field("sys_tbman", &self.sys_tbman())
            .field("ref_ticks", &self.ref_ticks())
            .field("sys_ticks", &self.sys_ticks())
            .field("sys_timer0", &self.sys_timer0())
            .field("sys_timer1", &self.sys_timer1())
            .field("sys_trng", &self.sys_trng())
            .field("peri_uart0", &self.peri_uart0())
            .field("sys_uart0", &self.sys_uart0())
            .field("peri_uart1", &self.peri_uart1())
            .field("sys_uart1", &self.sys_uart1())
            .field("sys_usbctrl", &self.sys_usbctrl())
            .field("usb", &self.usb())
            .field("sys_watchdog", &self.sys_watchdog())
            .field("sys_xip", &self.sys_xip())
            .field("sys_xosc", &self.sys_xosc())
            .finish()
    }
}

#[derive(Copy, Clone)]
/// Provides refs to the CLOCKS block.
struct ShareableClocks {
    _internal: (),
}

impl ShareableClocks {
    fn new(_clocks: &mut CLOCKS) -> Self {
        ShareableClocks { _internal: () }
    }

    unsafe fn get(&self) -> &pac::clocks::RegisterBlock {
        &*CLOCKS::ptr()
    }
}

/// Something when wrong setting up the clock
#[non_exhaustive]
#[derive(Copy, Clone, Debug, PartialEq, Eq, Hash)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum ClockError {
    /// The frequency desired is higher than the source frequency
    CantIncreaseFreq,
    /// The desired frequency is to high (would overflow an u32)
    FrequencyTooHigh,
    /// The desired frequency is too low (divider can't reach the desired value)
    FrequencyTooLow,
}

/// For clocks
pub trait Clock: Sealed + Sized {
    /// Enum with valid source clocks register values for `Clock`
    type Variant;

    /// Get operating frequency
    fn freq(&self) -> HertzU32;

    /// Configure this clock based on a clock source and desired frequency
    fn configure_clock<S: ValidSrc<Self>>(
        &mut self,
        src: &S,
        freq: HertzU32,
    ) -> Result<(), ClockError>;
}

/// For clocks with a divider
trait ClockDivision {
    /// Set integer divider value.
    fn set_div(&mut self, div: u32);
    /// Get integer diveder value.
    fn get_div(&self) -> u32;
}

/// Clock with glitchless source
trait GlitchlessClock {
    /// Self type to hand to ChangingClockToken
    type Clock: Clock;

    /// Await switching clock sources without glitches. Needs a token that is returned when setting
    fn await_select(
        &self,
        clock_token: &ChangingClockToken<Self::Clock>,
    ) -> nb::Result<(), Infallible>;
}

/// Token which can be used to await the glitchless switch
pub struct ChangingClockToken<G: Clock> {
    clock_nr: u8,
    clock: PhantomData<G>,
}

/// For clocks that can be disabled
pub trait StoppableClock: Sealed {
    /// Enables the clock.
    fn enable(&mut self);

    /// Disables the clock.
    fn disable(&mut self);

    /// Kills the clock.
    fn kill(&mut self);
}

/// Trait for things that can be used as clock source
pub trait ClockSource: Sealed {
    /// Get the operating frequency for this source
    ///
    /// Used to determine the divisor
    fn get_freq(&self) -> HertzU32;
}

/// Trait to contrain which ClockSource is valid for which Clock
pub trait ValidSrc<C: Clock>: Sealed + ClockSource {
    /// Is this a ClockSource for src or aux?
    fn is_aux(&self) -> bool;
    /// Get register value for this ClockSource
    fn variant(&self) -> C::Variant;
}

clocks! {
    /// GPIO Output 0 Clock Generator
    ///
    /// Clock output to GPIO. Can be used to clock external devices or debug on
    /// chip clocks with a logic analyser or oscilloscope.
    struct GpioOutput0Clock {
        init_freq: 0,
        reg: clk_gpout0,
        auxsrc: {
            PllSys: CLKSRC_PLL_SYS,
            GPin0: CLKSRC_GPIN0,
            GPin1: CLKSRC_GPIN1,
            PllUsb: CLKSRC_PLL_USB,
            /* CLKSRC_PLL_USB_PRIMARY_REF_OPCG */
            Rosc: ROSC_CLKSRC,
            Xosc: XOSC_CLKSRC,
            LpOsc: LPOSC_CLKSRC,
            SystemClock: CLK_SYS,
            UsbClock: CLK_USB,
            AdcClock: CLK_ADC,
            ReferenceClock: CLK_REF,
            PeripheralClock: CLK_PERI,
            HstxClock: CLK_HSTX
            /* OTP_CLK2FC */
        }
    }
    /// GPIO Output 1 Clock Generator
    ///
    /// Clock output to GPIO. Can be used to clock external devices or debug on
    /// chip clocks with a logic analyser or oscilloscope.
    struct GpioOutput1Clock {
        init_freq: 0,
        reg: clk_gpout1,
        auxsrc: {
            PllSys: CLKSRC_PLL_SYS,
            GPin0: CLKSRC_GPIN0,
            GPin1: CLKSRC_GPIN1,
            PllUsb: CLKSRC_PLL_USB,
            /* CLKSRC_PLL_USB_PRIMARY_REF_OPCG */
            Rosc: ROSC_CLKSRC,
            Xosc: XOSC_CLKSRC,
            LpOsc: LPOSC_CLKSRC,
            SystemClock: CLK_SYS,
            UsbClock: CLK_USB,
            AdcClock: CLK_ADC,
            ReferenceClock: CLK_REF,
            PeripheralClock: CLK_PERI,
            HstxClock: CLK_HSTX
            /* OTP_CLK2FC */
        }
    }
    /// GPIO Output 2 Clock Generator
    ///
    /// Clock output to GPIO. Can be used to clock external devices or debug on
    /// chip clocks with a logic analyser or oscilloscope.
    struct GpioOutput2Clock {
        init_freq: 0,
        reg: clk_gpout2,
        auxsrc: {
            PllSys: CLKSRC_PLL_SYS,
            GPin0: CLKSRC_GPIN0,
            GPin1: CLKSRC_GPIN1,
            PllUsb: CLKSRC_PLL_USB,
            /* CLKSRC_PLL_USB_PRIMARY_REF_OPCG */
            Rosc: ROSC_CLKSRC_PH,
            Xosc: XOSC_CLKSRC,
            LpOsc: LPOSC_CLKSRC,
            SystemClock: CLK_SYS,
            UsbClock: CLK_USB,
            AdcClock: CLK_ADC,
            ReferenceClock: CLK_REF,
            PeripheralClock: CLK_PERI,
            HstxClock: CLK_HSTX
            /* OTP_CLK2FC */
        }
    }
    /// GPIO Output 3 Clock Generator
    ///
    /// Clock output to GPIO. Can be used to clock external devices or debug on
    /// chip clocks with a logic analyser or oscilloscope.
    struct GpioOutput3Clock {
        init_freq: 0,
        reg: clk_gpout3,
        auxsrc: {
            PllSys: CLKSRC_PLL_SYS,
            GPin0: CLKSRC_GPIN0,
            GPin1: CLKSRC_GPIN1,
            PllUsb: CLKSRC_PLL_USB,
            /* CLKSRC_PLL_USB_PRIMARY_REF_OPCG */
            Rosc: ROSC_CLKSRC_PH,
            Xosc: XOSC_CLKSRC,
            LpOsc: LPOSC_CLKSRC,
            SystemClock: CLK_SYS,
            UsbClock: CLK_USB,
            AdcClock: CLK_ADC,
            ReferenceClock: CLK_REF,
            PeripheralClock: CLK_PERI,
            HstxClock: CLK_HSTX
            /* OTP_CLK2FC */
        }
    }
    /// Reference clock that is always running 6 - 12MHz unless in DORMANT mode.
    ///
    /// Runs from Ring Oscillator (ROSC) at power-up but can be switched to
    /// Crystal Oscillator (XOSC) for more accuracy.
    struct ReferenceClock {
        init_freq: 12_000_000,
        // Starts from ROSC which actually varies with input voltage etc,
        // but 12 MHz seems to be a good value
        reg: clk_ref,
        src: {
            Rosc: ROSC_CLKSRC_PH,
            /* Aux: CLKSRC_CLK_REF_AUX, */
            Xosc: XOSC_CLKSRC,
            LpOsc: LPOSC_CLKSRC
        },
        auxsrc: {
            PllUsb:CLKSRC_PLL_USB,
            GPin0:CLKSRC_GPIN0,
            GPin1:CLKSRC_GPIN1
            /* CLKSRC_PLL_USB_PRIMARY_REF_OPCG */
        }
    }
    /// System clock that is always running unless in DORMANT mode.
    ///
    /// Runs from clk_ref at power-up but is typically switched to a PLL.
    struct SystemClock {
        init_freq: 12_000_000,  // ref_clk is 12 MHz
        reg: clk_sys,
        src: {
            ReferenceClock: CLK_REF,
            SystemClock: CLKSRC_CLK_SYS_AUX
        },
        auxsrc: {
            PllSys: CLKSRC_PLL_SYS,
            PllUsb: CLKSRC_PLL_USB,
            Rosc: ROSC_CLKSRC,
            Xosc: XOSC_CLKSRC,
            GPin0: CLKSRC_GPIN0,
            GPin1: CLKSRC_GPIN1
        }
    }
    /// Peripheral clock.
    ///
    /// Typically runs from 12 - 150MHz clk_sys but allows peripherals to run at
    /// a consistent speed if clk_sys is changed by software.
    struct PeripheralClock {
        init_freq: 12_000_000,  // sys_clk is 12 MHz
        reg: clk_peri,
        auxsrc: {
            SystemClock: CLK_SYS,
            PllSys: CLKSRC_PLL_SYS,
            PllUsb:CLKSRC_PLL_USB,
            Rosc: ROSC_CLKSRC_PH,
            Xosc: XOSC_CLKSRC,
            GPin0: CLKSRC_GPIN0,
            GPin1: CLKSRC_GPIN1
        },
        div: false
    }
    /// HSTX (High-Speed Transmitter) Clock
    struct HstxClock {
        init_freq: 0,
        reg: clk_hstx,
        auxsrc: {
            SystemClock: CLK_SYS,
            PllUsb: CLKSRC_PLL_USB,
            PllSys: CLKSRC_PLL_SYS,
            GPin0: CLKSRC_GPIN0,
            GPin1: CLKSRC_GPIN1
        }
    }
    /// USB reference clock.
    ///
    /// Must be 48MHz.
    struct UsbClock {
        init_freq: 0,
        reg: clk_usb,
        auxsrc: {
            PllUsb: CLKSRC_PLL_USB,
            PllSys: CLKSRC_PLL_SYS,
            Rosc: ROSC_CLKSRC_PH,
            Xosc: XOSC_CLKSRC,
            GPin0: CLKSRC_GPIN0,
            GPin1: CLKSRC_GPIN1
        }
    }
    /// ADC reference clock.
    ///
    /// Must be 48MHz.
    struct AdcClock {
        init_freq: 0,
        reg: clk_adc,
        auxsrc: {
            PllUsb: CLKSRC_PLL_USB,
            PllSys: CLKSRC_PLL_SYS,
            Rosc: ROSC_CLKSRC_PH,
            Xosc: XOSC_CLKSRC,
            GPin0: CLKSRC_GPIN0,
            GPin1: CLKSRC_GPIN1
        }
    }
}

impl SystemClock {
    fn get_default_clock_source(&self) -> pac::clocks::clk_sys_ctrl::SRC_A {
        pac::clocks::clk_sys_ctrl::SRC_A::CLK_REF
    }

    fn get_aux_source(&self) -> pac::clocks::clk_sys_ctrl::SRC_A {
        pac::clocks::clk_sys_ctrl::SRC_A::CLKSRC_CLK_SYS_AUX
    }
}

impl ReferenceClock {
    fn get_default_clock_source(&self) -> pac::clocks::clk_ref_ctrl::SRC_A {
        pac::clocks::clk_ref_ctrl::SRC_A::ROSC_CLKSRC_PH
    }

    fn get_aux_source(&self) -> pac::clocks::clk_ref_ctrl::SRC_A {
        pac::clocks::clk_ref_ctrl::SRC_A::CLKSRC_CLK_REF_AUX
    }
}

impl ClocksManager {
    /// Initialize the clocks to a sane default
    pub fn init_default(
        &mut self,

        xosc: &CrystalOscillator<Stable>,
        pll_sys: &PhaseLockedLoop<Locked, PLL_SYS>,
        pll_usb: &PhaseLockedLoop<Locked, PLL_USB>,
    ) -> Result<(), ClockError> {
        // Configure clocks
        // CLK_REF = XOSC (12MHz) / 1 = 12MHz
        self.reference_clock
            .configure_clock(xosc, xosc.get_freq())?;

        // CLK SYS = PLL SYS (150MHz) / 1 = 150MHz
        self.system_clock
            .configure_clock(pll_sys, pll_sys.get_freq())?;

        // CLK USB = PLL USB (48MHz) / 1 = 48MHz
        self.usb_clock
            .configure_clock(pll_usb, pll_usb.get_freq())?;

        // CLK ADC = PLL USB (48MHZ) / 1 = 48MHz
        self.adc_clock
            .configure_clock(pll_usb, pll_usb.get_freq())?;

        // CLK PERI = clk_sys. Used as reference clock for Peripherals. No dividers so just select and enable
        // Normally choose clk_sys or clk_usb
        self.peripheral_clock
            .configure_clock(&self.system_clock, self.system_clock.freq())
    }

    /// Configure the clocks staying ON during deep-sleep.
    pub fn configure_sleep_enable(&mut self, clock_gate: ClockGate) {
        self.clocks
            .sleep_en0()
            .write(|w| unsafe { w.bits(clock_gate.0 as u32) });
        self.clocks
            .sleep_en1()
            .write(|w| unsafe { w.bits((clock_gate.0 >> 32) as u32) });
    }

    /// Read the clock gate configuration while the device is in its (deep) sleep state.
    pub fn sleep_enable(&self) -> ClockGate {
        ClockGate(
            (u64::from(self.clocks.sleep_en1().read().bits()) << 32)
                | u64::from(self.clocks.sleep_en0().read().bits()),
        )
    }

    /// Read the clock gate configuration while the device is in its wake state.
    pub fn wake_enable(&self) -> ClockGate {
        ClockGate(
            (u64::from(self.clocks.wake_en1().read().bits()) << 32)
                | u64::from(self.clocks.wake_en0().read().bits()),
        )
    }

    /// Releases the CLOCKS block
    pub fn free(self) -> CLOCKS {
        self.clocks
    }
}

/// Possible init errors
#[derive(Debug)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum InitError {
    /// Something went wrong setting up the Xosc
    XoscErr(XoscError),
    /// Something went wrong setting up the Pll
    PllError(PllError),
    /// Something went wrong setting up the Clocks
    ClockError(ClockError),
}

/// Initialize the clocks and plls according to the reference implementation
pub fn init_clocks_and_plls(
    xosc_crystal_freq: u32,
    xosc_dev: XOSC,
    clocks_dev: CLOCKS,
    pll_sys_dev: PLL_SYS,
    pll_usb_dev: PLL_USB,
    resets: &mut RESETS,
    watchdog: &mut Watchdog,
) -> Result<ClocksManager, InitError> {
    let xosc = setup_xosc_blocking(xosc_dev, xosc_crystal_freq.Hz()).map_err(InitError::XoscErr)?;

    // Configure watchdog tick generation to tick over every microsecond
    watchdog.enable_tick_generation((xosc_crystal_freq / 1_000_000) as u16);

    let mut clocks = ClocksManager::new(clocks_dev);

    let pll_sys = setup_pll_blocking(
        pll_sys_dev,
        xosc.operating_frequency(),
        PLL_SYS_150MHZ,
        &mut clocks,
        resets,
    )
    .map_err(InitError::PllError)?;
    let pll_usb = setup_pll_blocking(
        pll_usb_dev,
        xosc.operating_frequency(),
        PLL_USB_48MHZ,
        &mut clocks,
        resets,
    )
    .map_err(InitError::PllError)?;

    clocks
        .init_default(&xosc, &pll_sys, &pll_usb)
        .map_err(InitError::ClockError)?;
    Ok(clocks)
}

// Calculates (numerator<<16)/denominator.
fn fractional_div(numerator: u32, denominator: u32) -> Option<u32> {
    let numerator: u64 = u64::from(numerator) * 65536;
    let denominator: u64 = u64::from(denominator);
    let result = numerator / denominator;
    result.try_into().ok()
}
