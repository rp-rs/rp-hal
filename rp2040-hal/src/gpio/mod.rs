//! General Purpose Input and Output (GPIO)
//!
//! ## Basic usage
//! ```no_run
//! use embedded_hal::digital::{InputPin, OutputPin};
//! use rp2040_hal::{clocks::init_clocks_and_plls, gpio::Pins, watchdog::Watchdog, pac, Sio};
//! let mut peripherals = pac::Peripherals::take().unwrap();
//! let mut watchdog = Watchdog::new(peripherals.WATCHDOG);
//! const XOSC_CRYSTAL_FREQ: u32 = 12_000_000; // Typically found in BSP crates
//! let mut clocks = init_clocks_and_plls(XOSC_CRYSTAL_FREQ, peripherals.XOSC, peripherals.CLOCKS, peripherals.PLL_SYS, peripherals.PLL_USB, &mut peripherals.RESETS, &mut watchdog).ok().unwrap();
//!
//! let mut pac = pac::Peripherals::take().unwrap();
//! let sio = Sio::new(pac.SIO);
//! let pins = rp2040_hal::gpio::Pins::new(pac.IO_BANK0, pac.PADS_BANK0, sio.gpio_bank0, &mut pac.RESETS);
//! // Set a pin to drive output
//! let mut output_pin = pins.gpio25.into_push_pull_output();
//! // Drive output to 3.3V
//! output_pin.set_high().unwrap();
//! // Drive output to 0V
//! output_pin.set_low().unwrap();
//! // Set a pin to input
//! let mut input_pin = pins.gpio24.into_floating_input();
//! // pinstate will be true if the pin is above 2V
//! let pinstate = input_pin.is_high().unwrap();
//! // pinstate_low will be true if the pin is below 1.15V
//! let pinstate_low = input_pin.is_low().unwrap();
//! // you'll want to pull-up or pull-down a switch if it's not done externally
//! let button_pin = pins.gpio23.into_pull_down_input();
//! let button2_pin = pins.gpio22.into_pull_up_input();
//! ```
//! See [examples/gpio_in_out.rs](https://github.com/rp-rs/rp-hal/tree/main/rp2040-hal-examples/src/bin/gpio_in_out.rs) for a more practical example

// Design Notes:
//
// - The user must not be able to instantiate by themselves nor obtain an instance of the Type-level
//   structure.
// - non-typestated features (overrides, irq configuration, pads' output disable, pad's input
//   enable, drive strength, schmitt, slew rate, sio's in sync bypass) are considered somewhat
//   advanced usage of the pin (relative to reading/writing a gpio) and it is the responsibility of
//   the user to make sure these are in a correct state when converting and passing the pin around.

pub use embedded_hal::digital::PinState;

use crate::{
    atomic_register_access::{write_bitmask_clear, write_bitmask_set},
    pac,
    sio::Sio,
    typelevel::{self, Sealed},
};

mod func;
pub(crate) mod pin;
mod pin_group;
mod pull;

pub use func::*;
pub use pin::{DynBankId, DynPinId, PinId};
pub use pin_group::PinGroup;
pub use pull::*;

/// The amount of current that a pin can drive when used as an output.
#[allow(clippy::enum_variant_names)]
#[derive(Clone, Copy, Eq, PartialEq, Debug)]
pub enum OutputDriveStrength {
    /// 2 mA
    TwoMilliAmps,
    /// 4 mA
    FourMilliAmps,
    /// 8 mA
    EightMilliAmps,
    /// 12 mA
    TwelveMilliAmps,
}

/// The slew rate of a pin when used as an output.
#[derive(Clone, Copy, Eq, PartialEq, Debug)]
pub enum OutputSlewRate {
    /// Slew slow
    Slow,
    /// Slew fast
    Fast,
}

/// Interrupt kind.
#[derive(Clone, Copy, Eq, PartialEq, Debug)]
pub enum Interrupt {
    /// While low
    LevelLow,
    /// While high
    LevelHigh,
    /// On falling edge
    EdgeLow,
    /// On rising edge
    EdgeHigh,
}
impl Interrupt {
    fn mask(&self) -> u32 {
        match self {
            Interrupt::LevelLow => 0b0001,
            Interrupt::LevelHigh => 0b0010,
            Interrupt::EdgeLow => 0b0100,
            Interrupt::EdgeHigh => 0b1000,
        }
    }
}

/// Interrupt override state.
#[derive(Clone, Copy, Eq, PartialEq, Debug)]
pub enum InterruptOverride {
    /// Don't invert the interrupt.
    Normal = 0,
    /// Invert the interrupt.
    Invert = 1,
    /// Drive interrupt low.
    AlwaysLow = 2,
    /// Drive interrupt high.
    AlwaysHigh = 3,
}

/// Input override state.
#[derive(Clone, Copy, Eq, PartialEq, Debug)]
pub enum InputOverride {
    /// Don't invert the peripheral input.
    Normal = 0,
    /// Invert the peripheral input.
    Invert = 1,
    /// Drive peripheral input low.
    AlwaysLow = 2,
    /// Drive peripheral input high.
    AlwaysHigh = 3,
}

#[derive(Clone, Copy, Eq, PartialEq, Debug)]
/// Output enable override state.
pub enum OutputEnableOverride {
    /// Use the original output enable signal from selected peripheral.
    Normal = 0,
    /// Invert the output enable signal from selected peripheral.
    Invert = 1,
    /// Disable output.
    Disable = 2,
    /// Enable output.
    Enable = 3,
}

#[derive(Clone, Copy, Eq, PartialEq, Debug)]
/// Output override state.
pub enum OutputOverride {
    /// Use the original output signal from selected peripheral.
    DontInvert = 0,
    /// Invert the output signal from selected peripheral.
    Invert = 1,
    /// Drive output low.
    AlwaysLow = 2,
    /// Drive output high.
    AlwaysHigh = 3,
}

/// Represents a pin, with a given ID (e.g. Gpio3), a given function (e.g. FunctionUart) and a given pull type
/// (e.g. pull-down).
#[derive(Debug)]
pub struct Pin<I: PinId, F: func::Function, P: PullType> {
    id: I,
    function: F,
    pull_type: P,
}

/// Create a new pin instance.
///
/// # Safety
/// The uniqueness of the pin is not verified. User must make sure no other instance of that specific
/// pin exists at the same time.
pub unsafe fn new_pin(id: DynPinId) -> Pin<DynPinId, DynFunction, DynPullType> {
    use pac::io_bank0::gpio::gpio_ctrl::FUNCSEL_A;
    use pin::pin_sealed::PinIdOps;

    let funcsel = id
        .io_ctrl()
        .read()
        .funcsel()
        .variant()
        .expect("Invalid funcsel read from register.");
    let function = match funcsel {
        FUNCSEL_A::JTAG => DynFunction::Xip,
        FUNCSEL_A::SPI => DynFunction::Spi,
        FUNCSEL_A::UART => DynFunction::Uart,
        FUNCSEL_A::I2C => DynFunction::I2c,
        FUNCSEL_A::PWM => DynFunction::Pwm,
        FUNCSEL_A::SIO => {
            let mask = id.mask();
            let cfg = if id.sio_oe().read().bits() & mask == mask {
                DynSioConfig::Output
            } else {
                DynSioConfig::Input
            };
            DynFunction::Sio(cfg)
        }
        FUNCSEL_A::PIO0 => DynFunction::Pio0,
        FUNCSEL_A::PIO1 => DynFunction::Pio1,
        FUNCSEL_A::CLOCK => DynFunction::Clock,
        FUNCSEL_A::USB => DynFunction::Usb,
        FUNCSEL_A::NULL => DynFunction::Null,
    };
    let pad = id.pad_ctrl().read();
    let pull_type = match (pad.pue().bit_is_set(), pad.pde().bit_is_set()) {
        (true, true) => DynPullType::BusKeep,
        (true, false) => DynPullType::Up,
        (false, true) => DynPullType::Down,
        (false, false) => DynPullType::None,
    };

    Pin {
        id,
        function,
        pull_type,
    }
}

impl<I: PinId, F: func::Function, P: PullType> Pin<I, F, P> {
    /// Pin ID.
    pub fn id(&self) -> DynPinId {
        self.id.as_dyn()
    }

    /// # Safety
    /// This method does not check if the pin is actually configured as the target function or pull
    /// mode. This may lead to inconsistencies between the type-state and the actual state of the
    /// pin's configuration.
    pub unsafe fn into_unchecked<F2: func::Function, P2: PullType>(self) -> Pin<I, F2, P2> {
        Pin {
            id: self.id,
            function: F2::from(self.function.as_dyn()),
            pull_type: P2::from(self.pull_type.as_dyn()),
        }
    }

    /// Convert the pin from one state to the other.
    pub fn reconfigure<F2, P2>(self) -> Pin<I, F2, P2>
    where
        F2: func::Function,
        P2: PullType,
        I: func::ValidFunction<F2>,
    {
        self.into_function().into_pull_type()
    }

    /// Convert the pin function.
    #[deprecated(
        note = "Misleading name `mode` when it changes the `function`. Please use `into_function` instead.",
        since = "0.9.0"
    )]
    pub fn into_mode<F2>(self) -> Pin<I, F2, P>
    where
        F2: func::Function,
        I: func::ValidFunction<F2>,
    {
        self.into_function()
    }

    /// Convert the pin function.
    pub fn into_function<F2>(self) -> Pin<I, F2, P>
    where
        F2: func::Function,
        I: func::ValidFunction<F2>,
    {
        // Thanks to type-level validation, we know F2 is valid for I
        let prev_function = self.function.as_dyn();
        let function = F2::from(prev_function);
        let new_function = function.as_dyn();

        if prev_function != new_function {
            pin::set_function(&self.id, new_function);
        }

        Pin {
            function,
            id: self.id,
            pull_type: self.pull_type,
        }
    }

    /// Convert the pin pull type.
    pub fn into_pull_type<M2: PullType>(self) -> Pin<I, F, M2> {
        let prev_pull_type = self.pull_type.as_dyn();
        let pull_type = M2::from(prev_pull_type);
        let new_pull_type = pull_type.as_dyn();

        if prev_pull_type != new_pull_type {
            pin::set_pull_type(&self.id, new_pull_type);
        }

        Pin {
            pull_type,
            id: self.id,
            function: self.function,
        }
    }

    /// Erase the Pin ID type check.
    pub fn into_dyn_pin(self) -> Pin<DynPinId, F, P> {
        Pin {
            id: self.id.as_dyn(),
            function: self.function,
            pull_type: self.pull_type,
        }
    }

    /// Get the pin's pull type.
    pub fn pull_type(&self) -> DynPullType {
        self.pull_type.as_dyn()
    }

    //==============================================================================
    // Typical pin conversions.
    //==============================================================================

    /// Disable the pin and set it to float
    #[inline]
    pub fn into_floating_disabled(self) -> Pin<I, FunctionNull, PullNone>
    where
        I: ValidFunction<FunctionNull>,
    {
        self.reconfigure()
    }

    /// Disable the pin and set it to pull down
    #[inline]
    pub fn into_pull_down_disabled(self) -> Pin<I, FunctionNull, PullDown>
    where
        I: ValidFunction<FunctionNull>,
    {
        self.reconfigure()
    }

    /// Disable the pin and set it to pull up
    #[inline]
    pub fn into_pull_up_disabled(self) -> Pin<I, FunctionNull, PullUp>
    where
        I: ValidFunction<FunctionNull>,
    {
        self.reconfigure()
    }

    /// Configure the pin to operate as a floating input
    #[inline]
    pub fn into_floating_input(self) -> Pin<I, FunctionSio<SioInput>, PullNone>
    where
        I: ValidFunction<FunctionSio<SioInput>>,
    {
        self.into_function().into_pull_type()
    }

    /// Configure the pin to operate as a pulled down input
    #[inline]
    pub fn into_pull_down_input(self) -> Pin<I, FunctionSio<SioInput>, PullDown>
    where
        I: ValidFunction<FunctionSio<SioInput>>,
    {
        self.into_function().into_pull_type()
    }

    /// Configure the pin to operate as a pulled up input
    #[inline]
    pub fn into_pull_up_input(self) -> Pin<I, FunctionSio<SioInput>, PullUp>
    where
        I: ValidFunction<FunctionSio<SioInput>>,
    {
        self.into_function().into_pull_type()
    }

    /// Configure the pin to operate as a bus keep input
    #[inline]
    pub fn into_bus_keep_input(self) -> Pin<I, FunctionSio<SioInput>, PullBusKeep>
    where
        I: ValidFunction<FunctionSio<SioInput>>,
    {
        self.into_function().into_pull_type()
    }

    /// Configure the pin to operate as a push-pull output.
    ///
    /// If you want to specify the initial pin state, use [`Pin::into_push_pull_output_in_state`].
    #[inline]
    pub fn into_push_pull_output(self) -> Pin<I, FunctionSio<SioOutput>, P>
    where
        I: ValidFunction<FunctionSio<SioOutput>>,
    {
        self.into_function()
    }

    /// Configure the pin to operate as a push-pull output, specifying an initial
    /// state which is applied immediately.
    #[inline]
    pub fn into_push_pull_output_in_state(
        mut self,
        state: PinState,
    ) -> Pin<I, FunctionSio<SioOutput>, P>
    where
        I: ValidFunction<FunctionSio<SioOutput>>,
    {
        match state {
            PinState::High => self._set_high(),
            PinState::Low => self._set_low(),
        }
        self.into_push_pull_output()
    }

    /// Configure the pin to operate as a readable push pull output.
    ///
    /// If you want to specify the initial pin state, use [`Pin::into_readable_output_in_state`].
    #[inline]
    #[deprecated(
        note = "All gpio are readable, use `.into_push_pull_output()` instead.",
        since = "0.9.0"
    )]
    pub fn into_readable_output(self) -> Pin<I, FunctionSio<SioOutput>, P>
    where
        I: ValidFunction<FunctionSio<SioOutput>>,
    {
        self.into_function()
    }

    /// Configure the pin to operate as a readable push pull output, specifying an initial
    /// state which is applied immediately.
    #[inline]
    #[deprecated(
        note = "All gpio are readable, use `.into_push_pull_output_in_state()` instead.",
        since = "0.9.0"
    )]
    pub fn into_readable_output_in_state(self, state: PinState) -> Pin<I, FunctionSio<SioOutput>, P>
    where
        I: ValidFunction<FunctionSio<SioOutput>>,
    {
        self.into_push_pull_output_in_state(state)
    }

    //==============================================================================
    // methods available for all pins.
    //==============================================================================

    // =======================
    // Pad related methods

    /// Get the current drive strength of the pin.
    #[inline]
    pub fn get_drive_strength(&self) -> OutputDriveStrength {
        use pac::pads_bank0::gpio::DRIVE_A;
        match self.id.pad_ctrl().read().drive().variant() {
            DRIVE_A::_2M_A => OutputDriveStrength::TwoMilliAmps,
            DRIVE_A::_4M_A => OutputDriveStrength::FourMilliAmps,
            DRIVE_A::_8M_A => OutputDriveStrength::EightMilliAmps,
            DRIVE_A::_12M_A => OutputDriveStrength::TwelveMilliAmps,
        }
    }

    /// Set the drive strength for the pin.
    #[inline]
    pub fn set_drive_strength(&mut self, strength: OutputDriveStrength) {
        use pac::pads_bank0::gpio::DRIVE_A;
        let variant = match strength {
            OutputDriveStrength::TwoMilliAmps => DRIVE_A::_2M_A,
            OutputDriveStrength::FourMilliAmps => DRIVE_A::_4M_A,
            OutputDriveStrength::EightMilliAmps => DRIVE_A::_8M_A,
            OutputDriveStrength::TwelveMilliAmps => DRIVE_A::_12M_A,
        };
        self.id.pad_ctrl().modify(|_, w| w.drive().variant(variant))
    }

    /// Get the slew rate for the pin.
    #[inline]
    pub fn get_slew_rate(&self) -> OutputSlewRate {
        if self.id.pad_ctrl().read().slewfast().bit_is_set() {
            OutputSlewRate::Fast
        } else {
            OutputSlewRate::Slow
        }
    }

    /// Set the slew rate for the pin.
    #[inline]
    pub fn set_slew_rate(&mut self, rate: OutputSlewRate) {
        self.id
            .pad_ctrl()
            .modify(|_, w| w.slewfast().bit(OutputSlewRate::Fast == rate));
    }

    /// Get whether the schmitt trigger (hysteresis) is enabled.
    #[inline]
    pub fn get_schmitt_enabled(&self) -> bool {
        self.id.pad_ctrl().read().schmitt().bit_is_set()
    }

    /// Enable/Disable the schmitt trigger.
    #[inline]
    pub fn set_schmitt_enabled(&self, enable: bool) {
        self.id.pad_ctrl().modify(|_, w| w.schmitt().bit(enable));
    }

    /// Get the state of the digital output circuitry of the pad.
    #[inline]
    pub fn get_output_disable(&mut self) -> bool {
        self.id.pad_ctrl().read().od().bit_is_set()
    }

    /// Set the digital output circuitry of the pad.
    #[inline]
    pub fn set_output_disable(&mut self, disable: bool) {
        self.id.pad_ctrl().modify(|_, w| w.od().bit(disable));
    }

    /// Get the state of the digital input circuitry of the pad.
    #[inline]
    pub fn get_input_enable(&mut self) -> bool {
        self.id.pad_ctrl().read().ie().bit_is_set()
    }

    /// Set the digital input circuitry of the pad.
    #[inline]
    pub fn set_input_enable(&mut self, enable: bool) {
        self.id.pad_ctrl().modify(|_, w| w.ie().bit(enable));
    }

    // =======================
    // IO related methods

    /// Get the input override.
    #[inline]
    pub fn get_input_override(&self) -> InputOverride {
        use pac::io_bank0::gpio::gpio_ctrl::INOVER_A;
        match self.id.io_ctrl().read().inover().variant() {
            INOVER_A::NORMAL => InputOverride::Normal,
            INOVER_A::INVERT => InputOverride::Invert,
            INOVER_A::LOW => InputOverride::AlwaysLow,
            INOVER_A::HIGH => InputOverride::AlwaysHigh,
        }
    }

    /// Set the input override.
    #[inline]
    pub fn set_input_override(&mut self, override_value: InputOverride) {
        use pac::io_bank0::gpio::gpio_ctrl::INOVER_A;
        let variant = match override_value {
            InputOverride::Normal => INOVER_A::NORMAL,
            InputOverride::Invert => INOVER_A::INVERT,
            InputOverride::AlwaysLow => INOVER_A::LOW,
            InputOverride::AlwaysHigh => INOVER_A::HIGH,
        };
        self.id.io_ctrl().modify(|_, w| w.inover().variant(variant));
    }

    /// Get the output enable override.
    #[inline]
    pub fn get_output_enable_override(&self) -> OutputEnableOverride {
        use pac::io_bank0::gpio::gpio_ctrl::OEOVER_A;
        match self.id.io_ctrl().read().oeover().variant() {
            OEOVER_A::NORMAL => OutputEnableOverride::Normal,
            OEOVER_A::INVERT => OutputEnableOverride::Invert,
            OEOVER_A::DISABLE => OutputEnableOverride::Disable,
            OEOVER_A::ENABLE => OutputEnableOverride::Enable,
        }
    }

    /// Set the output enable override.
    #[inline]
    pub fn set_output_enable_override(&mut self, override_value: OutputEnableOverride) {
        use pac::io_bank0::gpio::gpio_ctrl::OEOVER_A;
        let variant = match override_value {
            OutputEnableOverride::Normal => OEOVER_A::NORMAL,
            OutputEnableOverride::Invert => OEOVER_A::INVERT,
            OutputEnableOverride::Disable => OEOVER_A::DISABLE,
            OutputEnableOverride::Enable => OEOVER_A::ENABLE,
        };
        self.id.io_ctrl().modify(|_, w| w.oeover().variant(variant));
    }

    /// Get the output override.
    #[inline]
    pub fn get_output_override(&self) -> OutputOverride {
        use pac::io_bank0::gpio::gpio_ctrl::OUTOVER_A;
        match self.id.io_ctrl().read().outover().variant() {
            OUTOVER_A::NORMAL => OutputOverride::DontInvert,
            OUTOVER_A::INVERT => OutputOverride::Invert,
            OUTOVER_A::LOW => OutputOverride::AlwaysLow,
            OUTOVER_A::HIGH => OutputOverride::AlwaysHigh,
        }
    }

    /// Set the output override.
    #[inline]
    pub fn set_output_override(&mut self, override_value: OutputOverride) {
        use pac::io_bank0::gpio::gpio_ctrl::OUTOVER_A;
        let variant = match override_value {
            OutputOverride::DontInvert => OUTOVER_A::NORMAL,
            OutputOverride::Invert => OUTOVER_A::INVERT,
            OutputOverride::AlwaysLow => OUTOVER_A::LOW,
            OutputOverride::AlwaysHigh => OUTOVER_A::HIGH,
        };
        self.id
            .io_ctrl()
            .modify(|_, w| w.outover().variant(variant));
    }

    /// Get the interrupt override.
    #[inline]
    pub fn get_interrupt_override(&self) -> InterruptOverride {
        use pac::io_bank0::gpio::gpio_ctrl::IRQOVER_A;
        match self.id.io_ctrl().read().irqover().variant() {
            IRQOVER_A::NORMAL => InterruptOverride::Normal,
            IRQOVER_A::INVERT => InterruptOverride::Invert,
            IRQOVER_A::LOW => InterruptOverride::AlwaysLow,
            IRQOVER_A::HIGH => InterruptOverride::AlwaysHigh,
        }
    }

    /// Set the interrupt override.
    #[inline]
    pub fn set_interrupt_override(&mut self, override_value: InterruptOverride) {
        use pac::io_bank0::gpio::gpio_ctrl::IRQOVER_A;
        let variant = match override_value {
            InterruptOverride::Normal => IRQOVER_A::NORMAL,
            InterruptOverride::Invert => IRQOVER_A::INVERT,
            InterruptOverride::AlwaysLow => IRQOVER_A::LOW,
            InterruptOverride::AlwaysHigh => IRQOVER_A::HIGH,
        };
        self.id
            .io_ctrl()
            .modify(|_, w| w.irqover().variant(variant));
    }

    // =======================
    // SIO related methods

    #[inline]
    #[allow(clippy::bool_comparison)] // more explicit this way
    pub(crate) fn _is_low(&self) -> bool {
        let mask = self.id.mask();
        self.id.sio_in().read().bits() & mask == 0
    }

    #[inline]
    #[allow(clippy::bool_comparison)] // more explicit this way
    pub(crate) fn _is_high(&self) -> bool {
        !self._is_low()
    }

    #[inline]
    pub(crate) fn _set_low(&mut self) {
        let mask = self.id.mask();
        self.id
            .sio_out_clr()
            .write(|w| unsafe { w.gpio_out_clr().bits(mask) });
    }

    #[inline]
    pub(crate) fn _set_high(&mut self) {
        let mask = self.id.mask();
        self.id
            .sio_out_set()
            .write(|w| unsafe { w.gpio_out_set().bits(mask) });
    }

    #[inline]
    pub(crate) fn _toggle(&mut self) {
        let mask = self.id.mask();
        self.id
            .sio_out_xor()
            .write(|w| unsafe { w.gpio_out_xor().bits(mask) });
    }

    #[inline]
    pub(crate) fn _is_set_low(&self) -> bool {
        let mask = self.id.mask();
        self.id.sio_out().read().bits() & mask == 0
    }

    #[inline]
    pub(crate) fn _is_set_high(&self) -> bool {
        !self._is_set_low()
    }

    // =======================
    // Interrupt related methods

    /// Clear interrupt.
    #[inline]
    pub fn clear_interrupt(&mut self, interrupt: Interrupt) {
        let (reg, offset) = self.id.intr();
        let mask = interrupt.mask();
        reg.write(|w| unsafe { w.bits(mask << offset) });
    }

    /// Interrupt status.
    #[inline]
    pub fn interrupt_status(&self, interrupt: Interrupt) -> bool {
        let (reg, offset) = self.id.proc_ints(Sio::core());
        let mask = interrupt.mask();
        (reg.read().bits() >> offset) & mask == mask
    }

    /// Is interrupt enabled.
    #[inline]
    pub fn is_interrupt_enabled(&self, interrupt: Interrupt) -> bool {
        let (reg, offset) = self.id.proc_inte(Sio::core());
        let mask = interrupt.mask();
        (reg.read().bits() >> offset) & mask == mask
    }

    /// Enable or disable interrupt.
    #[inline]
    pub fn set_interrupt_enabled(&self, interrupt: Interrupt, enabled: bool) {
        let (reg, offset) = self.id.proc_inte(Sio::core());
        let mask = interrupt.mask();
        unsafe {
            if enabled {
                write_bitmask_set(reg.as_ptr(), mask << offset);
            } else {
                write_bitmask_clear(reg.as_ptr(), mask << offset);
            }
        }
    }

    /// Is interrupt forced.
    #[inline]
    pub fn is_interrupt_forced(&self, interrupt: Interrupt) -> bool {
        let (reg, offset) = self.id.proc_intf(Sio::core());
        let mask = interrupt.mask();
        (reg.read().bits() >> offset) & mask == mask
    }

    /// Force or release interrupt.
    #[inline]
    pub fn set_interrupt_forced(&self, interrupt: Interrupt, forced: bool) {
        let (reg, offset) = self.id.proc_intf(Sio::core());
        let mask = interrupt.mask();
        unsafe {
            if forced {
                write_bitmask_set(reg.as_ptr(), mask << offset);
            } else {
                write_bitmask_clear(reg.as_ptr(), mask << offset);
            }
        }
    }

    /// Dormant wake status.
    #[inline]
    pub fn dormant_wake_status(&self, interrupt: Interrupt) -> bool {
        let (reg, offset) = self.id.dormant_wake_ints();
        let mask = interrupt.mask();
        (reg.read().bits() >> offset) & mask == mask
    }

    /// Is dormant wake enabled.
    #[inline]
    pub fn is_dormant_wake_enabled(&self, interrupt: Interrupt) -> bool {
        let (reg, offset) = self.id.dormant_wake_inte();
        let mask = interrupt.mask();
        (reg.read().bits() >> offset) & mask == mask
    }

    /// Enable or disable dormant wake.
    #[inline]
    pub fn set_dormant_wake_enabled(&self, interrupt: Interrupt, enabled: bool) {
        let (reg, offset) = self.id.dormant_wake_inte();
        let mask = interrupt.mask();
        unsafe {
            if enabled {
                write_bitmask_set(reg.as_ptr(), mask << offset);
            } else {
                write_bitmask_clear(reg.as_ptr(), mask << offset);
            }
        }
    }

    /// Is dormant wake forced.
    #[inline]
    pub fn is_dormant_wake_forced(&self, interrupt: Interrupt) -> bool {
        let (reg, offset) = self.id.dormant_wake_intf();
        let mask = interrupt.mask();
        (reg.read().bits() >> offset) & mask == mask
    }

    /// Force dormant wake.
    #[inline]
    pub fn set_dormant_wake_forced(&mut self, interrupt: Interrupt, forced: bool) {
        let (reg, offset) = self.id.dormant_wake_intf();
        let mask = interrupt.mask();
        unsafe {
            if forced {
                write_bitmask_set(reg.as_ptr(), mask << offset);
            } else {
                write_bitmask_clear(reg.as_ptr(), mask << offset);
            }
        }
    }

    /// Return a wrapper that implements InputPin.
    ///
    /// This allows to read from the pin independent of the selected function.
    /// Depending on the pad configuration, reading from the pin may not return a
    /// meaningful result.
    ///
    /// Calling this function does not set the pad's input enable bit.
    pub fn as_input(&self) -> AsInputPin<I, F, P> {
        AsInputPin(self)
    }
}
impl<I: PinId, C: SioConfig, P: PullType> Pin<I, FunctionSio<C>, P> {
    /// Is bypass enabled
    #[inline]
    pub fn is_sync_bypass(&self) -> bool {
        let mask = self.id.mask();
        self.id.proc_in_by_pass().read().bits() & mask == mask
    }

    /// Bypass the input sync stages.
    ///
    /// This saves two clock cycles in the input signal's path at the risks of introducing metastability.
    #[inline]
    pub fn set_sync_bypass(&mut self, bypass: bool) {
        let mask = self.id.mask();
        let reg = self.id.proc_in_by_pass();
        unsafe {
            if bypass {
                write_bitmask_set(reg.as_ptr(), mask);
            } else {
                write_bitmask_clear(reg.as_ptr(), mask);
            }
        }
    }
}
impl<F: func::Function, P: PullType> Pin<DynPinId, F, P> {
    /// Try to return to a type-checked pin id.
    ///
    /// This method may fail if the target pin id differs from the current dynamic one.
    pub fn try_into_pin<P2: pin::pin_sealed::TypeLevelPinId>(self) -> Result<Pin<P2, F, P>, Self> {
        if P2::ID == self.id {
            Ok(Pin {
                id: P2::new(),
                function: self.function,
                pull_type: self.pull_type,
            })
        } else {
            Err(self)
        }
    }

    /// Try to change the pin's function.
    pub fn try_into_function<F2>(self) -> Result<Pin<DynPinId, F2, P>, Pin<DynPinId, F, P>>
    where
        F2: func::Function,
    {
        // Thanks to type-level validation, we know F2 is valid for I
        let prev_function = self.function.as_dyn();
        let function = F2::from(prev_function);
        let function_as_dyn = function.as_dyn();

        use func_sealed::Function;
        if function_as_dyn.is_valid(&self.id) {
            if function_as_dyn != prev_function.as_dyn() {
                pin::set_function(&self.id, function_as_dyn);
            }
            Ok(Pin {
                function,
                id: self.id,
                pull_type: self.pull_type,
            })
        } else {
            Err(self)
        }
    }
}
impl<I: PinId, P: PullType> Pin<I, DynFunction, P> {
    /// Try to set the pin's function.
    ///
    /// This method may fail if the requested function is not supported by the pin, eg `FunctionXiP`
    /// on a gpio from `Bank0`.
    pub fn try_set_function(&mut self, function: DynFunction) -> Result<(), func::InvalidFunction> {
        use func_sealed::Function;
        if !function.is_valid(&self.id) {
            return Err(func::InvalidFunction);
        } else if function != self.function.as_dyn() {
            pin::set_function(&self.id, function);
            self.function = function;
        }
        Ok(())
    }

    /// Gets the pin's function.
    pub fn function(&self) -> DynFunction {
        use func_sealed::Function;
        self.function.as_dyn()
    }
}

impl<I: PinId, F: func::Function> Pin<I, F, DynPullType> {
    /// Set the pin's pull type.
    pub fn set_pull_type(&mut self, pull_type: DynPullType) {
        if pull_type != self.pull_type {
            pin::set_pull_type(&self.id, pull_type);
            self.pull_type = pull_type;
        }
    }
}

/// Wrapper providing input pin functions for GPIO pins independent of the configured mode.
pub struct AsInputPin<'a, I: PinId, F: func::Function, P: PullType>(&'a Pin<I, F, P>);

//==============================================================================
//  Embedded-HAL
//==============================================================================

/// GPIO error type.
pub type Error = core::convert::Infallible;

impl<I, P> embedded_hal_0_2::digital::v2::OutputPin for Pin<I, FunctionSio<SioOutput>, P>
where
    I: PinId,
    P: PullType,
{
    type Error = Error;

    fn set_low(&mut self) -> Result<(), Self::Error> {
        self._set_low();
        Ok(())
    }

    fn set_high(&mut self) -> Result<(), Self::Error> {
        self._set_high();
        Ok(())
    }
}

/// Deprecated: Instead of implicitly implementing InputPin for function SioOutput,
/// use `pin.as_input()` to get access to input values independent of the selected function.
impl<I, P> embedded_hal_0_2::digital::v2::InputPin for Pin<I, FunctionSio<SioOutput>, P>
where
    I: PinId,
    P: PullType,
{
    type Error = Error;

    fn is_high(&self) -> Result<bool, Self::Error> {
        Ok(self._is_high())
    }

    fn is_low(&self) -> Result<bool, Self::Error> {
        Ok(self._is_low())
    }
}

impl<I: PinId, F: func::Function, P: PullType> embedded_hal_0_2::digital::v2::InputPin
    for AsInputPin<'_, I, F, P>
{
    type Error = core::convert::Infallible;

    fn is_high(&self) -> Result<bool, Self::Error> {
        Ok(self.0._is_high())
    }

    fn is_low(&self) -> Result<bool, Self::Error> {
        Ok(self.0._is_low())
    }
}

impl<I, P> embedded_hal_0_2::digital::v2::StatefulOutputPin for Pin<I, FunctionSio<SioOutput>, P>
where
    I: PinId,
    P: PullType,
{
    fn is_set_high(&self) -> Result<bool, Self::Error> {
        Ok(self._is_set_high())
    }

    fn is_set_low(&self) -> Result<bool, Self::Error> {
        Ok(self._is_set_low())
    }
}

impl<I, P> embedded_hal_0_2::digital::v2::ToggleableOutputPin for Pin<I, FunctionSio<SioOutput>, P>
where
    I: PinId,
    P: PullType,
{
    type Error = Error;

    fn toggle(&mut self) -> Result<(), Self::Error> {
        self._toggle();
        Ok(())
    }
}
impl<I, P> embedded_hal_0_2::digital::v2::InputPin for Pin<I, FunctionSio<SioInput>, P>
where
    I: PinId,
    P: PullType,
{
    type Error = Error;

    fn is_high(&self) -> Result<bool, Self::Error> {
        Ok(self._is_high())
    }

    fn is_low(&self) -> Result<bool, Self::Error> {
        Ok(self._is_low())
    }
}

//==============================================================================
//  Pins
//==============================================================================

/// Default type state of a pin after reset of the pads, io and sio.
pub trait DefaultTypeState: crate::typelevel::Sealed {
    /// Default function.
    type Function: Function;
    /// Default pull type.
    type PullType: PullType;
}

// Clear input enable for pins of bank0.
// Pins 26-29 are ADC pins. If the pins are connected to an analog input,
// the signal level may not be valid for a digital input. Therefore, input
// should be disabled by default.
// For the other GPIO pins, the same setting is applied for consistency.
macro_rules! reset_ie {
    ( Bank0, $pads:ident ) => {
        for id in (0..=29) {
            $pads.gpio(id).modify(|_, w| w.ie().clear_bit());
        }
    };
    ( Qspi, $pads:ident ) => {};
}

macro_rules! gpio {
    ( $bank:ident:$prefix:ident, [ $(($id:expr, $pull_type:ident, $func:ident)),* ] ) => {
        paste::paste!{
            #[doc = "Pin bank " [<$bank>] ]
            pub mod [<$bank:snake>] {
                use $crate::pac::{[<IO_ $bank:upper>],[<PADS_ $bank:upper>]};
                use crate::sio::[<SioGpio $bank>];
                use super::{Pin, pin, pull, func};
                $(pub use super::pin::[<$bank:lower>]::[<$prefix $id>];)*

                $(
                    impl super::DefaultTypeState for [<$prefix $id>] {
                        type Function = super::[<Function $func>];
                        type PullType = super::[<Pull $pull_type>];
                    }
                 )*
                gpio!(struct: $bank $prefix $([<$prefix $id>], $id, $func, $pull_type),*);

                impl Pins {
                    /// Take ownership of the PAC peripherals and SIO slice and split it into discrete [`Pin`]s
                    ///
                    /// This clears the input-enable flag for all Bank0 pads.
                    pub fn new(io : [<IO_ $bank:upper>], pads: [<PADS_ $bank:upper>], sio: [<SioGpio $bank>], reset : &mut $crate::pac::RESETS) -> Self {
                        use $crate::resets::SubsystemReset;
                        pads.reset_bring_down(reset);
                        io.reset_bring_down(reset);

                        {
                            use $crate::gpio::pin::DynBankId;
                            // SAFETY: this function owns the whole bank that will be affected.
                            let sio = unsafe { &*$crate::pac::SIO::PTR };
                            if DynBankId::$bank == DynBankId::Bank0 {
                                sio.gpio_oe().reset();
                                sio.gpio_out().reset();
                            } else {
                                sio.gpio_hi_oe().reset();
                                sio.gpio_hi_out().reset();
                            }
                        }

                        io.reset_bring_up(reset);
                        pads.reset_bring_up(reset);
                        reset_ie!($bank, pads);
                        gpio!(members: io, pads, sio, $(([<$prefix $id>], $func, $pull_type)),+)
                    }
                }
            }
        }
    };
    (struct: $bank:ident $prefix:ident $($PXi:ident, $id:expr, $func:ident, $pull_type:ident),*) => {
        paste::paste!{
                /// Collection of all the individual [`Pin`]s
                #[derive(Debug)]
                pub struct Pins {
                    _io: [<IO_ $bank:upper>],
                    _pads: [<PADS_ $bank:upper>],
                    _sio: [<SioGpio $bank>],
                    $(
                        #[doc = "Pin " [<$PXi>] ]
                        pub [<$PXi:snake>]: Pin<pin::[<$bank:lower>]::[<$prefix $id>] , func::[<Function $func>], pull::[<Pull $pull_type>]>,
                     )*
                }
        }
    };
    (members: $io:ident, $pads:ident, $sio:ident, $(($PXi:ident, $func:ident, $pull_type:ident)),+) => {
        paste::paste!{
            Self {
                _io: $io,
                _pads: $pads,
                _sio: $sio,
                $(
                    [<$PXi:snake>]: Pin {
                        id: [<$PXi>] (()),
                        function: func::[<Function $func>] (()),
                        pull_type: pull::[<Pull $pull_type>] (())
                    },
                )+
            }
        }
    };
}

gpio!(
    Bank0: Gpio,
    [
        (0, Down, Null),
        (1, Down, Null),
        (2, Down, Null),
        (3, Down, Null),
        (4, Down, Null),
        (5, Down, Null),
        (6, Down, Null),
        (7, Down, Null),
        (8, Down, Null),
        (9, Down, Null),
        (10, Down, Null),
        (11, Down, Null),
        (12, Down, Null),
        (13, Down, Null),
        (14, Down, Null),
        (15, Down, Null),
        (16, Down, Null),
        (17, Down, Null),
        (18, Down, Null),
        (19, Down, Null),
        (20, Down, Null),
        (21, Down, Null),
        (22, Down, Null),
        (23, Down, Null),
        (24, Down, Null),
        (25, Down, Null),
        (26, Down, Null),
        (27, Down, Null),
        (28, Down, Null),
        (29, Down, Null)
    ]
);

gpio!(
    Qspi: Qspi,
    [
        (Sclk, Down, Null),
        (Ss, Up, Null),
        (Sd0, None, Null),
        (Sd1, None, Null),
        (Sd2, None, Null),
        (Sd3, None, Null)
    ]
);

pub use bank0::Pins;

//==============================================================================
//  AnyPin
//==============================================================================

/// Type class for [`Pin`] types.
///
/// This trait uses the [`AnyKind`] trait pattern to create a [type class] for
/// [`Pin`] types. See the `AnyKind` documentation for more details on the
/// pattern.
///
/// [`AnyKind`]: crate::typelevel#anykind-trait-pattern
/// [type class]: crate::typelevel#type-classes
pub trait AnyPin: Sealed
where
    Self: typelevel::Sealed,
    Self: typelevel::Is<Type = SpecificPin<Self>>,
{
    /// [`PinId`] of the corresponding [`Pin`]
    type Id: PinId;
    /// [`func::Function`] of the corresponding [`Pin`]
    type Function: func::Function;
    /// [`PullType`] of the corresponding [`Pin`]
    type Pull: PullType;
}

impl<I, F, P> Sealed for Pin<I, F, P>
where
    I: PinId,
    F: func::Function,
    P: PullType,
{
}

impl<I, F, P> AnyPin for Pin<I, F, P>
where
    I: PinId,
    F: func::Function,
    P: PullType,
{
    type Id = I;
    type Function = F;
    type Pull = P;
}

/// Type alias to recover the specific [`Pin`] type from an implementation of [`AnyPin`].
///
/// See the [`AnyKind`] documentation for more details on the pattern.
///
/// [`AnyKind`]: crate::typelevel#anykind-trait-pattern
pub type SpecificPin<P> = Pin<<P as AnyPin>::Id, <P as AnyPin>::Function, <P as AnyPin>::Pull>;

//==============================================================================
//  bsp_pins helper macro
//==============================================================================

/// Helper macro to give meaningful names to GPIO pins
///
/// The normal [`Pins`] struct names each [`Pin`] according to its [`PinId`].
/// However, BSP authors would prefer to name each [`Pin`] according to its
/// function. This macro defines a new `Pins` struct with custom field names
/// for each [`Pin`].
///
/// # Example
/// Calling the macro like this:
/// ```rust
/// use rp2040_hal::bsp_pins;
/// bsp_pins! {
///     #[cfg(feature = "gpio")]
///     Gpio0 {
///          /// Doc gpio0
///          name: gpio0,
///          aliases: { FunctionPio0, PullNone: PioPin }
///     },
///     Gpio1 {
///          name: led,
///          aliases: { FunctionPwm, PullDown: LedPwm }
///      },
/// }
/// ```
///
/// Is roughly equivalent to the following source code (excluding the docs strings below):
/// ```
/// use ::rp2040_hal as hal;
/// use hal::gpio;
/// pub struct Pins {
///     /// Doc gpio0
///     #[cfg(feature = "gpio")]
///     pub gpio0: gpio::Pin<
///         gpio::bank0::Gpio0,
///         <gpio::bank0::Gpio0 as gpio::DefaultTypeState>::Function,
///         <gpio::bank0::Gpio0 as gpio::DefaultTypeState>::PullType,
///     >,
///     pub led: gpio::Pin<
///         gpio::bank0::Gpio1,
///         <gpio::bank0::Gpio1 as gpio::DefaultTypeState>::Function,
///         <gpio::bank0::Gpio1 as gpio::DefaultTypeState>::PullType,
///     >,
/// }
/// impl Pins {
///     #[inline]
///     pub fn new(
///         io: hal::pac::IO_BANK0,
///         pads: hal::pac::PADS_BANK0,
///         sio: hal::sio::SioGpioBank0,
///         reset: &mut hal::pac::RESETS,
///     ) -> Self {
///         let mut pins = gpio::Pins::new(io, pads, sio, reset);
///         Self {
///             #[cfg(feature = "gpio")]
///             gpio0: pins.gpio0,
///             led: pins.gpio1,
///         }
///     }
/// }
/// pub type PioPin = gpio::Pin<gpio::bank0::Gpio0, gpio::FunctionPio0, gpio::PullNone>;
/// pub type LedPwm = gpio::Pin<gpio::bank0::Gpio1, gpio::FunctionPwm, gpio::PullDown>;
/// ```
#[macro_export]
macro_rules! bsp_pins {
    (
        $(
            $( #[$id_cfg:meta] )*
            $Id:ident {
                $( #[$name_doc:meta] )*
                name: $name:ident $(,)?
                $(
                    aliases: {
                        $(
                            $( #[$alias_cfg:meta] )*
                            $Function:ty, $PullType:ident: $Alias:ident
                        ),+
                    }
                )?
            } $(,)?
        )+
    ) => {
        $crate::paste::paste! {

            /// BSP replacement for the HAL
            /// [`Pins`](rp2040_hal::gpio::Pins) type
            ///
            /// This type is intended to provide more meaningful names for the
            /// given pins.
            ///
            /// To enable specific functions of the pins you can use the
            /// [rp2040_hal::gpio::pin::Pin::into_function] function with
            /// one of:
            /// - [rp2040_hal::gpio::FunctionI2C]
            /// - [rp2040_hal::gpio::FunctionPwm]
            /// - [rp2040_hal::gpio::FunctionSpi]
            /// - [rp2040_hal::gpio::FunctionXip]
            /// - [rp2040_hal::gpio::FunctionPio0]
            /// - [rp2040_hal::gpio::FunctionPio1]
            /// - [rp2040_hal::gpio::FunctionUart]
            ///
            /// like this:
            ///```no_run
            /// use rp2040_hal::{pac, gpio::{bank0::Gpio12, Pin, Pins}, sio::Sio};
            ///
            /// let mut peripherals = pac::Peripherals::take().unwrap();
            /// let sio = Sio::new(peripherals.SIO);
            /// let pins = Pins::new(peripherals.IO_BANK0,peripherals.PADS_BANK0,sio.gpio_bank0, &mut peripherals.RESETS);
            ///
            /// let _spi_sclk = pins.gpio2.into_function::<rp2040_hal::gpio::FunctionSpi>();
            /// let _spi_mosi = pins.gpio3.into_function::<rp2040_hal::gpio::FunctionSpi>();
            /// let _spi_miso = pins.gpio4.into_function::<rp2040_hal::gpio::FunctionSpi>();
            ///```
            ///
            /// **See also [rp2040_hal::gpio] for more in depth information about this**!
            pub struct Pins {
                $(
                    $( #[$id_cfg] )*
                    $( #[$name_doc] )*
                    pub $name: $crate::gpio::Pin<
                        $crate::gpio::bank0::$Id,
                        <$crate::gpio::bank0::$Id as $crate::gpio::DefaultTypeState>::Function,
                        <$crate::gpio::bank0::$Id as $crate::gpio::DefaultTypeState>::PullType,
                    >,
                )+
            }

            impl Pins {
                /// Take ownership of the PAC [`PORT`] and split it into
                /// discrete [`Pin`]s.
                ///
                /// This struct serves as a replacement for the HAL [`Pins`]
                /// struct. It is intended to provide more meaningful names for
                /// each [`Pin`] in a BSP. Any [`Pin`] not defined by the BSP is
                /// dropped.
                ///
                /// [`Pin`](rp2040_hal::gpio::Pin)
                /// [`Pins`](rp2040_hal::gpio::Pins)
                #[inline]
                pub fn new(io : $crate::pac::IO_BANK0, pads: $crate::pac::PADS_BANK0, sio: $crate::sio::SioGpioBank0, reset : &mut $crate::pac::RESETS) -> Self {
                    let mut pins = $crate::gpio::Pins::new(io,pads,sio,reset);
                    Self {
                        $(
                            $( #[$id_cfg] )*
                            $name: pins.[<$Id:lower>],
                        )+
                    }
                }
            }
            $(
                $( #[$id_cfg] )*
                $crate::bsp_pins!(@aliases, $( $( $( #[$alias_cfg] )* $Id $Function $PullType $Alias )+ )? );
            )+
        }
    };
    ( @aliases, $( $( $( #[$attr:meta] )* $Id:ident $Function:ident $PullType:ident $Alias:ident )+ )? ) => {
        $crate::paste::paste! {
            $(
                $(
                    $( #[$attr] )*
                    /// Alias for a configured [`Pin`](rp2040_hal::gpio::Pin)
                    pub type $Alias = $crate::gpio::Pin<
                        $crate::gpio::bank0::$Id,
                        $crate::gpio::$Function,
                        $crate::gpio::$PullType
                    >;
                )+
            )?
        }
    };
}

//==============================================================================
//  InOutPin
//==============================================================================

/// A wrapper [`AnyPin`]`<Function = `[`FunctionSioOutput`]`>` emulating open-drain function.
///
/// This wrapper implements both InputPin and OutputPin, to simulate an open-drain pin as needed for
/// example by the wire protocol the DHT11 sensor speaks.
///
/// <https://how2electronics.com/interfacing-dht11-temperature-humidity-sensor-with-raspberry-pi-pico/>
pub struct InOutPin<T: AnyPin> {
    inner: Pin<T::Id, FunctionSioOutput, T::Pull>,
}

impl<T: AnyPin> InOutPin<T> {
    /// Create a new wrapper
    pub fn new(inner: T) -> InOutPin<T>
    where
        T::Id: ValidFunction<FunctionSioOutput>,
    {
        let mut inner = inner.into();
        inner.set_output_enable_override(OutputEnableOverride::Disable);

        // into Pin<_, FunctionSioOutput, _>
        let inner = inner.into_push_pull_output_in_state(PinState::Low);

        Self { inner }
    }
}

impl<T> InOutPin<T>
where
    T: AnyPin,
    T::Id: ValidFunction<T::Function>,
{
    /// Releases the pin reverting to its previous function.
    pub fn release(self) -> T {
        // restore the previous typestate first
        let mut inner = self.inner.reconfigure();
        // disable override
        inner.set_output_enable_override(OutputEnableOverride::Normal);
        // typelevel-return
        T::from(inner)
    }
}

impl<T: AnyPin> embedded_hal_0_2::digital::v2::InputPin for InOutPin<T> {
    type Error = Error;
    fn is_high(&self) -> Result<bool, Error> {
        self.inner.is_high()
    }

    fn is_low(&self) -> Result<bool, Error> {
        self.inner.is_low()
    }
}

impl<T: AnyPin> embedded_hal_0_2::digital::v2::OutputPin for InOutPin<T> {
    type Error = Error;
    fn set_low(&mut self) -> Result<(), Error> {
        // The pin is already set to output low but this is inhibited by the override.
        self.inner
            .set_output_enable_override(OutputEnableOverride::Enable);
        Ok(())
    }

    fn set_high(&mut self) -> Result<(), Error> {
        // To set the open-drain pin to high, just disable the output driver by configuring the
        // output override. That way, the DHT11 can still pull the data line down to send its response.
        self.inner
            .set_output_enable_override(OutputEnableOverride::Disable);
        Ok(())
    }
}

mod eh1 {
    use embedded_hal::digital::{ErrorType, InputPin, OutputPin, StatefulOutputPin};

    use super::{
        func, AnyPin, AsInputPin, Error, FunctionSio, InOutPin, OutputEnableOverride, Pin, PinId,
        PullType, SioConfig, SioInput, SioOutput,
    };

    impl<I, P, S> ErrorType for Pin<I, FunctionSio<S>, P>
    where
        I: PinId,
        P: PullType,
        S: SioConfig,
    {
        type Error = Error;
    }

    impl<I, P> OutputPin for Pin<I, FunctionSio<SioOutput>, P>
    where
        I: PinId,
        P: PullType,
    {
        fn set_low(&mut self) -> Result<(), Self::Error> {
            self._set_low();
            Ok(())
        }

        fn set_high(&mut self) -> Result<(), Self::Error> {
            self._set_high();
            Ok(())
        }
    }

    impl<I, P> StatefulOutputPin for Pin<I, FunctionSio<SioOutput>, P>
    where
        I: PinId,
        P: PullType,
    {
        fn is_set_high(&mut self) -> Result<bool, Self::Error> {
            Ok(self._is_set_high())
        }

        fn is_set_low(&mut self) -> Result<bool, Self::Error> {
            Ok(self._is_set_low())
        }

        fn toggle(&mut self) -> Result<(), Self::Error> {
            self._toggle();
            Ok(())
        }
    }

    impl<I, P> InputPin for Pin<I, FunctionSio<SioInput>, P>
    where
        I: PinId,
        P: PullType,
    {
        fn is_high(&mut self) -> Result<bool, Self::Error> {
            Ok(self._is_high())
        }

        fn is_low(&mut self) -> Result<bool, Self::Error> {
            Ok(self._is_low())
        }
    }

    impl<I, F, P> ErrorType for AsInputPin<'_, I, F, P>
    where
        I: PinId,
        F: func::Function,
        P: PullType,
    {
        type Error = Error;
    }

    impl<I: PinId, F: func::Function, P: PullType> InputPin for AsInputPin<'_, I, F, P> {
        fn is_high(&mut self) -> Result<bool, Self::Error> {
            Ok(self.0._is_high())
        }

        fn is_low(&mut self) -> Result<bool, Self::Error> {
            Ok(self.0._is_low())
        }
    }

    impl<I> ErrorType for InOutPin<I>
    where
        I: AnyPin,
    {
        type Error = Error;
    }

    impl<I> OutputPin for InOutPin<I>
    where
        I: AnyPin,
    {
        fn set_low(&mut self) -> Result<(), Self::Error> {
            // The pin is already set to output low but this is inhibited by the override.
            self.inner
                .set_output_enable_override(OutputEnableOverride::Enable);
            Ok(())
        }

        fn set_high(&mut self) -> Result<(), Self::Error> {
            // To set the open-drain pin to high, just disable the output driver by configuring the
            // output override. That way, the DHT11 can still pull the data line down to send its response.
            self.inner
                .set_output_enable_override(OutputEnableOverride::Disable);
            Ok(())
        }
    }

    impl<I> InputPin for InOutPin<I>
    where
        I: AnyPin,
    {
        fn is_high(&mut self) -> Result<bool, Self::Error> {
            Ok(self.inner._is_high())
        }

        fn is_low(&mut self) -> Result<bool, Self::Error> {
            Ok(self.inner._is_low())
        }
    }
}
