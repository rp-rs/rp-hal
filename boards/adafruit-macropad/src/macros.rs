//!
//! You probably want a `main()` routine that starts out like this:
//!
//! ```
//! use rp_pico as bsp;
//! let mut pac = pac::Peripherals::take().unwrap();
//! let core = pac::CorePeripherals::take().unwrap();
//! let mut watchdog = Watchdog::new(pac.WATCHDOG);
//! let clocks = macropad_clocks!(pac, watchdog);
//! let mut delay = cortex_m::delay::Delay::new(core.SYST, clocks.system_clock.freq().integer());
//! let pins = macropad_pins!(pac);
//! let timer = Timer::new(pac.TIMER, &mut pac.RESETS);
//! //
//! let mut disp = macropad_oled!(pins, clocks, delay, pac);
//! let mut led_pin = pins.led.into_push_pull_output();
//! let mut neopixels = macropad_neopixels!(pins, clocks, timer, pac);
//! //
//! let mut rotary = macropad_rotary_encoder!(pins);
//! let keys = macropad_keypad!(pins);
//! ```
//!
//! If there is a peripheral you don't need, you can probably omit it.
//!
pub use crate::hal::clocks::init_clocks_and_plls;
pub use crate::hal::clocks::Clock;
pub use crate::hal::gpio::bank0::{
    Gpio1, Gpio10, Gpio11, Gpio12, Gpio2, Gpio3, Gpio4, Gpio5, Gpio6, Gpio7, Gpio8, Gpio9,
};
pub use crate::hal::gpio::{FunctionSpi, Pin, PullUpInput, PushPullOutput};
pub use crate::hal::sio::Sio;
pub use crate::hal::Spi;
pub use core::convert::Infallible;
pub use embedded_hal::digital::v2::InputPin;
pub use embedded_hal::spi::MODE_0;
pub use embedded_time::rate::Extensions;
pub use sh1106::prelude::GraphicsMode;

// XXX missing: speaker, I2C side port

#[macro_export]
macro_rules! macropad_neopixels {
    ($pins:expr, $pio:expr, $sm0: expr, $clocks:expr, $timer:expr) => {{
        ws2812_pio::Ws2812::new(
            $pins.neopixel.into_mode(),
            &mut $pio,
            $sm0,
            $crate::Clock::freq(&$clocks.peripheral_clock),
            $timer.count_down(),
        )
    }};
    ($pins: expr, $clocks:expr, $timer:expr, $pac:expr) => {{
        let (mut pio, sm0, _, _, _) = $pac.PIO0.split(&mut $pac.RESETS);
        macropad_neopixels!($pins, pio, sm0, $clocks, $timer)
    }};
}

#[macro_export]
macro_rules! macropad_oled {
    ($pins:expr, $clocks:expr, $delay: expr, $pac:expr) => {{
        let _spi_sclk = $pins.sclk.into_mode::<$crate::FunctionSpi>();
        let _spi_sclk = $pins.mosi.into_mode::<$crate::FunctionSpi>();

        let spi1 = $crate::Spi::<_, _, 8>::new($pac.SPI1).init(
            &mut $pac.RESETS,
            $crate::Clock::freq(&$clocks.peripheral_clock),
            $crate::Extensions::Hz(16_000_000u32),
            // (16_000_000u32 as embedded_time::rate::Extensions).Hz(),
            &$crate::MODE_0,
        );

        let mut oled_reset = $pins.oled_reset.into_push_pull_output();

        let mut disp: $crate::GraphicsMode<_> = sh1106::Builder::new()
            .connect_spi(
                spi1,
                $pins.oled_dc.into_push_pull_output(),
                $pins.oled_cs.into_push_pull_output(),
            )
            .into();

        disp.reset(&mut oled_reset, &mut $delay).unwrap();

        disp.init().unwrap();
        disp
    }};
}

#[macro_export]
macro_rules! macropad_pins {
    ($pac:expr) => {{
        let sio = $crate::Sio::new($pac.SIO);
        $crate::Pins::new(
            $pac.IO_BANK0,
            $pac.PADS_BANK0,
            sio.gpio_bank0,
            &mut $pac.RESETS,
        )
    }};
}

#[macro_export]
macro_rules! macropad_clocks {
    ($pac:expr, $watchdog:expr) => {
        $crate::init_clocks_and_plls(
            $crate::XOSC_CRYSTAL_FREQ,
            $pac.XOSC,
            $pac.CLOCKS,
            $pac.PLL_SYS,
            $pac.PLL_USB,
            &mut $pac.RESETS,
            &mut $watchdog,
        )
        .ok()
        .unwrap()
    };
}

#[macro_export]
macro_rules! macropad_rotary_encoder {
    ($pins:expr) => {
        Rotary::new(
            $pins.encoder_rota.into_pull_up_input(),
            $pins.encoder_rotb.into_pull_up_input(),
        )
    };
}

#[macro_export]
macro_rules! macropad_keypad {
    ($pins:expr) => {
        KeysTwelve {
            key1: $pins.key1.into_pull_up_input(),
            key2: $pins.key2.into_pull_up_input(),
            key3: $pins.key3.into_pull_up_input(),
            key4: $pins.key4.into_pull_up_input(),
            key5: $pins.key5.into_pull_up_input(),
            key6: $pins.key6.into_pull_up_input(),
            key7: $pins.key7.into_pull_up_input(),
            key8: $pins.key8.into_pull_up_input(),
            key9: $pins.key9.into_pull_up_input(),
            key10: $pins.key10.into_pull_up_input(),
            key11: $pins.key11.into_pull_up_input(),
            key12: $pins.key12.into_pull_up_input(),
        }
    };
}

pub struct KeysTwelve {
    pub key1: Pin<Gpio1, PullUpInput>,
    pub key2: Pin<Gpio2, PullUpInput>,
    pub key3: Pin<Gpio3, PullUpInput>,
    pub key4: Pin<Gpio4, PullUpInput>,
    pub key5: Pin<Gpio5, PullUpInput>,
    pub key6: Pin<Gpio6, PullUpInput>,
    pub key7: Pin<Gpio7, PullUpInput>,
    pub key8: Pin<Gpio8, PullUpInput>,
    pub key9: Pin<Gpio9, PullUpInput>,
    pub key10: Pin<Gpio10, PullUpInput>,
    pub key11: Pin<Gpio11, PullUpInput>,
    pub key12: Pin<Gpio12, PullUpInput>,
}

impl KeysTwelve {
    pub fn get_0based(&self, idx: i8) -> Option<&dyn InputPin<Error = Infallible>> {
        self.get_1based(1 + idx)
    }

    pub fn get_1based(&self, idx: i8) -> Option<&dyn InputPin<Error = Infallible>> {
        match idx {
            1 => Some(&self.key1),
            2 => Some(&self.key2),
            3 => Some(&self.key3),
            4 => Some(&self.key4),
            5 => Some(&self.key5),
            6 => Some(&self.key6),
            7 => Some(&self.key7),
            8 => Some(&self.key8),
            9 => Some(&self.key9),
            10 => Some(&self.key10),
            11 => Some(&self.key11),
            12 => Some(&self.key12),
            _ => None,
        }
    }

    pub fn array_0based(&self) -> [&dyn InputPin<Error = Infallible>; 12] {
        [
            &self.key1,
            &self.key2,
            &self.key3,
            &self.key4,
            &self.key5,
            &self.key6,
            &self.key7,
            &self.key8,
            &self.key9,
            &self.key10,
            &self.key11,
            &self.key12,
        ]
    }
}
