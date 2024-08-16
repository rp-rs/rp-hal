//! # ADC FIFO Interrupt Example
//!
//! This application demonstrates how to read ADC samples in free-running mode,
//! using the FIFO interrupt.
//!
//! It utilizes `rtic` (cortex-m-rtic crate) to safely share peripheral access between
//! initialization code and interrupt handlers.
//!
//! It may need to be adapted to your particular board layout and/or pin assignment.
//!
//! See the top-level `README.md` file for Copyright and license details.

#![no_std]
#![no_main]

use panic_halt as _;

/// The linker will place this boot block at the start of our program image. We
/// need this to help the ROM bootloader get our code up and running.
/// Note: This boot block is not necessary when using a rp-hal based BSP
/// as the BSPs already perform this step.
#[link_section = ".boot2"]
#[used]
pub static BOOT2: [u8; 256] = rp2040_boot2::BOOT_LOADER_GENERIC_03H;

#[rtic::app(device = rp2040_hal::pac)]
mod app {
    use core::fmt::Write;
    use fugit::RateExtU32;
    use hal::Clock;
    use rp2040_hal as hal;

    /// External high-speed crystal on the Raspberry Pi Pico board is 12 MHz. Adjust
    /// if your board has a different frequency
    const XTAL_FREQ_HZ: u32 = 12_000_000u32;

    // This example will capture 1000 samples to `shared.buf`.
    // When it is done, it will stop the ADC, set `shared.done` to true,
    // print the result and loop forever.

    const SAMPLE_COUNT: usize = 1000;

    type Uart = hal::uart::UartPeripheral<
        hal::uart::Enabled,
        hal::pac::UART0,
        (
            hal::gpio::Pin<hal::gpio::bank0::Gpio0, hal::gpio::FunctionUart, hal::gpio::PullDown>,
            hal::gpio::Pin<hal::gpio::bank0::Gpio1, hal::gpio::FunctionUart, hal::gpio::PullDown>,
        ),
    >;

    #[shared]
    struct Shared {
        done: bool,
        buf: [u16; SAMPLE_COUNT],
        uart: Uart,
    }

    #[local]
    struct Local {
        adc_fifo: Option<hal::adc::AdcFifo<'static, u16>>,
    }

    #[init(local = [adc: Option<hal::Adc> = None])]
    fn init(c: init::Context) -> (Shared, Local, init::Monotonics) {
        // Soft-reset does not release the hardware spinlocks
        // Release them now to avoid a deadlock after debug or watchdog reset
        unsafe {
            hal::sio::spinlock_reset();
        }

        let mut resets = c.device.RESETS;
        let mut watchdog = hal::Watchdog::new(c.device.WATCHDOG);
        let clocks = hal::clocks::init_clocks_and_plls(
            XTAL_FREQ_HZ,
            c.device.XOSC,
            c.device.CLOCKS,
            c.device.PLL_SYS,
            c.device.PLL_USB,
            &mut resets,
            &mut watchdog,
        )
        .unwrap();
        let sio = hal::Sio::new(c.device.SIO);
        let pins = hal::gpio::Pins::new(
            c.device.IO_BANK0,
            c.device.PADS_BANK0,
            sio.gpio_bank0,
            &mut resets,
        );

        // UART TX (characters sent from pico) on pin 1 (GPIO0) and RX (on pin 2 (GPIO1)
        let uart_pins = (
            pins.gpio0.into_function::<hal::gpio::FunctionUart>(),
            pins.gpio1.into_function::<hal::gpio::FunctionUart>(),
        );

        // Create a UART driver
        let uart = hal::uart::UartPeripheral::new(c.device.UART0, uart_pins, &mut resets)
            .enable(
                hal::uart::UartConfig::new(
                    115200.Hz(),
                    hal::uart::DataBits::Eight,
                    None,
                    hal::uart::StopBits::One,
                ),
                clocks.peripheral_clock.freq(),
            )
            .unwrap();

        // the ADC is put into a local, to gain static lifetime
        *c.local.adc = Some(hal::Adc::new(c.device.ADC, &mut resets));
        let adc = c.local.adc.as_mut().unwrap();

        let mut adc_pin_0 = hal::adc::AdcPin::new(pins.gpio26.into_floating_input()).unwrap();

        uart.write_full_blocking(b"ADC FIFO interrupt example\r\n");

        let adc_fifo = adc
            .build_fifo()
            // Set clock divider to target a sample rate of 1000 samples per second (1ksps).
            // The value was calculated by `(48MHz / 1ksps) - 1 = 47999.0`.
            // Please check the `clock_divider` method documentation for details.
            .clock_divider(47999, 0)
            .set_channel(&mut adc_pin_0)
            .enable_interrupt(1)
            .start();

        (
            Shared {
                done: false,
                buf: [0; SAMPLE_COUNT],
                uart,
            },
            Local {
                adc_fifo: Some(adc_fifo),
            },
            init::Monotonics(),
        )
    }

    #[idle(shared = [done, buf, uart])]
    fn idle(mut c: idle::Context) -> ! {
        loop {
            let finished = (&mut c.shared.done, &mut c.shared.buf, &mut c.shared.uart).lock(
                |done, buf, uart| {
                    if *done {
                        for sample in buf {
                            writeln!(uart, "Sample: {}\r", sample).unwrap();
                        }
                        writeln!(uart, "All done, going to sleep 😴\r").unwrap();
                        true
                    } else {
                        false
                    }
                },
            );

            if finished {
                break;
            }
        }

        #[allow(clippy::empty_loop)]
        loop {}
    }

    #[task(
        binds = ADC_IRQ_FIFO,
        priority = 1,
        shared = [done, buf],
        local = [adc_fifo, counter: usize = 0]
    )]
    fn adc_irq_fifo(mut c: adc_irq_fifo::Context) {
        let sample = c.local.adc_fifo.as_mut().unwrap().read();
        let i = *c.local.counter;
        c.shared.buf.lock(|buf| buf[i] = sample);
        *c.local.counter += 1;

        if *c.local.counter == SAMPLE_COUNT {
            c.local.adc_fifo.take().unwrap().stop();
            c.shared.done.lock(|done| *done = true);
        }
    }
}
