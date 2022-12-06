//! # Pico USB to UART passthrough
//!
//! Creates a USB Serial device on a Pico board, with the USB driver running in
//! the USB interrupt.
//!
//! This will create a USB Serial device passing the data through to a UART. UART incoming
//! data in turn is then sent back over the USB.
//!
//! See the `Cargo.toml` file for Copyright and license details.
#![no_std]
#![no_main]

use panic_halt as _;
#[rtic::app(device = rp_pico::hal::pac, peripherals = true, dispatchers = [I2C0_IRQ])]
mod app {

    // GPIO traits
    use fugit::RateExtU32;
    use fugit::ExtU64;

    // Ensure we halt the program on panic (if we don't mention this crate it won't
    // be linked)
    use panic_halt as _;

    // Pull in any important traits
    use rp_pico::hal::prelude::*;

    // A shorter alias for the Peripheral Access Crate, which provides low-level
    // register access
    use rp_pico::hal::pac;

    // USB Device support
    use usb_device::{class_prelude::*, prelude::*};

    // USB Communications Class Device support
    use usbd_serial::SerialPort;

    /// Import the GPIO pins we use
    use hal::gpio::pin::bank0::{Gpio0, Gpio1};

    // UART related types
    use hal::uart::{DataBits, StopBits, UartConfig};

    // These are the traits we need from Embedded HAL to treat our hardware
    // objects as generic embedded devices.
    use embedded_hal::{
        digital::v2::OutputPin,
        serial::{Read, Write},
    };

    /// Alias the type for our UART pins to make things clearer.
    type UartPins = (
        hal::gpio::Pin<Gpio0, hal::gpio::Function<hal::gpio::Uart>>,
        hal::gpio::Pin<Gpio1, hal::gpio::Function<hal::gpio::Uart>>,
    );

    /// Alias the type for our UART to make things clearer.
    type Uart = hal::uart::UartPeripheral<hal::uart::Enabled, pac::UART0, UartPins>;

    use heapless::spsc::{Consumer, Producer, Queue};
    use rp_pico::{
        hal::{
            self,
            clocks::init_clocks_and_plls,
            watchdog::Watchdog,
            Sio,
            timer::{monotonic::Monotonic, Alarm0},
        },
        XOSC_CRYSTAL_FREQ,
    };
    const BUF_SIZE: usize = 64;
    const DOUBLE_BUF_SIZE: usize = BUF_SIZE * 2;

    /// Used to leak a &'static reference on the USB allocator
    static mut USB_BUS: Option<UsbBusAllocator<hal::usb::UsbBus>> = None;

    /// Used for &'static references for the queues
    static mut USB_QUEUE: Queue<u8, DOUBLE_BUF_SIZE> = Queue::new();
    static mut UART_QUEUE: Queue<u8, DOUBLE_BUF_SIZE> = Queue::new();

    #[shared]
    struct Shared {
        led: hal::gpio::Pin<hal::gpio::pin::bank0::Gpio25, hal::gpio::PushPullOutput>,
        uart: Uart,
        serial: SerialPort<'static, hal::usb::UsbBus>,
    }

    #[local]
    struct Local {
        usb_device: UsbDevice<'static, hal::usb::UsbBus>,
        usb_producer: Producer<'static, u8, DOUBLE_BUF_SIZE>,
        usb_consumer: Consumer<'static, u8, DOUBLE_BUF_SIZE>,
        uart_producer: Producer<'static, u8, DOUBLE_BUF_SIZE>,
        uart_consumer: Consumer<'static, u8, DOUBLE_BUF_SIZE>,
    }

    #[monotonic(binds = TIMER_IRQ_0, default = true)]
    type MyMono = Monotonic<Alarm0>;

    #[init]
    fn init(c: init::Context) -> (Shared, Local, init::Monotonics) {
        // Soft-reset does not release the hardware spinlocks
        // Release them now to avoid a deadlock after debug or watchdog reset
        unsafe {
            hal::sio::spinlock_reset();
        }
        let mut resets = c.device.RESETS;
        let mut watchdog = Watchdog::new(c.device.WATCHDOG);
        let clocks = init_clocks_and_plls(
            XOSC_CRYSTAL_FREQ,
            c.device.XOSC,
            c.device.CLOCKS,
            c.device.PLL_SYS,
            c.device.PLL_USB,
            &mut resets,
            &mut watchdog,
        )
            .ok()
            .unwrap();

        let sio = Sio::new(c.device.SIO);
        let pins = rp_pico::Pins::new(
            c.device.IO_BANK0,
            c.device.PADS_BANK0,
            sio.gpio_bank0,
            &mut resets,
        );
        let mut led = pins.led.into_push_pull_output();
        led.set_low().unwrap();

        let usb_bus = UsbBusAllocator::new(hal::usb::UsbBus::new(
            c.device.USBCTRL_REGS,
            c.device.USBCTRL_DPRAM,
            clocks.usb_clock,
            true,
            &mut resets,
        ));

        unsafe {
            // Note (safety): This is safe as interrupts haven't been started yet
            USB_BUS = Some(usb_bus);
        }

        // Grab a reference to the USB Bus allocator. We are promising to the
        // compiler not to take mutable access to this global variable whilst this
        // reference exists!
        let bus_ref = unsafe { USB_BUS.as_ref().unwrap() };

        let serial = SerialPort::new(bus_ref);

        let usb_bus = unsafe { USB_BUS.as_mut().unwrap() };

        // Create a USB device with a fake VID and PID
        let usb_device = UsbDeviceBuilder::new(usb_bus, UsbVidPid(0x16c0, 0x27dd))
            .manufacturer("Fake company")
            .product("Serial port")
            .serial_number("MJVN")
            .device_class(2) // from: https://www.usb.org/defined-class-codes
            .build();

        let uart_pins = (
            // UART TX (characters sent from RP2040) on pin 1 (GPIO0)
            pins.gpio0.into_mode::<hal::gpio::FunctionUart>(),
            // UART RX (characters received by RP2040) on pin 2 (GPIO1)
            pins.gpio1.into_mode::<hal::gpio::FunctionUart>(),
        );

        // Make a UART on the given pins
        let mut uart = hal::uart::UartPeripheral::new(c.device.UART0, uart_pins, &mut resets)
            .enable(
                UartConfig::new(115200.Hz(), DataBits::Eight, None, StopBits::One),
                clocks.peripheral_clock.freq(),
            )
            .unwrap();

        // Tell the UART to raise its interrupt line on the NVIC when the RX FIFO
        // has data in it.
        uart.enable_rx_interrupt();

        let (usb_producer, usb_consumer) = unsafe { USB_QUEUE.split() };
        let (uart_producer, uart_consumer) = unsafe { UART_QUEUE.split() };

        let mut timer = hal::Timer::new(c.device.TIMER, &mut resets);
        let alarm = timer.alarm_0().unwrap();
        blink_led::spawn_after(500.millis()).unwrap();

        (
            Shared {
                led,
                serial,
                uart,
            },
            Local {
                usb_producer,
                usb_consumer,
                uart_producer,
                uart_consumer,
                usb_device,
            },
            init::Monotonics(Monotonic::new(timer, alarm)),
        )
    }

    #[task(
        binds = UART0_IRQ,
        priority = 1,
        shared = [uart],
        local = [usb_producer]
    )]
    fn uart_read(mut c: uart_read::Context) {
        let capacity = c.local.usb_producer.capacity();
        let len = c.local.usb_producer.len();
        // Check if we can take at least the buffer
        let ready = capacity - len > BUF_SIZE;
        if !ready {
            // Queue is full. Retry in 1ms
            pac::NVIC::pend(hal::pac::Interrupt::UART0_IRQ);
        }

        while let Ok(byte) = c.shared.uart.lock(|uart| uart.read()) {
            c.local.usb_producer.enqueue(byte).unwrap();

            let len = c.local.usb_producer.len();
            if len > BUF_SIZE {
                usb_write::spawn_after(0.millis()).unwrap();
            } else if len == DOUBLE_BUF_SIZE {
                // Queue is full. Retry again
                pac::NVIC::pend(hal::pac::Interrupt::UART0_IRQ);
            }
        }
        usb_write::spawn_after(0.millis()).unwrap();
    }

    #[task(
        binds = USBCTRL_IRQ,
        priority = 1,
        shared = [serial],
        local = [usb_device, uart_producer]
    )]
    fn usb_read(mut c: usb_read::Context) {
        let mut buf = [0u8; BUF_SIZE];
        loop {
            let capacity = c.local.uart_producer.capacity();
            let len = c.local.uart_producer.len();
            // Check if we can take at least the buffer
            let ready = capacity - len > BUF_SIZE;
            if !ready {
                // Queue is full. Retry again
                pac::NVIC::pend(hal::pac::Interrupt::USBCTRL_IRQ);
                break;
            }
            let usb_device = &mut c.local.usb_device;
            let uart_producer = &mut c.local.uart_producer;
            let count = c.shared.serial.lock(|serial| {
                if usb_device.poll(&mut [serial]) {
                    match serial.read(&mut buf) {
                        Ok(count) => {
                            buf.iter().take(count).for_each(|b|
                                uart_producer.enqueue(*b).ok().unwrap()
                            );
                            uart_write::spawn_after(0.millis()).unwrap();
                            count
                        }
                        _ => 0
                    }
                } else {
                    0
                }
            });

            if count == 0 {
                break;
            }
        }
    }

    #[task(
        priority = 1,
        shared = [uart],
        local = [uart_consumer]
    )]
    fn uart_write(mut c: uart_write::Context) {
        while let Some(b) = c.local.uart_consumer.dequeue() {
            let _ = c.shared.uart.lock(|uart| uart.write(b));
        }
    }

    #[task(
        priority = 1,
        shared = [serial],
        local = [usb_consumer]
    )]
    fn usb_write(mut c: usb_write::Context) {
        let mut buf = [0u8; BUF_SIZE];
        let mut ptr = 0usize;
        while let Some(b) = c.local.usb_consumer.dequeue() {
            buf[ptr] = b;
            ptr += 1;
            if ptr == BUF_SIZE {
                let _ = c.shared.serial.lock(|serial| serial.write(&buf));
                ptr = 0;
            }
        }
        if ptr > 0 {
            let buf_ptr = &buf[0..ptr];
            let _ = c.shared.serial.lock(|serial| serial.write(buf_ptr));
        }
    }

    #[task(
        shared = [led],
        local = [tog: bool = true],
    )]
    fn blink_led(mut c: blink_led::Context) {
        if *c.local.tog {
            c.shared.led.lock(|l| l.set_high().unwrap());
        } else {
            c.shared.led.lock(|l| l.set_low().unwrap());
        }
        *c.local.tog = !*c.local.tog;

        blink_led::spawn_after(500.millis()).unwrap();
    }
}
