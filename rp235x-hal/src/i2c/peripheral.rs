//! # I2C Peripheral (slave) implementation
//!
//! The RP2350 I2C block can behave as a peripheral node on an I2C bus.
//!
//! In order to handle peripheral transactions this driver exposes an iterator streaming I2C event
//! that the usercode must handle to properly handle the I2C communitation. See [`Event`] for a
//! list of events to handle.
//!
//! Although [`Start`](Event::Start), [`Restart`](Event::Restart) and [`Stop`](Event::Stop)
//! events may not require any action on the device, [`TransferRead`](Event::TransferRead) and
//! [`TransferWrite`](Event::TransferWrite) require some action:
//!
//! - [`TransferRead`](Event::TransferRead): A controller is attempting to read from this peripheral.  
//!   The I2C block holds the SCL line low (clock stretching) until data is pushed to the transmission
//!   FIFO by the user application using [`write`](I2C::write).  
//!   Data remaining in the FIFO when the bus constroller stops the transfer are ignored & the fifo
//!   is flushed.
//! - [`TransferWrite`](Event::TransferWrite): A controller is sending data to this peripheral.  
//!   The I2C block holds the SCL line (clock stretching) until there is room for more data in the
//!   Rx FIFO using [`read`](I2C::read).  
//!   Data are automatically acknowledged by the I2C block and it is not possible to NACK incoming
//!   data coming to the RP2350.
//!
//! ## Warning
//!
//! `Start`, `Restart` and `Stop` events may not be reported before or after a write operations.
//! This is because several write operation may take place and complete before the core has time to
//! react. All the data sent will be stored in the peripheral FIFO but it will not be possible to
//! identify between which bytes should the start/restart/stop events precicely took place.
//!
//! Because a Read operation will always cause a pause waiting for the firmware's input, a `Start`
//! (or `Restart` if the peripheral is already active) will always be reported. However, this does
//! not mean no other event occurred in the mean time.
//!
//! For example, let's consider the following sequence:
//!
//! `Start, Write, Stop, Start, Write, Restart, Read, Stop.`
//!
//! Depending on the firmware's and bus' speed, this driver may only report:
//! - `Start, Write, Restart, Read, Stop`

use core::{ops::Deref, task::Poll};

use embedded_hal::i2c::AddressMode;

use super::{Peripheral, ValidAddress, ValidPinScl, ValidPinSda, I2C};
use crate::{
    async_utils::{sealed::Wakeable, AsyncPeripheral, CancellablePollFn},
    pac::{i2c0::RegisterBlock, RESETS},
    resets::SubsystemReset,
};

/// I2C bus events
#[derive(Debug, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum Event {
    /// Start condition has been detected.
    Start,
    /// Restart condition has been detected.
    Restart,
    /// The controller requests data.
    TransferRead,
    /// The controller sends data.
    TransferWrite,
    /// Stop condition detected.
    Stop,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub(crate) enum State {
    Idle,
    Active,
    Read,
    Write,
}

impl<T, Sda, Scl> I2C<T, (Sda, Scl), Peripheral>
where
    T: SubsystemReset + Deref<Target = RegisterBlock>,
    Sda: ValidPinSda<T>,
    Scl: ValidPinScl<T>,
{
    /// Configures the I2C peripheral to work in peripheral mode
    ///
    /// The bus *MUST* be idle when this method is called.
    #[allow(clippy::type_complexity)]
    pub fn new_peripheral_event_iterator<A: ValidAddress + AddressMode>(
        i2c: T,
        sda_pin: Sda,
        scl_pin: Scl,
        resets: &mut RESETS,
        addr: A,
    ) -> Self {
        i2c.reset_bring_down(resets);
        i2c.reset_bring_up(resets);

        i2c.ic_enable().write(|w| w.enable().disabled());

        // TODO: Validate address?
        let addr = addr.into();
        // SAFETY: Only address by this function. IC_SAR spec filters out bits 15:10.
        // Any value is valid for the controller. They may not be for the bus itself though.
        i2c.ic_sar().write(|w| unsafe { w.ic_sar().bits(addr) });
        // select peripheral mode & speed
        i2c.ic_con().modify(|_, w| {
            // run in fast mode
            w.speed().fast();
            // setup slave mode
            w.master_mode().disabled();
            w.ic_slave_disable().slave_enabled();
            // hold scl when fifo's full
            w.rx_fifo_full_hld_ctrl().enabled();
            w.ic_restart_en().enabled();
            w.ic_10bitaddr_slave().variant(A::BIT_ADDR_S);
            w
        });

        // Clear FIFO threshold
        // SAFETY: Only address by this function. The field is 8bit long. 0 is a valid value.
        i2c.ic_tx_tl().write(|w| unsafe { w.tx_tl().bits(0) });
        i2c.ic_rx_tl().write(|w| unsafe { w.rx_tl().bits(0) });

        i2c.ic_clr_intr().read();

        let mut me = Self {
            i2c,
            pins: (sda_pin, scl_pin),
            mode: Peripheral { state: State::Idle },
        };
        me.unmask_intr();
        // Enable I2C block
        me.i2c.ic_enable().write(|w| w.enable().enabled());

        me
    }
}

fn unmask_intr(i2c: &RegisterBlock) {
    // SAFETY: 0 is a valid value meaning all irq masked.
    // This operation is atomic, `write_with_zero` only writes to the register.
    unsafe {
        i2c.ic_intr_mask().write_with_zero(|w| {
            // Only keep these IRQ enabled.
            w.m_start_det()
                .disabled()
                .m_rd_req()
                .disabled()
                .m_rx_full()
                .disabled()
                .m_stop_det()
                .disabled()
        });
    }
}

/// SAFETY: Takes a non-mutable reference to RegisterBlock but mutates its `ic_intr_mask` register.
unsafe fn mask_intr(i2c: &RegisterBlock) {
    // 0 is a valid value and means all flag masked.
    unsafe { i2c.ic_intr_mask().write_with_zero(|w| w) }
}

impl<T: Deref<Target = RegisterBlock>, PINS> I2C<T, PINS, Peripheral> {
    fn unmask_intr(&mut self) {
        unmask_intr(&self.i2c)
    }
    fn mask_intr(&mut self) {
        // SAFETY: We are the only owner of this register block.
        unsafe { mask_intr(&self.i2c) }
    }

    /// Push up to `usize::min(TX_FIFO_SIZE, buf.len())` bytes to the TX FIFO.
    /// Returns the number of bytes pushed to the FIFO. Note this does *not* reflect how many bytes
    /// are effectively received by the controller.
    pub fn write(&mut self, buf: &[u8]) -> usize {
        // just in case, clears previous tx abort.
        self.i2c.ic_clr_tx_abrt().read();

        let mut sent = 0;
        for &b in buf.iter() {
            if self.tx_fifo_full() {
                break;
            }

            // SAFETY: dat field is 8bits long. All values are valid.
            self.i2c.ic_data_cmd().write(|w| unsafe { w.dat().bits(b) });
            sent += 1;
        }
        // serve a pending read request
        self.i2c.ic_clr_rd_req().read();
        sent
    }

    /// Pull up to `usize::min(RX_FIFO_SIZE, buf.len())` bytes from the RX FIFO.
    pub fn read(&mut self, buf: &mut [u8]) -> usize {
        buf.iter_mut().zip(self).map(|(b, r)| *b = r).count()
    }
}

/// This allows I2C to be used with `core::iter::Extend`.
impl<T: Deref<Target = RegisterBlock>, PINS> Iterator for I2C<T, PINS, Peripheral> {
    type Item = u8;

    fn next(&mut self) -> Option<Self::Item> {
        if self.rx_fifo_empty() {
            None
        } else {
            Some(self.i2c.ic_data_cmd().read().dat().bits())
        }
    }
}
impl<T: Deref<Target = RegisterBlock>, PINS> I2C<T, PINS, Peripheral> {
    /// Returns the next i2c event if any.
    pub fn next_event(&mut self) -> Option<Event> {
        let stat = self.i2c.ic_raw_intr_stat().read();

        match self.mode.state {
            State::Idle if stat.start_det().bit_is_set() => {
                self.i2c.ic_clr_start_det().read();
                self.mode.state = State::Active;
                Some(Event::Start)
            }
            State::Active if stat.rd_req().bit_is_set() => {
                // Clearing `rd_req` is used by the hardware to detect when the I2C block can stop
                // stretching the clock and start process the data pushed to the FIFO (if any).
                // This is done in `Self::write`.
                self.mode.state = State::Read;
                // If stop_det is set at this point we know that it is related to a previous request,
                // It cannot be due the end of the current request as SCL is held low while waiting
                // for user input.
                if stat.stop_det().bit_is_set() {
                    self.i2c.ic_clr_stop_det().read();
                }
                Some(Event::TransferRead)
            }
            State::Active if !self.rx_fifo_empty() => {
                self.mode.state = State::Write;
                Some(Event::TransferWrite)
            }

            State::Read if stat.rd_req().bit_is_set() => Some(Event::TransferRead),
            State::Write if !self.rx_fifo_empty() => Some(Event::TransferWrite),

            State::Read | State::Write if stat.restart_det().bit_is_set() => {
                self.i2c.ic_clr_restart_det().read();
                self.i2c.ic_clr_start_det().read();
                self.mode.state = State::Active;
                Some(Event::Restart)
            }

            _ if stat.stop_det().bit_is_set() => {
                self.i2c.ic_clr_stop_det().read();
                self.i2c.ic_clr_tx_abrt().read();
                self.mode.state = State::Idle;
                Some(Event::Stop)
            }

            _ => None,
        }
    }
}

macro_rules! impl_wakeable {
    ($i2c:ty) => {
        impl<PINS> AsyncPeripheral for I2C<$i2c, PINS, Peripheral>
        where
            I2C<$i2c, PINS, Peripheral>: $crate::async_utils::sealed::Wakeable,
        {
            /// Wakes an async task (if any) & masks irqs
            fn on_interrupt() {
                unsafe {
                    // This is equivalent to stealing from pac::Peripherals
                    let i2c = &*<$i2c>::ptr();

                    mask_intr(i2c);
                }

                // interrupts are now masked, we can wake the task and return from this handler.
                Self::waker().wake();
            }
        }
    };
}
impl_wakeable!(rp235x_pac::I2C0);
impl_wakeable!(rp235x_pac::I2C1);

impl<T, PINS> I2C<T, PINS, Peripheral>
where
    I2C<T, PINS, Peripheral>: AsyncPeripheral,
    T: Deref<Target = RegisterBlock>,
{
    /// Asynchronously waits for an Event.
    pub async fn wait_next(&mut self) -> Event {
        loop {
            if let Some(evt) = self.next_event() {
                return evt;
            }

            CancellablePollFn::new(
                self,
                |me| {
                    let stat = me.i2c.ic_raw_intr_stat().read();
                    if stat.start_det().bit_is_set()
                        || stat.restart_det().bit_is_set()
                        || stat.stop_det().bit_is_set()
                        || stat.rd_req().bit_is_set()
                        || stat.rx_full().bit_is_set()
                    {
                        Poll::Ready(())
                    } else {
                        Poll::Pending
                    }
                },
                Self::unmask_intr,
                Self::mask_intr,
            )
            .await;
        }
    }
}
