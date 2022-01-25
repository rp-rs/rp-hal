use core::{marker::PhantomData, ops::Deref};

use crate::{
    gpio::pin::bank0::BankPinId,
    gpio::pin::{FunctionI2C, Pin, PinId},
    resets::SubsystemReset,
};
use pac::{i2c0::RegisterBlock as I2CBlock, RESETS};

use super::{Peripheral, SclPin, SdaPin, I2C};

/// I2C bus events
#[derive(Debug, PartialEq, Eq)]
pub enum I2CEvent {
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

#[derive(Debug, Clone, Copy)]
enum State {
    Idle,
    Active,
    Read,
    Write,
}

/// Provides Async features to I2C peripheral.
pub struct I2CPeripheralEventIterator<Block, Pins> {
    i2c: I2C<Block, Pins, Peripheral>,
    state: State,
}

impl<T, Sda, Scl> I2C<T, (Pin<Sda, FunctionI2C>, Pin<Scl, FunctionI2C>), Peripheral>
where
    T: SubsystemReset + Deref<Target = I2CBlock>,
    Sda: PinId + BankPinId,
    Scl: PinId + BankPinId,
{
    /// Configures the I2C peripheral to work in peripheral mode
    ///
    /// The bus *MUST* be idle when this method is called.
    #[allow(clippy::type_complexity)]
    pub fn new_peripheral_event_iterator(
        i2c: T,
        sda_pin: Pin<Sda, FunctionI2C>,
        scl_pin: Pin<Scl, FunctionI2C>,
        resets: &mut RESETS,
        addr: u16,
    ) -> I2CPeripheralEventIterator<T, (Pin<Sda, FunctionI2C>, Pin<Scl, FunctionI2C>)>
    where
        Sda: SdaPin<T>,
        Scl: SclPin<T>,
    {
        i2c.reset_bring_down(resets);
        i2c.reset_bring_up(resets);

        i2c.ic_enable.write(|w| w.enable().disabled());

        // TODO: rp2040 supports 10bits addressing
        //i2c_reserved_addr(addr)
        i2c.ic_sar.write(|w| unsafe { w.ic_sar().bits(addr) });
        // select peripheral mode & speed
        i2c.ic_con.modify(|_, w| {
            // run in fast mode
            w.speed().fast();
            // setup slave mode
            w.master_mode().disabled();
            w.ic_slave_disable().slave_enabled();
            // hold scl when fifo's full
            w.rx_fifo_full_hld_ctrl().enabled();
            w.ic_restart_en().enabled();
            w.tx_empty_ctrl().enabled()
        });

        // Clear FIFO threshold
        i2c.ic_tx_tl.write(|w| unsafe { w.tx_tl().bits(0) });
        i2c.ic_rx_tl.write(|w| unsafe { w.rx_tl().bits(0) });

        // Enable I2C block
        i2c.ic_enable.write(|w| w.enable().enabled());

        I2CPeripheralEventIterator {
            i2c: Self {
                i2c,
                pins: (sda_pin, scl_pin),
                mode: PhantomData,
            },
            state: State::Idle,
        }
    }
}

impl<T: Deref<Target = I2CBlock>, PINS> I2CPeripheralEventIterator<T, PINS> {
    /// Push up to `usize::min(TX_FIFO_SIZE, buf.len())` bytes to the TX FIFO.
    /// Returns the number of bytes pushed to the FIFO. Note this does *not* reflect how many bytes
    /// are effectively received by the controller.
    pub fn write(&mut self, buf: &[u8]) -> usize {
        // just in case, clears previous tx abort.
        self.i2c.i2c.ic_clr_tx_abrt.read();

        let mut sent = 0;
        for &b in buf.iter() {
            if self.i2c.tx_fifo_full() {
                break;
            }

            self.i2c
                .i2c
                .ic_data_cmd
                .write(|w| unsafe { w.dat().bits(b) });
            sent += 1;
        }
        // serve a pending read request
        self.i2c.i2c.ic_clr_rd_req.read();
        sent
    }

    /// Pull up to `usize::min(RX_FIFO_SIZE, buf.len())` bytes from the RX FIFO.
    pub fn read(&mut self, buf: &mut [u8]) -> usize {
        let mut read = 0;

        for b in buf.iter_mut() {
            if self.i2c.rx_fifo_empty() {
                break;
            }

            *b = self.i2c.i2c.ic_data_cmd.read().dat().bits();
            read += 1;
        }
        read
    }
}
impl<T: Deref<Target = I2CBlock>, PINS> Iterator for I2CPeripheralEventIterator<T, PINS> {
    type Item = I2CEvent;

    fn next(&mut self) -> Option<Self::Item> {
        let stat = self.i2c.i2c.ic_raw_intr_stat.read();
        self.i2c.i2c.ic_clr_activity.read();

        match self.state {
            State::Idle if stat.start_det().bit_is_set() => {
                self.i2c.i2c.ic_clr_start_det.read();
                self.state = State::Active;
                Some(I2CEvent::Start)
            }
            State::Active if !self.i2c.rx_fifo_empty() => {
                self.state = State::Write;
                Some(I2CEvent::TransferWrite)
            }
            State::Active if stat.rd_req().bit_is_set() => {
                // Clearing `rd_req` is used by the hardware to detect when the I2C block can stop
                // stretching the clock and start process the data pushed to the FIFO (if any).
                // This is done in `Self::write`.
                self.state = State::Read;
                Some(I2CEvent::TransferRead)
            }
            State::Read if stat.rd_req().bit_is_set() => Some(I2CEvent::TransferRead),
            State::Read if stat.restart_det().bit_is_set() => {
                self.i2c.i2c.ic_clr_restart_det.read();
                self.state = State::Active;
                Some(I2CEvent::Restart)
            }
            State::Write if !self.i2c.rx_fifo_empty() => Some(I2CEvent::TransferWrite),
            State::Write if stat.restart_det().bit_is_set() => {
                self.i2c.i2c.ic_clr_restart_det.read();
                self.state = State::Active;
                Some(I2CEvent::Restart)
            }
            _ if stat.stop_det().bit_is_set() => {
                self.i2c.i2c.ic_clr_stop_det.read();
                self.state = State::Idle;
                Some(I2CEvent::Stop)
            }
            _ => None,
        }
    }
}

impl<Block, Sda, Scl>
    I2CPeripheralEventIterator<Block, (Pin<Sda, FunctionI2C>, Pin<Scl, FunctionI2C>)>
where
    Block: SubsystemReset + Deref<Target = I2CBlock>,
    Sda: PinId + BankPinId,
    Scl: PinId + BankPinId,
{
    /// Releases the I2C peripheral and associated pins
    #[allow(clippy::type_complexity)]
    pub fn free(
        self,
        resets: &mut RESETS,
    ) -> (Block, (Pin<Sda, FunctionI2C>, Pin<Scl, FunctionI2C>)) {
        self.i2c.free(resets)
    }
}
