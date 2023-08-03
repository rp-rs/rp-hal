use usbh::{
    bus::{
        HostBus,
        PollResult,
        Event,
        Error,
    },
    types::{
        ConnectionSpeed,
        DeviceAddress,
        TransferType,
    },
};
use usb_device::UsbDirection;

use critical_section::Mutex;
use core::cell::RefCell;
use crate::clocks::UsbClock;
use crate::pac::{
    RESETS,
    USBCTRL_DPRAM,
    USBCTRL_REGS,
};
use crate::resets::SubsystemReset;

const CONTROL_BUFFER_SIZE: usize = 64;

pub struct UsbHostBus {
    inner: Mutex<RefCell<Inner>>,
}

impl UsbHostBus {
    pub fn new(
        ctrl_reg: USBCTRL_REGS,
        ctrl_dpram: USBCTRL_DPRAM,
        _pll: UsbClock,
        resets: &mut RESETS,
    ) -> Self {
        ctrl_reg.reset_bring_down(resets);
        ctrl_reg.reset_bring_up(resets);

        Self { inner: Mutex::new(RefCell::new(Inner { ctrl_reg, ctrl_dpram })) }
    }
}

impl HostBus for UsbHostBus {
    fn reset_controller(&mut self) {
        critical_section::with(|cs| {
            let mut inner = self.inner.borrow(cs).borrow_mut();
            unsafe {
                let raw_ctrl_reg =
                    core::slice::from_raw_parts_mut(USBCTRL_REGS::ptr() as *mut u32, 1 + 0x98 / 4);
                raw_ctrl_reg.fill(0);

                let raw_ctrl_pdram =
                    core::slice::from_raw_parts_mut(USBCTRL_DPRAM::ptr() as *mut u32, 1 + 0xfc / 4);
                raw_ctrl_pdram.fill(0);
            }
            inner.ctrl_reg.usb_muxing.modify(|_, w| {
                w.to_phy().set_bit();
                w.softcon().set_bit()
            });
            inner.ctrl_reg.usb_pwr.modify(|_, w| {
                w.vbus_detect().set_bit()
                    .vbus_detect_override_en().set_bit()
            });
            inner.ctrl_reg.sie_ctrl.modify(|_, w| {
                w.sof_en().set_bit()
                    .keep_alive_en().set_bit()
                    .pulldown_en().set_bit()
            });
            inner.ctrl_reg.main_ctrl.modify(|_, w| {
                w.host_ndevice().set_bit()
                    .controller_en().set_bit()
            });

            inner.ctrl_reg.inte.modify(|_, w| {
                w.buff_status().set_bit()
                    .host_conn_dis().set_bit()
                    .host_resume().set_bit()
                    .stall().set_bit()
                    .trans_complete().set_bit()
                    .error_rx_timeout().set_bit()
                    .error_data_seq().set_bit()
                    .error_crc().set_bit()
                    .error_bit_stuff().set_bit()
                    .error_rx_overflow().set_bit()
            });
        });
    }

    fn enable_sof(&mut self) {
        critical_section::with(|cs| {
            let inner = self.inner.borrow(cs).borrow_mut();
            inner.ctrl_reg.sie_ctrl.modify(|_, w| {
                w.sof_en().set_bit().keep_alive_en().set_bit().pulldown_en().set_bit()
            });
        });
    }

    fn sof_enabled(&self) -> bool {
        critical_section::with(|cs| {
            let inner = self.inner.borrow(cs).borrow_mut();
            let sie_ctrl = inner.ctrl_reg.sie_ctrl.read();
            sie_ctrl.sof_en().bit_is_set() && sie_ctrl.keep_alive_en().bit_is_set()
        })
    }

    fn reset_bus(&mut self) {
        critical_section::with(|cs| {
            let inner = self.inner.borrow(cs).borrow_mut();
            inner.ctrl_reg.sie_ctrl.modify(|_, w| w.reset_bus().set_bit());
        });
    }

    fn set_recipient(&mut self, dev_addr: Option<usbh::types::DeviceAddress>, endpoint: u8, transfer_type: TransferType) {
        critical_section::with(|cs| {
            let inner = self.inner.borrow(cs).borrow_mut();
            inner.ctrl_reg.addr_endp.write(|w| {
                unsafe {
                    w.address().bits(dev_addr.map(|addr| u8::from(addr)).unwrap_or(0));
                    w.endpoint().bits(endpoint)
                }
            });

            inner.ctrl_dpram.epx_control.modify(|_, w| {
                unsafe {
                    w.enable().set_bit()
                        .interrupt_per_buff().set_bit()
                        .buffer_address().bits(0x180);
                    match transfer_type {
                        TransferType::Control => w.endpoint_type().control(),
                        TransferType::Isochronous => w.endpoint_type().isochronous(),
                        TransferType::Bulk => w.endpoint_type().bulk(),
                        TransferType::Interrupt => w.endpoint_type().interrupt(),
                    }
                }
            });
        });
    }

    fn write_setup(&mut self, setup: usbh::types::SetupPacket) {
        critical_section::with(|cs| {
            let inner = self.inner.borrow(cs).borrow_mut();
            inner.ctrl_dpram.setup_packet_low.write(|w| {
                unsafe {
                    w.bmrequesttype().bits(setup.request_type);
                    w.brequest().bits(setup.request);
                    w.wvalue().bits(setup.value)
                }
            });
            inner.ctrl_dpram.setup_packet_high.write(|w| {
                unsafe {
                    w.windex().bits(setup.index);
                    w.wlength().bits(setup.length)
                }
            });
            inner.ctrl_reg.sie_ctrl.modify(|_, w| w.send_data().clear_bit().receive_data().clear_bit().send_setup().set_bit().start_trans().set_bit());
        });
    }

    fn write_data_in(&mut self, length: u16, pid: bool) {
        critical_section::with(|cs| {
            let inner = self.inner.borrow(cs).borrow_mut();
            inner.ctrl_dpram.ep_buffer_control[0].write(|w| {
                unsafe {
                    w.available_0().set_bit();
                    w.pid_0().bit(pid);
                    w.full_0().clear_bit()
                        .length_0().bits(length)
                        .last_0().set_bit()
                        .reset().set_bit()
                }
            });
            inner.ctrl_reg.sie_ctrl.modify(|_, w| w.send_data().clear_bit().send_setup().clear_bit().receive_data().set_bit().start_trans().set_bit());
        });
    }

    fn prepare_data_out(&mut self, data: &[u8]) {
        critical_section::with(|cs| {
            let mut inner = self.inner.borrow(cs).borrow_mut();
            inner.control_buffer_mut()[0..data.len()].copy_from_slice(data);
            inner.ctrl_dpram.ep_buffer_control[0].write(|w| {
                unsafe {
                    w.available_0().set_bit()
                        .pid_0().set_bit()
                        .full_0().set_bit()
                        .length_0().bits(data.len() as u16)
                        .last_0().set_bit()
                        .reset().set_bit()
                }
            });
        });
    }

    fn write_data_out_prepared(&mut self) {
        critical_section::with(|cs| {
            let inner = self.inner.borrow(cs).borrow_mut();
            inner.ctrl_reg.sie_ctrl.modify(|_, w| w.send_setup().clear_bit().receive_data().clear_bit().send_data().set_bit().start_trans().set_bit());
        });
    }

    fn poll(&mut self) -> PollResult {
        critical_section::with(|cs| {
            let mut inner = self.inner.borrow(cs).borrow_mut();
            PollResult {
                event: inner.ints_to_event(),
                poll_again_after: None,
            }
        })
    }

    fn process_received_data<F: FnOnce(&[u8]) -> T, T>(&self, f: F) -> T {
        critical_section::with(|cs| {
            let inner = self.inner.borrow(cs).borrow_mut();
            let len = inner.ctrl_dpram.ep_buffer_control[0].read().length_0().bits() as usize;
            f(&inner.control_buffer()[0..len])
        })
    }

    unsafe fn control_buffer(&self, len: usize) -> &[u8] {
        const DPRAM_BASE: *const u8 = USBCTRL_DPRAM::ptr() as *const u8;
        unsafe { core::slice::from_raw_parts(DPRAM_BASE.offset(0x180), len) }
    }

    fn create_interrupt_pipe(&mut self, device_address: DeviceAddress, endpoint_number: u8, direction: UsbDirection, size: u16, interval: u8) -> Option<(*mut u8, u8)> {
        critical_section::with(|cs| {
            let inner = self.inner.borrow(cs).borrow_mut();
            inner.ctrl_dpram.ep_control[0].write(|w| {
                unsafe {
                    w.bits(
                        (0x180 + CONTROL_BUFFER_SIZE as u32) |
                        ((TransferType::Interrupt as u8 as u32) << 26) |
                        (1 << 29) | // interrupt per buf
                        (1 << 31) | // enable
                        ((interval as u32 - 1) << 16)
                    )
                }
            });
            inner.ctrl_reg.sie_ctrl.modify(|_, w| w.sof_sync().set_bit());
            let ep_ctrl_bits = inner.ctrl_dpram.ep_control[0].read().bits();
            inner.ctrl_dpram.ep_buffer_control[2].write(|w| {
                w.last_0().set_bit();
                w.pid_0().clear_bit();
                w.full_0().clear_bit();
                w.reset().set_bit();
                unsafe { w.length_0().bits(size) }
            });
            cortex_m::asm::delay(12);
            inner.ctrl_dpram.ep_buffer_control[2].modify(|_, w| w.available_0().set_bit());
            let buf_control = inner.ctrl_dpram.ep_buffer_control[2].read().bits();
            inner.ctrl_reg.addr_endp1.write(|w| {
                unsafe {
                    w.intep_preamble().clear_bit();
                    w.address().bits(device_address.into());
                    w.endpoint().bits(endpoint_number);
                    w.intep_dir().bit(direction == UsbDirection::Out)
                }
            });
            cortex_m::asm::delay(12);
            inner.ctrl_reg.int_ep_ctrl.modify(|_, w| {
                unsafe { w.int_ep_active().bits(0x01) }
            });

            const DPRAM_BASE: *mut u8 = USBCTRL_DPRAM::ptr() as *mut u8;
            let buf_ptr = unsafe { DPRAM_BASE.offset(0x180 + CONTROL_BUFFER_SIZE as isize) };
            Some((buf_ptr, 2))
        })
    }

    fn release_interrupt_pipe(&mut self, pipe_ref: u8) {}

    fn received_len(&self) -> u16 {
        critical_section::with(|cs| {
            let inner = self.inner.borrow(cs).borrow_mut();
            inner.ctrl_dpram.ep_buffer_control[0].read().length_0().bits()
        })
    }

    fn dump_dpram(&self) {
        const DPRAM_BASE: *const u8 = USBCTRL_DPRAM::ptr() as *const u8;
        let dpram = unsafe { core::slice::from_raw_parts(DPRAM_BASE, 0x180) };
        defmt::info!("DPRAM: {}", dpram);
        let regs = unsafe { core::slice::from_raw_parts(USBCTRL_REGS::ptr() as *const u8, 156) };
        defmt::info!("REGS: {}", regs);
    }

    fn pipe_buf(&self, pipe_index: u8) -> &[u8] {
        const DPRAM_BASE: *const u8 = USBCTRL_DPRAM::ptr() as *const u8;
        unsafe { core::slice::from_raw_parts(DPRAM_BASE.offset(0x180 + CONTROL_BUFFER_SIZE as isize), 8) }
    }

    fn pipe_continue(&self, pipe_index: u8) {
        critical_section::with(|cs| {
            let inner = self.inner.borrow(cs).borrow_mut();
            let buf_control = inner.ctrl_dpram.ep_buffer_control[2].read().bits();
            //defmt::debug!("BUF CONTROL (continue): {:#X}, ep int ctr: {:#X}", buf_control, inner.ctrl_reg.int_ep_ctrl.read().bits());
            inner.ctrl_dpram.ep_buffer_control[2].modify(|r, w| {
                w.last_0().set_bit();
                w.pid_0().bit(!r.pid_0().bit());
                w.full_0().clear_bit();
                w.reset().set_bit();
                unsafe { w.length_0().bits(8) }
            });
            cortex_m::asm::delay(12);
            inner.ctrl_dpram.ep_buffer_control[2].modify(|_, w| w.available_0().set_bit());
            // inner.ctrl_dpram.ep_buffer_control[2].modify(|_, w| {
            //     w.full_0().clear_bit();
            //     w.available_0().set_bit()
            // });
        })
    }

    fn interrupt_on_sof(&mut self, enable: bool) {
        critical_section::with(|cs| {
            let inner = self.inner.borrow(cs).borrow_mut();
            inner.ctrl_reg.inte.modify(|_, w| w.host_sof().bit(enable));
        });
    }
}

unsafe impl Send for UsbHostBus {}

struct Inner {
    ctrl_reg: USBCTRL_REGS,
    ctrl_dpram: USBCTRL_DPRAM,
}

impl Inner {
    fn control_buffer(&self) -> &[u8] {
        const DPRAM_BASE: *const u8 = USBCTRL_DPRAM::ptr() as *const u8;
        unsafe { core::slice::from_raw_parts(DPRAM_BASE.offset(0x180), CONTROL_BUFFER_SIZE) }
    }

    fn control_buffer_mut(&mut self) -> &mut [u8] {
        const DPRAM_BASE: *mut u8 = USBCTRL_DPRAM::ptr() as *mut u8;
        unsafe { core::slice::from_raw_parts_mut(DPRAM_BASE.offset(0x180), CONTROL_BUFFER_SIZE) }
    }

    fn ints_to_event(&mut self) -> Option<Event> {
        let ints = self.ctrl_reg.ints.read();

        if ints.host_conn_dis().bit_is_set() {
            let event = match self.ctrl_reg.sie_status.read().speed().bits() {
                0b01 => Event::Attached(ConnectionSpeed::Low),
                0b10 => Event::Attached(ConnectionSpeed::Full),
                _ => Event::Detached,
            };
            self.ctrl_reg.sie_status.modify(|_, w| unsafe { w.speed().bits(0b11) });
            return Some(event)
        }
        if ints.host_resume().bit_is_set() {
            self.ctrl_reg.sie_status.modify(|_, w| w.resume().set_bit());
            return Some(Event::Resume);
        }
        if ints.stall().bit_is_set() {
            self.ctrl_reg.sie_status.modify(|_, w| w.stall_rec().set_bit());
            return Some(Event::Stall);
        }
        if ints.error_crc().bit_is_set() {
            self.ctrl_reg.sie_status.modify(|_, w| w.crc_error().set_bit());
            return Some(Event::Error(Error::Crc));
        }
        if ints.error_bit_stuff().bit_is_set() {
            self.ctrl_reg.sie_status.modify(|_, w| w.bit_stuff_error().set_bit());
            return Some(Event::Error(Error::BitStuffing));
        }
        if ints.error_rx_overflow().bit_is_set() {
            self.ctrl_reg.sie_status.modify(|_, w| w.rx_overflow().set_bit());
            return Some(Event::Error(Error::RxOverflow));
        }
        if ints.error_rx_timeout().bit_is_set() {
            self.ctrl_reg.sie_status.modify(|_, w| w.rx_timeout().set_bit());
            return Some(Event::Error(Error::RxTimeout));
        }
        if ints.error_data_seq().bit_is_set() {
            self.ctrl_reg.sie_status.modify(|_, w| w.data_seq_error().set_bit());
            return Some(Event::Error(Error::DataSequence));
        }
        if ints.buff_status().bit_is_set() {
            let status = self.ctrl_reg.buff_status.read().bits();
            // TODO: handle buffer updates more gracefully. Currently we always wait for TransComplete,
            //   which only works for transfers that fit into a single buffer.

            for i in 0..32 {
                if (status >> i) & 1 == 1 {
                    self.ctrl_reg.buff_status.modify(|_, w| unsafe { w.bits(1 << i) });
                    // control transfers (buffer 0) 
                    if i != 0 {
                        return Some(Event::InterruptPipe(i as u8));
                    }
                }
            }
        }
        if ints.trans_complete().bit_is_set() {
            self.ctrl_reg.sie_status.modify(|_, w| w.trans_complete().set_bit());
            return Some(Event::TransComplete);
        }
        if ints.host_sof().bit_is_set() {
            return Some(Event::Sof);
        }
        None
    }
}
