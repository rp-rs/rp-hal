//! Universal Serial Bus (USB)
// See [Chapter 4 Section 1](https://datasheets.raspberrypi.org/rp2040/rp2040_datasheet.pdf) for more details
//! ## Usage
//!
//! Initialize the Usb Bus forcing the VBUS detection.
//! ```no_run
//! use rp2040_hal::{clocks::init_clocks_and_plls, pac, Sio, usb::UsbBus, watchdog::Watchdog};
//! use usb_device::class_prelude::UsbBusAllocator;
//!
//! const XOSC_CRYSTAL_FREQ: u32 = 12_000_000; // Typically found in BSP crates
//!
//! let mut pac = pac::Peripherals::take().unwrap();
//! let sio = Sio::new(pac.SIO);
//! let mut watchdog = Watchdog::new(pac.WATCHDOG);
//! let mut clocks = init_clocks_and_plls(
//!     XOSC_CRYSTAL_FREQ,
//!     pac.XOSC,
//!     pac.CLOCKS,
//!     pac.PLL_SYS,
//!     pac.PLL_USB,
//!     &mut pac.RESETS,
//!     &mut watchdog
//! ).ok().unwrap();
//!
//! let usb_bus = UsbBusAllocator::new(UsbBus::new(
//!         pac.USBCTRL_REGS,
//!         pac.USBCTRL_DPRAM,
//!         clocks.usb_clock,
//!         true,
//!         &mut pac.RESETS,
//!     ));
//! // Use the usb_bus as usual.
//! ```
//!
//! See [pico_usb_serial.rs](https://github.com/rp-rs/rp-hal/tree/main/boards/pico/examples/pico_usb_serial.rs) for more complete examples
//!
//!
//! ## Enumeration issue with small EP0 max packet size
//!
//! During enumeration Windows hosts send a `StatusOut` after the `DataIn` packet of the first
//! `Get Descriptor` resquest even if the `DataIn` isn't completed (typically when the `max_packet_size_ep0`
//! is less than 18bytes). The next request request is a `Set Address` that expect a `StatusIn`.
//!
//! The issue is that by the time the previous `DataIn` packet is acknoledged and the `StatusOut`
//! followed by `Setup` are received, the usb stack may have already prepared the next `DataIn` payload
//! in the EP0 IN mailbox resulting in the payload being transmitted to the host instead of the
//! `StatusIn` for the `Set Address` request as expected by the host.
//!
//! To avoid that issue, the EP0 In mailbox should be invalidated between the `Setup` packet and the
//! next `StatusIn` initiated by the host. The workaround implemented clears the available bit of the
//! EP0 In endpoint's buffer to stop the device from sending the data instead of the status packet.
//! This workaround has the caveat that the poll function must be called between those two which
//! are only separated by a few microseconds.
//!
//! If the required timing cannot be met, using an maximum packet size of the endpoint 0 above 18bytes
//! (e.g. `.max_packet_size_ep0(64)`) should avoid that issue.

use core::cell::RefCell;

use crate::clocks::UsbClock;
use crate::pac::RESETS;
use crate::pac::USBCTRL_DPRAM;
use crate::pac::USBCTRL_REGS;
use crate::resets::SubsystemReset;

use cortex_m::interrupt::{self, Mutex};

use usb_device::{
    bus::{PollResult, UsbBus as UsbBusTrait},
    endpoint::{EndpointAddress, EndpointType},
    Result as UsbResult, UsbDirection, UsbError,
};

fn ep_addr_to_ep_buf_ctrl_idx(ep_addr: EndpointAddress) -> usize {
    ep_addr.index() * 2 + (if ep_addr.is_in() { 0 } else { 1 })
}
#[derive(Debug)]
struct Endpoint {
    ep_type: EndpointType,
    max_packet_size: u16,
    buffer_offset: u16,
}
impl Endpoint {
    unsafe fn get_buf_parts(&self) -> (*mut u8, usize) {
        const DPRAM_BASE: *mut u8 = USBCTRL_DPRAM::ptr() as *mut u8;
        if self.ep_type == EndpointType::Control {
            (DPRAM_BASE.offset(0x100), self.max_packet_size as usize)
        } else {
            (
                DPRAM_BASE.offset(0x180 + (self.buffer_offset * 64) as isize),
                self.max_packet_size as usize,
            )
        }
    }

    fn get_buf(&self) -> &'static [u8] {
        unsafe {
            let (base, len) = self.get_buf_parts();
            core::slice::from_raw_parts(base as *const _, len)
        }
    }
    fn get_buf_mut(&self) -> &'static mut [u8] {
        unsafe {
            let (base, len) = self.get_buf_parts();
            core::slice::from_raw_parts_mut(base, len)
        }
    }
}

struct Inner {
    ctrl_reg: USBCTRL_REGS,
    ctrl_dpram: USBCTRL_DPRAM,
    in_endpoints: [Option<Endpoint>; 16],
    out_endpoints: [Option<Endpoint>; 16],
    next_offset: u16,
    read_setup: bool,
}
impl Inner {
    fn new(ctrl_reg: USBCTRL_REGS, ctrl_dpram: USBCTRL_DPRAM) -> Self {
        Self {
            ctrl_reg,
            ctrl_dpram,
            in_endpoints: Default::default(),
            out_endpoints: Default::default(),
            next_offset: 0,
            read_setup: false,
        }
    }

    fn ep_allocate(
        &mut self,
        ep_addr: Option<EndpointAddress>,
        ep_dir: UsbDirection,
        ep_type: EndpointType,
        max_packet_size: u16,
    ) -> UsbResult<EndpointAddress> {
        let ep_addr = ep_addr
            .or_else(|| {
                let eps = if ep_dir == UsbDirection::In {
                    self.in_endpoints.iter()
                } else {
                    self.out_endpoints.iter()
                };
                // find free end point
                let mut iter = eps.enumerate();
                // reserve ep0 for the control endpoint
                if ep_type != EndpointType::Control {
                    iter.next();
                }
                iter.find(|(_, ep)| ep.is_none())
                    .map(|(index, _)| EndpointAddress::from_parts(index, ep_dir))
            })
            .ok_or(UsbError::EndpointOverflow)?;

        let is_ep0 = ep_addr.index() == 0;
        let is_ctrl_ep = ep_type == EndpointType::Control;
        if !(is_ep0 ^ !is_ctrl_ep) {
            return Err(UsbError::Unsupported);
        }

        let eps = if ep_addr.is_in() {
            &mut self.in_endpoints
        } else {
            &mut self.out_endpoints
        };
        let maybe_ep = eps
            .get_mut(ep_addr.index())
            .ok_or(UsbError::EndpointOverflow)?;
        if maybe_ep.is_some() {
            return Err(UsbError::InvalidEndpoint);
        }

        // Validate buffer size. From datasheet (4.1.2.5):
        // Data Buffers are typically 64 bytes long as this is the max normal packet size for most FS packets.
        // For Isochronous endpoints a maximum buffer size of 1023 bytes is supported.
        // For other packet types the maximum size is 64 bytes per buffer.
        if (ep_type != EndpointType::Isochronous && max_packet_size > 64) || max_packet_size > 1023
        {
            return Err(UsbError::Unsupported);
        }

        if ep_addr.index() == 0 {
            *maybe_ep = Some(Endpoint {
                ep_type,
                max_packet_size,
                buffer_offset: 0, // not used on CTRL ep
            });
        } else {
            // size in 64bytes units.
            // NOTE: the compiler is smart enough to recognize /64 as a 6bit right shift so let's
            // keep the division here for the sake of clarity
            let aligned_sized = (max_packet_size + 63) / 64;
            if (self.next_offset + aligned_sized) > (4096 / 64) {
                return Err(UsbError::EndpointMemoryOverflow);
            }

            let buffer_offset = self.next_offset;
            self.next_offset += aligned_sized;

            *maybe_ep = Some(Endpoint {
                ep_type,
                max_packet_size,
                buffer_offset,
            });
        }
        Ok(ep_addr)
    }

    fn ep_reset_all(&mut self) {
        self.ctrl_reg
            .sie_ctrl
            .modify(|_, w| w.ep0_int_1buf().set_bit());
        // expect ctrl ep to receive on DATA first
        self.ctrl_dpram.ep_buffer_control[0].write(|w| w.pid_0().set_bit());
        self.ctrl_dpram.ep_buffer_control[1].write(|w| {
            w.available_0().set_bit();
            w.pid_0().set_bit()
        });

        for (index, ep) in itertools::interleave(
            self.in_endpoints.iter().skip(1),  // skip control endpoint
            self.out_endpoints.iter().skip(1), // skip control endpoint
        )
        .enumerate()
        .filter_map(|(i, ep)| ep.as_ref().map(|ep| (i, ep)))
        {
            use pac::usbctrl_dpram::ep_control::ENDPOINT_TYPE_A;
            let ep_type = match ep.ep_type {
                EndpointType::Bulk => ENDPOINT_TYPE_A::BULK,
                EndpointType::Isochronous => ENDPOINT_TYPE_A::ISOCHRONOUS,
                EndpointType::Control => ENDPOINT_TYPE_A::CONTROL,
                EndpointType::Interrupt => ENDPOINT_TYPE_A::INTERRUPT,
            };
            // configure
            // ep 0 in&out are not part of index (skipped before enumeration)
            self.ctrl_dpram.ep_control[index].modify(|_, w| unsafe {
                w.endpoint_type().variant(ep_type);
                w.interrupt_per_buff().set_bit();
                w.enable().set_bit();
                w.buffer_address().bits(0x180 + (ep.buffer_offset << 6))
            });
            // reset OUT ep and prepare IN ep to accept data
            let buf_control = &self.ctrl_dpram.ep_buffer_control[index + 2];
            if (index & 1) == 0 {
                // first write occur on DATA0 so prepare the pid bit to be flipped
                buf_control.write(|w| w.pid_0().set_bit());
            } else {
                buf_control.write(|w| unsafe {
                    w.available_0().set_bit();
                    w.pid_0().clear_bit();
                    w.length_0().bits(ep.max_packet_size)
                });
            }
        }
    }

    fn ep_write(&mut self, ep_addr: EndpointAddress, buf: &[u8]) -> UsbResult<usize> {
        let index = ep_addr.index();
        let ep = self
            .in_endpoints
            .get_mut(index)
            .and_then(Option::as_mut)
            .ok_or(UsbError::InvalidEndpoint)?;

        let buf_control = &self.ctrl_dpram.ep_buffer_control[index * 2];
        if buf_control.read().available_0().bit_is_set() {
            return Err(UsbError::WouldBlock);
        }

        let ep_buf = ep.get_buf_mut();
        if ep_buf.len() < buf.len() {
            return Err(UsbError::BufferOverflow);
        }
        ep_buf[..buf.len()].copy_from_slice(buf);

        buf_control.modify(|r, w| unsafe {
            w.available_0().set_bit();
            w.length_0().bits(buf.len() as u16);
            w.full_0().set_bit();
            w.pid_0().bit(!r.pid_0().bit())
        });

        Ok(buf.len())
    }

    fn ep_read(&mut self, ep_addr: EndpointAddress, buf: &mut [u8]) -> UsbResult<usize> {
        let index = ep_addr.index();
        let ep = self
            .out_endpoints
            .get_mut(index)
            .and_then(Option::as_mut)
            .ok_or(UsbError::InvalidEndpoint)?;

        let buf_control = &self.ctrl_dpram.ep_buffer_control[index * 2 + 1];
        let buf_control_val = buf_control.read();

        let process_setup = index == 0 && self.read_setup;
        let (ep_buf, len) = if process_setup {
            // assume we want to read the setup request

            // Next packet will be on DATA1 so clear pid_0 so it gets flipped by next buf config
            self.ctrl_dpram.ep_buffer_control[0].modify(|_, w| w.pid_0().clear_bit());
            // the OUT packet will be either data or a status zlp
            // clear setup request flag
            self.ctrl_reg.sie_status.write(|w| w.setup_rec().set_bit());
            (
                unsafe { core::slice::from_raw_parts(USBCTRL_DPRAM::ptr() as *const u8, 8) },
                8,
            )
        } else {
            if buf_control_val.full_0().bit_is_clear() {
                return Err(UsbError::WouldBlock);
            }
            let len: usize = buf_control_val.length_0().bits().into();
            (ep.get_buf(), len)
        };

        if len > buf.len() {
            return Err(UsbError::BufferOverflow);
        }

        buf[..len].copy_from_slice(&ep_buf[..len]);

        if process_setup {
            self.read_setup = false;

            // clear any out standing out flag e.g. in case a zlp got discarded
            self.ctrl_reg.buff_status.write(|w| unsafe { w.bits(2) });

            let data_length = u16::from(buf[6]) | (u16::from(buf[7]) << 8);
            let is_in_request = (buf[0] & 0x80) == 0x80;
            let expect_data_or_zlp = is_in_request || data_length != 0;
            buf_control.modify(|_, w| unsafe {
                // enable if and only if a dataphase is expected.
                w.available_0().bit(expect_data_or_zlp);
                w.length_0().bits(ep.max_packet_size);
                w.full_0().clear_bit();
                w.pid_0().set_bit()
            });
        } else {
            buf_control.modify(|r, w| unsafe {
                w.available_0().set_bit();
                w.length_0().bits(ep.max_packet_size);
                w.full_0().clear_bit();
                w.pid_0().bit(!r.pid_0().bit())
            });
            // Clear OUT flag once it is read.
            self.ctrl_reg
                .buff_status
                .write(|w| unsafe { w.bits(1 << (index * 2 + 1)) });
        }

        Ok(len)
    }
}

/// Usb bus
pub struct UsbBus {
    inner: Mutex<RefCell<Inner>>,
}

impl UsbBus {
    /// Create new usb bus struct and bring up usb as device.
    pub fn new(
        ctrl_reg: USBCTRL_REGS,
        ctrl_dpram: USBCTRL_DPRAM,
        _pll: UsbClock,
        force_vbus_detect_bit: bool,
        resets: &mut RESETS,
    ) -> Self {
        ctrl_reg.reset_bring_down(resets);
        ctrl_reg.reset_bring_up(resets);

        unsafe {
            let raw_ctrl_reg =
                core::slice::from_raw_parts_mut(USBCTRL_REGS::ptr() as *mut u32, 1 + 0x98 / 4);
            raw_ctrl_reg.fill(0);

            let raw_ctrl_pdram =
                core::slice::from_raw_parts_mut(USBCTRL_DPRAM::ptr() as *mut u32, 1 + 0xfc / 4);
            raw_ctrl_pdram.fill(0);
        }

        ctrl_reg.usb_muxing.modify(|_, w| {
            w.to_phy().set_bit();
            w.softcon().set_bit()
        });

        if force_vbus_detect_bit {
            ctrl_reg.usb_pwr.modify(|_, w| {
                w.vbus_detect().set_bit();
                w.vbus_detect_override_en().set_bit()
            });
        }
        ctrl_reg.main_ctrl.modify(|_, w| {
            w.sim_timing().clear_bit();
            w.host_ndevice().clear_bit();
            w.controller_en().set_bit()
        });

        Self {
            inner: Mutex::new(RefCell::new(Inner::new(ctrl_reg, ctrl_dpram))),
        }
    }
}

impl UsbBusTrait for UsbBus {
    fn alloc_ep(
        &mut self,
        ep_dir: UsbDirection,
        ep_addr: Option<EndpointAddress>,
        ep_type: EndpointType,
        max_packet_size: u16,
        _interval: u8,
    ) -> UsbResult<EndpointAddress> {
        interrupt::free(|cs| {
            let mut inner = self.inner.borrow(cs).borrow_mut();

            inner.ep_allocate(ep_addr, ep_dir, ep_type, max_packet_size)
        })
    }

    fn enable(&mut self) {
        interrupt::free(|cs| {
            let inner = self.inner.borrow(cs).borrow_mut();
            // at this stage ep's are expected to be in their reset state
            // TODO: is it worth having a debug_assert for that here?

            // Enable interrupt generation when a buffer is done, when the bus is reset,
            // and when a setup packet is received
            // this should be sufficient for device mode, will need more for host.
            inner.ctrl_reg.inte.modify(|_, w| {
                w.buff_status()
                    .set_bit()
                    .bus_reset()
                    .set_bit()
                    .setup_req()
                    .set_bit()
            });

            // enable pull up to let the host know we exist.
            inner
                .ctrl_reg
                .sie_ctrl
                .modify(|_, w| w.pullup_en().set_bit());
        })
    }
    fn reset(&self) {
        interrupt::free(|cs| {
            let mut inner = self.inner.borrow(cs).borrow_mut();

            // clear reset flag
            inner.ctrl_reg.sie_status.write(|w| w.bus_reset().set_bit());
            inner
                .ctrl_reg
                .buff_status
                .write(|w| unsafe { w.bits(0xFFFF_FFFF) });

            // reset all endpoints
            inner.ep_reset_all();

            // Reset address register
            inner.ctrl_reg.addr_endp.reset();
            // TODO: RP2040-E5: work around implementation
            // TODO: reset all endpoints & buffer statuses
        })
    }
    fn set_device_address(&self, addr: u8) {
        interrupt::free(|cs| {
            let inner = self.inner.borrow(cs).borrow_mut();
            inner
                .ctrl_reg
                .addr_endp
                .modify(|_, w| unsafe { w.address().bits(addr & 0x3F) });
            // reset ep0
            inner.ctrl_dpram.ep_buffer_control[0].modify(|_, w| w.pid_0().set_bit());
            inner.ctrl_dpram.ep_buffer_control[1].modify(|_, w| w.pid_0().set_bit());
        })
    }
    fn write(&self, ep_addr: EndpointAddress, buf: &[u8]) -> UsbResult<usize> {
        interrupt::free(|cs| {
            let mut inner = self.inner.borrow(cs).borrow_mut();
            inner.ep_write(ep_addr, buf)
        })
    }
    fn read(&self, ep_addr: EndpointAddress, buf: &mut [u8]) -> UsbResult<usize> {
        interrupt::free(|cs| {
            let mut inner = self.inner.borrow(cs).borrow_mut();
            inner.ep_read(ep_addr, buf)
        })
    }
    fn set_stalled(&self, ep_addr: EndpointAddress, stalled: bool) {
        interrupt::free(|cs| {
            let inner = self.inner.borrow(cs).borrow_mut();

            if ep_addr.index() == 0 {
                inner.ctrl_reg.ep_stall_arm.modify(|_, w| {
                    if ep_addr.is_in() {
                        w.ep0_in().bit(stalled)
                    } else {
                        w.ep0_out().bit(stalled)
                    }
                });
            }

            let index = ep_addr_to_ep_buf_ctrl_idx(ep_addr);
            inner.ctrl_dpram.ep_buffer_control[index].modify(|_, w| w.stall().bit(stalled));
        })
    }
    fn is_stalled(&self, ep_addr: EndpointAddress) -> bool {
        interrupt::free(|cs| {
            let inner = self.inner.borrow(cs).borrow_mut();
            let index = ep_addr_to_ep_buf_ctrl_idx(ep_addr);
            inner.ctrl_dpram.ep_buffer_control[index]
                .read()
                .stall()
                .bit_is_set()
        })
    }
    fn suspend(&self) {
        todo!()
    }
    fn resume(&self) {
        todo!()
    }
    fn poll(&self) -> PollResult {
        interrupt::free(|cs| {
            let mut inner = self.inner.borrow(cs).borrow_mut();
            // TODO: check for suspend request
            // TODO: check for resume request

            // check for bus reset
            let sie_status = inner.ctrl_reg.sie_status.read();
            if sie_status.bus_reset().bit_is_set() {
                return PollResult::Reset;
            }

            let (mut ep_out, mut ep_in_complete, mut ep_setup): (u16, u16, u16) = (0, 0, 0);

            let buff_status = inner.ctrl_reg.buff_status.read().bits();
            if buff_status != 0 {
                // IN Complete shall only be reported once.
                inner
                    .ctrl_reg
                    .buff_status
                    .write(|w| unsafe { w.bits(0x5555_5555) });

                for i in 0..32u32 {
                    let mask = 1 << i;
                    if (buff_status & mask) == mask {
                        if (i & 1) == 0 {
                            ep_in_complete |= 1 << (i / 2);
                        } else {
                            ep_out |= 1 << (i / 2);
                        }
                    }
                }
            }
            // check for setup request
            // Only report setup if OUT has been cleared.
            if sie_status.setup_rec().bit_is_set() {
                // Small max_packet_size_ep0 Work-Around
                inner.ctrl_dpram.ep_buffer_control[0].modify(|_, w| w.available_0().clear_bit());

                ep_setup |= 1;
                inner.read_setup = true;
            }

            if let (0, 0, 0) = (ep_out, ep_in_complete, ep_setup) {
                return PollResult::None;
            }
            PollResult::Data {
                ep_out,
                ep_in_complete,
                ep_setup,
            }
        })
    }

    const QUIRK_SET_ADDRESS_BEFORE_STATUS: bool = false;
}
