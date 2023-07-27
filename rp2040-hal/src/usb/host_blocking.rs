#[cfg(feature = "defmt")]
use defmt::Format;

use defmt::info;

use usb_device::{UsbDirection, control::{Request, RequestType, Recipient}};

use crate::clocks::UsbClock;
use crate::pac::{
    RESETS,
    USBCTRL_DPRAM,
    USBCTRL_REGS,
};
use crate::resets::SubsystemReset;

pub struct BlockingHost {
    ctrl_reg: USBCTRL_REGS,
    ctrl_dpram: USBCTRL_DPRAM,
}

impl BlockingHost {
    pub fn new(
        ctrl_reg: USBCTRL_REGS,
        ctrl_dpram: USBCTRL_DPRAM,
        _pll: UsbClock,
        resets: &mut RESETS,
    ) -> Self {
        ctrl_reg.reset_bring_down(resets);
        ctrl_reg.reset_bring_up(resets);

        Self { ctrl_reg, ctrl_dpram }
    }

    pub fn run(&mut self, delay: &mut cortex_m::delay::Delay) {
        let event = self.wait_for(|e| matches!(e, Event::ConnDis(Some(_))));
        info!("New USB device: {}", event);

        delay.delay_ms(20);

        self.ctrl_dpram.epx_control.write(|w| {
            unsafe {
                w.enable().set_bit()
                    .interrupt_per_buff().set_bit()
                    .endpoint_type().control()
                    .buffer_address().bits(0x180)
            }
        });

        let buf = self.control_in(RequestType::Standard, Recipient::Device, Request::GET_DESCRIPTOR, 1 << 8, 0, 18);
        info!("Received device descriptor from ADDR0/EDPT0: {}", buf);

        self.control_out(RequestType::Standard, Recipient::Device, Request::SET_ADDRESS, 42, 0, &[]);
        info!("Assigned address 42 to device");
        self.ctrl_reg.addr_endp.write(|w| {
            unsafe { w.address().bits(42) }
        });

        let buf = self.control_in(RequestType::Standard, Recipient::Device, Request::GET_DESCRIPTOR, 1 << 8, 0, 18);
        print_device_descriptor(buf);

        let buf = self.control_in(RequestType::Standard, Recipient::Device, Request::GET_DESCRIPTOR, 2 << 8, 0, 9);
        info!("Get config descriptor: received first 9 bytes: {}", buf);
        let total_length = ((buf[3] as u16) << 8) | (buf[2] as u16);

        let buf = self.control_in(RequestType::Standard, Recipient::Device, Request::GET_DESCRIPTOR, 2 << 8, 0, total_length);
        print_config_descriptor(buf);

        let cfg_value = buf[5] as u16;

        info!("SetConfiguration({})", cfg_value);
        self.control_out(RequestType::Standard, Recipient::Device, Request::SET_CONFIGURATION, cfg_value, 0, &[]);

        info!("Set configuration complete!");
        // self.write_setup_packet(UsbDirection::In, RequestType::Standard, Recipient::Device, Request::SET_CONFIGURATION, cfg_value, 0, 0);
        // self.ctrl_reg.sie_ctrl.write(|w| w.send_setup().set_bit().start_trans().set_bit());
        // info!("SetConfiguration({})", cfg_value);
        // self.wait_for(|e| matches!(e, Event::TransComplete));
        // self.ctrl_dpram.ep_buffer_control[0].write(|w| {
        //     w.pid_0().set_bit();
        //     w.full_0().clear_bit();
        //     w.available_0().set_bit();
        //     unsafe { w.length_0().bits(0) }
        // });
        // self.ctrl_reg.sie_ctrl.write(|w| w.receive_data().set_bit().start_trans().set_bit());
        // self.wait_for(|e| matches!(e, Event::TransComplete));

        // self.ctrl_reg.addr_endp1.write(|w| {
        //     unsafe { w.address().bits(42).endpoint().bits(1).intep_dir().clear_bit() }
        // });

        let buf = self.control_in(RequestType::Standard, Recipient::Interface, Request::GET_DESCRIPTOR, 0x22 << 8, 0, 54);
        print_hid_report(buf);

        defmt::info!("SET PROTOCOL (boot)");
        self.control_out(RequestType::Class, Recipient::Interface, 0x0b, 0, 0, &[]);

        delay.delay_us(100);

        let sequence = [
            0b001,
            0b010,
            0b100,
            0b111,
            0b000,
            0b111,
            0b000,
            0b100,
            0b010,
            0b001,
            0b111,
            0b000,
            0b111,
            0b000,
        ];

        let mut index = 0;

        loop {
            defmt::info!("SET REPORT {}", sequence[index]); 
            self.control_out(RequestType::Class, Recipient::Interface, 0x09, 2 << 8, 0, &[sequence[index]]);
            self.ctrl_reg.sie_ctrl.write(|w| w.keep_alive_en().set_bit());
            delay.delay_ms(200);
            index += 1;
            if index == sequence.len() {
                index = 0;
            }
        }
            

        // let mut state = 1;

        // loop {
            // self.control_out(RequestType::Class, Recipient::Interface, 0x09, 2 << 8, 0, &[state]);
        //     info!("toggled");
        //     delay.delay_ms(1000);
        //     state = !state;
        // }

        // self.ctrl_reg.addr_endp.write(|w| {
        //     unsafe { w.endpoint().bits(1) }
        // });
        // self.ctrl_dpram.ep_control[0].write(|w| {
        //     w.enable().set_bit();
        //     w.endpoint_type().interrupt();
        //     w.interrupt_per_buff().set_bit();
        //     unsafe { w.buffer_address().bits(0x280) }
        // });
        // self.ctrl_dpram.ep_buffer_control[1].write(|w| {
        //     w.pid_0().set_bit();
        //     w.full_0().clear_bit();
        //     w.available_0().set_bit();
        //     w.reset().set_bit();
        //     unsafe { w.length_0().bits(54) }
        // });

        // self.ctrl_reg.sie_ctrl.write(|w| w.receive_data().set_bit().start_trans().set_bit());

        // self.wait_for(|e| matches!(e, Event::TransComplete));

        // info!("INT TRANS COMPLETE");

        // self.ctrl_reg.int_ep_ctrl.write(|w| {
        //     unsafe { w.int_ep_active().bits(1) }
        // });

        // info!("Configured interrupt endpoint");

        // loop {
        //     let e = self.wait_for(|e| matches!(e, Event::BuffStatus(_)));
        //     info!("Buff status... {}", e);

        //     const DPRAM_BASE: *mut u8 = USBCTRL_DPRAM::ptr() as *mut u8;
        //     let data = unsafe { core::slice::from_raw_parts(DPRAM_BASE.offset(0x280), 54) };
        //     info!("INT DATA: {}", data);
        // }
    }

    fn wait_for<F: Fn(Event) -> bool>(&mut self, f: F) -> Event {
        loop {
            let event = self.wait_for_event();
            if f(event) {
                return event
            }

            match event {
                Event::ErrCrc | Event::ErrDataSeq | Event::ErrBitStuff | Event::ErrRxTimeout | Event::ErrRxOverflow | Event::Stall | Event::ConnDis(_) => {
                    defmt::error!("ERROR: {}", event);
                },
                _ => {}
            }
        }
    }

    fn wait_for_event(&mut self) -> Event {
        let ints = self.ctrl_reg.ints.read();

        if ints.host_conn_dis().bit_is_set() {
            let speed = match self.ctrl_reg.sie_status.read().speed().bits() {
                0b01 => Some(ConnectionSpeed::Low),
                0b10 => Some(ConnectionSpeed::Full),
                _ => None,
            };
            self.ctrl_reg.sie_status.write(|w| unsafe { w.speed().bits(0b11) });
            return Event::ConnDis(speed)
        }
        if ints.host_resume().bit_is_set() {
            self.ctrl_reg.sie_status.write(|w| w.resume().set_bit());
            return Event::Resume;
        }
        if ints.stall().bit_is_set() {
            self.ctrl_reg.sie_status.write(|w| w.stall_rec().set_bit());
            return Event::Stall;
        }
        if ints.trans_complete().bit_is_set() {
            self.ctrl_reg.sie_status.write(|w| w.trans_complete().set_bit());
            return Event::TransComplete;
        }
        if ints.error_crc().bit_is_set() {
            self.ctrl_reg.sie_status.write(|w| w.crc_error().set_bit());
            return Event::ErrCrc;
        }
        if ints.error_bit_stuff().bit_is_set() {
            self.ctrl_reg.sie_status.write(|w| w.bit_stuff_error().set_bit());
            return Event::ErrBitStuff;
        }            
        if ints.error_rx_overflow().bit_is_set() {
            self.ctrl_reg.sie_status.write(|w| w.rx_overflow().set_bit());
            return Event::ErrRxOverflow;
        }
        if ints.error_rx_timeout().bit_is_set() {
            self.ctrl_reg.sie_status.write(|w| w.rx_timeout().set_bit());
            return Event::ErrRxTimeout;
        }
        if ints.error_data_seq().bit_is_set() {
            self.ctrl_reg.sie_status.write(|w| w.data_seq_error().set_bit());
            return Event::ErrDataSeq;
        }
        if ints.host_sof().bit_is_set() {
            let count = self.ctrl_reg.sof_rd.read().count().bits();
            return Event::Sof(count);
        };
        if ints.buff_status().bit_is_set() {
            let status = self.ctrl_reg.buff_status.read().bits();
            self.ctrl_reg.buff_status.write(|w| unsafe { w.bits(0xFFFFFFFF) });
            return Event::BuffStatus(status);
        }
        if ints.bus_reset().bit_is_set() {
            self.ctrl_reg.sie_status.write(|w| w.bus_reset().set_bit());
            return Event::BusReset;
        }
        Event::None
    }

    pub fn reset(&self) {
        defmt::debug!("RESET");
        unsafe {
            let raw_ctrl_reg =
                core::slice::from_raw_parts_mut(USBCTRL_REGS::ptr() as *mut u32, 1 + 0x98 / 4);
            raw_ctrl_reg.fill(0);

            let raw_ctrl_pdram =
                core::slice::from_raw_parts_mut(USBCTRL_DPRAM::ptr() as *mut u32, 1 + 0xfc / 4);
            raw_ctrl_pdram.fill(0);
        }

        self.ctrl_reg.usb_muxing.modify(|_, w| {
            w.to_phy().set_bit();
            w.softcon().set_bit()
        });

        // Taken from hcd_rp2040.c, with comment:
        // // Force VBUS detect to always present, for now we assume vbus is always provided (without using VBUS En)
        self.ctrl_reg.usb_pwr.modify(|_, w| {
            w.vbus_detect().set_bit()
                .vbus_detect_override_en().set_bit()
        });

        self.ctrl_reg.sie_ctrl.modify(|_, w| {
            // The host needs to send keep-alive packets to the device every 1ms to keep the device from suspending. In Full Speed
            // mode this is done by sending a SOF (start of frame) packet. In Low Speed mode, an EOP (end of packet) is sent. When
            // setting up the controller, SIE_CTRL.KEEP_ALIVE_EN and SIE_CTRL.SOF_EN should be set to enable these packets.
            w.sof_en().set_bit()
                .keep_alive_en().set_bit()
                .pulldown_en().set_bit()
                .sof_sync().set_bit()
        });

        self.ctrl_reg.main_ctrl.modify(|_, w| {
            w.host_ndevice().set_bit()
                .controller_en().set_bit()
        });

        self.ctrl_reg.inte.modify(|_, w| {
            w.buff_status().set_bit()
                .bus_reset().set_bit()
                .host_conn_dis().set_bit()
                .host_resume().set_bit()
                .stall().set_bit()
                .trans_complete().set_bit()
                .error_rx_timeout().set_bit()
                .error_data_seq().set_bit()
                .host_sof().set_bit()
                .ep_stall_nak().set_bit()
                .error_crc().set_bit()
                .error_bit_stuff().set_bit()
                .error_rx_overflow().set_bit()
        });
    }

    fn control_in(&mut self, request_type: RequestType, recipient: Recipient, request: u8, value: u16, index: u16, length: u16) -> &[u8] {
        // 1. Setup packet
        self.write_setup_packet(UsbDirection::In, request_type, recipient, request, value, index, length);
        self.ctrl_reg.sie_ctrl.write(|w| w.send_setup().set_bit().start_trans().set_bit());
        self.wait_for(|e| matches!(e, Event::TransComplete));
        defmt::debug!("IN: setup complete");

        // 2. IN packet(s)
        self.ctrl_dpram.ep_buffer_control[0].write(|w| {
            w.pid_0().set_bit();
            w.full_0().clear_bit();
            w.available_0().set_bit();
            unsafe { w.length_0().bits(length) }
        });
        self.ctrl_reg.sie_ctrl.write(|w| w.receive_data().set_bit().start_trans().set_bit());
        self.wait_for(|e| matches!(e, Event::TransComplete));
        defmt::debug!("IN: data complete");

        let received_len = self.ctrl_dpram.ep_buffer_control[0].read().length_0().bits() as usize;

        // 3. Confirmation (empty OUT packet)
        self.ctrl_dpram.ep_buffer_control[0].modify(|r, w| {
            unsafe {
                w.available_0().set_bit()
                    .pid_0().set_bit()
                    .full_0().set_bit()
                    .length_0().bits(0)
                    .last_0().set_bit()
                    .reset().set_bit()
            }
        });
        self.ctrl_reg.sie_ctrl.write(|w| w.send_data().set_bit().start_trans().set_bit());

        self.wait_for(|e| matches!(e, Event::TransComplete));
        defmt::debug!("IN: confirm complete");

        self.control_buffer(received_len)
    }

    fn control_out(&mut self, request_type: RequestType, recipient: Recipient, request: u8, value: u16, index: u16, buf: &[u8]) {
        // 1. Setup packet
        self.write_setup_packet(UsbDirection::Out, request_type, recipient, request, value, index, buf.len() as u16);
        self.ctrl_reg.sie_ctrl.write(|w| w.send_setup().set_bit().start_trans().set_bit());
        self.wait_for(|e| matches!(e, Event::TransComplete));
        defmt::debug!("OUT: setup complete");
        if buf.len() > 0 {
            // 2. OUT data
            self.control_buffer_mut(buf.len()).copy_from_slice(buf);
            self.ctrl_dpram.ep_buffer_control[0].modify(|r, w| {
                unsafe {
                    w.available_0().set_bit()
                        .pid_0().set_bit()
                        .full_0().set_bit()
                        .length_0().bits(buf.len() as u16)
                        .last_0().set_bit()
                        .reset().set_bit()
                }
            });
            self.ctrl_reg.sie_ctrl.write(|w| w.send_data().set_bit().start_trans().set_bit());
            self.wait_for(|e| matches!(e, Event::TransComplete));
            defmt::debug!("OUT: send complete");
        }
        // 3. IN data (empty packet) for confirmation
        self.ctrl_dpram.ep_buffer_control[0].modify(|r, w| {
            unsafe {
                w.available_0().set_bit()
                    .pid_0().set_bit()
                    .full_0().clear_bit()
                    .length_0().bits(0)
                    .last_0().set_bit()
                    .reset().set_bit()
            }
        });
        self.ctrl_reg.sie_ctrl.write(|w| w.receive_data().set_bit().start_trans().set_bit());
        self.wait_for(|e| matches!(e, Event::TransComplete));
        defmt::debug!("OUT: confirm complete");
    }

    pub fn write_setup_packet(
        &self,
        direction: UsbDirection,
        request_type: RequestType,
        recipient: Recipient,
        request: u8,
        value: u16,
        index: u16,
        length: u16
    ) {
        self.ctrl_dpram.setup_packet_low.write(|w| {
            unsafe {
                w.bmrequesttype().bits(
                    (direction as u8) |
                    ((request_type as u8) << 5) |
                    (recipient as u8)
                );
                w.brequest().bits(request);
                w.wvalue().bits(value)
            }
        });

        self.ctrl_dpram.setup_packet_high.write(|w| {
            unsafe {
                w.windex().bits(index);
                w.wlength().bits(length)
            }
        });

        self.dump_setup_packet();
    }

    fn dump_setup_packet(&self) {
        let low = self.ctrl_dpram.setup_packet_low.read().bits();
        let high = self.ctrl_dpram.setup_packet_high.read().bits();
        defmt::debug!("SETUP PACKET: {:#X} {:#X} {:#X} {:#X} {:#X} {:#X} {:#X} {:#X}",
                      low & 0x000000FF,
                      (low & 0x0000FF00) >> 8,
                      (low & 0x00FF0000) >> 16,
                      (low & 0xFF000000) >> 24,
                      high & 0x000000FF,
                      (high & 0x0000FF00) >> 8,
                      (high & 0x00FF0000) >> 16,
                      (high & 0xFF000000) >> 24,
        );
    }

    pub fn control_buffer(&self, len: usize) -> &[u8] {
        const DPRAM_BASE: *mut u8 = USBCTRL_DPRAM::ptr() as *mut u8;
        unsafe { core::slice::from_raw_parts(DPRAM_BASE.offset(0x180), len) }
    }

    fn control_buffer_mut(&self, len: usize) -> &mut [u8] {
        const DPRAM_BASE: *mut u8 = USBCTRL_DPRAM::ptr() as *mut u8;
        unsafe { core::slice::from_raw_parts_mut(DPRAM_BASE.offset(0x180), len) }
    }
}

#[cfg_attr(feature = "defmt", derive(Format))]
#[derive(Copy, Clone)]
pub enum Event {
    None,
    ConnDis(Option<ConnectionSpeed>),
    Stall,
    Resume,
    ErrCrc,
    ErrBitStuff,
    ErrRxOverflow,
    ErrRxTimeout,
    ErrDataSeq,
    Sof(u16),
    TransComplete,
    BuffStatus(u32),
    BusReset,
}

#[derive(Copy, Clone)]
pub enum ConnectionSpeed {
    Low,
    Full,
}

#[cfg(feature = "defmt")]
impl defmt::Format for ConnectionSpeed {
    fn format(&self, fmt: defmt::Formatter) {
        defmt::write!(fmt, "{}", match self {
            ConnectionSpeed::Low => "low-speed",
            ConnectionSpeed::Full => "full-speed",
        })
    }
}

fn print_device_descriptor(bytes: &[u8]) -> usize {
    let size = bytes[0];
    if size != bytes.len() as u8 {
        defmt::error!("Invalid device descriptor? bLength={} does not match buffer size {}", size, bytes.len());
    } else if size != 18 {
        defmt::info!("Device descriptor too short ({} bytes), not printing for now.", size);
    } else {
        defmt::info!("Device descriptor:\n - Desc. Type:\t\t{}\n - USB Version:\t\t{}{}.{}.{}\n - Device Class:\t{}\n - vendor/product:\t{:#X}:{:#X}\n - #configurations:\t{}",
                     bytes[1], (bytes[3] & 0xF0) >> 4, bytes[3] & 0x0F, (bytes[2] & 0xF0) >> 4, bytes[2] & 0x0F, bytes[4],
                     ((bytes[9] as u16) << 8) | (bytes[8] as u16),
                     ((bytes[11] as u16) << 8) | (bytes[10] as u16),
                     bytes[17]
        );
    }
    bytes.len()
}

fn print_config_descriptor(bytes: &[u8]) -> usize {
    defmt::info!("Config descriptor:\n - Num Interfaces:\t{}\n - Config Value:\t{}\n - Self powered:\t{}\n - Remote wakeup:\t{}\n - Max power (mA):\t{}\n",
                 bytes[4], bytes[5], (bytes[7] >> 6) & 1 == 1, (bytes[7] >> 5) & 1 == 1, bytes[8] * 2
    );
    let mut consumed = 9; // size of config descriptor itself

    loop {
        if bytes.len() > consumed {
            consumed += print_descriptor(&bytes[consumed..]);
        } else {
            break
        }
    }

    consumed
}

fn print_interface_descriptor(bytes: &[u8]) -> usize {
    defmt::info!(
        "Interface descriptor:\n - Interface num:\t{}\n - Alt setting: \t{}\n - Num endpoints:\t{}\n - Interface class:\t{}\n - Iface subclass:\t{}\n - Iface protocol:\t{}\n",
        bytes[2], bytes[3], bytes[4], bytes[5], bytes[6], bytes[7]
    );
    9
}

fn print_endpoint_descriptor(bytes: &[u8]) -> usize {
    defmt::info!(
        "Endpoint descriptor:\n - EP Address:\t{} ({})\n - Transfer type:\t{}\n - Sync type:\t{}\n - Usage type:\t{}\n - Max packet:\t{}\n - Poll interval:\t{}\n",
        bytes[2] & 0b111,
        if (bytes[2] >> 7) & 1 == 1 { "IN" } else { "OUT" },
        match bytes[3] & 0b11 {
            0 => "control",
            1 => "isochronous",
            2 => "bulk",
            3 => "interrupt",
            _ => unreachable!(),
        },
        match (bytes[3] >> 2) & 0b11 {
            0 => "none",
            1 => "asynchronous",
            2 => "adaptive",
            3 => "synchronous",
            _ => unreachable!(),
        },
        match (bytes[3] >> 4) & 0b11 {
            0 => "data",
            1 => "feedback",
            2 => "explicit feedback",
            3 => "(reserved)",
            _ => unreachable!(),
        },
        bytes[4] as u16 | ((bytes[5] as u16) << 8),
        bytes[6]
    );
    7
}

fn print_hid_kbd_descriptor(bytes: &[u8]) -> usize {
    defmt::info!(
        "HID (Keyboard) Descriptor:\n - HID version:\t{}{}.{}.{}\n - Country code:\t{}\n - Report type:\t{:#X}\n - Report length:\t{}\n",
        (bytes[3] & 0xF0) >> 4, bytes[3] & 0x0F, (bytes[2] & 0xF0) >> 4, bytes[2] & 0x0F,
        bytes[4], bytes[6], bytes[7]
    );
    9
}

fn print_descriptor(bytes: &[u8]) -> usize {
    match bytes[1] {
        1 => print_device_descriptor(bytes),
        2 => print_config_descriptor(bytes),
        4 => print_interface_descriptor(bytes),
        5 => print_endpoint_descriptor(bytes),
        0x21 => print_hid_kbd_descriptor(bytes),
        n => {
            defmt::error!("Not implemented: {} ({} bytes)", n, bytes[0]);
            bytes[0] as usize
        }
    }
}

fn print_hid_report(bytes: &[u8]) {
    let mut consumed = 0;
    loop {
        consumed += print_hid_item(&bytes[consumed..]);
        if consumed == bytes.len() {
            return
        }
    }
}

fn print_hid_item(bytes: &[u8]) -> usize {
    if bytes[0] == 0b11111110 {
        // long item
        let size = bytes[1];
        let tag = bytes[2];
        let data = &bytes[3..(3 + size as usize)];
        defmt::info!("HID ITEM (long): size={} tag={} data={}", size, tag, data);
        3 + size as usize
    } else {
        // short item
        let size = bytes[0] & 0b11;

        let type_ = match (bytes[0] >> 2) & 0b11 {
            0 => "main",
            1 => "global",
            2 => "local",
            3 => "reserved",
            _ => unreachable!(),
        };

        let tag = (bytes[0] >> 4) & 0b1111;

        defmt::info!("HID ITEM: size={}, type={}, tag={}, data={}", size, type_, tag, &bytes[1..(1 + size) as usize]);

        match (bytes[0] >> 2) & 0b11 {
            0 => { // MAIN
                if size > 0 {
                    match bytes[1] | 0b11111100 {
                        0b10000000 => defmt::info!("  INPUT, size {}", bytes[1] | 0b11),
                        0b10010000 => defmt::info!("  OUTPUT, size {}", bytes[1] | 0b11),
                        0b10110000 => defmt::info!("  FEATURE, size {}", bytes[1] | 0b11),
                        0b10100000 => defmt::info!("  COLLECTION, size {}", bytes[1] | 0b11),
                        0b11000000 => defmt::info!("  END COLLECTION, size {}", bytes[1] | 0b11),
                        _ => defmt::error!("Not sure how to interpret {:#X}", bytes[1]),
                    }
                }
            },
            1 => { // GLOBAL
            },
            2 => { // LOCAL
            },
            3 => { /* reserved */ }
            _ => unreachable!(),
        }

        1 + size as usize
    }
}
