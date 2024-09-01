//! Functions for rebooting the chip using the ROM.

/// Types of reboot we support
#[derive(Debug, Clone, PartialEq, Eq)]
pub enum RebootKind {
    /// A normal reboot
    Normal,
    /// Boot like BOOTSEL is held down
    BootSel {
        /// Disable the picoboot interface
        picoboot_disabled: bool,
        /// Disable the Mass Storage Device interface
        msd_disabled: bool,
    },
    /// Boot into RAM
    Ram {
        /// The start of the RAM area to boot from
        start_addr: *const u32,
        /// The size in bytes of that area
        size: usize,
    },
    /// Reboot but prefer the flash partition you just wrote
    FlashUpdate {
        /// The start of the flash area you want to boot from
        start_addr: *const u32,
    },
    /// Reboot into the given Program Counter and Stack Pointer
    PcSp {
        /// The new program counter
        pc: fn() -> !,
        /// The new stack pointer
        sp: *const u32,
    },
}

/// Which architecture should we reboot into
#[derive(Debug, Copy, Clone, PartialEq, Eq)]
pub enum RebootArch {
    /// No architecture preference
    Normal,
    /// Prefer to boot into ARM mode
    Arm,
    /// Prefer to boot into RISC-V mode
    Riscv,
}

/// Reboot the chip
pub fn reboot(kind: RebootKind, arch: RebootArch) -> ! {
    let options = match arch {
        RebootArch::Normal => 0x0000,
        RebootArch::Arm => 0x0010,
        RebootArch::Riscv => 0x0020,
    };

    match kind {
        RebootKind::Normal => {
            #[allow(clippy::identity_op)]
            crate::rom_data::reboot(0x0000 | options, 500, 0, 0);
        }
        RebootKind::BootSel {
            picoboot_disabled,
            msd_disabled,
        } => {
            let mut flags = 0;
            if picoboot_disabled {
                flags |= 2;
            }
            if msd_disabled {
                flags |= 1;
            }
            crate::rom_data::reboot(0x0002 | options, 500, 0, flags);
        }
        RebootKind::Ram { start_addr, size } => {
            crate::rom_data::reboot(0x0003 | options, 500, start_addr as u32, size as u32);
        }
        RebootKind::FlashUpdate { start_addr } => {
            crate::rom_data::reboot(0x0004 | options, 500, start_addr as u32, 0);
        }
        RebootKind::PcSp { pc, sp } => {
            crate::rom_data::reboot(0x000d | options, 500, pc as usize as u32, sp as u32);
        }
    }
    // Wait for the reboot (might take a few ms - it's asynchronous)
    loop {
        core::hint::spin_loop();
    }
}
