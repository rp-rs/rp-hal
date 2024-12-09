//! Hazard3 Interrupt Controller (Xh3irq) Driver
//!
//! > Xh3irq controls up to 512 external interrupts, with up to 16 levels of
//! > pre-emption. It is architected as a layer on top of the standard mip.meip
//! > external interrupt line, and all standard RISC-V interrupt behaviour still
//! > applies.
//!
//! See [Section 3.8.6.1](https://rptl.io/rp2350-datasheet#extension-xh3irq-section) for more details

/// The Machine External Interrupt Enable Array
///
/// The array contains a read-write bit for each external interrupt request: a
/// `1` bit indicates that interrupt is currently enabled. At reset, all
/// external interrupts are disabled.
///
/// If enabled, an external interrupt can cause assertion of the standard RISC-V
/// machine external interrupt pending flag (`mip.meip`), and therefore cause
/// the processor to enter the external interrupt vector. See `meipa`.
///
/// There are up to 512 external interrupts. The upper half of this register
/// contains a 16-bit window into the full 512-bit vector. The window is indexed
/// by the 5 LSBs of the write data.
pub const RVCSR_MEIEA_OFFSET: u32 = 0xbe0;

/// Machine External Interrupt Pending Array
///
/// Contains a read-only bit for each external interrupt request. Similarly to
/// `meiea`, this register is a window into an array of up to 512 external
/// interrupt flags. The status appears in the upper 16 bits of the value read
/// from `meipa`, and the lower 5 bits of the value _written_ by the same CSR
/// instruction (or 0 if no write takes place) select a 16-bit window of the
/// full interrupt pending array.
///
/// A `1` bit indicates that interrupt is currently asserted. IRQs are assumed
/// to be level-sensitive, and the relevant `meipa` bit is cleared by servicing
/// the requestor so that it deasserts its interrupt request.
///
/// When any interrupt of sufficient priority is both set in `meipa` and enabled
/// in `meiea`, the standard RISC-V external interrupt pending bit `mip.meip` is
/// asserted. In other words, `meipa` is filtered by `meiea` to generate the
/// standard `mip.meip` flag.
pub const RVCSR_MEIPA_OFFSET: u32 = 0xbe1;

/// Machine External Interrupt Force Array
//
/// Contains a read-write bit for every interrupt request. Writing a 1 to a bit
/// in the interrupt force array causes the corresponding bit to become pending
/// in `meipa`. Software can use this feature to manually trigger a particular
/// interrupt.
///
/// There are no restrictions on using `meifa` inside of an interrupt. The more
/// useful case here is to schedule some lower- priority handler from within a
/// high-priority interrupt, so that it will execute before the core returns to
/// the foreground code. Implementers may wish to reserve some external IRQs
/// with their external inputs tied to 0 for this purpose.
///
/// Bits can be cleared by software, and are cleared automatically by hardware
/// upon a read of `meinext` which returns the corresponding IRQ number in
/// `meinext.irq` with `mienext.noirq` clear (no matter whether `meinext.update`
/// is written).
///
/// `meifa` implements the same array window indexing scheme as `meiea` and
/// `meipa`.
pub const RVCSR_MEIFA_OFFSET: u32 = 0xbe2;

/// Machine External Interrupt Priority Array
///
/// Each interrupt has an (up to) 4-bit priority value associated with it, and
/// each access to this register reads and/or writes a 16-bit window containing
/// four such priority values. When less than 16 priority levels are available,
/// the LSBs of the priority fields are hardwired to 0.
///
/// When an interrupt's priority is lower than the current preemption priority
/// `meicontext.preempt`, it is treated as not being pending for the purposes of
/// `mip.meip`. The pending bit in `meipa` will still assert, but the machine
/// external interrupt pending bit `mip.meip` will not, so the processor will
/// ignore this interrupt. See `meicontext`.
pub const RVCSR_MEIPRA_OFFSET: u32 = 0xbe3;

/// Machine External Get Next Interrupt
///
/// Contains the index of the highest-priority external interrupt which is both
/// asserted in `meipa` and enabled in `meiea`, left- shifted by 2 so that it
/// can be used to index an array of 32-bit function pointers. If there is no
/// such interrupt, the MSB is set.
///
/// When multiple interrupts of the same priority are both pending and enabled,
/// the lowest-numbered wins. Interrupts with priority less than
/// `meicontext.ppreempt` -- the _previous_ preemption priority -- are treated
/// as though they are not pending. This is to ensure that a preempting
/// interrupt frame does not service interrupts which may be in progress in the
/// frame that was preempted.
pub const RVCSR_MEINEXT_OFFSET: u32 = 0xbe4;

/// Check if a given interrupt is pending
#[cfg(all(target_arch = "riscv32", target_os = "none"))]
pub fn is_pending(irq: rp235x_pac::Interrupt) -> bool {
    let (index, bits) = interrupt_to_mask(irq);
    let index = index as u32;
    let mut csr_rdata: u32;
    // Do a CSR Read-Set on RVCSR_MEIPA_OFFSET
    unsafe {
        core::arch::asm!(
            "csrrs {0}, 0xbe1, {1}",
            out(reg) csr_rdata,
            in(reg) index
        );
    }
    let bitmask = (bits as u32) << 16;
    (csr_rdata & bitmask) != 0
}

/// Enable an interrupt
///
/// # Safety
///
/// This function is unsafe because it can break mask-based critical
/// sections. Do not call inside a critical section.
#[cfg(all(target_arch = "riscv32", target_os = "none"))]
pub unsafe fn unmask(irq: rp235x_pac::Interrupt) {
    let mask_index = interrupt_to_mask_index(irq);
    // Do a RISC-V CSR Set on RVCSR_MEIEA_OFFSET
    unsafe {
        core::arch::asm!(
            "csrs 0xbe0, {0}",
            in(reg) mask_index
        );
    }
}

/// Disable an interrupt
#[cfg(all(target_arch = "riscv32", target_os = "none"))]
pub fn mask(irq: rp235x_pac::Interrupt) {
    let mask_index = interrupt_to_mask_index(irq);
    // Do a RISC-V CSR Clear on RVCSR_MEIEA_OFFSET
    unsafe {
        core::arch::asm!(
            "csrc 0xbe0, {0}",
            in(reg) mask_index
        );
    }
}

/// Check if an interrupt is enabled
#[cfg(all(target_arch = "riscv32", target_os = "none"))]
pub fn is_enabled(irq: rp235x_pac::Interrupt) -> bool {
    let (index, bits) = interrupt_to_mask(irq);
    let index = index as u32;
    let mut csr_rdata: u32;
    // Do a CSR Read-Set on RVCSR_MEIEA_OFFSET
    unsafe {
        core::arch::asm!(
            "csrrs {0}, 0xbe0, {1}",
            out(reg) csr_rdata,
            in(reg) index
        );
    }
    let bitmask = (bits as u32) << 16;
    (csr_rdata & bitmask) != 0
}

/// Set an interrupt as pending, even if it isn't.
#[cfg(all(target_arch = "riscv32", target_os = "none"))]
pub fn pend(irq: rp235x_pac::Interrupt) {
    let mask_index = interrupt_to_mask_index(irq);
    // Do a RISC-V CSR Set on RVCSR_MEIFA_OFFSET
    unsafe {
        core::arch::asm!(
            "csrs 0xbe2, {0}",
            in(reg) mask_index
        );
    }
}

/// Check which interrupt should be handled next
///
/// Also updates the state so next time you call this you'll get a different
/// answer.
#[cfg(all(target_arch = "riscv32", target_os = "none"))]
pub fn get_next_interrupt() -> Option<rp235x_pac::Interrupt> {
    const NOIRQ: u32 = 0x8000_0000;

    let mut csr_rdata: u32;
    // Do a CSR Read-Set-Immediate on MEINEXT to set the UPDATE bit
    unsafe {
        core::arch::asm!(
            "csrrsi {0}, 0xbe4, 0x01",
            out(reg) csr_rdata,
        );
    }

    if (csr_rdata & NOIRQ) != 0 {
        return None;
    }

    let irq_no = (csr_rdata >> 2) as u8;
    let irq = rp235x_pac::Interrupt::try_from(irq_no).unwrap();
    Some(irq)
}

/// Convert an IRQ into a window selection value and separate a bitmask for that
/// window.
pub const fn interrupt_to_mask(irq: rp235x_pac::Interrupt) -> (u16, u16) {
    let irq = irq as u16;
    // Select a bank of 16 interrupts out of the 512 total
    let index = irq / 16;
    // Which interrupt out of the 16 we've selected
    let bitmask = 1 << (irq % 16);
    (index, bitmask)
}

/// Convert an IRQ into a 32-bit value that selects a window and a bit within
/// that window.
pub const fn interrupt_to_mask_index(irq: rp235x_pac::Interrupt) -> u32 {
    let (index, bits) = interrupt_to_mask(irq);
    (bits as u32) << 16 | index as u32
}

#[cfg(test)]
mod test {
    use super::*;

    #[test]
    fn test_interrupt_to_mask() {
        let (index, bitmask) = interrupt_to_mask(rp235x_pac::Interrupt::ADC_IRQ_FIFO);
        assert_eq!(rp235x_pac::Interrupt::ADC_IRQ_FIFO as u16, 35);
        // ADC_IRQ_FIFO is IRQ 35, so that's index 2, bit 3
        assert_eq!(index, 2);
        assert_eq!(bitmask, 1 << 3);
    }

    #[test]
    fn test_interrupt_to_mask_index() {
        let mask = interrupt_to_mask_index(rp235x_pac::Interrupt::ADC_IRQ_FIFO);
        assert_eq!(rp235x_pac::Interrupt::ADC_IRQ_FIFO as u16, 35);
        // ADC_IRQ_FIFO is IRQ 35, so that's index 2, bit 3
        // This value is in (bitmask | index) format, and 1 << 3 is 0x0008
        assert_eq!(mask, 0x0008_0002);
    }
}

// End of file
