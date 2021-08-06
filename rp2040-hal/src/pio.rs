//! Programmable IO (PIO)
/// See [Chapter 3](https://rptl.io/pico-datasheet) for more details.
use crate::resets::SubsystemReset;
use pio::{Program, SideSet, Wrap};

const PIO_INSTRUCTION_COUNT: usize = 32;

/// PIO Instance
pub trait Instance:
    core::ops::Deref<Target = rp2040_pac::pio0::RegisterBlock> + SubsystemReset
{
}

impl Instance for rp2040_pac::PIO0 {}
impl Instance for rp2040_pac::PIO1 {}

/// Programmable IO Block
pub struct PIO<P: Instance> {
    used_instruction_space: core::cell::Cell<u32>, // bit for each PIO_INSTRUCTION_COUNT
    pio: P,
    state_machines: [StateMachine<P>; 4],
    interrupts: [Interrupt<P>; 2],
}

impl<P: Instance> core::fmt::Debug for PIO<P> {
    fn fmt(&self, f: &mut core::fmt::Formatter) -> core::fmt::Result {
        f.debug_struct("PIO")
            .field("used_instruction_space", &self.used_instruction_space)
            .field("pio", &"PIO { .. }")
            .finish()
    }
}

// Safety: `PIO` provides exclusive access to PIO registers.
unsafe impl<P: Instance + Send> Send for PIO<P> {}

impl<P: Instance> PIO<P> {
    /// Create a new PIO wrapper.
    pub fn new(pio: P, resets: &mut pac::RESETS) -> Self {
        pio.reset_bring_up(resets);

        PIO {
            used_instruction_space: core::cell::Cell::new(0),
            state_machines: [
                StateMachine {
                    id: 0,
                    block: pio.deref(),
                    _phantom: core::marker::PhantomData,
                },
                StateMachine {
                    id: 1,
                    block: pio.deref(),
                    _phantom: core::marker::PhantomData,
                },
                StateMachine {
                    id: 2,
                    block: pio.deref(),
                    _phantom: core::marker::PhantomData,
                },
                StateMachine {
                    id: 3,
                    block: pio.deref(),
                    _phantom: core::marker::PhantomData,
                },
            ],
            interrupts: [
                Interrupt {
                    id: 0,
                    block: pio.deref(),
                    _phantom: core::marker::PhantomData,
                },
                Interrupt {
                    id: 1,
                    block: pio.deref(),
                    _phantom: core::marker::PhantomData,
                },
            ],
            pio,
        }
    }

    /// Free this instance.
    pub fn free(self) -> P {
        self.pio
    }

    /// This PIO's state machines.
    pub fn state_machines(&self) -> &[StateMachine<P>; 4] {
        &self.state_machines
    }

    /// This PIO's interrupts.
    pub fn interrupts(&self) -> &[Interrupt<P>; 2] {
        &self.interrupts
    }

    /// Clear PIO's IRQ flags indicated by the bits.
    ///
    /// The PIO has 8 IRQ flags, of which 4 are visible to the host processor. Each bit of `flags` corresponds to one of
    /// the IRQ flags.
    pub fn clear_irq(&self, flags: u8) {
        self.pio
            .deref()
            .irq
            .write(|w| unsafe { w.irq().bits(flags) });
    }

    /// Force PIO's IRQ flags indicated by the bits.
    ///
    /// The PIO has 8 IRQ flags, of which 4 are visible to the host processor. Each bit of `flags` corresponds to one of
    /// the IRQ flags.
    pub fn force_irq(&self, flags: u8) {
        self.pio
            .deref()
            .irq_force
            .write(|w| unsafe { w.irq_force().bits(flags) });
    }

    fn find_offset_for_instructions(&self, i: &[u16], origin: Option<u8>) -> Option<usize> {
        if i.len() > PIO_INSTRUCTION_COUNT {
            None
        } else {
            let mask = (1 << i.len()) - 1;
            if let Some(origin) = origin {
                if origin as usize > PIO_INSTRUCTION_COUNT - i.len()
                    || self.used_instruction_space.get() & (mask << origin) != 0
                {
                    None
                } else {
                    Some(origin as usize)
                }
            } else {
                for i in (0..=32 - i.len()).rev() {
                    if self.used_instruction_space.get() & (mask << i) == 0 {
                        return Some(i);
                    }
                }
                None
            }
        }
    }

    fn add_program(&self, instructions: &[u16], origin: Option<u8>) -> Option<usize> {
        if let Some(offset) = self.find_offset_for_instructions(instructions, origin) {
            for (i, instr) in instructions.iter().enumerate() {
                let instr = if instr & 0xe000 != 0 {
                    *instr
                } else {
                    // JMP instruction. We need to apply offset here
                    let address = instr & 0x001f;
                    assert!(
                        address + (offset as u16) <= 0x001f,
                        "Invalid JMP out of the program after offset addition"
                    );
                    instr + offset as u16
                };

                self.pio.instr_mem[i + offset].write(|w| unsafe { w.bits(instr as u32) })
            }
            self.used_instruction_space
                .set(self.used_instruction_space.get() | ((1 << instructions.len()) - 1));
            Some(offset)
        } else {
            None
        }
    }
}

/// PIO State Machine.
#[derive(Debug)]
pub struct StateMachine<P: Instance> {
    id: u8,
    block: *const rp2040_pac::pio0::RegisterBlock,
    _phantom: core::marker::PhantomData<P>,
}

// `StateMachine` doesn't implement `Send` because it sometimes accesses shared registers, e.g. `sm_enable`.
// unsafe impl<P: Instance + Send> Send for StateMachine<P> {}

impl<P: Instance> StateMachine<P> {
    /// Start and stop the state machine.
    pub fn set_enabled(&self, enabled: bool) {
        let mask = 1 << self.id;
        if enabled {
            self.block()
                .ctrl
                .modify(|r, w| unsafe { w.sm_enable().bits(r.sm_enable().bits() | mask) })
        } else {
            self.block()
                .ctrl
                .modify(|r, w| unsafe { w.sm_enable().bits(r.sm_enable().bits() & !mask) })
        }
    }

    fn restart(&self) {
        self.block()
            .ctrl
            .write(|w| unsafe { w.sm_restart().bits(1 << self.id) });
    }

    fn reset_clock(&self) {
        self.block()
            .ctrl
            .write(|w| unsafe { w.clkdiv_restart().bits(1 << self.id) });
    }

    fn set_clock_divisor(&self, divisor: f32) {
        // sm frequency = clock freq / (CLKDIV_INT + CLKDIV_FRAC / 256)
        let int = divisor as u16;
        let frac = ((divisor - int as f32) * 256.0) as u8;

        self.sm().sm_clkdiv.write(|w| {
            unsafe {
                w.int().bits(int);
                w.frac().bits(frac);
            }

            w
        });
    }

    /// The address of the instruction currently being executed.
    pub fn instruction_address(&self) -> u32 {
        self.sm().sm_addr.read().bits()
    }

    /// Set the current instruction.
    pub fn set_instruction(&self, instruction: u16) {
        self.sm()
            .sm_instr
            .write(|w| unsafe { w.sm0_instr().bits(instruction) })
    }

    /// Pull a word from the RX FIFO
    pub fn pull(&self) -> u32 {
        self.block().rxf[self.id as usize].read().bits()
    }

    /// Push a word into the TX FIFO
    pub fn push(&self, word: u32) {
        self.block().txf[self.id as usize].write(|w| unsafe { w.bits(word) })
    }

    /// Check if the current instruction is stalled.
    pub fn stalled(&self) -> bool {
        self.sm().sm_execctrl.read().exec_stalled().bits()
    }

    fn block(&self) -> &rp2040_pac::pio0::RegisterBlock {
        unsafe { &*self.block }
    }

    fn sm(&self) -> &rp2040_pac::pio0::SM {
        &self.block().sm[self.id as usize]
    }

    /// Get the next element from RX FIFO.
    ///
    /// Returns `None` if the FIFO is empty.
    pub fn read_rx(&self) -> Option<u32> {
        let is_empty = self.block().fstat.read().rxempty().bits() & (1 << self.id) != 0;

        if is_empty {
            return None;
        }

        Some(self.block().rxf[self.id as usize].read().bits())
    }

    /// Write an element to TX FIFO.
    ///
    /// Returns `true` if the value was written to FIFO, `false` otherwise.
    pub fn write_tx(&self, value: u32) -> bool {
        let is_full = self.block().fstat.read().txfull().bits() & (1 << self.id) != 0;

        if is_full {
            return false;
        }

        self.block().txf[self.id as usize].write(|w| unsafe { w.bits(value) });

        true
    }
}

/// PIO Interrupt controller.
#[derive(Debug)]
pub struct Interrupt<P: Instance> {
    id: u8,
    block: *const rp2040_pac::pio0::RegisterBlock,
    _phantom: core::marker::PhantomData<P>,
}

// Safety: `Interrupt` provides exclusive access to interrupt registers.
unsafe impl<P: Instance + Send> Send for Interrupt<P> {}

impl<P: Instance> Interrupt<P> {
    /// Enable interrupts raised by state machines.
    ///
    /// The PIO peripheral has 4 outside visible interrupts that can be raised by the state machines. Note that this
    /// don't correspond with the state machine index; any state machine can raise any one of the four interrupts.
    pub fn enable_sm_interrupt(&self, id: u8) {
        match id {
            0 => self.irq().irq_inte.write(|w| w.sm0().set_bit()),
            1 => self.irq().irq_inte.write(|w| w.sm1().set_bit()),
            2 => self.irq().irq_inte.write(|w| w.sm2().set_bit()),
            3 => self.irq().irq_inte.write(|w| w.sm3().set_bit()),
            _ => panic!("invalid state machine interrupt number"),
        }
    }

    /// Disable interrupts raised by state machines.
    ///
    /// See [`Self::enable_sm_interrupt`] for info about the index.
    pub fn disable_sm_interrupt(&self, id: u8) {
        match id {
            0 => self.irq().irq_inte.write(|w| w.sm0().clear_bit()),
            1 => self.irq().irq_inte.write(|w| w.sm1().clear_bit()),
            2 => self.irq().irq_inte.write(|w| w.sm2().clear_bit()),
            3 => self.irq().irq_inte.write(|w| w.sm3().clear_bit()),
            _ => panic!("invalid state machine interrupt number"),
        }
    }

    /// Force state machine interrupt.
    ///
    /// Note that this doesn't affect the state seen by the state machine. For that, see [`PIO::force_irq`].
    ///
    /// See [`Self::enable_sm_interrupt`] for info about the index.
    pub fn force_sm_interrupt(&self, id: u8) {
        match id {
            0 => self.irq().irq_intf.write(|w| w.sm0().set_bit()),
            1 => self.irq().irq_intf.write(|w| w.sm1().set_bit()),
            2 => self.irq().irq_intf.write(|w| w.sm2().set_bit()),
            3 => self.irq().irq_intf.write(|w| w.sm3().set_bit()),
            _ => panic!("invalid state machine interrupt number"),
        }
    }

    /// Enable TX FIFO not full interrupt.
    ///
    /// Each of the 4 state machines have their own TX FIFO. This interrupt is raised when the TX FIFO is not full, i.e.
    /// one could push more data to it.
    pub fn enable_tx_not_full_interrupt(&self, id: u8) {
        match id {
            0 => self.irq().irq_inte.write(|w| w.sm0_txnfull().set_bit()),
            1 => self.irq().irq_inte.write(|w| w.sm1_txnfull().set_bit()),
            2 => self.irq().irq_inte.write(|w| w.sm2_txnfull().set_bit()),
            3 => self.irq().irq_inte.write(|w| w.sm3_txnfull().set_bit()),
            _ => panic!("invalid state machine interrupt number"),
        }
    }

    /// Disable TX FIFO not full interrupt.
    ///
    /// See [`Self::enable_tx_not_full_interrupt`] for info about the index.
    pub fn disable_tx_not_full_interrupt(&self, id: u8) {
        match id {
            0 => self.irq().irq_inte.write(|w| w.sm0_txnfull().clear_bit()),
            1 => self.irq().irq_inte.write(|w| w.sm1_txnfull().clear_bit()),
            2 => self.irq().irq_inte.write(|w| w.sm2_txnfull().clear_bit()),
            3 => self.irq().irq_inte.write(|w| w.sm3_txnfull().clear_bit()),
            _ => panic!("invalid state machine interrupt number"),
        }
    }

    /// Force TX FIFO not full interrupt.
    ///
    /// See [`Self::enable_tx_not_full_interrupt`] for info about the index.
    pub fn force_tx_not_full_interrupt(&self, id: u8) {
        match id {
            0 => self.irq().irq_intf.write(|w| w.sm0_txnfull().set_bit()),
            1 => self.irq().irq_intf.write(|w| w.sm1_txnfull().set_bit()),
            2 => self.irq().irq_intf.write(|w| w.sm2_txnfull().set_bit()),
            3 => self.irq().irq_intf.write(|w| w.sm3_txnfull().set_bit()),
            _ => panic!("invalid state machine interrupt number"),
        }
    }

    /// Enable RX FIFO not empty interrupt.
    ///
    /// Each of the 4 state machines have their own RX FIFO. This interrupt is raised when the RX FIFO is not empty,
    /// i.e. one could read more data from it.
    pub fn enable_rx_not_empty_interrupt(&self, id: u8) {
        match id {
            0 => self.irq().irq_inte.write(|w| w.sm0_rxnempty().set_bit()),
            1 => self.irq().irq_inte.write(|w| w.sm1_rxnempty().set_bit()),
            2 => self.irq().irq_inte.write(|w| w.sm2_rxnempty().set_bit()),
            3 => self.irq().irq_inte.write(|w| w.sm3_rxnempty().set_bit()),
            _ => panic!("invalid state machine interrupt number"),
        }
    }

    /// Disable RX FIFO not empty interrupt.
    ///
    /// See [`Self::enable_rx_not_empty_interrupt`] for info about the index.
    pub fn disable_rx_not_empty_interrupt(&self, id: u8) {
        match id {
            0 => self.irq().irq_inte.write(|w| w.sm0_rxnempty().clear_bit()),
            1 => self.irq().irq_inte.write(|w| w.sm1_rxnempty().clear_bit()),
            2 => self.irq().irq_inte.write(|w| w.sm2_rxnempty().clear_bit()),
            3 => self.irq().irq_inte.write(|w| w.sm3_rxnempty().clear_bit()),
            _ => panic!("invalid state machine interrupt number"),
        }
    }

    /// Force RX FIFO not empty interrupt.
    ///
    /// See [`Self::enable_rx_not_empty_interrupt`] for info about the index.
    pub fn force_rx_not_empty_interrupt(&self, id: u8) {
        match id {
            0 => self.irq().irq_intf.write(|w| w.sm0_rxnempty().set_bit()),
            1 => self.irq().irq_intf.write(|w| w.sm1_rxnempty().set_bit()),
            2 => self.irq().irq_intf.write(|w| w.sm2_rxnempty().set_bit()),
            3 => self.irq().irq_intf.write(|w| w.sm3_rxnempty().set_bit()),
            _ => panic!("invalid state machine interrupt number"),
        }
    }

    /// Get the raw interrupt state.
    ///
    /// This is the state of the interrupts without interrupt masking and forcing.
    pub fn raw(&self) -> InterruptState {
        InterruptState(self.block().intr.read().bits())
    }

    /// Get the interrupt state.
    ///
    /// This is the state of the interrupts after interrupt masking and forcing.
    pub fn state(&self) -> InterruptState {
        InterruptState(self.irq().irq_ints.read().bits())
    }

    fn block(&self) -> &rp2040_pac::pio0::RegisterBlock {
        unsafe { &*self.block }
    }

    fn irq(&self) -> &rp2040_pac::pio0::SM_IRQ {
        &self.block().sm_irq[self.id as usize]
    }
}

/// Provides easy access for decoding PIO's interrupt state.
#[derive(Debug, Clone, Copy, PartialEq, Eq, PartialOrd, Ord, Hash)]
pub struct InterruptState(u32);

macro_rules! raw_interrupt_accessor {
    ($name:ident, $doc:literal, $idx:expr) => {
        #[doc = concat!("Check whether interrupt ", $doc, " has been raised.")]
        pub fn $name(self) -> bool {
            self.0 & (1 << $idx) != 0
        }
    };
}
impl InterruptState {
    raw_interrupt_accessor!(sm0_rx_not_empty, "SM0_RXNEMPTY", 0);
    raw_interrupt_accessor!(sm1_rx_not_empty, "SM1_RXNEMPTY", 1);
    raw_interrupt_accessor!(sm2_rx_not_empty, "SM2_RXNEMPTY", 2);
    raw_interrupt_accessor!(sm3_rx_not_empty, "SM3_RXNEMPTY", 3);

    raw_interrupt_accessor!(sm0_tx_not_full, "SM0_TXNFULL", 4);
    raw_interrupt_accessor!(sm1_tx_not_full, "SM1_TXNFULL", 5);
    raw_interrupt_accessor!(sm2_tx_not_full, "SM2_TXNFULL", 6);
    raw_interrupt_accessor!(sm3_tx_not_full, "SM3_TXNFULL", 7);

    raw_interrupt_accessor!(sm0, "SM0", 8);
    raw_interrupt_accessor!(sm1, "SM1", 9);
    raw_interrupt_accessor!(sm2, "SM2", 10);
    raw_interrupt_accessor!(sm3, "SM3", 11);
}

/// Comparison used for `mov x, status` instruction.
#[derive(Debug, Clone, Copy)]
pub enum MovStatusConfig {
    /// The `mov x, status` instruction returns all ones if TX FIFO level is below the set status, otherwise all zeros.
    Tx(u8),
    /// The `mov x, status` instruction returns all ones if RX FIFO level is below the set status, otherwise all zeros.
    Rx(u8),
}

/// Shift direction for input and output shifting.
#[derive(Debug, Clone, Copy)]
pub enum ShiftDirection {
    /// Shift register to left.
    Left,
    /// Shift register to right.
    Right,
}

impl ShiftDirection {
    fn bit(self) -> bool {
        match self {
            Self::Left => false,
            Self::Right => true,
        }
    }
}

/// Builder to deploy a fully configured PIO program on one of the state
/// machines.
#[derive(Debug)]
pub struct PIOBuilder<'a> {
    /// Clock divisor.
    clock_divisor: f32,

    /// Instructions of the program.
    instructions: &'a [u16],
    /// Origin where this program should be loaded.
    instruction_origin: Option<u8>,
    /// Wrapping behavior.
    wrap: Wrap,
    /// Side-set behavior.
    side_set: SideSet,
    /// GPIO pin used by `jmp pin` instruction.
    jmp_pin: u8,

    /// Continuously assert the most recent OUT/SET to the pins.
    out_sticky: bool,
    /// Use a bit of OUT data as an auxilary write enable.
    ///
    /// When [`out_sticky`](Self::out_sticky) is enabled, setting the bit to 0 deasserts for that instr.
    inline_out: Option<u8>,
    /// Config for `mov x, status` instruction.
    mov_status: MovStatusConfig,

    /// Config for FIFO joining.
    fifo_join: Buffers,

    /// Number of bits shifted out of `OSR` before autopull or conditional pull will take place.
    pull_threshold: u8,
    /// Number of bits shifted into `ISR` before autopush or conditional push will take place.
    push_threshold: u8,
    // Shift direction for `OUT` instruction.
    out_shiftdir: ShiftDirection,
    // Shift direction for `IN` instruction.
    in_shiftdir: ShiftDirection,
    // Enable autopull.
    auto_pull: bool,
    // Enable autopush.
    auto_push: bool,

    /// Number of pins asserted by a `SET`.
    set_count: u8,
    /// Number of pins asserted by an `OUT PINS`, `OUT PINDIRS` or `MOV PINS` instruction.
    out_count: u8,
    /// The first pin that is assigned in state machine's `IN` data bus.
    in_base: u8,
    /// The first pin that is affected by side-set operations.
    side_set_base: u8,
    /// The first pin that is affected by `SET PINS` or `SET PINDIRS` instructions.
    set_base: u8,
    /// The first pin that is affected by `OUT PINS`, `OUT PINDIRS` or `MOV PINS` instructions.
    out_base: u8,
}

impl<'a> Default for PIOBuilder<'a> {
    fn default() -> Self {
        PIOBuilder {
            clock_divisor: 1.0,
            instructions: &[],
            instruction_origin: None,
            wrap: Wrap {
                source: 31,
                target: 0,
            },
            side_set: SideSet::default(),
            jmp_pin: 0,
            out_sticky: false,
            inline_out: None,
            mov_status: MovStatusConfig::Tx(0),
            fifo_join: Buffers::RxTx,
            pull_threshold: 0,
            push_threshold: 0,
            out_shiftdir: ShiftDirection::Left,
            in_shiftdir: ShiftDirection::Left,
            auto_pull: false,
            auto_push: false,
            set_count: 5,
            out_count: 0,
            in_base: 0,
            side_set_base: 0,
            set_base: 0,
            out_base: 0,
        }
    }
}

/// Buffer sharing configuration.
#[derive(Debug, Clone, Copy)]
pub enum Buffers {
    /// No sharing.
    RxTx,
    /// The memory of the RX FIFO is given to the TX FIFO to double its depth.
    OnlyTx,
    /// The memory of the TX FIFO is given to the RX FIFO to double its depth.
    OnlyRx,
}

/// Errors that occurred during `PIOBuilder::build`.
#[derive(Debug)]
pub enum BuildError {
    /// There was not enough space for the instructions on the selected PIO.
    NoSpace,
}

impl<'a> PIOBuilder<'a> {
    /// Set config settings based on information from the given [`pio::Program`].
    /// Additional configuration may be needed in addition to this.
    pub fn with_program(mut self, p: &'a Program) -> Self {
        self.instruction_origin = p.origin;
        self.instructions(&p.instructions)
            .wrap(p.wrap)
            .side_set(p.side_set)
    }

    /// Set the instructions of the program.
    pub fn instructions(mut self, instructions: &'a [u16]) -> Self {
        self.instructions = instructions;
        self
    }

    /// Set the wrap source and target.
    ///
    /// The program will automatically jump from the wrap bottom to the wrap top in 0 cycles.
    pub fn wrap(mut self, wrap: Wrap) -> Self {
        self.wrap = wrap;
        self
    }

    /// Sets the pins asserted by `SET` instruction.
    ///
    /// The least-significant bit of `SET` instruction asserts the state of the pin indicated by `base`, the next bit
    /// asserts the state of the next pin, and so on up to `count` pins. The pin numbers are considered modulo 32.
    pub fn set_pins(mut self, base: u8, count: u8) -> Self {
        assert!(count <= 5);
        self.set_base = base;
        self.set_count = count;
        // self.in_base = base;
        // self.out_base = base;
        // self.out_count = count;
        self
    }

    /// Sets the pins asserted by `OUT` instruction.
    ///
    /// The least-significant bit of `OUT` instruction asserts the state of the pin indicated by `base`, the next bit
    /// asserts the state of the next pin, and so on up to `count` pins. The pin numbers are considered modulo 32.
    pub fn out_pins(mut self, base: u8, count: u8) -> Self {
        assert!(count <= 5);
        self.out_base = base;
        self.out_count = count;
        self
    }

    pub fn in_pin_base(mut self, base: u8) -> Self {
        self.in_base = base;
        self
    }

    pub fn jmp_pin(mut self, pin: u8) -> Self {
        self.jmp_pin = pin;
        self
    }

    pub fn side_set_pin_base(mut self, base: u8) -> Self {
        self.side_set_base = base;
        self
    }

    /// Set the side-set status.
    pub fn side_set(mut self, side_set: SideSet) -> Self {
        self.side_set = side_set;
        self
    }

    /// Set buffer sharing.
    ///
    /// See [`Buffers`] for more information.
    pub fn buffers(mut self, buffers: Buffers) -> Self {
        self.fifo_join = buffers;
        self
    }

    /// Set the clock divisor.
    ///
    /// The is based on the sys_clk. Set 1 for full speed. A clock divisor of `n` will cause the state machine to run 1
    /// cycle every `n` clock cycles. For small values of `n`, a fractional divisor may introduce unacceptable jitter.
    pub fn clock_divisor(mut self, divisor: f32) -> Self {
        self.clock_divisor = divisor;
        self
    }

    /// Build the config and deploy it to a StateMachine.
    pub fn build<P: Instance>(self, pio: &PIO<P>, sm: &StateMachine<P>) -> Result<(), BuildError> {
        let offset = match pio.add_program(self.instructions, self.instruction_origin) {
            Some(o) => o,
            None => return Err(BuildError::NoSpace),
        };

        // Stop the SM
        // TODO: This should probably do before we write the program source code
        sm.set_enabled(false);

        // Write all configuration bits
        sm.set_clock_divisor(self.clock_divisor);

        sm.sm().sm_execctrl.write(|w| {
            w.side_en().bit(self.side_set.optional());
            w.side_pindir().bit(self.side_set.pindirs());

            unsafe {
                w.jmp_pin().bits(self.jmp_pin);
            }

            if let Some(inline_out) = self.inline_out {
                w.inline_out_en().bit(true);
                unsafe {
                    w.out_en_sel().bits(inline_out);
                }
            } else {
                w.inline_out_en().bit(false);
            }

            w.out_sticky().bit(self.out_sticky);

            unsafe {
                w.wrap_top().bits(offset as u8 + self.wrap.source);
                w.wrap_bottom().bits(offset as u8 + self.wrap.target);
            }

            let n = match self.mov_status {
                MovStatusConfig::Tx(n) => {
                    w.status_sel().bit(false);
                    n
                }
                MovStatusConfig::Rx(n) => {
                    w.status_sel().bit(true);
                    n
                }
            };
            unsafe {
                w.status_n().bits(n);
            }

            w
        });

        sm.sm().sm_shiftctrl.write(|w| {
            let (fjoin_rx, fjoin_tx) = match self.fifo_join {
                Buffers::RxTx => (false, false),
                Buffers::OnlyTx => (false, true),
                Buffers::OnlyRx => (true, false),
            };
            w.fjoin_rx().bit(fjoin_rx);
            w.fjoin_tx().bit(fjoin_tx);

            unsafe {
                // TODO: Encode 32 as zero, and error on 0
                w.pull_thresh().bits(self.pull_threshold);
                w.push_thresh().bits(self.push_threshold);
            }

            w.out_shiftdir().bit(self.out_shiftdir.bit());
            w.in_shiftdir().bit(self.in_shiftdir.bit());

            w.autopull().bit(self.auto_pull);
            w.autopush().bit(self.auto_push);

            w
        });

        sm.sm().sm_pinctrl.write(|w| {
            unsafe {
                w.sideset_count().bits(self.side_set.bits());
                w.set_count().bits(self.set_count);
                w.out_count().bits(self.out_count);

                w.in_base().bits(self.in_base);
                w.sideset_base().bits(self.side_set_base);
                w.set_base().bits(self.set_base);
                w.out_base().bits(self.out_base);
            }

            w
        });

        // Restart SM and its clock
        sm.restart();
        sm.reset_clock();

        // Set starting location by setting the state machine to execute a jmp
        // to the beginning of the program we loaded in.
        #[allow(clippy::unusual_byte_groupings)]
        let mut instr = 0b000_00000_000_00000; // JMP 0
        instr |= offset as u16;
        sm.set_instruction(instr);

        // Enable SM
        sm.set_enabled(true);

        Ok(())
    }
}
