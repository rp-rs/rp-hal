//! Programmable IO (PIO)
//! See [Chapter 3 of the datasheet](https://rptl.io/rp2040-datasheet#section_pio) for more details.
use core::ops::Deref;

use crate::{
    atomic_register_access::{write_bitmask_clear, write_bitmask_set},
    dma::{EndlessReadTarget, EndlessWriteTarget, ReadTarget, WriteTarget},
    gpio::{Function, FunctionPio0, FunctionPio1},
    resets::SubsystemReset,
    typelevel::Sealed,
};
use pio::{Instruction, InstructionOperands, Program, SideSet, Wrap};
use rp2040_pac::{dma::ch::ch_ctrl_trig::TREQ_SEL_A, pio0::RegisterBlock, PIO0, PIO1};

const PIO_INSTRUCTION_COUNT: usize = 32;

impl crate::typelevel::Sealed for PIO0 {}
impl crate::typelevel::Sealed for PIO1 {}

/// PIO Instance
pub trait PIOExt: Deref<Target = RegisterBlock> + SubsystemReset + Sized + Send + Sealed {
    /// Associated Pin Function.
    type PinFunction: Function;

    /// Create a new PIO wrapper and split the state machines into individual objects.
    #[allow(clippy::type_complexity)] // Required for symmetry with PIO::free().
    fn split(
        self,
        resets: &mut pac::RESETS,
    ) -> (
        PIO<Self>,
        UninitStateMachine<(Self, SM0)>,
        UninitStateMachine<(Self, SM1)>,
        UninitStateMachine<(Self, SM2)>,
        UninitStateMachine<(Self, SM3)>,
    ) {
        self.reset_bring_down(resets);
        self.reset_bring_up(resets);

        let sm0 = UninitStateMachine {
            block: self.deref(),
            sm: &self.deref().sm[0],
            _phantom: core::marker::PhantomData,
        };
        let sm1 = UninitStateMachine {
            block: self.deref(),
            sm: &self.deref().sm[1],
            _phantom: core::marker::PhantomData,
        };
        let sm2 = UninitStateMachine {
            block: self.deref(),
            sm: &self.deref().sm[2],
            _phantom: core::marker::PhantomData,
        };
        let sm3 = UninitStateMachine {
            block: self.deref(),
            sm: &self.deref().sm[3],
            _phantom: core::marker::PhantomData,
        };
        (
            PIO {
                used_instruction_space: 0,
                pio: self,
            },
            sm0,
            sm1,
            sm2,
            sm3,
        )
    }

    /// Number of this PIO (0..1).
    fn id() -> usize;
}

impl PIOExt for PIO0 {
    type PinFunction = FunctionPio0;
    fn id() -> usize {
        0
    }
}
impl PIOExt for PIO1 {
    type PinFunction = FunctionPio1;
    fn id() -> usize {
        1
    }
}

#[allow(clippy::upper_case_acronyms)]
/// Programmable IO Block
pub struct PIO<P: PIOExt> {
    used_instruction_space: u32, // bit for each PIO_INSTRUCTION_COUNT
    pio: P,
}

impl<P: PIOExt> core::fmt::Debug for PIO<P> {
    fn fmt(&self, f: &mut core::fmt::Formatter) -> core::fmt::Result {
        f.debug_struct("PIO")
            .field("used_instruction_space", &self.used_instruction_space)
            .field("pio", &"PIO { .. }")
            .finish()
    }
}

// Safety: `PIO` only provides access to those registers which are not directly used by
// `StateMachine`.
unsafe impl<P: PIOExt> Send for PIO<P> {}

// Safety: `PIO` is marked Send so ensure all accesses remain atomic and no new concurrent accesses
// are added.
impl<P: PIOExt> PIO<P> {
    /// Free this instance.
    ///
    /// All output pins are left in their current state.
    pub fn free(
        self,
        _sm0: UninitStateMachine<(P, SM0)>,
        _sm1: UninitStateMachine<(P, SM1)>,
        _sm2: UninitStateMachine<(P, SM2)>,
        _sm3: UninitStateMachine<(P, SM3)>,
    ) -> P {
        // All state machines have already been stopped.
        self.pio
    }

    /// This PIO0's interrupts.
    pub fn irq0(&self) -> Interrupt<'_, P, 0> {
        Interrupt {
            block: self.pio.deref(),
            _phantom: core::marker::PhantomData,
        }
    }

    /// This PIO0's interrupts.
    pub fn irq1(&self) -> Interrupt<'_, P, 1> {
        Interrupt {
            block: self.pio.deref(),
            _phantom: core::marker::PhantomData,
        }
    }

    /// Get raw irq flags.
    ///
    /// The PIO has 8 IRQ flags, of which 4 are visible to the host processor. Each bit of `flags` corresponds to one of
    /// the IRQ flags.
    pub fn get_irq_raw(&self) -> u8 {
        self.pio.irq.read().irq().bits()
    }

    /// Clear PIO's IRQ flags indicated by the bits.
    ///
    /// The PIO has 8 IRQ flags, of which 4 are visible to the host processor. Each bit of `flags` corresponds to one of
    /// the IRQ flags.
    // Safety: PIOExt provides exclusive access to the pio.irq register, this must be preserved to
    // satisfy Send trait.
    pub fn clear_irq(&self, flags: u8) {
        self.pio.irq.write(|w| unsafe { w.irq().bits(flags) });
    }

    /// Force PIO's IRQ flags indicated by the bits.
    ///
    /// The PIO has 8 IRQ flags, of which 4 are visible to the host processor. Each bit of `flags` corresponds to one of
    /// the IRQ flags.
    // Safety: PIOExt provides exclusive access to the pio.irq register, this must be preserved to
    // satisfy Send trait.
    pub fn force_irq(&self, flags: u8) {
        self.pio
            .irq_force
            .write(|w| unsafe { w.irq_force().bits(flags) });
    }

    /// Calculates a mask with the `len` right-most bits set.
    fn instruction_mask(len: usize) -> u32 {
        if len < 32 {
            (1 << len) - 1
        } else {
            0xffffffff
        }
    }

    /// Tries to find an appropriate offset for the instructions, in range 0..=31.
    fn find_offset_for_instructions(&self, i: &[u16], origin: Option<u8>) -> Option<u8> {
        if i.len() > PIO_INSTRUCTION_COUNT || i.is_empty() {
            None
        } else {
            let mask = Self::instruction_mask(i.len());
            if let Some(origin) = origin {
                if origin as usize > PIO_INSTRUCTION_COUNT - i.len()
                    || self.used_instruction_space & (mask << origin) != 0
                {
                    None
                } else {
                    Some(origin)
                }
            } else {
                for i in (0..=32 - (i.len() as u8)).rev() {
                    if self.used_instruction_space & (mask << i) == 0 {
                        return Some(i);
                    }
                }
                None
            }
        }
    }

    /// Allocates space in instruction memory and installs the program.
    ///
    /// The function returns a handle to the installed program that can be used to configure a
    /// `StateMachine` via `PIOBuilder`. The program can be uninstalled to free instruction memory
    /// via `uninstall()` once the state machine using the program has been uninitialized.
    // Safety: PIOExt is marked send and should be the only object allowed to access pio.instr_mem
    pub fn install(
        &mut self,
        p: &Program<{ pio::RP2040_MAX_PROGRAM_SIZE }>,
    ) -> Result<InstalledProgram<P>, InstallError> {
        if let Some(offset) = self.find_offset_for_instructions(&p.code, p.origin) {
            p.code
                .iter()
                .cloned()
                .map(|instr| {
                    if instr & 0b1110_0000_0000_0000 == 0 {
                        // this is a JMP instruction -> add offset to address
                        let address = (instr & 0b11111) as u8;
                        let address = address + offset;
                        assert!(
                            address < pio::RP2040_MAX_PROGRAM_SIZE as u8,
                            "Invalid JMP out of the program after offset addition"
                        );
                        instr & (!0b11111) | address as u16
                    } else {
                        // this is not a JMP instruction -> keep it unchanged
                        instr
                    }
                })
                .enumerate()
                .for_each(|(i, instr)| {
                    self.pio.instr_mem[i + offset as usize]
                        .write(|w| unsafe { w.instr_mem0().bits(instr) })
                });
            self.used_instruction_space |= Self::instruction_mask(p.code.len()) << offset;
            Ok(InstalledProgram {
                offset,
                length: p.code.len() as u8,
                side_set: p.side_set,
                wrap: p.wrap,
                _phantom: core::marker::PhantomData,
            })
        } else {
            Err(InstallError::NoSpace)
        }
    }

    /// Removes the specified program from instruction memory, freeing the allocated space.
    pub fn uninstall(&mut self, p: InstalledProgram<P>) {
        let instr_mask = Self::instruction_mask(p.length as usize) << p.offset as u32;
        self.used_instruction_space &= !instr_mask;
    }
}

/// Handle to a program that was placed in the PIO's instruction memory.
///
/// Objects of this type can be reused for multiple state machines of the same PIO block to save
/// memory if multiple state machines are supposed to perform the same function (for example, if
/// one PIO block is used to implement multiple I2C busses).
///
/// `PIO::uninstall(program)` can be used to free the space occupied by the program once it is no
/// longer used.
///
/// # Examples
///
/// ```no_run
/// use rp2040_hal::{pac, pio::PIOBuilder, pio::PIOExt};
/// let mut peripherals = pac::Peripherals::take().unwrap();
/// let (mut pio, sm0, _, _, _) = peripherals.PIO0.split(&mut peripherals.RESETS);
/// // Install a program in instruction memory.
/// let program = pio_proc::pio_asm!(
///     ".wrap_target",
///     "set pins, 1 [31]",
///     "set pins, 0 [31]",
///     ".wrap"
/// ).program;
/// let installed = pio.install(&program).unwrap();
/// // Configure a state machine to use the program.
/// let (sm, rx, tx) = PIOBuilder::from_program(installed).build(sm0);
/// // Uninitialize the state machine again, freeing the program.
/// let (sm, installed) = sm.uninit(rx, tx);
/// // Uninstall the program to free instruction memory.
/// pio.uninstall(installed);
/// ```
///
/// # Safety
///
/// Objects of this type can outlive their `PIO` object. If the PIO block is reinitialized, the API
/// does not prevent the user from calling `uninstall()` when the PIO block does not actually hold
/// the program anymore. The user must therefore make sure that `uninstall()` is only called on the
/// PIO object which was used to install the program.
///
/// ```ignore
/// let (mut pio, sm0, sm1, sm2, sm3) = pac.PIO0.split(&mut pac.RESETS);
/// // Install a program in instruction memory.
/// let installed = pio.install(&program).unwrap();
/// // Reinitialize PIO.
/// let pio0 = pio.free(sm0, sm1, sm2, sm3);
/// let (mut pio, _, _, _, _) = pio0.split(&mut pac.RESETS);
/// // Do not do the following, the program is not in instruction memory anymore!
/// pio.uninstall(installed);
/// ```
#[derive(Debug)]
pub struct InstalledProgram<P> {
    offset: u8,
    length: u8,
    side_set: SideSet,
    wrap: Wrap,
    _phantom: core::marker::PhantomData<P>,
}

impl<P: PIOExt> InstalledProgram<P> {
    /// Change the source and/or target for automatic program wrapping.
    ///
    /// This replaces the current wrap bounds with a new set. This can be useful if you are running
    /// multiple state machines with the same program but using different wrap bounds.
    ///
    /// # Returns
    ///
    /// * [`Ok`] containing a new program with the provided wrap bounds
    /// * [`Err`] containing the old program if the provided wrap was invalid (outside the bounds of
    ///   the program length)
    pub fn set_wrap(self, wrap: Wrap) -> Result<Self, Self> {
        if wrap.source < self.length && wrap.target < self.length {
            Ok(InstalledProgram { wrap, ..self })
        } else {
            Err(self)
        }
    }

    /// Get the wrap target (entry point) of the installed program.
    pub fn wrap_target(&self) -> u8 {
        self.offset + self.wrap.target
    }

    /// Get the offset the program is installed at.
    pub fn offset(&self) -> u8 {
        self.offset
    }

    /// Clones this program handle so that it can be executed by two state machines at the same
    /// time.
    ///
    /// # Safety
    ///
    /// This function is marked as unsafe because, once this function has been called, the
    /// resulting handle can be used to call `PIO::uninstall()` while the program is still running.
    ///
    /// The user has to make sure to call `PIO::uninstall()` only once and only after all state
    /// machines using the program have been uninitialized.
    pub unsafe fn share(&self) -> InstalledProgram<P> {
        InstalledProgram {
            offset: self.offset,
            length: self.length,
            side_set: self.side_set,
            wrap: self.wrap,
            _phantom: core::marker::PhantomData,
        }
    }
}

/// State machine identifier (without a specified PIO block).
pub trait StateMachineIndex: Send + Sealed {
    /// Numerical index of the state machine (0 to 3).
    fn id() -> usize;
}

/// First state machine.
pub struct SM0;
/// Second state machine.
pub struct SM1;
/// Third state machine.
pub struct SM2;
/// Fourth state machine.
pub struct SM3;

impl StateMachineIndex for SM0 {
    fn id() -> usize {
        0
    }
}

impl Sealed for SM0 {}

impl StateMachineIndex for SM1 {
    fn id() -> usize {
        1
    }
}

impl Sealed for SM1 {}

impl StateMachineIndex for SM2 {
    fn id() -> usize {
        2
    }
}

impl Sealed for SM2 {}

impl StateMachineIndex for SM3 {
    fn id() -> usize {
        3
    }
}

impl Sealed for SM3 {}

/// Trait to identify a single state machine, as a generic type parameter to `UninitStateMachine`,
/// `InitStateMachine`, etc.
pub trait ValidStateMachine: Sealed {
    /// The PIO block to which this state machine belongs.
    type PIO: PIOExt;

    /// The index of this state machine (between 0 and 3).
    fn id() -> usize;
    /// The DREQ number for which TX DMA requests are triggered.
    fn tx_dreq() -> u8;
    /// The DREQ number for which RX DMA requests are triggered.
    fn rx_dreq() -> u8;
}

/// First state machine of the first PIO block.
pub type PIO0SM0 = (PIO0, SM0);
/// Second state machine of the first PIO block.
pub type PIO0SM1 = (PIO0, SM1);
/// Third state machine of the first PIO block.
pub type PIO0SM2 = (PIO0, SM2);
/// Fourth state machine of the first PIO block.
pub type PIO0SM3 = (PIO0, SM3);
/// First state machine of the second PIO block.
pub type PIO1SM0 = (PIO1, SM0);
/// Second state machine of the second PIO block.
pub type PIO1SM1 = (PIO1, SM1);
/// Third state machine of the second PIO block.
pub type PIO1SM2 = (PIO1, SM2);
/// Fourth state machine of the second PIO block.
pub type PIO1SM3 = (PIO1, SM3);

impl<P: PIOExt, SM: StateMachineIndex> ValidStateMachine for (P, SM) {
    type PIO = P;
    fn id() -> usize {
        SM::id()
    }
    fn tx_dreq() -> u8 {
        ((P::id() << 3) | SM::id()) as u8
    }
    fn rx_dreq() -> u8 {
        ((P::id() << 3) | SM::id() | 0x4) as u8
    }
}

/// Pin State in the PIO
///
/// Note the GPIO is able to override/invert that.
#[derive(Debug, PartialEq, Eq, Clone, Copy)]
pub enum PinState {
    /// Pin in Low state.
    High,
    /// Pin in Low state.
    Low,
}

/// Pin direction in the PIO
///
/// Note the GPIO is able to override/invert that.
#[derive(Debug, PartialEq, Eq, Clone, Copy)]
pub enum PinDir {
    /// Pin set as an Input
    Input,
    /// Pin set as an Output.
    Output,
}

/// PIO State Machine (uninitialized, without a program).
#[derive(Debug)]
pub struct UninitStateMachine<SM: ValidStateMachine> {
    block: *const RegisterBlock,
    sm: *const rp2040_pac::pio0::SM,
    _phantom: core::marker::PhantomData<SM>,
}

// Safety: `UninitStateMachine` only uses atomic accesses to shared registers.
unsafe impl<SM: ValidStateMachine + Send> Send for UninitStateMachine<SM> {}

// Safety: `UninitStateMachine` is marked Send so ensure all accesses remain atomic and no new
// concurrent accesses are added.
impl<SM: ValidStateMachine> UninitStateMachine<SM> {
    /// Start and stop the state machine.
    fn set_enabled(&mut self, enabled: bool) {
        // Bits 3:0 are SM_ENABLE.
        let mask = 1 << SM::id();
        if enabled {
            self.set_ctrl_bits(mask);
        } else {
            self.clear_ctrl_bits(mask);
        }
    }

    fn restart(&mut self) {
        // Bits 7:4 are SM_RESTART.
        self.set_ctrl_bits(1 << (SM::id() + 4));
    }

    fn reset_clock(&mut self) {
        // Bits 11:8 are CLKDIV_RESTART.
        self.set_ctrl_bits(1 << (SM::id() + 8));
    }

    // Safety: All ctrl set access should go through this function to ensure atomic access.
    fn set_ctrl_bits(&mut self, bits: u32) {
        // Safety: We only use the atomic alias of the register.
        unsafe {
            write_bitmask_set((*self.block).ctrl.as_ptr(), bits);
        }
    }

    // Safety: All ctrl clear access should go through this function to ensure atomic access.
    fn clear_ctrl_bits(&mut self, bits: u32) {
        // Safety: We only use the atomic alias of the register.
        unsafe {
            write_bitmask_clear((*self.block).ctrl.as_ptr(), bits);
        }
    }

    // Safety: The Send trait assumes this is the only write to sm_clkdiv
    fn set_clock_divisor(&self, int: u16, frac: u8) {
        // Safety: This is the only write to this register
        unsafe {
            self.sm()
                .sm_clkdiv
                .write(|w| w.int().bits(int).frac().bits(frac));
        }
    }

    unsafe fn sm(&self) -> &rp2040_pac::pio0::SM {
        &*self.sm
    }

    unsafe fn pio(&self) -> &RegisterBlock {
        &*self.block
    }
}

/// PIO State Machine with an associated program.
pub struct StateMachine<SM: ValidStateMachine, State> {
    sm: UninitStateMachine<SM>,
    program: InstalledProgram<SM::PIO>,
    _phantom: core::marker::PhantomData<State>,
}

/// Marker for an initialized, but stopped state machine.
pub struct Stopped;
/// Marker for an initialized and running state machine.
pub struct Running;

/// Id for the PIO's IRQ
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum PioIRQ {
    #[allow(missing_docs)]
    Irq0,
    #[allow(missing_docs)]
    Irq1,
}
impl PioIRQ {
    const fn to_index(self) -> usize {
        match self {
            PioIRQ::Irq0 => 0,
            PioIRQ::Irq1 => 1,
        }
    }
}

impl<SM: ValidStateMachine, State> StateMachine<SM, State> {
    /// Stops the state machine if it is still running and returns its program.
    ///
    /// The program can be uninstalled to free space once it is no longer used by any state
    /// machine.
    pub fn uninit(
        mut self,
        _rx: Rx<SM>,
        _tx: Tx<SM>,
    ) -> (UninitStateMachine<SM>, InstalledProgram<SM::PIO>) {
        self.sm.set_enabled(false);
        (self.sm, self.program)
    }

    /// The address of the instruction currently being executed.
    pub fn instruction_address(&self) -> u32 {
        // Safety: Read only access without side effect
        unsafe { self.sm.sm().sm_addr.read().bits() }
    }

    #[deprecated(note = "Renamed to exec_instruction")]
    /// Execute the instruction immediately.
    pub fn set_instruction(&mut self, instruction: u16) {
        let instruction =
            Instruction::decode(instruction, self.program.side_set).expect("Invalid instruction");
        self.exec_instruction(instruction);
    }

    /// Execute the instruction immediately.
    ///
    /// If an instruction written to INSTR stalls, it is stored in the same instruction latch used
    /// by OUT EXEC and MOV EXEC, and will overwrite an in-progress instruction there. If EXEC
    /// instructions are used, instructions written to INSTR must not stall.
    pub fn exec_instruction(&mut self, instruction: Instruction) {
        let instruction = instruction.encode(self.program.side_set);

        // Safety: all accesses to this register are controlled by this instance
        unsafe {
            self.sm
                .sm()
                .sm_instr
                .write(|w| w.sm0_instr().bits(instruction))
        }
    }

    /// Check if the current instruction is stalled.
    pub fn stalled(&self) -> bool {
        // Safety: read only access without side effect
        unsafe { self.sm.sm().sm_execctrl.read().exec_stalled().bit() }
    }

    /// Drain Tx fifo.
    pub fn drain_tx_fifo(&mut self) {
        // According to the datasheet 3.5.4.2 Page 358:
        //
        // When autopull is enabled, the behaviour of 'PULL'  is  altered:  it  becomes  a  no-op
        // if  the  OSR  is  full.  This  is  to  avoid  a  race  condition  against  the  system
        // DMA.  It behaves as a fence: either an autopull has already taken place, in which case
        // the 'PULL' has no effect, or the program will stall on the 'PULL' until data becomes
        // available in the FIFO.

        // TODO: encode at compile time once pio 0.3.0 is out
        const OUT: InstructionOperands = InstructionOperands::OUT {
            destination: pio::OutDestination::NULL,
            bit_count: 32,
        };
        const PULL: InstructionOperands = InstructionOperands::PULL {
            if_empty: false,
            block: false,
        };

        // Safety: all accesses to these registers are controlled by this instance
        unsafe {
            let sm = &self.sm.sm();
            let sm_pinctrl = &sm.sm_pinctrl;
            let sm_instr = &sm.sm_instr;
            let fstat = &self.sm.pio().fstat;

            let operands = if sm.sm_shiftctrl.read().autopull().bit_is_set() {
                OUT
            } else {
                PULL
            }
            .encode();

            // Safety: sm0_instr may be accessed from SM::exec_instruction.
            let mut saved_sideset_count = 0;
            sm_pinctrl.modify(|r, w| {
                saved_sideset_count = r.sideset_count().bits();
                w.sideset_count().bits(0)
            });

            let mask = 1 << SM::id();
            // white tx fifo is not empty
            while (fstat.read().txempty().bits() & mask) == 0 {
                sm_instr.write(|w| w.sm0_instr().bits(operands))
            }

            if saved_sideset_count != 0 {
                sm_pinctrl.modify(|_, w| w.sideset_count().bits(saved_sideset_count));
            }
        }
    }

    /// Change the clock divider of a state machine.
    ///
    /// Changing the clock divider of a running state machine is allowed
    /// and guaranteed to not cause any glitches, but the exact timing of
    /// clock pulses during the change is not specified.
    pub fn set_clock_divisor(&mut self, divisor: f32) {
        // sm frequency = clock freq / (CLKDIV_INT + CLKDIV_FRAC / 256)
        let int = divisor as u16;
        let frac = ((divisor - int as f32) * 256.0) as u8;

        self.sm.set_clock_divisor(int, frac);
    }

    /// Change the clock divider of a state machine using a 16.8 fixed point value.
    ///
    /// Changing the clock divider of a running state machine is allowed
    /// and guaranteed to not cause any glitches, but the exact timing of
    /// clock pulses during the change is not specified.
    pub fn clock_divisor_fixed_point(&mut self, int: u16, frac: u8) {
        self.sm.set_clock_divisor(int, frac);
    }
}

// Safety: All shared register accesses are atomic.
unsafe impl<SM: ValidStateMachine + Send, State> Send for StateMachine<SM, State> {}

// Safety: `StateMachine` is marked Send so ensure all accesses remain atomic and no new concurrent
// accesses are added.
impl<SM: ValidStateMachine> StateMachine<SM, Stopped> {
    /// Starts execution of the selected program.
    pub fn start(mut self) -> StateMachine<SM, Running> {
        // Enable SM
        self.sm.set_enabled(true);

        StateMachine {
            sm: self.sm,
            program: self.program,
            _phantom: core::marker::PhantomData,
        }
    }

    /// Sets the pin state for the specified pins.
    ///
    /// The user has to make sure that they do not select any pins that are in use by any
    /// other state machines of the same PIO block.
    ///
    /// The iterator's item are pairs of `(pin_number, pin_state)`.
    pub fn set_pins(&mut self, pins: impl IntoIterator<Item = (u8, PinState)>) {
        // TODO: turn those three into const once pio 0.3.0 is released
        let set_high_instr = InstructionOperands::SET {
            destination: pio::SetDestination::PINS,
            data: 1,
        }
        .encode();
        let set_low_instr = InstructionOperands::SET {
            destination: pio::SetDestination::PINS,
            data: 0,
        }
        .encode();

        // Safety: all accesses to these registers are controlled by this instance
        unsafe {
            let sm = self.sm.sm();
            let sm_pinctrl = &sm.sm_pinctrl;
            let sm_execctrl = &sm.sm_execctrl;
            let sm_instr = &sm.sm_instr;

            // sideset_count is implicitly set to 0 when the set_base/set_count are written (rather
            // than modified)
            let saved_pin_ctrl = sm_pinctrl.read().bits();
            let mut saved_execctrl = 0;

            sm_execctrl.modify(|r, w| {
                saved_execctrl = r.bits();
                w.out_sticky().clear_bit()
            });

            for (pin_num, pin_state) in pins {
                sm_pinctrl.write(|w| w.set_base().bits(pin_num).set_count().bits(1));
                let instruction = if pin_state == PinState::High {
                    set_high_instr
                } else {
                    set_low_instr
                };

                sm_instr.write(|w| w.sm0_instr().bits(instruction))
            }

            sm_pinctrl.write(|w| w.bits(saved_pin_ctrl));
            sm_execctrl.write(|w| w.bits(saved_execctrl));
        }
    }

    /// Set pin directions.
    ///
    /// The user has to make sure that they do not select any pins that are in use by any
    /// other state machines of the same PIO block.
    ///
    /// The iterator's item are pairs of `(pin_number, pin_dir)`.
    pub fn set_pindirs(&mut self, pindirs: impl IntoIterator<Item = (u8, PinDir)>) {
        // TODO: turn those three into const once pio 0.3.0 is released
        let set_output_instr = InstructionOperands::SET {
            destination: pio::SetDestination::PINDIRS,
            data: 1,
        }
        .encode();
        let set_input_instr = InstructionOperands::SET {
            destination: pio::SetDestination::PINDIRS,
            data: 0,
        }
        .encode();

        // Safety: all accesses to these registers are controlled by this instance
        unsafe {
            let sm = self.sm.sm();
            let sm_pinctrl = &sm.sm_pinctrl;
            let sm_execctrl = &sm.sm_execctrl;
            let sm_instr = &sm.sm_instr;

            // sideset_count is implicitly set to 0 when the set_base/set_count are written (rather
            // than modified)
            let saved_pin_ctrl = sm_pinctrl.read().bits();
            let mut saved_execctrl = 0;

            sm_execctrl.modify(|r, w| {
                saved_execctrl = r.bits();
                w.out_sticky().clear_bit()
            });

            for (pin_num, pin_dir) in pindirs {
                sm_pinctrl.write(|w| w.set_base().bits(pin_num).set_count().bits(1));
                let instruction = if pin_dir == PinDir::Output {
                    set_output_instr
                } else {
                    set_input_instr
                };

                sm_instr.write(|w| w.sm0_instr().bits(instruction))
            }

            sm_pinctrl.write(|w| w.bits(saved_pin_ctrl));
            sm_execctrl.write(|w| w.bits(saved_execctrl));
        }
    }
}

impl<P: PIOExt, SM: StateMachineIndex> StateMachine<(P, SM), Stopped> {
    /// Restarts the clock dividers for the specified state machines.
    ///
    /// As a result, the clock will be synchronous for the state machines, which is a precondition
    /// for synchronous operation.
    ///
    /// The function returns an object that, once destructed, restarts the clock dividers. This
    /// object allows further state machines to be added if more than two shall be synchronized.
    ///
    /// # Example
    ///
    /// ```ignore
    /// sm0.synchronize_with(sm1).and_with(sm2);
    /// ```
    pub fn synchronize_with<'sm, SM2: StateMachineIndex>(
        &'sm mut self,
        _other_sm: &'sm mut StateMachine<(P, SM2), Stopped>,
    ) -> Synchronize<'sm, (P, SM)> {
        let sm_mask = (1 << SM::id()) | (1 << SM2::id());
        Synchronize { sm: self, sm_mask }
    }
}

impl<P: PIOExt, SM: StateMachineIndex, State> StateMachine<(P, SM), State> {
    /// Create a group of state machines, which can be started/stopped synchronously
    pub fn with<SM2: StateMachineIndex>(
        self,
        other_sm: StateMachine<(P, SM2), State>,
    ) -> StateMachineGroup2<P, SM, SM2, State> {
        StateMachineGroup2 {
            sm1: self,
            sm2: other_sm,
        }
    }
}

/// Group of 2 state machines, which can be started/stopped synchronously.
pub struct StateMachineGroup2<
    P: PIOExt,
    SM1Idx: StateMachineIndex,
    SM2Idx: StateMachineIndex,
    State,
> {
    sm1: StateMachine<(P, SM1Idx), State>,
    sm2: StateMachine<(P, SM2Idx), State>,
}

/// Group of 3 state machines, which can be started/stopped synchronously.
pub struct StateMachineGroup3<
    P: PIOExt,
    SM1Idx: StateMachineIndex,
    SM2Idx: StateMachineIndex,
    SM3Idx: StateMachineIndex,
    State,
> {
    sm1: StateMachine<(P, SM1Idx), State>,
    sm2: StateMachine<(P, SM2Idx), State>,
    sm3: StateMachine<(P, SM3Idx), State>,
}

/// Group of 4 state machines, which can be started/stopped synchronously.
pub struct StateMachineGroup4<
    P: PIOExt,
    SM1Idx: StateMachineIndex,
    SM2Idx: StateMachineIndex,
    SM3Idx: StateMachineIndex,
    SM4Idx: StateMachineIndex,
    State,
> {
    sm1: StateMachine<(P, SM1Idx), State>,
    sm2: StateMachine<(P, SM2Idx), State>,
    sm3: StateMachine<(P, SM3Idx), State>,
    sm4: StateMachine<(P, SM4Idx), State>,
}

impl<P: PIOExt, SM1Idx: StateMachineIndex, SM2Idx: StateMachineIndex, State>
    StateMachineGroup2<P, SM1Idx, SM2Idx, State>
{
    /// Split the group, releasing the contained state machines
    #[allow(clippy::type_complexity)]
    pub fn free(
        self,
    ) -> (
        StateMachine<(P, SM1Idx), State>,
        StateMachine<(P, SM2Idx), State>,
    ) {
        (self.sm1, self.sm2)
    }

    /// Add another state machine to the group
    pub fn with<SM3Idx: StateMachineIndex>(
        self,
        other_sm: StateMachine<(P, SM3Idx), State>,
    ) -> StateMachineGroup3<P, SM1Idx, SM2Idx, SM3Idx, State> {
        StateMachineGroup3 {
            sm1: self.sm1,
            sm2: self.sm2,
            sm3: other_sm,
        }
    }

    fn mask(&self) -> u32 {
        (1 << SM1Idx::id()) | (1 << SM2Idx::id())
    }
}

impl<
        P: PIOExt,
        SM1Idx: StateMachineIndex,
        SM2Idx: StateMachineIndex,
        SM3Idx: StateMachineIndex,
        State,
    > StateMachineGroup3<P, SM1Idx, SM2Idx, SM3Idx, State>
{
    /// Split the group, releasing the contained state machines
    #[allow(clippy::type_complexity)]
    pub fn free(
        self,
    ) -> (
        StateMachine<(P, SM1Idx), State>,
        StateMachine<(P, SM2Idx), State>,
        StateMachine<(P, SM3Idx), State>,
    ) {
        (self.sm1, self.sm2, self.sm3)
    }

    /// Add another state machine to the group
    pub fn with<SM4Idx: StateMachineIndex>(
        self,
        other_sm: StateMachine<(P, SM4Idx), State>,
    ) -> StateMachineGroup4<P, SM1Idx, SM2Idx, SM3Idx, SM4Idx, State> {
        StateMachineGroup4 {
            sm1: self.sm1,
            sm2: self.sm2,
            sm3: self.sm3,
            sm4: other_sm,
        }
    }

    fn mask(&self) -> u32 {
        (1 << SM1Idx::id()) | (1 << SM2Idx::id()) | (1 << SM3Idx::id())
    }
}

impl<
        P: PIOExt,
        SM1Idx: StateMachineIndex,
        SM2Idx: StateMachineIndex,
        SM3Idx: StateMachineIndex,
        SM4Idx: StateMachineIndex,
        State,
    > StateMachineGroup4<P, SM1Idx, SM2Idx, SM3Idx, SM4Idx, State>
{
    /// Split the group, releasing the contained state machines
    #[allow(clippy::type_complexity)]
    pub fn free(
        self,
    ) -> (
        StateMachine<(P, SM1Idx), State>,
        StateMachine<(P, SM2Idx), State>,
        StateMachine<(P, SM3Idx), State>,
        StateMachine<(P, SM4Idx), State>,
    ) {
        (self.sm1, self.sm2, self.sm3, self.sm4)
    }

    fn mask(&self) -> u32 {
        (1 << SM1Idx::id()) | (1 << SM2Idx::id()) | (1 << SM3Idx::id()) | (1 << SM4Idx::id())
    }
}

impl<P: PIOExt, SM1Idx: StateMachineIndex, SM2Idx: StateMachineIndex>
    StateMachineGroup2<P, SM1Idx, SM2Idx, Stopped>
{
    /// Start grouped state machines
    pub fn start(mut self) -> StateMachineGroup2<P, SM1Idx, SM2Idx, Running> {
        self.sm1.sm.set_ctrl_bits(self.mask());
        StateMachineGroup2 {
            sm1: StateMachine {
                sm: self.sm1.sm,
                program: self.sm1.program,
                _phantom: core::marker::PhantomData,
            },
            sm2: StateMachine {
                sm: self.sm2.sm,
                program: self.sm2.program,
                _phantom: core::marker::PhantomData,
            },
        }
    }

    /// Sync grouped state machines
    pub fn sync(mut self) -> Self {
        self.sm1.sm.set_ctrl_bits(self.mask() << 8);
        self
    }
}

impl<
        P: PIOExt,
        SM1Idx: StateMachineIndex,
        SM2Idx: StateMachineIndex,
        SM3Idx: StateMachineIndex,
    > StateMachineGroup3<P, SM1Idx, SM2Idx, SM3Idx, Stopped>
{
    /// Start grouped state machines
    pub fn start(mut self) -> StateMachineGroup3<P, SM1Idx, SM2Idx, SM3Idx, Running> {
        self.sm1.sm.set_ctrl_bits(self.mask());
        StateMachineGroup3 {
            sm1: StateMachine {
                sm: self.sm1.sm,
                program: self.sm1.program,
                _phantom: core::marker::PhantomData,
            },
            sm2: StateMachine {
                sm: self.sm2.sm,
                program: self.sm2.program,
                _phantom: core::marker::PhantomData,
            },
            sm3: StateMachine {
                sm: self.sm3.sm,
                program: self.sm3.program,
                _phantom: core::marker::PhantomData,
            },
        }
    }

    /// Sync grouped state machines
    pub fn sync(mut self) -> Self {
        self.sm1.sm.set_ctrl_bits(self.mask() << 8);
        self
    }
}

impl<
        P: PIOExt,
        SM1Idx: StateMachineIndex,
        SM2Idx: StateMachineIndex,
        SM3Idx: StateMachineIndex,
        SM4Idx: StateMachineIndex,
    > StateMachineGroup4<P, SM1Idx, SM2Idx, SM3Idx, SM4Idx, Stopped>
{
    /// Start grouped state machines
    pub fn start(mut self) -> StateMachineGroup4<P, SM1Idx, SM2Idx, SM3Idx, SM4Idx, Running> {
        self.sm1.sm.set_ctrl_bits(self.mask());
        StateMachineGroup4 {
            sm1: StateMachine {
                sm: self.sm1.sm,
                program: self.sm1.program,
                _phantom: core::marker::PhantomData,
            },
            sm2: StateMachine {
                sm: self.sm2.sm,
                program: self.sm2.program,
                _phantom: core::marker::PhantomData,
            },
            sm3: StateMachine {
                sm: self.sm3.sm,
                program: self.sm3.program,
                _phantom: core::marker::PhantomData,
            },
            sm4: StateMachine {
                sm: self.sm4.sm,
                program: self.sm4.program,
                _phantom: core::marker::PhantomData,
            },
        }
    }

    /// Sync grouped state machines
    pub fn sync(mut self) -> Self {
        self.sm1.sm.set_ctrl_bits(self.mask() << 8);
        self
    }
}

impl<P: PIOExt, SM1Idx: StateMachineIndex, SM2Idx: StateMachineIndex>
    StateMachineGroup2<P, SM1Idx, SM2Idx, Running>
{
    /// Stop grouped state machines
    pub fn stop(mut self) -> StateMachineGroup2<P, SM1Idx, SM2Idx, Stopped> {
        self.sm1.sm.clear_ctrl_bits(self.mask());
        StateMachineGroup2 {
            sm1: StateMachine {
                sm: self.sm1.sm,
                program: self.sm1.program,
                _phantom: core::marker::PhantomData,
            },
            sm2: StateMachine {
                sm: self.sm2.sm,
                program: self.sm2.program,
                _phantom: core::marker::PhantomData,
            },
        }
    }
}

impl<
        P: PIOExt,
        SM1Idx: StateMachineIndex,
        SM2Idx: StateMachineIndex,
        SM3Idx: StateMachineIndex,
    > StateMachineGroup3<P, SM1Idx, SM2Idx, SM3Idx, Running>
{
    /// Stop grouped state machines
    pub fn stop(mut self) -> StateMachineGroup3<P, SM1Idx, SM2Idx, SM3Idx, Stopped> {
        self.sm1.sm.clear_ctrl_bits(self.mask());
        StateMachineGroup3 {
            sm1: StateMachine {
                sm: self.sm1.sm,
                program: self.sm1.program,
                _phantom: core::marker::PhantomData,
            },
            sm2: StateMachine {
                sm: self.sm2.sm,
                program: self.sm2.program,
                _phantom: core::marker::PhantomData,
            },
            sm3: StateMachine {
                sm: self.sm3.sm,
                program: self.sm3.program,
                _phantom: core::marker::PhantomData,
            },
        }
    }
}

impl<
        P: PIOExt,
        SM1Idx: StateMachineIndex,
        SM2Idx: StateMachineIndex,
        SM3Idx: StateMachineIndex,
        SM4Idx: StateMachineIndex,
    > StateMachineGroup4<P, SM1Idx, SM2Idx, SM3Idx, SM4Idx, Running>
{
    /// Stop grouped state machines
    pub fn stop(mut self) -> StateMachineGroup4<P, SM1Idx, SM2Idx, SM3Idx, SM4Idx, Stopped> {
        self.sm1.sm.clear_ctrl_bits(self.mask());
        StateMachineGroup4 {
            sm1: StateMachine {
                sm: self.sm1.sm,
                program: self.sm1.program,
                _phantom: core::marker::PhantomData,
            },
            sm2: StateMachine {
                sm: self.sm2.sm,
                program: self.sm2.program,
                _phantom: core::marker::PhantomData,
            },
            sm3: StateMachine {
                sm: self.sm3.sm,
                program: self.sm3.program,
                _phantom: core::marker::PhantomData,
            },
            sm4: StateMachine {
                sm: self.sm4.sm,
                program: self.sm4.program,
                _phantom: core::marker::PhantomData,
            },
        }
    }

    /// Sync grouped state machines
    pub fn sync(mut self) -> Self {
        self.sm1.sm.set_ctrl_bits(self.mask() << 8);
        self
    }
}

/// Type which, once destructed, restarts the clock dividers for all selected state machines,
/// effectively synchronizing them.
pub struct Synchronize<'sm, SM: ValidStateMachine> {
    sm: &'sm mut StateMachine<SM, Stopped>,
    sm_mask: u32,
}

impl<'sm, P: PIOExt, SM: StateMachineIndex> Synchronize<'sm, (P, SM)> {
    /// Adds another state machine to be synchronized.
    pub fn and_with<SM2: StateMachineIndex>(
        mut self,
        _other_sm: &'sm mut StateMachine<(P, SM2), Stopped>,
    ) -> Self {
        // Add another state machine index to the mask.
        self.sm_mask |= 1 << SM2::id();
        self
    }
}

impl<'sm, SM: ValidStateMachine> Drop for Synchronize<'sm, SM> {
    fn drop(&mut self) {
        // Restart the clocks of all state machines specified by the mask.
        // Bits 11:8 of CTRL contain CLKDIV_RESTART.
        let sm_mask = self.sm_mask << 8;
        self.sm.sm.set_ctrl_bits(sm_mask);
    }
}

impl<SM: ValidStateMachine> StateMachine<SM, Running> {
    /// Stops execution of the selected program.
    pub fn stop(mut self) -> StateMachine<SM, Stopped> {
        // Enable SM
        self.sm.set_enabled(false);

        StateMachine {
            sm: self.sm,
            program: self.program,
            _phantom: core::marker::PhantomData,
        }
    }

    /// Restarts the execution of the selected program from its wrap target.
    pub fn restart(&mut self) {
        // pause the state machine
        self.sm.set_enabled(false);

        // Safety: all accesses to these registers are controlled by this instance
        unsafe {
            let sm = self.sm.sm();
            let sm_pinctrl = &sm.sm_pinctrl;
            let sm_instr = &sm.sm_instr;

            // save exec_ctrl & make side_set optional
            let mut saved_sideset_count = 0;
            sm_pinctrl.modify(|r, w| {
                saved_sideset_count = r.sideset_count().bits();
                w.sideset_count().bits(0)
            });

            // revert it to its wrap target
            let instruction = InstructionOperands::JMP {
                condition: pio::JmpCondition::Always,
                address: self.program.wrap_target(),
            }
            .encode();
            sm_instr.write(|w| w.sm0_instr().bits(instruction));

            // restore exec_ctrl
            if saved_sideset_count != 0 {
                sm_pinctrl.modify(|_, w| w.sideset_count().bits(saved_sideset_count));
            }

            // clear osr/isr
            self.sm.restart();
        }

        // unpause the state machine
        self.sm.set_enabled(true);
    }
}

/// PIO RX FIFO handle.
pub struct Rx<SM: ValidStateMachine> {
    block: *const RegisterBlock,
    _phantom: core::marker::PhantomData<SM>,
}

// Safety: All shared register accesses are atomic.
unsafe impl<SM: ValidStateMachine + Send> Send for Rx<SM> {}

// Safety: `Rx` is marked Send so ensure all accesses remain atomic and no new concurrent accesses
// are added.
impl<SM: ValidStateMachine> Rx<SM> {
    unsafe fn block(&self) -> &pac::pio0::RegisterBlock {
        &*self.block
    }

    /// Gets the FIFO's address.
    ///
    /// This is useful if you want to DMA from this peripheral.
    ///
    /// NB: You are responsible for using the pointer correctly and not
    /// underflowing the buffer.
    pub fn fifo_address(&self) -> *const u32 {
        // Safety: returning the address is safe as such. The user is responsible for any
        // dereference ops at that address.
        unsafe { self.block().rxf[SM::id()].as_ptr() }
    }

    /// Gets the FIFO's `DREQ` value.
    ///
    /// This is a value between 0 and 39. Each FIFO on each state machine on
    /// each PIO has a unique value.
    pub fn dreq_value(&self) -> u8 {
        if self.block as usize == 0x5020_0000usize {
            TREQ_SEL_A::PIO0_RX0 as u8 + (SM::id() as u8)
        } else {
            TREQ_SEL_A::PIO1_RX0 as u8 + (SM::id() as u8)
        }
    }

    /// Get the next element from RX FIFO.
    ///
    /// Returns `None` if the FIFO is empty.
    pub fn read(&mut self) -> Option<u32> {
        if self.is_empty() {
            return None;
        }

        // Safety: The register is unique to this Rx instance.
        Some(unsafe { core::ptr::read_volatile(self.fifo_address()) })
    }

    /// Enable/Disable the autopush feature of the state machine.
    // Safety: This register is read by Rx, this is the only write.
    pub fn enable_autopush(&mut self, enable: bool) {
        // Safety: only instance reading/writing to autopush bit and no other write to this
        // register
        unsafe {
            self.block().sm[SM::id()]
                .sm_shiftctrl
                .modify(|_, w| w.autopush().bit(enable))
        }
    }

    /// Indicate if the rx FIFO is empty
    pub fn is_empty(&self) -> bool {
        // Safety: Read only access without side effect
        unsafe { self.block().fstat.read().rxempty().bits() & (1 << SM::id()) != 0 }
    }

    /// Indicate if the rx FIFO is full
    pub fn is_full(&self) -> bool {
        // Safety: Read only access without side effect
        unsafe { self.block().fstat.read().rxfull().bits() & (1 << SM::id()) != 0 }
    }

    /// Enable RX FIFO not empty interrupt.
    ///
    /// This interrupt is raised when the RX FIFO is not empty, i.e. one could read more data from it.
    pub fn enable_rx_not_empty_interrupt(&self, id: PioIRQ) {
        // Safety: Atomic write to a single bit owned by this instance
        unsafe {
            write_bitmask_set(
                self.block().sm_irq[id.to_index()].irq_inte.as_ptr(),
                1 << SM::id(),
            );
        }
    }

    /// Disable RX FIFO not empty interrupt.
    pub fn disable_rx_not_empty_interrupt(&self, id: PioIRQ) {
        // Safety: Atomic write to a single bit owned by this instance
        unsafe {
            write_bitmask_clear(
                self.block().sm_irq[id.to_index()].irq_inte.as_ptr(),
                1 << SM::id(),
            );
        }
    }

    /// Force RX FIFO not empty interrupt.
    pub fn force_rx_not_empty_interrupt(&self, id: PioIRQ, state: bool) {
        let action = if state {
            write_bitmask_set
        } else {
            write_bitmask_clear
        };
        // Safety: Atomic write to a single bit owned by this instance
        unsafe {
            action(
                self.block().sm_irq[id.to_index()].irq_intf.as_ptr(),
                1 << SM::id(),
            );
        }
    }
}

impl<SM: ValidStateMachine> ReadTarget for Rx<SM> {
    type ReceivedWord = u32;

    fn rx_treq() -> Option<u8> {
        Some(SM::rx_dreq())
    }

    fn rx_address_count(&self) -> (u32, u32) {
        (
            &unsafe { &*self.block }.rxf[SM::id()] as *const _ as u32,
            u32::MAX,
        )
    }

    fn rx_increment(&self) -> bool {
        false
    }
}

impl<SM: ValidStateMachine> EndlessReadTarget for Rx<SM> {}

/// PIO TX FIFO handle.
pub struct Tx<SM: ValidStateMachine> {
    block: *const RegisterBlock,
    _phantom: core::marker::PhantomData<SM>,
}

// Safety: All shared register accesses are atomic.
unsafe impl<SM: ValidStateMachine + Send> Send for Tx<SM> {}

// Safety: `Tx` is marked Send so ensure all accesses remain atomic and no new concurrent accesses
// are added.
impl<SM: ValidStateMachine> Tx<SM> {
    unsafe fn block(&self) -> &pac::pio0::RegisterBlock {
        &*self.block
    }

    fn write_generic<T>(&mut self, value: T) -> bool {
        if !self.is_full() {
            // Safety: Only accessed by this instance (unless DMA is used).
            unsafe {
                let reg_ptr = self.fifo_address() as *mut T;
                reg_ptr.write_volatile(value);
            }
            true
        } else {
            false
        }
    }

    /// Gets the FIFO's address.
    ///
    /// This is useful if you want to DMA to this peripheral.
    ///
    /// NB: You are responsible for using the pointer correctly and not
    /// overflowing the buffer.
    pub fn fifo_address(&self) -> *const u32 {
        // Safety: The only access to this register
        unsafe { self.block().txf[SM::id()].as_ptr() }
    }

    /// Gets the FIFO's `DREQ` value.
    ///
    /// This is a value between 0 and 39. Each FIFO on each state machine on
    /// each PIO has a unique value.
    pub fn dreq_value(&self) -> u8 {
        if self.block as usize == 0x5020_0000usize {
            TREQ_SEL_A::PIO0_TX0 as u8 + (SM::id() as u8)
        } else {
            TREQ_SEL_A::PIO1_TX0 as u8 + (SM::id() as u8)
        }
    }

    /// Write a u32 value to TX FIFO.
    ///
    /// Returns `true` if the value was written to FIFO, `false` otherwise.
    pub fn write(&mut self, value: u32) -> bool {
        self.write_generic(value)
    }

    /// Write a replicated u8 value to TX FIFO.
    ///
    /// Memory mapped register writes that are smaller than 32bits will trigger
    /// "Narrow IO Register Write" behaviour in RP2040 - the value written will
    /// be replicated to the rest of the register as described in
    /// [RP2040 Datasheet: 2.1.4. - Narrow IO Register Writes][section_2_1_4]
    ///
    ///
    /// This 8bit write will set all 4 bytes of the FIFO to `value`
    /// Eg: if you write `0xBA` the value written to the the FIFO will be
    /// `0xBABABABA`
    ///
    /// If you wish to write an 8bit number without replication,
    /// use `write(my_u8 as u32)` instead.
    ///
    /// Returns `true` if the value was written to FIFO, `false` otherwise.
    ///
    /// [section_2_1_4]: <https://datasheets.raspberrypi.com/rp2040/rp2040-datasheet.pdf#_narrow_io_register_writes>
    pub fn write_u8_replicated(&mut self, value: u8) -> bool {
        self.write_generic(value)
    }

    /// Write a replicated 16bit value to TX FIFO.
    ///
    /// Memory mapped register writes that are smaller than 32bits will trigger
    /// "Narrow IO Register Write" behaviour in RP2040 - the value written will
    /// be replicated to the rest of the register as described in
    /// [RP2040 Datasheet: 2.1.4. - Narrow IO Register Writes][section_2_1_4]
    ///
    /// This 16bit write will set both the upper and lower half of the FIFO entry to `value`.
    ///
    /// For example, if you write `0xC0DA` the value written to the FIFO will be
    /// `0xC0DAC0DA`
    ///
    /// If you wish to write a 16bit number without replication,
    /// use `write(my_u16 as u32)` instead.
    ///
    /// Returns `true` if the value was written to FIFO, `false` otherwise.
    ///
    /// [section_2_1_4]: <https://datasheets.raspberrypi.com/rp2040/rp2040-datasheet.pdf#_narrow_io_register_writes>
    pub fn write_u16_replicated(&mut self, value: u16) -> bool {
        self.write_generic(value)
    }

    /// Checks if the state machine has stalled on empty TX FIFO during a blocking PULL, or an OUT
    /// with autopull enabled.
    ///
    /// **Note this is a sticky flag and may not reflect the current state of the machine.**
    pub fn has_stalled(&self) -> bool {
        let mask = 1 << SM::id();
        // Safety: read-only access without side-effect
        unsafe { self.block().fdebug.read().txstall().bits() & mask == mask }
    }

    /// Clears the `tx_stalled` flag.
    pub fn clear_stalled_flag(&self) {
        let mask = 1 << SM::id();

        // Safety: These bits are WC, only the one corresponding to this SM is set.
        unsafe {
            self.block().fdebug.write(|w| w.txstall().bits(mask));
        }
    }

    /// Indicate if the tx FIFO is empty
    pub fn is_empty(&self) -> bool {
        // Safety: read-only access without side-effect
        unsafe { self.block().fstat.read().txempty().bits() & (1 << SM::id()) != 0 }
    }

    /// Indicate if the tx FIFO is full
    pub fn is_full(&self) -> bool {
        // Safety: read-only access without side-effect
        unsafe { self.block().fstat.read().txfull().bits() & (1 << SM::id()) != 0 }
    }

    /// Enable TX FIFO not full interrupt.
    ///
    /// This interrupt is raised when the TX FIFO is not full, i.e. one could push more data to it.
    pub fn enable_tx_not_full_interrupt(&self, id: PioIRQ) {
        // Safety: Atomic access to the register. Bit only modified by this Tx<SM>
        unsafe {
            write_bitmask_set(
                self.block().sm_irq[id.to_index()].irq_inte.as_ptr(),
                1 << (SM::id() + 4),
            );
        }
    }

    /// Disable TX FIFO not full interrupt.
    pub fn disable_tx_not_full_interrupt(&self, id: PioIRQ) {
        // Safety: Atomic access to the register. Bit only modified by this Tx<SM>
        unsafe {
            write_bitmask_clear(
                self.block().sm_irq[id.to_index()].irq_inte.as_ptr(),
                1 << (SM::id() + 4),
            );
        }
    }

    /// Force TX FIFO not full interrupt.
    pub fn force_tx_not_full_interrupt(&self, id: PioIRQ) {
        // Safety: Atomic access to the register. Bit only modified by this Tx<SM>
        unsafe {
            write_bitmask_set(
                self.block().sm_irq[id.to_index()].irq_intf.as_ptr(),
                1 << (SM::id() + 4),
            );
        }
    }
}

impl<SM: ValidStateMachine> WriteTarget for Tx<SM> {
    type TransmittedWord = u32;

    fn tx_treq() -> Option<u8> {
        Some(SM::tx_dreq())
    }

    fn tx_address_count(&mut self) -> (u32, u32) {
        (
            &unsafe { &*self.block }.txf[SM::id()] as *const _ as u32,
            u32::MAX,
        )
    }

    fn tx_increment(&self) -> bool {
        false
    }
}

impl<SM: ValidStateMachine> EndlessWriteTarget for Tx<SM> {}

/// PIO Interrupt controller.
#[derive(Debug)]
pub struct Interrupt<'a, P: PIOExt, const IRQ: usize> {
    block: *const RegisterBlock,
    _phantom: core::marker::PhantomData<&'a P>,
}

// Safety: `Interrupt` provides exclusive access to interrupt registers.
unsafe impl<'a, P: PIOExt, const IRQ: usize> Send for Interrupt<'a, P, IRQ> {}

// Safety: `Interrupt` is marked Send so ensure all accesses remain atomic and no new concurrent
// accesses are added.
// `Interrupt` provides exclusive access to `irq_intf` to `irq_inte` for it's state machine, this
// must remain true to satisfy Send.
impl<'a, P: PIOExt, const IRQ: usize> Interrupt<'a, P, IRQ> {
    /// Enable interrupts raised by state machines.
    ///
    /// The PIO peripheral has 4 outside visible interrupts that can be raised by the state machines. Note that this
    /// does not correspond with the state machine index; any state machine can raise any one of the four interrupts.
    pub fn enable_sm_interrupt(&self, id: u8) {
        assert!(id < 4, "invalid state machine interrupt number");
        // Safety: Atomic write to a single bit owned by this instance
        unsafe {
            write_bitmask_set(self.irq().irq_inte.as_ptr(), 1 << (id + 8));
        }
    }

    /// Disable interrupts raised by state machines.
    ///
    /// See [`Self::enable_sm_interrupt`] for info about the index.
    pub fn disable_sm_interrupt(&self, id: u8) {
        assert!(id < 4, "invalid state machine interrupt number");
        // Safety: Atomic write to a single bit owned by this instance
        unsafe {
            write_bitmask_clear(self.irq().irq_inte.as_ptr(), 1 << (id + 8));
        }
    }

    /// Force state machine interrupt.
    ///
    /// Note that this doesn't affect the state seen by the state machine. For that, see [`PIO::force_irq`].
    ///
    ///
    ///
    /// See [`Self::enable_sm_interrupt`] for info about the index.
    pub fn force_sm_interrupt(&self, id: u8, set: bool) {
        assert!(id < 4, "invalid state machine interrupt number");
        // Safety: Atomic write to a single bit owned by this instance
        unsafe {
            if set {
                write_bitmask_set(self.irq().irq_intf.as_ptr(), 1 << (id + 8));
            } else {
                write_bitmask_clear(self.irq().irq_intf.as_ptr(), 1 << (id + 8));
            }
        }
    }

    /// Enable TX FIFO not full interrupt.
    ///
    /// Each of the 4 state machines have their own TX FIFO. This interrupt is raised when the TX FIFO is not full, i.e.
    /// one could push more data to it.
    #[deprecated(
        since = "0.7.0",
        note = "Use the dedicated method on the state machine"
    )]
    pub fn enable_tx_not_full_interrupt(&self, id: u8) {
        assert!(id < 4, "invalid state machine interrupt number");
        // Safety: Atomic write to a single bit owned by this instance
        unsafe {
            write_bitmask_set(self.irq().irq_inte.as_ptr(), 1 << (id + 4));
        }
    }

    /// Disable TX FIFO not full interrupt.
    ///
    /// See [`Self::enable_tx_not_full_interrupt`] for info about the index.
    #[deprecated(
        since = "0.7.0",
        note = "Use the dedicated method on the state machine"
    )]
    pub fn disable_tx_not_full_interrupt(&self, id: u8) {
        assert!(id < 4, "invalid state machine interrupt number");
        // Safety: Atomic write to a single bit owned by this instance
        unsafe {
            write_bitmask_clear(self.irq().irq_inte.as_ptr(), 1 << (id + 4));
        }
    }

    /// Force TX FIFO not full interrupt.
    ///
    /// See [`Self::enable_tx_not_full_interrupt`] for info about the index.
    #[deprecated(
        since = "0.7.0",
        note = "Use the dedicated method on the state machine"
    )]
    pub fn force_tx_not_full_interrupt(&self, id: u8) {
        assert!(id < 4, "invalid state machine interrupt number");
        // Safety: Atomic write to a single bit owned by this instance
        unsafe {
            write_bitmask_set(self.irq().irq_intf.as_ptr(), 1 << (id + 4));
        }
    }

    /// Enable RX FIFO not empty interrupt.
    ///
    /// Each of the 4 state machines have their own RX FIFO. This interrupt is raised when the RX FIFO is not empty,
    /// i.e. one could read more data from it.
    #[deprecated(
        since = "0.7.0",
        note = "Use the dedicated method on the state machine"
    )]
    pub fn enable_rx_not_empty_interrupt(&self, id: u8) {
        assert!(id < 4, "invalid state machine interrupt number");
        // Safety: Atomic write to a single bit owned by this instance
        unsafe {
            write_bitmask_set(self.irq().irq_inte.as_ptr(), 1 << id);
        }
    }

    /// Disable RX FIFO not empty interrupt.
    ///
    /// See [`Self::enable_rx_not_empty_interrupt`] for info about the index.
    #[deprecated(
        since = "0.7.0",
        note = "Use the dedicated method on the state machine"
    )]
    pub fn disable_rx_not_empty_interrupt(&self, id: u8) {
        assert!(id < 4, "invalid state machine interrupt number");
        // Safety: Atomic write to a single bit owned by this instance
        unsafe {
            write_bitmask_clear(self.irq().irq_inte.as_ptr(), 1 << id);
        }
    }

    /// Force RX FIFO not empty interrupt.
    ///
    /// See [`Self::enable_rx_not_empty_interrupt`] for info about the index.
    #[deprecated(
        since = "0.7.0",
        note = "Use the dedicated method on the state machine"
    )]
    pub fn force_rx_not_empty_interrupt(&self, id: u8) {
        assert!(id < 4, "invalid state machine interrupt number");
        // Safety: Atomic write to a single bit owned by this instance
        unsafe {
            write_bitmask_set(self.irq().irq_intf.as_ptr(), 1 << id);
        }
    }

    /// Get the raw interrupt state.
    ///
    /// This is the state of the interrupts without interrupt masking and forcing.
    pub fn raw(&self) -> InterruptState {
        InterruptState(
            // Safety: Read only access without side effect
            unsafe { self.block().intr.read().bits() },
        )
    }

    /// Get the interrupt state.
    ///
    /// This is the state of the interrupts after interrupt masking and forcing.
    pub fn state(&self) -> InterruptState {
        InterruptState(
            // Safety: Read only access without side effect
            unsafe { self.irq().irq_ints.read().bits() },
        )
    }

    unsafe fn block(&self) -> &RegisterBlock {
        &*self.block
    }

    unsafe fn irq(&self) -> &rp2040_pac::pio0::SM_IRQ {
        &self.block().sm_irq[IRQ]
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
pub struct PIOBuilder<P> {
    /// Clock divisor.
    clock_divisor: (u16, u8),

    /// Program location and configuration.
    program: InstalledProgram<P>,
    /// GPIO pin used by `jmp pin` instruction.
    jmp_pin: u8,

    /// Continuously assert the most recent OUT/SET to the pins.
    out_sticky: bool,
    /// Use a bit of OUT data as an auxiliary write enable.
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
    /// Shift direction for `OUT` instruction.
    out_shiftdir: ShiftDirection,
    /// Shift direction for `IN` instruction.
    in_shiftdir: ShiftDirection,
    /// Enable autopull.
    autopull: bool,
    /// Enable autopush.
    autopush: bool,

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

/// Errors that occurred during `PIO::install`.
#[derive(Debug)]
pub enum InstallError {
    /// There was not enough space for the instructions on the selected PIO.
    NoSpace,
}

impl<P: PIOExt> PIOBuilder<P> {
    /// Set config settings based on information from the given [`pio::Program`].
    /// Additional configuration may be needed in addition to this.
    pub fn from_program(p: InstalledProgram<P>) -> Self {
        PIOBuilder {
            clock_divisor: (1, 0),
            program: p,
            jmp_pin: 0,
            out_sticky: false,
            inline_out: None,
            mov_status: MovStatusConfig::Tx(0),
            fifo_join: Buffers::RxTx,
            pull_threshold: 0,
            push_threshold: 0,
            out_shiftdir: ShiftDirection::Left,
            in_shiftdir: ShiftDirection::Left,
            autopull: false,
            autopush: false,
            set_count: 5,
            out_count: 0,
            in_base: 0,
            side_set_base: 0,
            set_base: 0,
            out_base: 0,
        }
    }

    /// Set the config for when the status register is set to true.
    ///
    /// See `MovStatusConfig` for more info.
    pub fn set_mov_status_config(mut self, mov_status: MovStatusConfig) -> Self {
        self.mov_status = mov_status;

        self
    }

    /// Set the pins asserted by `SET` instruction.
    ///
    /// The least-significant bit of `SET` instruction asserts the state of the pin indicated by `base`, the next bit
    /// asserts the state of the next pin, and so on up to `count` pins. The pin numbers are considered modulo 32.
    pub fn set_pins(mut self, base: u8, count: u8) -> Self {
        assert!(count <= 5);
        self.set_base = base;
        self.set_count = count;
        self
    }

    /// Set the pins asserted by `OUT` instruction.
    ///
    /// The least-significant bit of `OUT` instruction asserts the state of the pin indicated by `base`, the next bit
    /// asserts the state of the next pin, and so on up to `count` pins. The pin numbers are considered modulo 32.
    pub fn out_pins(mut self, base: u8, count: u8) -> Self {
        assert!(count <= 32);
        self.out_base = base;
        self.out_count = count;
        self
    }

    /// Set the pins used by `IN` instruction.
    ///
    /// The `IN` instruction reads the least significant bit from the pin indicated by `base`, the next bit from the
    /// next pin, and so on. The pin numbers are considered modulo 32.
    pub fn in_pin_base(mut self, base: u8) -> Self {
        self.in_base = base;
        self
    }

    /// Set the pin used by `JMP PIN` instruction.
    ///
    /// When the pin set by this function is high, the jump is taken, otherwise not.
    pub fn jmp_pin(mut self, pin: u8) -> Self {
        self.jmp_pin = pin;
        self
    }

    /// Set the pins used by side-set instructions.
    ///
    /// The least-significant side-set bit asserts the state of the pin indicated by `base`, the next bit asserts the
    /// state of the next pin, and so on up to [`pio::SideSet::bits()`] bits as configured in
    /// [`pio::Program`].
    pub fn side_set_pin_base(mut self, base: u8) -> Self {
        self.side_set_base = base;
        self
    }
    // TODO: Update documentation above.

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
    #[deprecated(
        since = "0.7.0",
        note = "Pulls in floating points. Use the fixed point alternative: clock_divisor_fixed_point"
    )]
    pub fn clock_divisor(mut self, divisor: f32) -> Self {
        self.clock_divisor = (divisor as u16, (divisor * 256.0) as u8);
        self
    }

    /// The clock is based on the `sys_clk` and will execute an instruction every `int + (frac/256)` ticks.
    ///
    /// A clock divisor of `n` will cause the state machine to run 1 cycle every `n` clock cycles. If the integer part
    /// is 0 then the fractional part must be 0. This is interpreted by the device as the integer 65536.
    ///
    /// For small values of `int`, a fractional divisor may introduce unacceptable jitter.
    pub fn clock_divisor_fixed_point(mut self, int: u16, frac: u8) -> Self {
        assert!(int != 0 || frac == 0);
        self.clock_divisor = (int, frac);
        self
    }

    /// Set the output sticky state.
    ///
    /// When the output is set to be sticky, the PIO hardware continuously asserts the most recent `OUT`/`SET` to the
    /// pins.
    pub fn out_sticky(mut self, out_sticky: bool) -> Self {
        self.out_sticky = out_sticky;
        self
    }

    /// Set the inline `OUT` enable bit.
    ///
    /// When set to value, the given bit of `OUT` instruction's data is used as an auxiliary write enable. When used
    /// with [`Self::out_sticky`], writes with enable 0 will deassert the latest pin write.
    pub fn inline_out(mut self, inline_out: Option<u8>) -> Self {
        self.inline_out = inline_out;
        self
    }

    /// Set the autopush state.
    ///
    /// When autopush is enabled, the `IN` instruction automatically pushes the data once the number of bits reaches
    /// threshold set by [`Self::push_threshold`].
    pub fn autopush(mut self, autopush: bool) -> Self {
        self.autopush = autopush;
        self
    }

    /// Set the number of bits pushed into ISR before autopush or conditional push will take place.
    pub fn push_threshold(mut self, threshold: u8) -> Self {
        self.push_threshold = threshold;
        self
    }

    /// Set the autopull state.
    ///
    /// When autopull is enabled, the `OUT` instruction automatically pulls the data once the number of bits reaches
    /// threshold set by [`Self::pull_threshold`].
    pub fn autopull(mut self, autopull: bool) -> Self {
        self.autopull = autopull;
        self
    }

    /// Set the number of bits pulled from out of OSR before autopull or conditional pull will take place.
    pub fn pull_threshold(mut self, threshold: u8) -> Self {
        self.pull_threshold = threshold;
        self
    }

    /// Set the ISR shift direction for `IN` instruction.
    ///
    /// For example `ShiftDirection::Right` means that ISR is shifted to right, i.e. data enters from left.
    pub fn in_shift_direction(mut self, direction: ShiftDirection) -> Self {
        self.in_shiftdir = direction;
        self
    }

    /// Set the OSR shift direction for `OUT` instruction.
    ///
    /// For example `ShiftDirection::Right` means that OSR is shifted to right, i.e. data is taken from the right side.
    pub fn out_shift_direction(mut self, direction: ShiftDirection) -> Self {
        self.out_shiftdir = direction;
        self
    }

    /// Build the config and deploy it to a StateMachine.
    #[allow(clippy::type_complexity)] // The return type cannot really be simplified.
    pub fn build<SM: StateMachineIndex>(
        self,
        mut sm: UninitStateMachine<(P, SM)>,
    ) -> (StateMachine<(P, SM), Stopped>, Rx<(P, SM)>, Tx<(P, SM)>) {
        let offset = self.program.offset;

        // Stop the SM
        sm.set_enabled(false);

        // Write all configuration bits
        sm.set_clock_divisor(self.clock_divisor.0, self.clock_divisor.1);

        // Safety: Only instance owning the SM
        unsafe {
            sm.sm().sm_execctrl.write(|w| {
                w.side_en().bit(self.program.side_set.optional());
                w.side_pindir().bit(self.program.side_set.pindirs());

                w.jmp_pin().bits(self.jmp_pin);

                if let Some(inline_out) = self.inline_out {
                    w.inline_out_en().bit(true);
                    w.out_en_sel().bits(inline_out);
                } else {
                    w.inline_out_en().bit(false);
                }

                w.out_sticky().bit(self.out_sticky);

                w.wrap_top().bits(offset + self.program.wrap.source);
                w.wrap_bottom().bits(offset + self.program.wrap.target);

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
                w.status_n().bits(n)
            });

            sm.sm().sm_shiftctrl.write(|w| {
                let (fjoin_rx, fjoin_tx) = match self.fifo_join {
                    Buffers::RxTx => (false, false),
                    Buffers::OnlyTx => (false, true),
                    Buffers::OnlyRx => (true, false),
                };
                w.fjoin_rx().bit(fjoin_rx);
                w.fjoin_tx().bit(fjoin_tx);

                // TODO: Encode 32 as zero, and error on 0
                w.pull_thresh().bits(self.pull_threshold);
                w.push_thresh().bits(self.push_threshold);

                w.out_shiftdir().bit(self.out_shiftdir.bit());
                w.in_shiftdir().bit(self.in_shiftdir.bit());

                w.autopull().bit(self.autopull);
                w.autopush().bit(self.autopush)
            });

            sm.sm().sm_pinctrl.write(|w| {
                w.sideset_count().bits(self.program.side_set.bits());
                w.set_count().bits(self.set_count);
                w.out_count().bits(self.out_count);

                w.in_base().bits(self.in_base);
                w.sideset_base().bits(self.side_set_base);
                w.set_base().bits(self.set_base);
                w.out_base().bits(self.out_base)
            })
        }

        // Restart SM and its clock
        sm.restart();
        sm.reset_clock();

        // Set starting location by forcing the state machine to execute a jmp
        // to the beginning of the program we loaded in.
        let instr = InstructionOperands::JMP {
            condition: pio::JmpCondition::Always,
            address: offset,
        }
        .encode();
        // Safety: Only instance owning the SM
        unsafe {
            sm.sm().sm_instr.write(|w| w.sm0_instr().bits(instr));
        }

        let rx = Rx {
            block: sm.block,
            _phantom: core::marker::PhantomData,
        };
        let tx = Tx {
            block: sm.block,
            _phantom: core::marker::PhantomData,
        };
        (
            StateMachine {
                sm,
                program: self.program,
                _phantom: core::marker::PhantomData,
            },
            rx,
            tx,
        )
    }
}
