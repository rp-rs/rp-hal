//! Programmable IO (PIO)
/// See [Chapter 3](https://rptl.io/pico-datasheet) for more details.
use crate::resets::SubsystemReset;
use pio::{Program, SideSet, Wrap};

const PIO_INSTRUCTION_COUNT: usize = 32;

/// PIO Instance
pub trait PIOExt:
    core::ops::Deref<Target = rp2040_pac::pio0::RegisterBlock> + SubsystemReset + Sized
{
    /// Create a new PIO wrapper and split the state machines into individual objects.
    fn split(
        self,
        resets: &mut pac::RESETS,
    ) -> (
        PIO<Self>,
        UninitStateMachine<Self>,
        UninitStateMachine<Self>,
        UninitStateMachine<Self>,
        UninitStateMachine<Self>,
    ) {
        self.reset_bring_up(resets);

        let sm0 = UninitStateMachine {
            id: 0,
            block: self.deref(),
            sm: &self.deref().sm[0],
            _phantom: core::marker::PhantomData,
        };
        let sm1 = UninitStateMachine {
            id: 0,
            block: self.deref(),
            sm: &self.deref().sm[0],
            _phantom: core::marker::PhantomData,
        };
        let sm2 = UninitStateMachine {
            id: 0,
            block: self.deref(),
            sm: &self.deref().sm[0],
            _phantom: core::marker::PhantomData,
        };
        let sm3 = UninitStateMachine {
            id: 0,
            block: self.deref(),
            sm: &self.deref().sm[0],
            _phantom: core::marker::PhantomData,
        };
        (
            PIO {
                used_instruction_space: 0,
                interrupts: [
                    Interrupt {
                        id: 0,
                        block: self.deref(),
                        _phantom: core::marker::PhantomData,
                    },
                    Interrupt {
                        id: 1,
                        block: self.deref(),
                        _phantom: core::marker::PhantomData,
                    },
                ],
                pio: self,
            },
            sm0,
            sm1,
            sm2,
            sm3,
        )
    }
}

impl PIOExt for rp2040_pac::PIO0 {}
impl PIOExt for rp2040_pac::PIO1 {}

/// Programmable IO Block
pub struct PIO<P: PIOExt> {
    used_instruction_space: u32, // bit for each PIO_INSTRUCTION_COUNT
    pio: P,
    interrupts: [Interrupt<P>; 2],
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
unsafe impl<P: PIOExt + Send> Send for PIO<P> {}

impl<P: PIOExt> PIO<P> {
    /// Free this instance.
    ///
    /// All output pins are left in their current state.
    pub fn free(
        self,
        _sm0: UninitStateMachine<P>,
        _sm1: UninitStateMachine<P>,
        _sm2: UninitStateMachine<P>,
        _sm3: UninitStateMachine<P>,
    ) -> P {
        // All state machines have already been stopped.
        self.pio
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
        self.pio.irq.write(|w| unsafe { w.irq().bits(flags) });
    }

    /// Force PIO's IRQ flags indicated by the bits.
    ///
    /// The PIO has 8 IRQ flags, of which 4 are visible to the host processor. Each bit of `flags` corresponds to one of
    /// the IRQ flags.
    pub fn force_irq(&self, flags: u8) {
        self.pio
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
                    || self.used_instruction_space & (mask << origin) != 0
                {
                    None
                } else {
                    Some(origin as usize)
                }
            } else {
                for i in (0..=32 - i.len()).rev() {
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
    pub fn install(
        &mut self,
        p: &Program<{ pio::RP2040_MAX_PROGRAM_SIZE }>,
    ) -> Result<InstalledProgram<P>, InstallError> {
        if let Some(offset) = self.find_offset_for_instructions(&p.code, p.origin) {
            for (i, instr) in p
                .code
                .iter()
                .map(|instr| {
                    let mut instr = pio::Instruction::decode(*instr, p.side_set).unwrap();

                    instr.operands = match instr.operands {
                        pio::InstructionOperands::JMP { condition, address } => {
                            // JMP instruction. We need to apply offset here
                            let address = address + offset as u8;
                            assert!(
                                address < pio::RP2040_MAX_PROGRAM_SIZE as u8,
                                "Invalid JMP out of the program after offset addition"
                            );
                            pio::InstructionOperands::JMP { condition, address }
                        }
                        _ => instr.operands,
                    };

                    instr.encode(p.side_set)
                })
                .enumerate()
            {
                self.pio.instr_mem[i + offset].write(|w| unsafe { w.bits(instr as u32) })
            }
            self.used_instruction_space =
                self.used_instruction_space | (((1 << p.code.len()) - 1) << offset);
            Ok(InstalledProgram {
                offset: offset as u8,
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
        let instr_mask = (1 << p.length as u32) << p.offset as u32;
        self.used_instruction_space = self.used_instruction_space & !instr_mask;
    }

    /// Restarts the clock dividers for the specified state machines.
    ///
    /// As a result, the clock will be synchronous for the state machines, which is a precondition
    /// for synchronous operation.
    pub fn synchronize(sm_set: &[&mut StateMachine<P, Stopped>]) {
        if sm_set.len() == 0 {
            return;
        }
        let mut sm_mask = 0;
        for sm in sm_set {
            // Bits 11:8 of CTRL contain CLKDIV_RESTART.
            sm_mask |= ((1 << sm.sm.id) as u32) << 8;
        }
        const ATOMIC_SET_OFFSET: usize = 0x2000;
        // Safety: We only use the atomic alias of the register.
        unsafe {
            *(*sm_set[0].sm.block)
                .ctrl
                .as_ptr()
                .add(ATOMIC_SET_OFFSET / 4) = sm_mask;
        }
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
/// ```
/// let (mut pio, sm0, _, _, _) = pac.PIO0.split(&mut pac.RESETS);
/// // Install a program in instruction memory.
/// let installed = pio.install(&program).unwrap();
/// // Configure a state machine to use the program.
/// let sm = rp2040_hal::pio::PIOBuilder::from_program(installed).build(sm0);
/// // Uninitialize the state machine again, freeing the program.
/// let (sm, installed) = sm.uninit();
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
/// ```
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

/// PIO State Machine (uninitialized, without a program).
#[derive(Debug)]
pub struct UninitStateMachine<P: PIOExt> {
    id: u8,
    block: *const rp2040_pac::pio0::RegisterBlock,
    sm: *const rp2040_pac::pio0::SM,
    _phantom: core::marker::PhantomData<P>,
}

// Safety: `UninitStateMachine` only uses atomic accesses to shared registers.
unsafe impl<P: PIOExt + Send> Send for UninitStateMachine<P> {}

impl<P: PIOExt> UninitStateMachine<P> {
    /// Start and stop the state machine.
    fn set_enabled(&mut self, enabled: bool) {
        // Bits 3:0 are SM_ENABLE.
        let mask = 1 << self.id;
        if enabled {
            self.set_ctrl_bits(mask);
        } else {
            self.clear_ctrl_bits(mask);
        }
    }

    fn restart(&mut self) {
        // Bits 7:4 are SM_RESTART.
        self.set_ctrl_bits(1 << (self.id + 4));
    }

    fn reset_clock(&mut self) {
        // Bits 11:8 are CLKDIV_RESTART.
        self.set_ctrl_bits(1 << (self.id + 8));
    }

    fn set_ctrl_bits(&mut self, bits: u32) {
        const ATOMIC_SET_OFFSET: usize = 0x2000;
        // Safety: We only use the atomic alias of the register.
        unsafe {
            *(*self.block).ctrl.as_ptr().add(ATOMIC_SET_OFFSET / 4) = bits;
        }
    }

    fn clear_ctrl_bits(&mut self, bits: u32) {
        const ATOMIC_CLEAR_OFFSET: usize = 0x3000;
        // Safety: We only use the atomic alias of the register.
        unsafe {
            *(*self.block).ctrl.as_ptr().add(ATOMIC_CLEAR_OFFSET / 4) = bits;
        }
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

    /// Set the current instruction.
    fn set_instruction(&mut self, instruction: u16) {
        self.sm()
            .sm_instr
            .write(|w| unsafe { w.sm0_instr().bits(instruction) })
    }

    fn sm(&self) -> &rp2040_pac::pio0::SM {
        unsafe { &*self.sm }
    }
}

/// PIO State Machine with an associated program.
pub struct StateMachine<P: PIOExt, State> {
    sm: UninitStateMachine<P>,
    program: InstalledProgram<P>,
    _phantom: core::marker::PhantomData<State>,
}

/// Marker for an initialized, but stopped state machine.
pub struct Stopped;
/// Marker for an initialized and running state machine.
pub struct Running;

impl<P: PIOExt, State> StateMachine<P, State> {
    /// Stops the state machine if it is still running and returns its program.
    ///
    /// The program can be uninstalled to free space once it is no longer used by any state
    /// machine.
    pub fn uninit(mut self) -> (UninitStateMachine<P>, InstalledProgram<P>) {
        self.sm.set_enabled(false);
        (self.sm, self.program)
    }

    /// The address of the instruction currently being executed.
    pub fn instruction_address(&self) -> u32 {
        self.sm.sm().sm_addr.read().bits()
    }

    /// Set the current instruction.
    pub fn set_instruction(&mut self, instruction: u16) {
        // TODO: Check if this function is safe to call while the state machine is running.
        self.sm.set_instruction(instruction);
    }

    /// Check if the current instruction is stalled.
    pub fn stalled(&self) -> bool {
        self.sm.sm().sm_execctrl.read().exec_stalled().bits()
    }

    /// Get the next element from RX FIFO.
    ///
    /// Returns `None` if the FIFO is empty.
    pub fn read_rx(&mut self) -> Option<u32> {
        // Safety: The register is never written by software.
        let is_empty =
            unsafe { &*self.sm.block }.fstat.read().rxempty().bits() & (1 << self.sm.id) != 0;

        if is_empty {
            return None;
        }

        // Safety: The register is unique to this state machine.
        Some(
            unsafe { &*self.sm.block }.rxf[self.sm.id as usize]
                .read()
                .bits(),
        )
    }

    /// Write an element to TX FIFO.
    ///
    /// Returns `true` if the value was written to FIFO, `false` otherwise.
    pub fn write_tx(&mut self, value: u32) -> bool {
        // Safety: The register is never written by software.
        let is_full =
            unsafe { &*self.sm.block }.fstat.read().txfull().bits() & (1 << self.sm.id) != 0;

        if is_full {
            return false;
        }

        // Safety: The register is unique to this state machine.
        unsafe { &*self.sm.block }.txf[self.sm.id as usize].write(|w| unsafe { w.bits(value) });

        true
    }
}

impl<P: PIOExt> StateMachine<P, Stopped> {
    /// Starts execution of the selected program.
    pub fn start(mut self) -> StateMachine<P, Running> {
        // Enable SM
        self.sm.set_enabled(true);

        StateMachine {
            sm: self.sm,
            program: self.program,
            _phantom: core::marker::PhantomData,
        }
    }

    /// Sets the pin directions for the specified pins.
    ///
    /// The `pins` parameter specifies a set of pins as a mask, and `pindir` contains the
    /// directions that are configured for these pins. The bits in both masks correspond to the pin
    /// number. The user has to make sure that they do not select any pins that are in use by any
    /// other state machines of the same PIO block.
    ///
    /// This function needs to be called for sideset pins if they are supposed to be used as
    /// output pins.
    pub fn set_pindirs_with_mask(&mut self, mut pins: u32, pindir: u32) {
        let mut pin = 0;
        let prev_pinctrl = self.sm.sm().sm_pinctrl.read().bits();
        // For each pin in the mask, we select the pin as a SET pin and then execute "set PINDIRS,
        // <direction>".
        while pins != 0 {
            if (pins & 1) != 0 {
                self.sm.sm().sm_pinctrl.write(|w| {
                    unsafe {
                        w.set_count().bits(1);
                        w.set_base().bits(pin as u8);
                    }
                    w
                });
                self.sm.sm().sm_instr.write(|w| {
                    unsafe {
                        const SET_PINDIRS: u16 = 0xe080;
                        w.sm0_instr()
                            .bits(SET_PINDIRS | ((pindir >> pin) & 0x1) as u16);
                    }
                    w
                });
            }
            pin += 1;
            pins = pins >> 1;
        }
        // We modified PINCTRL, yet the program assumes a certain configuration, so restore the
        // previous value.
        self.sm.sm().sm_pinctrl.write(|w| {
            unsafe {
                w.bits(prev_pinctrl);
            }
            w
        });
    }
}

impl<P: PIOExt> StateMachine<P, Running> {
    /// Stops execution of the selected program.
    pub fn stop(mut self) -> StateMachine<P, Stopped> {
        // Enable SM
        self.sm.set_enabled(false);

        StateMachine {
            sm: self.sm,
            program: self.program,
            _phantom: core::marker::PhantomData,
        }
    }
}

/// PIO Interrupt controller.
#[derive(Debug)]
pub struct Interrupt<P: PIOExt> {
    id: u8,
    block: *const rp2040_pac::pio0::RegisterBlock,
    _phantom: core::marker::PhantomData<P>,
}

// Safety: `Interrupt` provides exclusive access to interrupt registers.
unsafe impl<P: PIOExt + Send> Send for Interrupt<P> {}

impl<P: PIOExt> Interrupt<P> {
    /// Enable interrupts raised by state machines.
    ///
    /// The PIO peripheral has 4 outside visible interrupts that can be raised by the state machines. Note that this
    /// don't correspond with the state machine index; any state machine can raise any one of the four interrupts.
    pub fn enable_sm_interrupt(&self, id: u8) {
        match id {
            0 => self.irq().irq_inte.modify(|_, w| w.sm0().set_bit()),
            1 => self.irq().irq_inte.modify(|_, w| w.sm1().set_bit()),
            2 => self.irq().irq_inte.modify(|_, w| w.sm2().set_bit()),
            3 => self.irq().irq_inte.modify(|_, w| w.sm3().set_bit()),
            _ => panic!("invalid state machine interrupt number"),
        }
    }

    /// Disable interrupts raised by state machines.
    ///
    /// See [`Self::enable_sm_interrupt`] for info about the index.
    pub fn disable_sm_interrupt(&self, id: u8) {
        match id {
            0 => self.irq().irq_inte.modify(|_, w| w.sm0().clear_bit()),
            1 => self.irq().irq_inte.modify(|_, w| w.sm1().clear_bit()),
            2 => self.irq().irq_inte.modify(|_, w| w.sm2().clear_bit()),
            3 => self.irq().irq_inte.modify(|_, w| w.sm3().clear_bit()),
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
            0 => self.irq().irq_intf.modify(|_, w| w.sm0().set_bit()),
            1 => self.irq().irq_intf.modify(|_, w| w.sm1().set_bit()),
            2 => self.irq().irq_intf.modify(|_, w| w.sm2().set_bit()),
            3 => self.irq().irq_intf.modify(|_, w| w.sm3().set_bit()),
            _ => panic!("invalid state machine interrupt number"),
        }
    }

    /// Enable TX FIFO not full interrupt.
    ///
    /// Each of the 4 state machines have their own TX FIFO. This interrupt is raised when the TX FIFO is not full, i.e.
    /// one could push more data to it.
    pub fn enable_tx_not_full_interrupt(&self, id: u8) {
        match id {
            0 => self.irq().irq_inte.modify(|_, w| w.sm0_txnfull().set_bit()),
            1 => self.irq().irq_inte.modify(|_, w| w.sm1_txnfull().set_bit()),
            2 => self.irq().irq_inte.modify(|_, w| w.sm2_txnfull().set_bit()),
            3 => self.irq().irq_inte.modify(|_, w| w.sm3_txnfull().set_bit()),
            _ => panic!("invalid state machine interrupt number"),
        }
    }

    /// Disable TX FIFO not full interrupt.
    ///
    /// See [`Self::enable_tx_not_full_interrupt`] for info about the index.
    pub fn disable_tx_not_full_interrupt(&self, id: u8) {
        match id {
            0 => self
                .irq()
                .irq_inte
                .modify(|_, w| w.sm0_txnfull().clear_bit()),
            1 => self
                .irq()
                .irq_inte
                .modify(|_, w| w.sm1_txnfull().clear_bit()),
            2 => self
                .irq()
                .irq_inte
                .modify(|_, w| w.sm2_txnfull().clear_bit()),
            3 => self
                .irq()
                .irq_inte
                .modify(|_, w| w.sm3_txnfull().clear_bit()),
            _ => panic!("invalid state machine interrupt number"),
        }
    }

    /// Force TX FIFO not full interrupt.
    ///
    /// See [`Self::enable_tx_not_full_interrupt`] for info about the index.
    pub fn force_tx_not_full_interrupt(&self, id: u8) {
        match id {
            0 => self.irq().irq_intf.modify(|_, w| w.sm0_txnfull().set_bit()),
            1 => self.irq().irq_intf.modify(|_, w| w.sm1_txnfull().set_bit()),
            2 => self.irq().irq_intf.modify(|_, w| w.sm2_txnfull().set_bit()),
            3 => self.irq().irq_intf.modify(|_, w| w.sm3_txnfull().set_bit()),
            _ => panic!("invalid state machine interrupt number"),
        }
    }

    /// Enable RX FIFO not empty interrupt.
    ///
    /// Each of the 4 state machines have their own RX FIFO. This interrupt is raised when the RX FIFO is not empty,
    /// i.e. one could read more data from it.
    pub fn enable_rx_not_empty_interrupt(&self, id: u8) {
        match id {
            0 => self
                .irq()
                .irq_inte
                .modify(|_, w| w.sm0_rxnempty().set_bit()),
            1 => self
                .irq()
                .irq_inte
                .modify(|_, w| w.sm1_rxnempty().set_bit()),
            2 => self
                .irq()
                .irq_inte
                .modify(|_, w| w.sm2_rxnempty().set_bit()),
            3 => self
                .irq()
                .irq_inte
                .modify(|_, w| w.sm3_rxnempty().set_bit()),
            _ => panic!("invalid state machine interrupt number"),
        }
    }

    /// Disable RX FIFO not empty interrupt.
    ///
    /// See [`Self::enable_rx_not_empty_interrupt`] for info about the index.
    pub fn disable_rx_not_empty_interrupt(&self, id: u8) {
        match id {
            0 => self
                .irq()
                .irq_inte
                .modify(|_, w| w.sm0_rxnempty().clear_bit()),
            1 => self
                .irq()
                .irq_inte
                .modify(|_, w| w.sm1_rxnempty().clear_bit()),
            2 => self
                .irq()
                .irq_inte
                .modify(|_, w| w.sm2_rxnempty().clear_bit()),
            3 => self
                .irq()
                .irq_inte
                .modify(|_, w| w.sm3_rxnempty().clear_bit()),
            _ => panic!("invalid state machine interrupt number"),
        }
    }

    /// Force RX FIFO not empty interrupt.
    ///
    /// See [`Self::enable_rx_not_empty_interrupt`] for info about the index.
    pub fn force_rx_not_empty_interrupt(&self, id: u8) {
        match id {
            0 => self
                .irq()
                .irq_intf
                .modify(|_, w| w.sm0_rxnempty().set_bit()),
            1 => self
                .irq()
                .irq_intf
                .modify(|_, w| w.sm1_rxnempty().set_bit()),
            2 => self
                .irq()
                .irq_intf
                .modify(|_, w| w.sm2_rxnempty().set_bit()),
            3 => self
                .irq()
                .irq_intf
                .modify(|_, w| w.sm3_rxnempty().set_bit()),
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
pub struct PIOBuilder<P> {
    /// Clock divisor.
    clock_divisor: f32,

    /// Program location and configuration.
    program: InstalledProgram<P>,
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
            clock_divisor: 1.0,
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
        assert!(count <= 5);
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
    /// state of the next pin, and so on up to number of bits set using [`Self::side_set`] function.
    pub fn side_set_pin_base(mut self, base: u8) -> Self {
        self.side_set_base = base;
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
    pub fn build(self, mut sm: UninitStateMachine<P>) -> StateMachine<P, Stopped> {
        let offset = self.program.offset;

        // Stop the SM
        sm.set_enabled(false);

        // Write all configuration bits
        sm.set_clock_divisor(self.clock_divisor);

        sm.sm().sm_execctrl.write(|w| {
            w.side_en().bit(self.program.side_set.optional());
            w.side_pindir().bit(self.program.side_set.pindirs());

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
                w.wrap_top().bits(offset as u8 + self.program.wrap.source);
                w.wrap_bottom()
                    .bits(offset as u8 + self.program.wrap.target);
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

            w.autopull().bit(self.autopull);
            w.autopush().bit(self.autopush);

            w
        });

        sm.sm().sm_pinctrl.write(|w| {
            unsafe {
                w.sideset_count().bits(self.program.side_set.bits());
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
        sm.set_instruction(
            pio::InstructionOperands::JMP {
                condition: pio::JmpCondition::Always,
                address: offset as u8,
            }
            .encode(),
        );

        StateMachine {
            sm: sm,
            program: self.program,
            _phantom: core::marker::PhantomData,
        }
    }
}
