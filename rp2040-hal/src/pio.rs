//! Programmable IO (PIO)
/// See [Chapter 3](https://rptl.io/pico-datasheet) for more details.

use rp2040_pac::Peripherals;
// use vcell::VolatileCell;

/*
/// aaa
pub fn aaa<P>(p: &Peripherals, program: &pio::Program<P>) {
    // atomic steps numbered

    let instrs: &[VolatileCell<u32>; 16] = unsafe { core::mem::transmute(&p.PIO0.instr_mem0) };
    // 1. Move program somewhere into the 32 instrs of memory in pio
    for (i, instr) in program.code().iter().enumerate() {
        instrs[i].set(*instr as u32);
    }

    // 2. Find a free state machine

    unsafe {
        // 8. Interact with the state machine.
        p.PIO0.sm0_addr.read().bits();
        p.PIO0.rxf0.read().bits();
        p.PIO0.txf0.write(|w| w.bits(0));
        p.PIO0.sm0_execctrl.read().exec_stalled().bit();
    }
}
*/

/// a
pub struct PIOBuilder<'a> {
    p: &'a Peripherals,
    // wrap program from top to bottom
    wrap_top: u8,
    wrap_bottom: u8,
    // sideset is optional
    side_en: bool,
    // sideset sets pindirs
    side_pindir: bool,
    // gpio pin used by `jmp pin` instr
    jmp_pin: u8,
    // continuously assert the most recent OUT/SET to the pins.
    out_sticky: bool,
    // use a bit of OUT data as an auxilary write enable.
    // when OUT_STICKY is enabled, setting the bit to 0 deasserts for that instr.
    inline_out_en: bool,
    // which bit to use
    out_en_sel: u8,
    // for `mov x, status`.
    // false -> all ones if tx fifo level < N
    // true -> all ones if rx fifo level < N
    status_sel: bool,
    // base = starting pin
    // count = number of pins
    in_base: u8,
    out_base: u8,
    out_count: u8,
    set_base: u8,
    set_count: u8,
    sideset_base: u8,
    sideset_count: u8,
    // rx fifo steals tx fifo storage to be twice as deep
    fjoin_rx: bool,
    // tx fifo steals rx fifo storage to be twice as deep
    fjoin_tx: bool,
    // enable autopull
    autopull: bool,
    // enable autopush
    autopush: bool,
    // threhold for autopull
    pull_thresh: u8,
    // threshold for autopush
    push_thresh: u8,
    // true = shift out of OSR to right
    in_shiftdir: bool,
    // true = shift into ISR from right
    out_shiftdir: bool,
    // sm frequency = clock freq / (CLKDIV_INT + CLKDIV_FRAC / 256)
    clkdiv_int: u16,
    clkdiv_frac: u8,
}

impl<'a> PIOBuilder<'a> {
    /// a
    pub fn new(p: &'a Peripherals) -> Self {
        PIOBuilder {
            p,
            wrap_top: 0,
            wrap_bottom: 31,
            side_en: false,
            side_pindir: false,
            jmp_pin: 0,
            out_sticky: false,
            inline_out_en: false,
            out_en_sel: 0,
            status_sel: false,
            in_base: 0,
            out_base: 0,
            out_count: 32,
            set_base: 0,
            set_count: 0,
            sideset_base: 0,
            sideset_count: 0,
            fjoin_rx: false,
            fjoin_tx: false,
            autopull: false,
            autopush: false,
            pull_thresh: 32,
            push_thresh: 32,
            in_shiftdir: true,
            out_shiftdir: true,
            clkdiv_int: 1,
            clkdiv_frac: 0,
        }
    }
}

/// a
pub enum Buffers {
    /// a
    RxTx,
    /// a
    OnlyTx,
    /// a
    OnlyRx,
}

impl<'a> PIOBuilder<'a> {
    /// a
    pub fn with_program<P>(&mut self, p: &pio::Program<P>) -> &mut Self {
        self.wrap_top = p.wrap().0;
        self.wrap_bottom = p.wrap().1;

        self.side_en = p.side_set().optional();
        self.side_pindir = p.side_set().pindirs();

        self.sideset_count = p.side_set().bits();

        self
    }

    /// a
    pub fn buffers(&mut self, buffers: Buffers) -> &mut Self {
        match buffers {
            Buffers::RxTx => {
                self.fjoin_tx = false;
                self.fjoin_rx = false;
            }
            Buffers::OnlyTx => {
                self.fjoin_rx = false;
                self.fjoin_tx = true;
            }
            Buffers::OnlyRx => {
                self.fjoin_rx = true;
                self.fjoin_tx = true;
            }
        }
        self
    }

    /// 1 for full speed. A clock divisor of `n` will cause the state macine to run 1 cycle every
    /// `n`. For a small `n`, a fractional divisor may introduce unacceptable jitter.
    pub fn clock_divisor(&mut self, div: f32) -> &mut Self {
        // sm frequency = clock freq / (CLKDIV_INT + CLKDIV_FRAC / 256)
        self.clkdiv_int = div as u16;
        self.clkdiv_frac = (div.fract() * 256.0) as u8;
        self
    }

    /// a
    pub fn build(self) {
        const SM_ID: u8 = 0;

        // ### STOP SM ####
        self.set_sm_enabled(SM_ID, false);

        // ### CONFIGURE SM ###
        self.p.PIO0.sm0_execctrl.write(|w| {
            unsafe {
                w.wrap_top().bits(self.wrap_top);
                w.wrap_bottom().bits(self.wrap_bottom);
            }

            w.side_en().bit(self.side_en);
            w.side_pindir().bit(self.side_pindir);

            unsafe {
                w.jmp_pin().bits(self.jmp_pin);
            }

            w.out_sticky().bit(self.out_sticky);

            w.inline_out_en().bit(self.inline_out_en);
            unsafe {
                w.out_en_sel().bits(self.out_en_sel);
            }

            w.status_sel().bit(self.status_sel);

            w
        });

        self.p.PIO0.sm0_pinctrl.write(|w| {
            unsafe {
                w.in_base().bits(self.in_base);
            }

            unsafe {
                w.out_base().bits(self.out_base);
                w.out_count().bits(self.out_count);
            }

            unsafe {
                w.set_base().bits(self.set_base);
                w.set_count().bits(self.set_count);
            }

            unsafe {
                w.sideset_base().bits(self.sideset_base);
                w.sideset_count().bits(self.sideset_count);
            }

            w
        });

        self.p.PIO0.sm0_shiftctrl.write(|w| {
            w.fjoin_rx().bit(self.fjoin_rx);
            w.fjoin_tx().bit(self.fjoin_tx);

            w.autopull().bit(self.autopull);
            w.autopush().bit(self.autopush);

            unsafe {
                w.pull_thresh().bits(self.pull_thresh);
                w.push_thresh().bits(self.push_thresh);
            }

            w.out_shiftdir().bit(self.out_shiftdir);
            w.in_shiftdir().bit(self.in_shiftdir);

            w
        });

        self.p.PIO0.sm0_clkdiv.write(|w| {
            unsafe {
                w.int().bits(self.clkdiv_int);
                w.frac().bits(self.clkdiv_frac);
            }

            w
        });

        // ### RESTART SM & RESET SM CLOCK ###
        self.p.PIO0.ctrl.write(|w| {
            unsafe {
                w.sm_restart()
                    .bits(self.p.PIO0.ctrl.read().sm_restart().bits() | 1 << SM_ID);
                w.clkdiv_restart()
                    .bits(self.p.PIO0.ctrl.read().clkdiv_restart().bits() | 1 << SM_ID);
            }

            w
        });

        // ### SET SM PC ###
        self.p.PIO0.sm0_instr.write(|w| {
            // set starting location by setting the state machine
            // to execute a jmp to the beginning of the program
            // we loaded in.
            unsafe {
                w.sm0_instr().bits(0b000_00000_000_00000); // jmp 0
            }
            w
        });

        // ### ENABLE SM ###
        self.set_sm_enabled(0, true);
    }
}

impl<'a> PIOBuilder<'a> {
    fn set_sm_enabled(&self, id: u8, enabled: bool) {
        let bits = self.p.PIO0.ctrl.read().sm_enable().bits();
        let bits = (bits & !(1 << id)) | ((enabled as u8) << id);
        self.p
            .PIO0
            .ctrl
            .write(|w| unsafe { w.sm_enable().bits(bits) });
    }
}
