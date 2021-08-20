//! Multicore support
// See [Chapter ?? Section ??](https://datasheets.raspberrypi.org/rp2040/rp2040_datasheet.pdf) for more details

use crate::pac;

#[cfg(feature = "alloc")]
extern crate alloc;

struct FIFO<'p> {
    sio: &'p mut pac::SIO,
}

impl<'p> FIFO<'p> {
    fn new(sio: &'p mut pac::SIO) -> Self {
        Self { sio }
    }

    #[inline(always)]
    fn read_ready(&self) -> bool {
        self.sio.fifo_st.read().vld().bit_is_set()
    }

    #[inline(always)]
    fn write_ready(&self) -> bool {
        self.sio.fifo_st.read().rdy().bit_is_set()
    }

    #[inline(always)]
    fn drain(&mut self) {
        while self.read_ready() {
            self.sio.fifo_rd.read().bits();
        }
    }

    fn push_blocking(&mut self, value: u32) {
        while !self.write_ready() {
            cortex_m::asm::nop();
        }

        self.sio.fifo_wr.write(|w| unsafe { w.bits(value) });
        cortex_m::asm::sev();
    }

    fn pop_blocking(&mut self) -> u32 {
        while !self.read_ready() {
            cortex_m::asm::wfe();
        }

        self.sio.fifo_rd.read().bits()
    }
}

static mut CORE1_STACK: [usize; 4096] = [0; 4096];
extern "C" {
    fn multicore_trampoline();
}

fn core1_setup(_stack_bottom: *mut ()) {
    // TODO: stack guard
    // TODO: irq priorities
}

/// Multicore execution management.
pub struct Multicore<'p> {
    psm: &'p mut pac::PSM,
    ppb: &'p mut pac::PPB,
    sio: &'p mut pac::SIO,
}

impl<'p> Multicore<'p> {
    /// Create a new |Multicore| instance.
    pub fn new(psm: &'p mut pac::PSM, ppb: &'p mut pac::PPB, sio: &'p mut pac::SIO) -> Self {
        Self { psm, ppb, sio }
    }

    fn reset(&mut self, id: usize) {
        assert_eq!(id, 1);

        self.psm.frce_off.modify(|_, w| w.proc1().set_bit());

        while !self.psm.frce_off.read().proc1().bit_is_set() {
            cortex_m::asm::nop();
        }

        self.psm.frce_off.modify(|_, w| w.proc1().clear_bit());
    }

    fn inner_spawn(&mut self, id: usize, wrapper: *mut (), entry: *mut ()) {
        assert_eq!(id, 1);

        self.reset(id);

        let vector_table = self.ppb.vtor.read().bits();

        let core1_stack = unsafe { &mut CORE1_STACK };
        let mut stack_ptr = unsafe { core1_stack.as_mut_ptr().add(core1_stack.len() - 1) };

        let mut push = |v: usize| unsafe {
            stack_ptr.write(v);
            stack_ptr = stack_ptr.sub(1);
        };

        push(wrapper as usize);
        push(core1_stack.as_mut_ptr() as usize);
        push(entry as usize);

        let cmd_seq = [
            0,
            0,
            1,
            vector_table as usize,
            stack_ptr as usize,
            multicore_trampoline as usize,
        ];

        let mut fifo = FIFO::new(self.sio);

        let mut seq = 0;
        loop {
            let cmd = cmd_seq[seq] as u32;
            if cmd == 0 {
                fifo.drain();
                cortex_m::asm::sev();
            }
            fifo.push_blocking(cmd);
            let response = fifo.pop_blocking();
            seq = if cmd == response { seq + 1 } else { 0 };
            if seq >= cmd_seq.len() {
                break;
            }
        }
    }

    /// Spawn a function on core |id|.
    #[cfg(not(feature = "alloc"))]
    pub fn spawn(&mut self, id: usize, entry: fn() -> !) {
        self.inner_spawn(id, core1_no_alloc as _, entry as _);

        #[allow(improper_ctypes_definitions)]
        extern "C" fn core1_no_alloc(entry: fn() -> !, stack_bottom: *mut ()) -> ! {
            core1_setup(stack_bottom);
            entry();
        }
    }

    /// Spawn a function on core |id|.
    #[cfg(feature = "alloc")]
    pub fn spawn<F>(&mut self, id: usize, entry: F)
    where
        F: FnOnce() -> bad::Never,
        F: Send + 'static,
    {
        use alloc::boxed::Box;

        let main: Box<dyn FnOnce() -> bad::Never> = Box::new(move || entry());
        let p = Box::into_raw(Box::new(main));
        self.inner_spawn(id, core1_alloc as _, p as _);

        extern "C" fn core1_alloc(entry: *mut (), stack_bottom: *mut ()) -> ! {
            core1_setup(stack_bottom);
            let main = unsafe { Box::from_raw(entry as *mut Box<dyn FnOnce() -> bad::Never>) };
            main();
        }
    }
}

// https://github.com/nvzqz/bad-rs/blob/master/src/never.rs
#[cfg(feature = "alloc")]
mod bad {
    pub(crate) type Never = <F as HasOutput>::Output;

    pub trait HasOutput {
        type Output;
    }

    impl<O> HasOutput for fn() -> O {
        type Output = O;
    }

    type F = fn() -> !;
}
