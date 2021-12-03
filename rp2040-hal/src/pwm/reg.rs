use super::dyn_slice::{DynSliceId, DynSliceMode};
use pac::pwm::CH;

/// # Safety
///
/// Users should only implement the [`id`] function. No default function
/// implementations should be overridden. The implementing type must also have
/// "control" over the corresponding slice ID, i.e. it must guarantee that each
/// slice ID is a singleton
pub(super) unsafe trait RegisterInterface {
    /// Provide a [`DynSliceId`] identifying the set of registers controlled by
    /// this type.
    fn id(&self) -> DynSliceId;

    #[inline]
    fn ch(&self) -> &CH {
        let num = self.id().num as usize;
        unsafe { &(*pac::PWM::ptr()).ch[num] }
    }

    #[inline]
    fn advance_phase(&mut self) {
        self.ch().csr.modify(|_, w| w.ph_adv().set_bit())
    }

    #[inline]
    fn retard_phase(&mut self) {
        self.ch().csr.modify(|_, w| w.ph_ret().set_bit())
    }

    #[inline]
    fn do_change_mode(&mut self, mode: DynSliceMode) {
        self.ch().csr.modify(|_, w| match mode {
            DynSliceMode::FreeRunning => w.divmode().div(),
            DynSliceMode::InputHighRunning => w.divmode().level(),
            DynSliceMode::CountRisingEdge => w.divmode().rise(),
            DynSliceMode::CountFallingEdge => w.divmode().fall(),
        })
    }

    #[inline]
    fn write_inv_a(&mut self, value: bool) {
        self.ch().csr.modify(|_, w| w.a_inv().bit(value));
    }

    #[inline]
    fn write_inv_b(&mut self, value: bool) {
        self.ch().csr.modify(|_, w| w.b_inv().bit(value));
    }

    #[inline]
    fn write_ph_correct(&mut self, value: bool) {
        self.ch().csr.modify(|_, w| w.ph_correct().bit(value));
    }

    #[inline]
    fn write_enable(&mut self, value: bool) {
        self.ch().csr.modify(|_, w| w.en().bit(value));
    }

    #[inline]
    fn write_div_int(&mut self, value: u8) {
        self.ch().div.modify(|_, w| unsafe { w.int().bits(value) });
    }
    #[inline]
    fn write_div_frac(&mut self, value: u8) {
        self.ch().div.modify(|_, w| unsafe { w.frac().bits(value) });
    }

    #[inline]
    fn write_ctr(&mut self, value: u16) {
        self.ch().ctr.write(|w| unsafe { w.ctr().bits(value) });
    }

    #[inline]
    fn read_ctr(&self) -> u16 {
        self.ch().ctr.read().ctr().bits()
    }
    #[inline]
    fn write_cc_a(&mut self, value: u16) {
        self.ch().cc.modify(|_, w| unsafe { w.a().bits(value) });
    }
    #[inline]
    fn read_cc_a(&self) -> u16 {
        self.ch().cc.read().a().bits()
    }

    #[inline]
    fn write_cc_b(&mut self, value: u16) {
        self.ch().cc.modify(|_, w| unsafe { w.b().bits(value) });
    }

    #[inline]
    fn read_cc_b(&self) -> u16 {
        self.ch().cc.read().b().bits()
    }
    #[inline]
    fn write_top(&mut self, value: u16) {
        self.ch().top.write(|w| unsafe { w.top().bits(value) });
    }
    #[inline]
    fn read_top(&self) -> u16 {
        self.ch().top.read().top().bits()
    }
}
