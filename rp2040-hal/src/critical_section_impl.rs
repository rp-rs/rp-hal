struct RpSpinlockCs;
critical_section::custom_impl!(RpSpinlockCs);

unsafe impl critical_section::Impl for RpSpinlockCs {
    unsafe fn acquire() -> u8 {
        loop {
            cortex_m::interrupt::disable();
            if (*pac::SIO::ptr()).spinlock31.read().bits() != 0 {
                break;
            }
            cortex_m::interrupt::enable();
        }
        0
    }

    unsafe fn release(_token: u8) {
        (*pac::SIO::ptr()).spinlock31.write_with_zero(|w| w.bits(1));
        cortex_m::interrupt::enable();
    }
}
