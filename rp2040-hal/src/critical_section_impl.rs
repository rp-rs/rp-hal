struct RpSpinlockCs;
critical_section::custom_impl!(RpSpinlockCs);

const LOCK_CORE_SENTINEL: u32 = 0xDEADBEEF;
// These may or may not be shared between cores
// If shared, we need to access them with interrupts disabled.
// TODO: move away from static mut to avoid potential static init issues.
static mut LOCK_CORE: u32 = LOCK_CORE_SENTINEL;
static mut LOCK_COUNT: u8 = 0;

unsafe impl critical_section::Impl for RpSpinlockCs {
    unsafe fn acquire() -> u8 {
        // Store the initial interrupt state and current core id in stack variables
        let interrupts_active = cortex_m::register::primask::read().is_active();
        let core = (*pac::SIO::ptr()).cpuid.read().bits();
        // Need interrupts to be disabled to check/set our static variables
        cortex_m::interrupt::disable();
        // Safety: LOCK_COUNT won't change if LOCK_CORE = current core
        if LOCK_CORE == core && LOCK_COUNT != 0 {
            // We already own the lock, so we must have called acquire within a critical_section.
            // Don't need to do anything here.
        } else {
            // Spin until we get the lock
            loop {
                // Need to disable interrupts to ensure that we will not deadlock
                // if an interrupt enters critical_section::Impl after we acquire the lock
                cortex_m::interrupt::disable();
                // Read the spinlock reserved for critical_section
                if (*pac::SIO::ptr()).spinlock31.read().bits() != 0 {
                    // We just acquired the lock.
                    // Store which core we are so we can tell if we're called recursively
                    LOCK_CORE = core;
                    break;
                }
                // We didn't get the lock, enable interrupts if they were enabled before we started
                if interrupts_active {
                    cortex_m::interrupt::enable();
                }
            }
            // If we broke out of the loop we have just acquired the lock.
        }
        LOCK_COUNT += 1;
        interrupts_active as _
    }

    unsafe fn release(token: u8) {
        // Decrement our lock count
        LOCK_COUNT -= 1;
        // Was this the outermost critical_section?
        if LOCK_COUNT == 0 {
            // Yes, we were the last.
            // Set LOCK_CORE to the sentinel value to ensure the next call checks spinlock instead
            LOCK_CORE = LOCK_CORE_SENTINEL;
            // Release our spinlock
            (*pac::SIO::ptr()).spinlock31.write_with_zero(|w| w.bits(1));
            // Re-enable interrupts if they were enabled when we first called acquire()
            // We only do this on the outermost critical_section to ensure interrupts stay disabled
            // for the whole time that we have the lock
            if token != 0 {
                cortex_m::interrupt::enable();
            }
        }
    }
}
