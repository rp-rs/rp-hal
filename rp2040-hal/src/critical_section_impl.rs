use core::sync::atomic::{AtomicU8, Ordering};

struct RpSpinlockCs;
critical_section::custom_impl!(RpSpinlockCs);

// Value to indicate no-one has the lock.
// Initialising LOCK_CORE to 0 means cheaper initialisation.
const LOCK_CORE_SENTINEL: u8 = 0;
static mut LOCK_CORE: AtomicU8 = AtomicU8::new(LOCK_CORE_SENTINEL);
// Value to indicate that we're not the inner loop
// Only needs to be not 0 or 1, but might as well make it unique
const INNER_LOOP_VALUE: u8 = 0xee;

unsafe impl critical_section::Impl for RpSpinlockCs {
    unsafe fn acquire() -> u8 {
        // Store the initial interrupt state and current core id in stack variables
        let interrupts_active = cortex_m::register::primask::read().is_active();
        let core = (*pac::SIO::ptr()).cpuid.read().bits() as u8 + 1_u8;
        if LOCK_CORE.load(Ordering::Acquire) == core {
            // We already own the lock, so we must have called acquire within a critical_section.
            // Return the magic inner-loop value so that we know not to re-enable interrupts in release()
            INNER_LOOP_VALUE
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
                    LOCK_CORE.store(core, Ordering::Release);
                    break;
                }
                // We didn't get the lock, enable interrupts if they were enabled before we started
                if interrupts_active {
                    cortex_m::interrupt::enable();
                }
            }
            // If we broke out of the loop we have just acquired the lock
            // As the outermost loop, we want to return the interrupt status to restore later
            interrupts_active as _
        }
    }

    unsafe fn release(token: u8) {
        // Was this the outermost critical_section?
        if token != INNER_LOOP_VALUE {
            // Yes, we were the outermost.
            // Set LOCK_CORE to the sentinel value to ensure the next call checks spinlock instead
            LOCK_CORE.store(LOCK_CORE_SENTINEL, Ordering::Release);
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
