#[cfg(feature = "rp2040")]
use rp2040_hal as hal;
#[cfg(feature = "rp235x")]
use rp235x_hal as hal;

use hal::pac;

/// When test cases are run from a debugger, there may not be a complete
/// system reset between test cases. To get clean initial conditions, reset
/// core1 and spinlocks.
///
/// This must only be called immediatly after booting core0, ie. at the start
/// of the `#[init]` function.
pub unsafe fn reset_cleanup() {
    unsafe {
        (*pac::PSM::PTR)
            .frce_off()
            .modify(|_, w| w.proc1().set_bit());
        while !(*pac::PSM::PTR).frce_off().read().proc1().bit_is_set() {
            cortex_m::asm::nop();
        }
        (*pac::PSM::PTR)
            .frce_off()
            .modify(|_, w| w.proc1().clear_bit());
        hal::sio::spinlock_reset();
    }
}
