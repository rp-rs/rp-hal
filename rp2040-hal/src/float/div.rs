use super::Float;
use crate::rom_data;
use crate::sio::save_divider;

trait ROMDiv {
    fn rom_div(self, b: Self) -> Self;
}

impl ROMDiv for f32 {
    fn rom_div(self, b: Self) -> Self {
        // ROM implementation uses the hardware divider, so we have to save it
        save_divider(|_sio| rom_data::float_funcs::fdiv(self, b))
    }
}

impl ROMDiv for f64 {
    fn rom_div(self, b: Self) -> Self {
        // ROM implementation uses the hardware divider, so we have to save it
        save_divider(|_sio| rom_data::double_funcs::ddiv(self, b))
    }
}

fn div<F: Float + ROMDiv>(a: F, b: F) -> F {
    if a.is_not_finite() {
        if b.is_not_finite() {
            // inf/NaN / inf/NaN = NaN
            return F::NAN;
        }

        if b.is_zero() {
            // inf/NaN / 0 = NaN
            return F::NAN;
        }

        return if b.is_sign_negative() {
            // [+/-]inf/NaN / (-X) = [-/+]inf/NaN
            a.negate()
        } else {
            // [-]inf/NaN / X = [-]inf/NaN
            a
        };
    }

    if b.is_nan() {
        // X / NaN = NaN
        return b;
    }

    // ROM handles X / 0 = [-]inf and X / [-]inf = [-]0, so we only
    // need to catch 0 / 0
    if b.is_zero() && a.is_zero() {
        return F::NAN;
    }

    a.rom_div(b)
}

intrinsics! {
    #[alias = __divsf3vfp]
    #[aeabi = __aeabi_fdiv]
    extern "C" fn __divsf3(a: f32, b: f32) -> f32 {
        div(a, b)
    }

    #[bootrom_v2]
    #[alias = __divdf3vfp]
    #[aeabi = __aeabi_ddiv]
    extern "C" fn __divdf3(a: f64, b: f64) -> f64 {
        div(a, b)
    }
}
