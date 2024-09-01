extern crate proc_macro;

use proc_macro::TokenStream;
use proc_macro2::Span;
use quote::quote;
use syn::{parse, parse_macro_input, Item, ItemFn, Stmt};

#[proc_macro_attribute]
pub fn entry(args: TokenStream, input: TokenStream) -> TokenStream {
    let mut f = parse_macro_input!(input as ItemFn);

    if !args.is_empty() {
        return parse::Error::new(Span::call_site(), "This attribute accepts no arguments")
            .to_compile_error()
            .into();
    }

    let clear_locks: TokenStream = quote!(unsafe {
        const SIO_BASE: u32 = 0xd0000000;
        const SPINLOCK0_PTR: *mut u32 = (SIO_BASE + 0x100) as *mut u32;
        const SPINLOCK_COUNT: usize = 32;
        for i in 0..SPINLOCK_COUNT {
            SPINLOCK0_PTR.wrapping_add(i).write_volatile(1);
        }
        #[cfg(target_arch = "arm")]
        {
            // Enable the Double-Co-Pro and the GPIO Co-Pro in the CPACR register.
            // We have to do this early, before there's a chance we might call
            // any accelerated functions.
            const SCB_CPACR_PTR: *mut u32 = 0xE000_ED88 as *mut u32;
            const SCB_CPACR_FULL_ACCESS: u32 = 0b11;
            // Do a R-M-W, because the FPU enable is here and that's already been enabled
            let mut temp = SCB_CPACR_PTR.read_volatile();
            // DCP Co-Pro is 4, two-bits per entry
            temp |= SCB_CPACR_FULL_ACCESS << (4 * 2);
            // GPIO Co-Pro is 0, two-bits per entry
            temp |= SCB_CPACR_FULL_ACCESS << (0 * 2);
            SCB_CPACR_PTR.write_volatile(temp);
            // Don't allow any DCP code to be moved before this fence.
            core::sync::atomic::compiler_fence(core::sync::atomic::Ordering::SeqCst);
        }
    })
    .into();
    let clear_locks = parse_macro_input!(clear_locks as Stmt);

    // statics must stay first so cortex_m_rt::entry still finds them
    let stmts = insert_after_static(f.block.stmts, clear_locks);
    f.block.stmts = stmts;

    quote!(
        #[rp235x_hal::arch_entry]
        #f
    )
    .into()
}

/// Insert new statements after initial block of statics
fn insert_after_static(stmts: impl IntoIterator<Item = Stmt>, insert: Stmt) -> Vec<Stmt> {
    let mut istmts = stmts.into_iter();
    let mut stmts = vec![];
    for stmt in istmts.by_ref() {
        match stmt {
            Stmt::Item(Item::Static(var)) => {
                stmts.push(Stmt::Item(Item::Static(var)));
            }
            _ => {
                stmts.push(insert);
                stmts.push(stmt);
                break;
            }
        }
    }
    stmts.extend(istmts);

    stmts
}
