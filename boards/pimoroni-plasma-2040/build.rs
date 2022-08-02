//! This build script makes sure the linker flag -Tdefmt.x is added
//! for the examples.

fn main() {
    println!("cargo:rustc-link-arg-examples=-Tdefmt.x");
}
