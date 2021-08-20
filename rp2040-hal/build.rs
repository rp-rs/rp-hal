fn main() {
    println!("cargo:rerun-if-changed=src/asm.S");

    cc::Build::new().file("src/asm.S").compile("asm");
}
