fn main() {
    // The memory map comes from embassy-stm32's `memory-x` feature.
    println!("cargo:rustc-link-arg-bins=-Tdefmt.x");
}
