fn main() {
    #[cfg(feature = "defmt")]
    println!("cargo:rustc-link-arg-bins=-Tdefmt.x");

    built::write_built_file().expect("Failed to acquire build-time information");
}
