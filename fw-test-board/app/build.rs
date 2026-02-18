fn main() {
    println!("cargo:rerun-if-env-changed=ASTERIA_ARTIFACT_TIMESTAMP_MS");

    #[cfg(feature = "defmt")]
    println!("cargo:rustc-link-arg-bins=-Tdefmt.x");

    if let Ok(ts) = std::env::var("ASTERIA_ARTIFACT_TIMESTAMP_MS") {
        println!("cargo:rustc-env=ASTERIA_ARTIFACT_TIMESTAMP_MS={ts}");
    }

    built::write_built_file().expect("Failed to acquire build-time information");
}
