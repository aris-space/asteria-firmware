use std::time::{SystemTime, UNIX_EPOCH};

fn main() {
    let timestamp = unix_timestamp();
    println!("cargo:rustc-link-arg-bins=-Tdefmt.x");
    println!("cargo:rustc-env=BUILT_UNIX_TS={timestamp}");

    built::write_built_file().expect("Failed to acquire build-time information");
}

fn unix_timestamp() -> u64 {
    std::env::var("SOURCE_DATE_EPOCH")
        .ok()
        .and_then(|s| s.parse::<u64>().ok())
        .unwrap_or_else(|| {
            SystemTime::now()
                .duration_since(UNIX_EPOCH)
                .unwrap()
                .as_secs()
        })
}
