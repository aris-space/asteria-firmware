use std::env;
use std::fs;
use std::path::PathBuf;

fn main() {
    let out = PathBuf::from(env::var("OUT_DIR").unwrap());

    // We supply the memory map ourselves (instead of embassy's `memory-x`). The
    // RAM_D3 / SRAM4 region is harmless when unused, so the map is the same either
    // way; only the `.sram4` section below is feature-gated.
    fs::copy("memory.x", out.join("memory.x")).unwrap();
    println!("cargo:rerun-if-changed=memory.x");
    println!("cargo:rustc-link-search={}", out.display());
    println!("cargo:rustc-link-arg-bins=-Tdefmt.x");

    // Only the BDMA build needs the SRAM4 bounce buffer. INSERT AFTER .uninit (not
    // .bss): cortex-m-rt folds `INSERT AFTER .bss` sections into its .bss-zeroing
    // by extending __ebss past them, which for a RAM_D3 section would make startup
    // zero all of memory. .uninit is past every RAM bound symbol.
    if env::var_os("CARGO_FEATURE_USE_I2C4").is_some() {
        fs::write(
            out.join("sram4.x"),
            "SECTIONS {\n  .sram4 (NOLOAD) : ALIGN(4) {\n    *(.sram4 .sram4.*)\n  } > RAM_D3\n} INSERT AFTER .uninit;\n",
        )
        .unwrap();
        println!("cargo:rustc-link-arg-bins=-Tsram4.x");
    }
}
