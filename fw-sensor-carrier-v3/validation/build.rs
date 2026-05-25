use std::env;
use std::fs;
use std::path::PathBuf;

fn main() {
    let out = PathBuf::from(env::var("OUT_DIR").unwrap());

    // We supply the memory map ourselves (instead of embassy's `memory-x`) so the
    // SRAM4 / RAM_D3 region exists for the BDMA bounce buffer.
    fs::copy("memory.x", out.join("memory.x")).unwrap();
    println!("cargo:rerun-if-changed=memory.x");

    // Place the `.sram4` section (the BDMA bounce buffer) into RAM_D3 / SRAM4.
    fs::write(
        out.join("sram4.x"),
        "SECTIONS {\n  .sram4 (NOLOAD) : ALIGN(4) {\n    *(.sram4 .sram4.*)\n  } > RAM_D3\n} INSERT AFTER .bss;\n",
    )
    .unwrap();

    println!("cargo:rustc-link-search={}", out.display());
    println!("cargo:rustc-link-arg-bins=-Tdefmt.x");
    println!("cargo:rustc-link-arg-bins=-Tsram4.x");
}
