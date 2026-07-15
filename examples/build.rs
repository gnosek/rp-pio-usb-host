//! Build script: make `memory.x` available to the linker and add the linker
//! scripts needed for an RP2040 + cortex-m-rt + defmt + embassy-rp build.
//!
//! Linker scripts pulled in (order matters):
//!   - link.x      : from cortex-m-rt (vectors, sections; INCLUDEs memory.x)
//!   - link-rp.x   : from embassy-rp   (places the boot2 stage into BOOT2)
//!   - defmt.x     : from defmt        (defmt log metadata sections)

use std::env;
use std::fs::File;
use std::io::Write;
use std::path::PathBuf;

fn main() {
    // Put `memory.x` where the linker can find it.
    let out = PathBuf::from(env::var_os("OUT_DIR").unwrap());
    File::create(out.join("memory.x"))
        .unwrap()
        .write_all(include_bytes!("memory.x"))
        .unwrap();
    println!("cargo:rustc-link-search={}", out.display());
    println!("cargo:rerun-if-changed=memory.x");
    println!("cargo:rerun-if-changed=build.rs");

    // Linker arguments (apply to binaries only).
    println!("cargo:rustc-link-arg-bins=--nmagic");
    println!("cargo:rustc-link-arg-bins=-Tlink.x");
    println!("cargo:rustc-link-arg-bins=-Tlink-rp.x");
    println!("cargo:rustc-link-arg-bins=-Tdefmt.x");
}
