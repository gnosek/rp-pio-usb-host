#![no_std]

//! PIO-based low-/full-speed USB host transport for RP2040/RP235x chips.
//!
//! The crate provides a direct root-port [`bus::Bus`] and an [`embassy`]
//! adapter for `embassy-usb-host`.

pub mod bus;
mod chip;
mod pio_instance;
