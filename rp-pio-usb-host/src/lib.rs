#![no_std]

//! PIO-based low-/full-speed USB host transport for RP2040/RP235x chips.
//!
//! The crate provides a direct root-port [`bus::Bus`] and an [`embassy`]
//! adapter for `embassy-usb-host`.

pub mod bus;
mod chip;
mod clock;
mod crc;
pub mod embassy;
mod encoding;
mod pid;
pub mod pio_instance;
mod ram;
mod rx_driver;
mod rx_pio;
mod tx_driver;
mod tx_pio;
pub mod frame_counter;
