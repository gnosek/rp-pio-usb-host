//! PIO instance abstraction used by timing-critical PAC access helpers.

use embassy_rp::pio::Instance;
use rp_pac as pac;

/// A PIO block usable by the PIO-USB host transport: an embassy-rp [`Instance`]
/// plus its concrete [`pac::pio::Pio`] register block (which `Instance` keeps
/// sealed). Implemented for `PIO0`/`PIO1` (and `PIO2` on RP2350); the `REGS`
/// const inlines to a constant MMIO base, so the RAM-resident hot paths take no
/// flash to reach it. Select the block through the PIO peripheral passed to
/// [`crate::bus::Bus::new`].
#[doc(hidden)]
pub trait UsbPioInstance: Instance {
    /// The PIO block's register base (e.g. `pac::PIO0`).
    const REGS: pac::pio::Pio;
}

impl UsbPioInstance for embassy_rp::peripherals::PIO0 {
    const REGS: pac::pio::Pio = pac::PIO0;
}

impl UsbPioInstance for embassy_rp::peripherals::PIO1 {
    const REGS: pac::pio::Pio = pac::PIO1;
}

#[cfg(any(feature = "rp235xa", feature = "rp235xb"))]
impl UsbPioInstance for embassy_rp::peripherals::PIO2 {
    const REGS: pac::pio::Pio = pac::PIO2;
}
