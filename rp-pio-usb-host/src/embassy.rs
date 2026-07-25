//! Async `embassy` integration for sharing the direct PIO USB bus.
//!
//! The wrapper serializes access to the physical root port, provides an idle task
//! for frame keep-alives, and exposes the host-controller/allocator types used by
//! `embassy-usb-host`.

use crate::bus::{Bus as PioUsbBus, Pulldown};
use crate::pio_instance::UsbPioInstance;
use embassy_rp::Peri;
use embassy_rp::interrupt::typelevel::Binding;
use embassy_rp::peripherals::PIO0;
use embassy_rp::pio::{Instance, InterruptHandler, PioPin};
use embassy_sync::blocking_mutex::raw::CriticalSectionRawMutex;
use embassy_sync::mutex::{Mutex, MutexGuard};

/// Shared, interior-mutable bus state referenced by the controller and all pipes.
///
/// Construct once (typically into a `StaticCell`) and hand out a controller via
/// [`Bus::controller`]; pipes obtained through the controller's allocator
/// borrow the same `&'d` shared bus.
pub struct Bus<'d, PIO: UsbPioInstance = PIO0> {
    /// The single physical bus shared by the controller and all allocated pipes.
    bus: Mutex<CriticalSectionRawMutex, PioUsbBus<'d, PIO>>,
}

impl<'d, PIO: UsbPioInstance> Bus<'d, PIO> {
    /// Wrap a freshly-constructed [`Bus`] for sharing.
    pub fn from_bus(bus: PioUsbBus<'d, PIO>) -> Self {
        Self {
            bus: Mutex::new(bus),
        }
    }

    /// Construct a shared async bus from the raw PIO peripheral, USB pins, and IRQ binding.
    pub fn new<Irq0>(
        pio: Peri<'d, PIO>,
        dp: Peri<'d, impl PioPin>,
        dm: Peri<'d, impl PioPin>,
        irq0: Irq0,
        pulldown: Pulldown,
    ) -> Self
    where
        Irq0: Binding<<PIO as Instance>::Interrupt, InterruptHandler<PIO>>,
    {
        let bus = PioUsbBus::new(pio, dp, dm, irq0, pulldown);
        Self::from_bus(bus)
    }

    /// Perform one non-blocking keep-alive check.
    ///
    /// If an in-flight transfer owns the bus, this frame is skipped because that
    /// transfer itself provides bus activity. [`Self::idle_task`] calls this once
    /// per frame.
    fn tick(&self) {
        if let Ok(mut bus) = self.bus.try_lock() {
            bus.keepalive();
        }
    }

    /// Run the keep-alive task for as long as the bus is in use.
    ///
    /// This future never returns. Spawn it once beside the USB host stack so full-speed
    /// devices receive SOFs and low-speed devices receive keep-alive EOPs between
    /// transfers.
    pub async fn idle_task(&self) {
        loop {
            self.tick();
            PioUsbBus::<PIO>::wait_for_next_frame().await;
        }
    }

    /// Lock and expose the underlying direct bus.
    ///
    /// Most callers should use [`Self::controller`] instead; this is intended for code
    /// that needs direct access to the root-port primitives.
    pub async fn lock(&'d self) -> MutexGuard<'d, CriticalSectionRawMutex, PioUsbBus<'d, PIO>> {
        self.bus.lock().await
    }
}
