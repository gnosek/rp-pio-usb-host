//! Async `embassy` integration for sharing the direct PIO USB bus.
//!
//! The wrapper serializes access to the physical root port, provides an idle task
//! for frame keep-alives, and exposes the host-controller/allocator types used by
//! `embassy-usb-host`.

use crate::bus::{Bus as PioUsbBus, Pulldown};
use crate::pio_instance::UsbPioInstance;
use core::marker::PhantomData;
use embassy_rp::Peri;
use embassy_rp::interrupt::typelevel::Binding;
use embassy_rp::peripherals::PIO0;
use embassy_rp::pio::{Instance, InterruptHandler, PioPin};
use embassy_sync::blocking_mutex::raw::CriticalSectionRawMutex;
use embassy_sync::mutex::{Mutex, MutexGuard};
use embassy_time::{Duration, Instant, Timer};
use embassy_usb_driver::EndpointInfo;
use embassy_usb_driver::host::{
    DeviceEvent, HostError, PipeError, SplitInfo, TimeoutConfig, UsbHostAllocator,
    UsbHostController, UsbPipe, pipe,
};

const FRAME_INTERVAL_US: u32 = 1000; // 1 ms

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

    /// Obtain the root-port controller handle.
    pub fn controller<'a>(&'a self) -> PioUsbController<'a, 'd, PIO> {
        PioUsbController { shared: self }
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

/// A single endpoint pipe; implements [`UsbPipe`]. Carries the addressing it needs
/// to build tokens at runtime plus its own data-toggle and timeout state.
pub struct PioPipe<'a, 'd, T: pipe::Type, D: pipe::Direction, PIO: UsbPioInstance = PIO0> {
    /// Shared root-port bus used to execute this pipe's transactions.
    shared: &'a Bus<'d, PIO>,
    /// USB device address.
    addr: u8,
    /// Endpoint number without direction bit.
    ep: u8,
    /// Endpoint maximum packet size.
    mps: u16,
    /// Polling interval for interrupt endpoints, in microseconds.
    interval_us: u32,
    /// Next OUT/IN data toggle (`true` ⇒ DATA1). Initialised to DATA0.
    toggle_data1: bool,
    /// Transfer timeouts supplied by the host stack.
    timeout: TimeoutConfig,
    /// Type-level endpoint kind and direction markers.
    _markers: PhantomData<(T, D)>,
}

impl<'a, 'd: 'a, T: pipe::Type, D: pipe::Direction, PIO: UsbPioInstance> UsbPipe<T, D>
    for PioPipe<'a, 'd, T, D, PIO>
{
    async fn control_in(&mut self, setup: &[u8; 8], buf: &mut [u8]) -> Result<usize, PipeError>
    where
        T: pipe::IsControl,
        D: pipe::IsIn,
    {
        // The trait contract is retry-until-timeout. Keep-alive is emitted at
        // transaction boundaries so retries also preserve the USB frame timebase.
        let deadline =
            Instant::now() + Duration::from_millis(self.timeout.data_timeout.as_millis() as u64);
        loop {
            let res = {
                let mut bus = self.shared.bus.lock().await;
                bus.control_in(self.addr, self.ep, self.mps, setup, buf)
            };

            match res {
                Err(PipeError::Timeout) if Instant::now() < deadline => {
                    Timer::after(Duration::from_micros(FRAME_INTERVAL_US as u64)).await;
                }
                _ => return res,
            }
        }
    }

    async fn control_out(&mut self, setup: &[u8; 8], buf: &[u8]) -> Result<(), PipeError>
    where
        T: pipe::IsControl,
        D: pipe::IsOut,
    {
        // Same retry-until-timeout contract as `control_in`. No-data control
        // writes use `no_data_timeout`; writes with an OUT data stage use
        // `data_timeout`.
        let timeout = if buf.is_empty() {
            self.timeout.no_data_timeout
        } else {
            self.timeout.data_timeout
        };
        let deadline = Instant::now() + Duration::from_millis(timeout.as_millis() as u64);
        loop {
            let res = {
                let mut bus = self.shared.bus.lock().await;
                bus.control_out(self.addr, self.ep, self.mps, setup, buf)
            };

            match res {
                Err(PipeError::Timeout) if Instant::now() < deadline => {
                    Timer::after(Duration::from_micros(FRAME_INTERVAL_US as u64)).await;
                }
                _ => return res,
            }
        }
    }

    async fn request_in(&mut self, buf: &mut [u8]) -> Result<usize, PipeError>
    where
        D: pipe::IsIn,
    {
        // Interrupt/bulk IN: NAK means no data yet; wait one endpoint polling
        // interval (minimum one frame) and retry. Callers impose cancellation by
        // dropping this future.
        loop {
            let res = {
                let mut bus = self.shared.bus.lock().await;
                bus.request_in(self.addr, self.ep, buf, &mut self.toggle_data1)
            };

            match res {
                Err(PipeError::Timeout) => {
                    Timer::after(Duration::from_micros(self.interval_us as u64)).await;
                }
                _ => return res,
            }
        }
    }

    async fn request_out(
        &mut self,
        buf: &[u8],
        _ensure_transaction_end: bool,
    ) -> Result<(), PipeError>
    where
        D: pipe::IsOut,
    {
        // Interrupt/bulk OUT, single packet. NAK means device busy; retry until
        // the request future is dropped.
        loop {
            let res = {
                let mut bus = self.shared.bus.lock().await;
                bus.request_out(self.addr, self.ep, buf, &mut self.toggle_data1)
            };

            match res {
                Err(PipeError::Timeout) => {
                    Timer::after(Duration::from_micros(self.interval_us as u64)).await;
                }
                _ => return res,
            }
        }
    }

    fn set_timeout(&mut self, timeout: TimeoutConfig)
    where
        T: pipe::IsControl,
    {
        self.timeout = timeout;
    }

    fn reset_data_toggle(&mut self)
    where
        T: pipe::IsBulkOrInterrupt,
    {
        self.toggle_data1 = false;
    }
}

/// Pipe allocator handle; implements [`UsbHostAllocator`]. Cloneable — every clone
/// shares the same underlying bus.
pub struct PioUsbAllocator<'a, 'd, PIO: UsbPioInstance = PIO0> {
    /// Shared root-port bus state.
    shared: &'a Bus<'d, PIO>,
}

// Manual `Clone` (not `#[derive]`): the only field is a shared reference, so cloning
// never touches `PIO` — deriving would spuriously require `PIO: Clone`, which the PIO
// peripheral singleton is not.
impl<'a, 'd, PIO: UsbPioInstance> Clone for PioUsbAllocator<'a, 'd, PIO> {
    fn clone(&self) -> Self {
        Self {
            shared: self.shared,
        }
    }
}

impl<'a, 'd: 'a, PIO: UsbPioInstance> UsbHostAllocator<'a> for PioUsbAllocator<'a, 'd, PIO> {
    type Pipe<T: pipe::Type, D: pipe::Direction> = PioPipe<'a, 'd, T, D, PIO>;

    fn alloc_pipe<T: pipe::Type, D: pipe::Direction>(
        &self,
        addr: u8,
        endpoint: &EndpointInfo,
        split: Option<SplitInfo>,
    ) -> Result<Self::Pipe<T, D>, HostError> {
        if split.is_some() {
            // `split == None` covers root-port devices AND full-speed devices behind a
            // full-speed hub (the host stack's hub routing yields no split there), so
            // those work. A `Some(split)` is either low-speed-behind-a-hub (legacy PRE)
            // or a high-speed Transaction-Translator split — neither implemented.
            return Err(HostError::Other(
                "split transactions (LS-via-hub PRE / HS TT) unsupported",
            ));
        }
        Ok(PioPipe {
            shared: self.shared,
            addr,
            ep: endpoint.addr.index() as u8,
            mps: endpoint.max_packet_size,
            interval_us: (u32::from(endpoint.interval_ms) * 1000).max(FRAME_INTERVAL_US),
            toggle_data1: false,
            timeout: TimeoutConfig::default(),
            _markers: PhantomData,
        })
    }
}

/// Root-port controller; implements [`UsbHostController`]. `'a` is the borrow of the
/// shared bus, `'d` the bus's own (peripheral) lifetime, with `'d: 'a`.
pub struct PioUsbController<'a, 'd, PIO: UsbPioInstance = PIO0> {
    /// Shared root-port bus state.
    shared: &'a Bus<'d, PIO>,
}

impl<'a, 'd: 'a, PIO: UsbPioInstance> UsbHostController<'a> for PioUsbController<'a, 'd, PIO> {
    type Allocator = PioUsbAllocator<'a, 'd, PIO>;

    fn allocator(&self) -> Self::Allocator {
        PioUsbAllocator {
            shared: self.shared,
        }
    }

    async fn wait_for_device_event(&mut self) -> DeviceEvent {
        loop {
            let event = {
                let mut bus = self.shared.bus.lock().await;
                let event = bus.poll_device_event();
                if matches!(event, Some(DeviceEvent::Connected(_))) {
                    bus.bus_reset().await;
                }
                event
            };
            if let Some(event) = event {
                return event;
            }
            PioUsbBus::<PIO>::wait_for_next_frame().await;
        }
    }

    async fn bus_reset(&mut self) {
        let mut bus = self.shared.bus.lock().await;
        bus.bus_reset().await
    }
}
