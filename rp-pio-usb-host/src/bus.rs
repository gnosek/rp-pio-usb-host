//! Direct root-port USB bus built from one RP PIO block and two adjacent GPIOs.
//!
//! This layer owns packet transmission, packet reception, speed detection, debounce,
//! reset, and low-/full-speed keep-alives. Higher-level adapters can build control,
//! bulk, and interrupt transfers on top of these primitives.

use crate::pio_instance::UsbPioInstance;
use crate::ram::now_us;
use crate::tx_driver::TxDriver;
use embassy_rp::Peri;
use embassy_rp::interrupt::typelevel::Binding;
use embassy_rp::pio::{Common, Instance, InterruptHandler, Pin, Pio, PioPin};
use embassy_time::{Duration, Timer};
use embassy_usb_driver::Speed;
use embassy_usb_driver::host::DeviceEvent;

/// Root-port reset SE0 duration.
///
/// USB 2.0 §7.1.7.5 "Reset Signaling" (T_DRST, table 7-14) requires at least
/// 10 ms of SE0; 15 ms leaves some margin.
const RESET_SE0_US: u64 = 15_000;

/// Reset-recovery hold: after releasing reset, emit SOF-only frames (no transactions)
/// for this many frames before talking to the device. USB 2.0 §7.1.7.5 reset-recovery
/// interval T_RSTRCY (table 7-14) is ≥10 ms; at one frame/ms (below) 15 frames ≈ 15 ms.
const RESET_RECOVERY_FRAMES: u32 = 15;

/// Debounce counter
///
/// A device has to be plugged in for at least this many frames (1 ms each) before we
/// consider it "attached".
const DEBOUNCE_FRAMES: u32 = 15;

/// Debounce cap
///
/// A device has to be unplugged for at least this many frames (1 ms each), but longer
/// than the time spent plugged in, before we consider it "detached".
const DEBOUNCE_CAP: u32 = 60;

/// Full-speed frame interval in microseconds.
///
/// USB 2.0 §7.1.12 and §8.4.3 define one full-speed frame every 1.000 ms
/// ±0.0005 ms. This is used as the SOF/low-speed keep-alive period, presence-poll
/// period, and retry cadence. The bus-idle test uses the RAM-safe [`now_us`]
/// helper because it runs in the synchronous transaction path.
const FRAME_INTERVAL_US: u32 = 1000;

/// Pull-down configuration for the USB bus.
///
/// The USB spec mandates 15k pull-downs on the D+/D- lines to detect device presence.
/// The PIO-USB host transport can either drive internal pull-downs
/// on the D+/D- lines or leave it to external pull-down resistors. External pull-downs
/// are preferable, as the internal ones do not meet the USB spec.
///
/// Using the internal pull-downs may let you get away with wiring the D+/D- lines
/// directly to a socket, but it is not recommended for production use.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Pulldown {
    /// Enable the RP's internal D+/D- pull-downs.
    Internal,
    /// Leave D+/D- pull-downs to external resistors.
    External,
}

/// Physical USB bus, implemented with a PIO block and two GPIO pins.
pub struct Bus<'a, PIO: UsbPioInstance> {
    /// Shared PIO ownership state kept alive for the loaded programs and claimed pins.
    _common: Common<'a, PIO>,
    /// D+ PIO pin
    dp: Pin<'a, PIO>,
    /// D- PIO pin
    dm: Pin<'a, PIO>,

    /// The speed the transport is currently configured for.
    speed: Speed,

    /// Whether a device is currently attached to the bus.
    attached: bool,

    /// Saturating attach/detach debounce accumulator.
    debounce: u32,

    /// Running SOF frame counter (11-bit), advanced by [`Self::sof`].
    frame: u16,

    /// Timestamp of the last packet put on the bus, in microseconds from [`now_us`].
    ///
    /// For LS devices, keepalives are sent 1 ms after the last bus activity
    /// (the deadline is 3 ms so we try to keep comfortable headroom).
    /// This is updated by [`Self::mark_activity`] from every TX path, including
    /// keep-alive itself.
    last_activity: u32,

    /// Timestamp of the last SOF packet sent, in microseconds from [`now_us`].
    ///
    /// For FS devices, we need to send one SOF frame every millisecond,
    /// *if* the bus is not busy at the moment.
    last_sof: u32,

    /// Transmit driver for sending packets on the bus.
    tx: TxDriver<'a, PIO>,
}

impl<'a, PIO: UsbPioInstance> Bus<'a, PIO> {
    /// Construct a root-port bus from a PIO block, adjacent D+/D- pins, and IRQ binding.
    ///
    /// The PIO state machines are assigned as TX on SM0, RX edge detection on SM1, and
    /// RX decoding on SM2. `dp` and `dm` must be adjacent GPIOs; the constructor panics
    /// otherwise.
    pub fn new<Irq0>(
        pio: Peri<'a, PIO>,
        dp: Peri<'a, impl PioPin>,
        dm: Peri<'a, impl PioPin>,
        irq0: Irq0,
        pulldown: Pulldown,
    ) -> Self
    where
        Irq0: Binding<<PIO as Instance>::Interrupt, InterruptHandler<PIO>>,
    {
        let (dpn, dmn) = (dp.pin(), dm.pin());
        assert_eq!(
            dpn.abs_diff(dmn),
            1,
            "PIO USB bus requires adjacent USB pins"
        );

        // All three USB state machines on the chosen PIO: TX = sm0, detector = sm1,
        // decoder = sm2.
        let Pio {
            mut common,
            sm0: tx_sm,
            ..
        } = Pio::new(pio, irq0);

        // Convert GPIO peripherals into PIO-owned pins before configuring pads.
        let mut dp = common.make_pio_pin(dp);
        let mut dm = common.make_pio_pin(dm);

        let pulldown_cfg = match pulldown {
            Pulldown::Internal => embassy_rp::gpio::Pull::Down,
            Pulldown::External => embassy_rp::gpio::Pull::None,
        };
        dp.set_pull(pulldown_cfg);
        dm.set_pull(pulldown_cfg);

        // The PIO programs are written for inverted line sense.
        // Most of the interesting states are SE0 (both low) and J/K (one high, one low).
        // PIO only supports waiting/conditionally jumping when a pin is high,
        // so invert the inputs to simplify the PIO programs.
        crate::chip::set_gpio_input_inversion(dpn, true);
        crate::chip::set_gpio_input_inversion(dmn, true);

        let gpio_high_window = crate::chip::configure_pio_gpio_base::<PIO>(dpn, dmn);

        let tx = TxDriver::init(&mut common, tx_sm, &dp, &dm, gpio_high_window);

        let now = now_us();
        Self {
            _common: common,
            dp,
            dm,
            speed: Speed::Full,
            attached: false,
            debounce: 0,
            frame: 0,
            last_activity: now,
            last_sof: now,
            tx,
        }
    }

    /// Sense the attached device speed from the idle line state.
    ///
    /// A full-speed device pulls D+ high; a low-speed device pulls D- high
    /// (USB 2.0 §7.1.5.1). Both GPIO inputs are inverted for the RX PIO programs,
    /// so a real high level reads as `false` through [`crate::chip::gpio_input_level`].
    /// Returns `None` for SE0/no-device or transient invalid states; callers use
    /// [`Self::poll_device_event`] to debounce that raw state.
    fn detect_speed(&self) -> Option<Speed> {
        let dp_high = !crate::chip::gpio_input_level(self.dp.pin()); // real D+ high (inverted input reads 0)
        let dm_high = !crate::chip::gpio_input_level(self.dm.pin()); // real D- high
        match (dp_high, dm_high) {
            (true, false) => Some(Speed::Full), // D+ pulled up ⇒ full-speed device
            (false, true) => Some(Speed::Low),  // D- pulled up ⇒ low-speed device
            _ => None,                          // SE0 (no device) or transient/invalid
        }
    }

    fn set_speed(&mut self, speed: Speed) {
        self.speed = speed;
        self.tx.set_speed(speed);
    }

    /// Drive a single SOF frame (keep-alive). Advances the internal frame counter.
    fn sof(&mut self) {
        self.last_sof = now_us();
        let sof = crate::encoding::build_sof(self.frame);
        self.tx.transmit(&sof);
        self.frame = self.frame.wrapping_add(1) & 0x7ff;
    }

    /// Low-speed keep-alive: send a single low-speed **EOP** via the TX player. Encoding
    /// an empty payload yields just `[SE0, COMP]`, so the player drives SE0 for its EOP
    /// slot — `irq 0 side 0b00 [7]` = 8 SM cycles = **1.33 µs at the LS clock = exactly 2
    /// LS bit-times** — then releases. A spec-correct keep-alive (USB 2.0 §7.1.7.4 /
    /// §11.8.4.1); LS devices have no SOF, so this per-frame EOP is what keeps them awake
    /// and gives them a bus-derived frame timebase.
    ///
    /// The precise width matters because reset detection can begin after roughly
    /// 2.5 µs of SE0 (T_DETRST, USB 2.0 table 7-14). Letting the PIO player time
    /// the EOP from the low-speed divider keeps it at two bit-times.
    fn ls_keepalive(&mut self) {
        self.mark_activity();
        self.tx.transmit(&crate::encoding::LS_KEEPALIVE_PACKET);
    }

    fn keepalive(&mut self) {
        if !self.attached {
            return;
        }

        match self.speed {
            Speed::Full => {
                if now_us().wrapping_sub(self.last_sof) >= FRAME_INTERVAL_US {
                    self.sof()
                }
            }
            Speed::Low => {
                if now_us().wrapping_sub(self.last_activity) >= FRAME_INTERVAL_US {
                    self.ls_keepalive()
                }
            }
            _ => (), // only FS and LS are supported by the PIO host transport
        }
    }

    /// Sample and debounce the root-port line state once without waiting.
    ///
    /// A connected event leaves reset to the caller so adapters can keep the
    /// shared bus locked across the state transition and reset, while releasing
    /// it between ordinary debounce samples.
    pub(crate) fn poll_device_event(&mut self) -> Option<DeviceEvent> {
        let speed = self.detect_speed();
        if speed.is_some() {
            self.debounce = (self.debounce + 1).min(DEBOUNCE_CAP);
        } else {
            self.debounce = self.debounce.saturating_sub(1);
        }

        if !self.attached
            && self.debounce >= DEBOUNCE_FRAMES
            && let Some(speed) = speed
        {
            self.set_speed(speed);
            self.attached = true;

            return Some(DeviceEvent::Connected(speed));
        }

        if self.attached && self.debounce == 0 && speed.is_none() {
            self.attached = false;
            self.tx.release_bus();
            return Some(DeviceEvent::Disconnected);
        }

        None
    }

    #[inline(always)]
    async fn wait_for_next_frame() {
        Timer::after_micros(FRAME_INTERVAL_US as u64).await;
    }

    async fn bus_reset(&mut self) {
        self.tx.drive_reset_se0();
        Timer::after(Duration::from_micros(RESET_SE0_US)).await;
        self.tx.release_reset();

        for _ in 0..RESET_RECOVERY_FRAMES {
            self.keepalive();
            Self::wait_for_next_frame().await;
        }
    }

    /// Wait until the debounced root port reports a connection or disconnection.
    ///
    /// On connection this also performs USB reset and reset recovery before returning
    /// the event, so callers can begin enumeration immediately.
    pub async fn wait_for_device_event(&mut self) -> DeviceEvent {
        loop {
            match self.poll_device_event() {
                Some(ev @ DeviceEvent::Connected(_)) => {
                    self.bus_reset().await;
                    return ev;
                }
                Some(ev) => {
                    return ev;
                }
                None => {
                    Self::wait_for_next_frame().await;
                }
            }
        }
    }

    /// Record that a packet was just transmitted.
    ///
    /// Any bus activity — SOF, keep-alive, or a transaction's token/DATA — resets
    /// the attached device's 3 ms suspend timer (USB 2.0 §7.1.7.6), so every TX
    /// path stamps this. Uses the RAM-safe [`now_us`] helper because it can run
    /// at transaction boundaries.
    #[inline(always)]
    fn mark_activity(&mut self) {
        self.last_activity = now_us();
    }
}
