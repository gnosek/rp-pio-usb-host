//! Direct root-port USB bus built from one RP PIO block and two adjacent GPIOs.
//!
//! This layer owns packet transmission, packet reception, speed detection, debounce,
//! reset, and low-/full-speed keep-alives. Higher-level adapters can build control,
//! bulk, and interrupt transfers on top of these primitives.

use crate::pio_instance::UsbPioInstance;
use crate::ram::now_us;
use crate::rx_driver::{RxDriver, RxPacketStatus};
use crate::tx_driver::TxDriver;
use embassy_rp::Peri;
use embassy_rp::interrupt::typelevel::Binding;
use embassy_rp::pio::{Common, Instance, InterruptHandler, Pin, Pio, PioPin};
use embassy_time::{Duration, Timer};
use embassy_usb_driver::Speed;
use embassy_usb_driver::host::{DeviceEvent, PipeError};

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

/// `control_out` STATUS-stage IN polls before giving up.
///
/// Devices may NAK the write-status IN while processing requests such as
/// `SET_CONFIGURATION`; retrying the whole transfer too early restarts the request.
const STATUS_POLL_ATTEMPTS: u32 = 400;

/// `control_in` DATA-stage NAK-poll budget per packet.
///
/// The budget resets after each successfully received packet so a slow multi-packet
/// descriptor cannot spend the entire allowance before later packets are ready.
const DATA_STALL_BUDGET: u32 = 64;

/// Absolute `control_in` poll cap for the whole DATA stage.
///
/// This bounds devices that keep returning full-size packets or NAK forever.
const DATA_TOTAL_CAP: u32 = 400;

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

/// Classified response to an IN token.
enum InReply {
    /// No response was captured after the IN token.
    NoReply,
    /// Device returned a NAK handshake.
    Nak,
    /// Device returned DATA0/DATA1.
    Data {
        /// Raw PID byte (`DATA0` or `DATA1`).
        pid: u8,
        /// Whether CRC16 and DATA PID checks passed.
        valid_crc: bool,
        /// Payload bytes, excluding SYNC, PID, and CRC16.
        payload_len: usize,
    },
    /// Any other non-terminal response.
    Other,
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

    /// Scratch buffer for the NRZI/bit-stuff encoder.
    enc: [u8; crate::encoding::MAX_ENCODED_PACKET_BYTES],

    /// Transmit driver for sending packets on the bus.
    tx: TxDriver<'a, PIO>,

    /// Receive driver for receiving packets from the bus.
    rx: RxDriver<'a, PIO>,
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
            sm1: rx_det_sm,
            sm2: rx_dec_sm,
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
        let rx = RxDriver::init(
            &mut common,
            rx_det_sm,
            rx_dec_sm,
            &dp,
            &dm,
            gpio_high_window,
        );

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
            enc: [0u8; crate::encoding::MAX_ENCODED_PACKET_BYTES],
            tx,
            rx,
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
        self.rx.set_speed(speed);
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
        self.mark_activity();
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

    /// Send an IN token, catch the device reply, and ACK valid DATA before returning.
    ///
    /// This spans the TX→RX turnaround (`transmit_for_reply` returns with RX armed, then
    /// `receive_data_and_ack` pre-stages/fires the host ACK), so keep the wrapper itself in
    /// RAM as well as the transmit/receive helpers it calls.
    #[unsafe(link_section = ".data.ram_func")]
    #[inline(never)]
    fn in_reply(&mut self, in_tok: &[u32], pkt: &mut [u8]) -> Result<InReply, PipeError> {
        self.transmit_for_reply(in_tok, None);
        let (n, status, ack_sent) = self.receive_data_and_ack(pkt);
        if status == RxPacketStatus::Overflow {
            return Err(PipeError::Babble);
        }
        if n < 2 {
            return Ok(InReply::NoReply);
        }

        use crate::pid;

        match (ack_sent, pkt[1]) {
            (_, pid::USB_PID_STALL) => Err(PipeError::Stall),
            (_, pid::USB_PID_NAK) => Ok(InReply::Nak),
            (_, pid @ (pid::USB_PID_DATA0 | pid::USB_PID_DATA1)) => {
                let valid_crc = status == RxPacketStatus::ValidData;
                if ack_sent && valid_crc {
                    self.tx.wait();
                }
                Ok(InReply::Data {
                    pid,
                    valid_crc,
                    payload_len: n.saturating_sub(4),
                })
            }
            _ => Ok(InReply::Other),
        }
    }

    /// Transmit one or two packets, then arm RX for the device reply.
    ///
    /// Used for token-only IN transactions and token+DATA OUT/SETUP transactions.
    #[unsafe(link_section = ".data.ram_func")]
    #[inline(never)]
    pub(crate) fn transmit_for_reply(&mut self, first: &[u32], second: Option<&[u32]>) {
        self.transmit_for_reply_inner(first, second);
    }

    /// Shared RAM-inlined body for [`transmit_for_reply`](Self::transmit_for_reply)
    /// and [`transmit_and_check_ack`](Self::transmit_and_check_ack).
    #[inline(always)]
    fn transmit_for_reply_inner(&mut self, first: &[u32], second: Option<&[u32]>) {
        self.rx.prepare_for_receive();
        self.tx.transmit(first);
        if let Some(second) = second {
            self.tx.transmit(second);
        }
        self.rx.start_receive();
    }

    /// Transmit a token or token+DATA pair and interpret the device handshake.
    ///
    /// Returns `Ok(true)` for ACK, `Ok(false)` for no reply/NAK/other non-ACK
    /// response, and `Err(PipeError::Stall)` for STALL.
    #[unsafe(link_section = ".data.ram_func")]
    #[inline(never)]
    pub(crate) fn transmit_and_check_ack(
        &mut self,
        first: &[u32],
        second: Option<&[u32]>,
    ) -> Result<bool, PipeError> {
        self.transmit_for_reply_inner(first, second);
        let mut hbuf = [0u8; 8];
        let (hlen, _) = self.rx.receive(&mut hbuf);
        self.mark_activity();
        if hlen < 2 {
            return Ok(false);
        }
        match hbuf[1] {
            crate::pid::USB_PID_ACK => Ok(true),
            crate::pid::USB_PID_STALL => Err(PipeError::Stall),
            _ => Ok(false),
        }
    }

    /// Receive one device DATA packet and ACK it immediately if valid.
    ///
    /// `out` receives `[SYNC, PID, payload..., CRC16_lo, CRC16_hi]`. Returns the
    /// packet length, receive status, and whether the pre-staged ACK was sent.
    ///
    /// USB handshakes have tight response timing after EOP (USB 2.0 §7.1.18.2), so
    /// the ACK is preloaded before receiving and fired with a single SM-enable write.
    /// CRC16 is updated per byte as data arrives, letting the EOP path decide validity
    /// with a residue comparison instead of a post-packet CRC pass.
    #[unsafe(link_section = ".data.ram_func")]
    #[inline(never)]
    pub(crate) fn receive_data_and_ack(&mut self, out: &mut [u8]) -> (usize, RxPacketStatus, bool) {
        // Pre-stage the ACK off the EOP→ACK path. Release the bus last so the
        // device reply is not collided with during capture.
        self.tx.prepare_ack_and_release_bus();

        let (n, status) = self.rx.receive(out);
        if status == RxPacketStatus::ValidData {
            // Fire the pre-staged ACK with one MMIO write.
            self.tx.start_tx();
        }
        self.mark_activity();
        (n, status, status == RxPacketStatus::ValidData)
    }

    /// One **OUT** transaction (interrupt/bulk or one control-OUT data packet): OUT token + DATA →
    /// device handshake. `data1` selects the DATA1/DATA0 toggle. Returns `Ok(true)`
    /// on device ACK, `Ok(false)` on NAK (caller retries), `Err(Stall)` on stall.
    ///
    /// This helper sends a single DATA packet; callers that need multi-packet OUT
    /// transfers must split the payload and manage toggles.
    pub(crate) fn out_once(
        &mut self,
        addr: u8,
        ep: u8,
        data1: bool,
        data: &[u8],
    ) -> Result<bool, PipeError> {
        let out_tok = crate::encoding::build_token(crate::pid::USB_PID_OUT, addr, ep);
        let pid = if data1 {
            crate::pid::USB_PID_DATA1
        } else {
            crate::pid::USB_PID_DATA0
        };
        let mut dbuf = [0u8; crate::encoding::MAX_DATA_PACKET_BYTES];

        let mut data_w = [0u32; crate::encoding::MAX_DATA_PACKET_WORDS];
        let data_w = crate::encoding::build_data(pid, data, &mut dbuf, &mut self.enc, &mut data_w)
            .ok_or(PipeError::BufferOverflow)?;

        self.transmit_and_check_ack(&out_tok, Some(data_w))
    }

    /// Send the SETUP token and DATA0 setup packet, expecting an ACK handshake.
    fn control_setup(&mut self, addr: u8, ep: u8, setup: &[u8; 8]) -> Result<(), PipeError> {
        self.keepalive();

        let setup_tok = crate::encoding::build_token(crate::pid::USB_PID_SETUP, addr, ep);
        let mut data0 = [0u8; crate::encoding::MAX_DATA_PACKET_BYTES];
        let mut data_w = [0u32; crate::encoding::MAX_DATA_PACKET_WORDS];
        let data_w = crate::encoding::build_data(
            crate::pid::USB_PID_DATA0,
            setup,
            &mut data0,
            &mut self.enc,
            &mut data_w,
        )
        .ok_or(PipeError::BufferOverflow)?;

        if self.transmit_and_check_ack(&setup_tok, Some(data_w))? {
            Ok(())
        } else {
            Err(PipeError::Timeout)
        }
    }

    /// Execute one control-IN transfer and copy the DATA stage into `data`.
    ///
    /// Sends SETUP, polls IN packets until the requested length or a short packet is
    /// received, ACKs each valid DATA packet, then completes the status OUT stage.
    /// Returns the number of payload bytes copied into `data`.
    pub fn control_in(
        &mut self,
        addr: u8,
        ep: u8,
        mps: u16,
        setup: &[u8; 8],
        data: &mut [u8],
    ) -> Result<usize, PipeError> {
        self.keepalive();
        // wLength from the SETUP request: the data stage also ends once this many bytes
        // are received (not only on a short packet) — essential when the configured `mps`
        // doesn't match the device's real mps0 (e.g. the bootstrap device-descriptor read
        // at mps=8, where the device's single 18-byte packet is never "< mps").
        let wlen = u16::from_le_bytes([setup[6], setup[7]]) as usize;
        if wlen > data.len() {
            return Err(PipeError::BufferOverflow);
        }

        // Build + encode the fixed packets for this transfer.
        let in_tok = crate::encoding::build_token(crate::pid::USB_PID_IN, addr, ep);

        self.control_setup(addr, ep, setup)?;

        // ---- DATA-IN stage: poll IN, ACK each DATA, accumulate until short. ----
        let mut total = 0usize;
        let mut expect_data1 = true; // first data packet of a control-IN is DATA1
        let mut pkt = [0u8; crate::encoding::MAX_DATA_PACKET_BYTES];
        let mut completed = wlen == 0;
        // Each packet gets its own NAK-poll budget, reset on a received packet;
        // DATA_TOTAL_CAP bounds a device that never sends a short packet.
        let mut stall_polls = 0u32;
        let mut total_polls = 0u32;
        loop {
            if stall_polls >= DATA_STALL_BUDGET || total_polls >= DATA_TOTAL_CAP || completed {
                break;
            }
            stall_polls += 1;
            total_polls += 1;

            let (is_data1, payload_len) = match self.in_reply(&in_tok, &mut pkt)? {
                InReply::NoReply => continue,
                InReply::Data {
                    pid: crate::pid::USB_PID_DATA0,
                    valid_crc: true,
                    payload_len,
                } => (false, payload_len),
                InReply::Data {
                    pid: crate::pid::USB_PID_DATA1,
                    valid_crc: true,
                    payload_len,
                } => (true, payload_len),
                InReply::Data {
                    valid_crc: false, ..
                } => continue,
                _ => continue,
            };

            if is_data1 == expect_data1 {
                if total + payload_len > data.len() {
                    return Err(PipeError::BufferOverflow);
                }
                for i in 0..payload_len {
                    data[total] = pkt[2 + i];
                    total += 1;
                }
                expect_data1 = !expect_data1;
                stall_polls = 0; // progress — give the next packet a fresh budget
                if payload_len < mps as usize || total >= wlen {
                    completed = true;
                }
            }
        }

        if !completed {
            return Err(PipeError::Timeout);
        }

        // ---- STATUS stage: host sends OUT + zero-length DATA1, expects ACK. ----
        for _ in 0..DATA_STALL_BUDGET {
            if self.out_once(addr, ep, true, &[])? {
                return Ok(total);
            }
        }
        Err(PipeError::Timeout)
    }

    /// One **control-OUT** transfer: SETUP(token + DATA0 request) → optional OUT **data
    /// stage** → STATUS (host IN → device returns a zero-length DATA1 → host ACK).
    ///
    /// With `data` empty this is a no-data control write (`SET_ADDRESS`,
    /// `SET_CONFIGURATION`, HID `SET_IDLE`/`SET_PROTOCOL`, hub port features). With `data`
    /// non-empty it is a control write *with* an OUT data stage — e.g. HID `SET_REPORT`.
    /// The data stage starts on the **DATA1** toggle and is split into `mps`-sized packets;
    /// each is retried on NAK. The control STATUS stage of a write is always an **IN**
    /// (device returns a ZLP), regardless of whether there was an OUT data stage.
    pub fn control_out(
        &mut self,
        addr: u8,
        ep: u8,
        mps: u16,
        setup: &[u8; 8],
        data: &[u8],
    ) -> Result<(), PipeError> {
        let in_tok = crate::encoding::build_token(crate::pid::USB_PID_IN, addr, ep);

        // ---- SETUP stage. ----
        self.control_setup(addr, ep, setup)?;

        // ---- OUT data stage (control write with data): DATA1, DATA0, … per `mps`. ----
        if !data.is_empty() {
            let mps = mps.max(1) as usize;
            let mut toggle_data1 = true; // first control-OUT data packet is DATA1
            let mut off = 0usize;
            while off < data.len() {
                let end = (off + mps).min(data.len());
                let chunk = &data[off..end];
                let mut acked = false;
                for _ in 0..DATA_STALL_BUDGET {
                    if self.out_once(addr, ep, toggle_data1, chunk)? {
                        acked = true;
                        break;
                    }
                    // NAK / no handshake — device busy, retry this same packet/toggle.
                }
                if !acked {
                    return Err(PipeError::Timeout);
                }
                toggle_data1 = !toggle_data1;
                off = end;
            }
        }

        // ---- STATUS stage: IN → device sends zero-length DATA1 → host ACK. ----
        // Poll the STATUS IN *persistently* (not just a few times): a device NAKs the
        // status IN while it completes the request — SET_CONFIGURATION on a multi-interface
        // device can take milliseconds to bring up all its endpoints. Giving up early lets
        // the caller's retry re-issue the SETUP, which RESTARTS the request before the
        // device can send its ZLP, so it's never caught (the device configures but our
        // control_out reports Timeout — which a host stack treats as failure). Returns Ok
        // the instant the ZLP arrives, so a fast device/link is unaffected.
        let mut pkt = [0u8; 8];
        for _ in 0..STATUS_POLL_ATTEMPTS {
            match self.in_reply(&in_tok, &mut pkt)? {
                InReply::Data {
                    valid_crc: true, ..
                } => return Ok(()),
                InReply::Data {
                    valid_crc: false, ..
                } => continue,
                _ => continue,
            }
        }
        Err(PipeError::Timeout)
    }
}
