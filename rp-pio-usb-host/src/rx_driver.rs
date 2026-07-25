use crate::pio_instance::UsbPioInstance;
use crate::rx_pio::{
    DECODER_PRIME_OSR, DECODER_PRIME_X, DECODER_TRIGGER, IRQ_RX_ALL, IRQ_RX_EOP, IRQ_RX_START,
    usb_edge_detector_program,
};
use crate::{chip, ram};
use embassy_rp::pio::{
    Common, Config as PioConfig, FifoJoin, LoadedProgram, Pin as PioGpioPin, ShiftConfig,
    ShiftDirection, StateMachine,
};
use embassy_usb_driver::Speed;

/// Edge-detector state-machine clock for full speed: 8x oversample of 12 Mb/s.
const DET_CLOCK_HZ_FS: u32 = 96_000_000;
/// Edge-detector state-machine clock for low speed: 8x oversample of 1.5 Mb/s.
const DET_CLOCK_HZ_LS: u32 = 12_000_000;

/// Maximum time to wait for SOP after arming RX.
///
/// USB low-/full-speed device turnaround is bounded in bit times (USB 2.0
/// §7.1.18.1). This window is intentionally wider than the nominal turnaround to
/// absorb software arm latency and slightly marginal devices.
const RX_SOP_TIMEOUT_US: u32 = 8;

/// Maximum idle gap between decoded bytes once a packet has started.
///
/// A full-speed byte is about 0.67 µs on the wire before bit-stuffing; this
/// timeout gives the PIO decoder/FIFO drain path slack while still terminating
/// malformed packets promptly.
const RX_DRAIN_TIMEOUT_US: u32 = 7;

/// Receive-side state: edge detector on SM1 and NRZI decoder on SM2.
pub(crate) struct RxDriver<'d, PIO: UsbPioInstance> {
    /// Edge-detector state machine. Raises packet-start, EOP, and bit-trigger IRQs.
    det_sm: StateMachine<'d, PIO, 1>,
    /// NRZI decoder state machine. Pushes decoded bytes into RX FIFO.
    dec_sm: StateMachine<'d, PIO, 2>,

    /// Loaded programs, retained so [`Self::set_speed`] can rebuild each state
    /// machine's configuration when the device speed changes.
    edge_loaded: LoadedProgram<'d, PIO>,
    dec_loaded: LoadedProgram<'d, PIO>,

    /// Absolute PIO instruction address of the edge detector's EOP wait loop.
    det_jmp_eop: u16,
    /// Absolute PIO instruction address of the decoder's `start` label.
    dec_jmp_start: u16,

    /// GPIO_BASE-relative pin number for D+.
    dp_pin: u8,
    /// GPIO_BASE-relative pin number for D-.
    dm_pin: u8,
}

/// Result classification for one receive attempt.
#[derive(Clone, Copy, Eq, PartialEq)]
pub(crate) enum RxPacketStatus {
    /// No packet start was observed before [`RX_SOP_TIMEOUT_US`].
    NoReply,
    /// A packet was observed but was not a valid DATA0/DATA1 packet with good CRC16.
    Invalid,
    /// More decoded bytes arrived than the caller's output buffer could hold.
    Overflow,
    /// A DATA0/DATA1 packet with valid CRC16 was received.
    ValidData,
}

impl<'d, PIO: UsbPioInstance> RxDriver<'d, PIO> {
    /// Load RX programs, configure GPIO input inversion, and initialize for full speed.
    pub(crate) fn init(
        pio_common: &mut Common<'d, PIO>,
        det_sm: StateMachine<'d, PIO, 1>,
        dec_sm: StateMachine<'d, PIO, 2>,
        dp: &PioGpioPin<'d, PIO>,
        dm: &PioGpioPin<'d, PIO>,
        gpio_high_window: bool,
    ) -> Self {
        let (dpn, dmn) = (dp.pin(), dm.pin());
        let dpn = chip::pio_local_pin(dpn, gpio_high_window);
        let dmn = chip::pio_local_pin(dmn, gpio_high_window);

        // RX SMs only read D+/D-. Sharing the PIO block with TX is safe because
        // IRQ flags do not overlap: TX uses flag 0, RX uses flags 1-4.
        let edge_prog = usb_edge_detector_program();
        let edge_loaded = pio_common.load_program(&edge_prog);
        let det_jmp_eop: u16 = edge_loaded.origin as u16;

        let dec_prog = crate::rx_pio::usb_nrzi_decoder_program();
        let dec_loaded = pio_common.load_program(&dec_prog);
        let dec_jmp_start: u16 = dec_loaded.origin as u16;

        let mut this = Self {
            det_sm,
            dec_sm,
            edge_loaded,
            dec_loaded,
            det_jmp_eop,
            dec_jmp_start,
            dp_pin: dpn,
            dm_pin: dmn,
        };
        this.set_speed(Speed::Full);
        this
    }

    /// Repoint RX sampling for full-speed or direct-root low-speed signalling.
    ///
    /// Low speed uses D- as logical J, so both the edge detector and decoder swap
    /// which physical line they sample.
    pub(crate) fn set_speed(&mut self, speed: Speed) {
        let low = speed == Speed::Low;

        // Quiesce both SMs while their pin selectors and clock dividers change.
        self.det_sm.set_enable(false);
        self.dec_sm.set_enable(false);

        let mut detcfg = PioConfig::default();
        detcfg.use_program(&self.edge_loaded, &[]);
        detcfg.clock_divider = crate::clock::pio_clkdiv_for(if low {
            DET_CLOCK_HZ_LS
        } else {
            DET_CLOCK_HZ_FS
        });
        detcfg.shift_in = ShiftConfig {
            threshold: 8,
            direction: ShiftDirection::Left,
            auto_fill: false,
        };
        // The decoder runs from the PIO clock but only advances on decoder IRQs.
        let mut deccfg = PioConfig::default();
        deccfg.use_program(&self.dec_loaded, &[]);
        deccfg.fifo_join = FifoJoin::RxOnly;
        deccfg.shift_in = ShiftConfig {
            threshold: 8,
            direction: ShiftDirection::Right,
            auto_fill: true,
        };
        self.det_sm.set_config(&detcfg);
        self.dec_sm.set_config(&deccfg);

        // FS: in_pin = D+, jmp_pin = D-. LS mirrors the pair.
        let (dpn, dmn) = (self.dp_pin, self.dm_pin);
        let (det_in, det_jmp): (u8, u8) = if low { (dmn, dpn) } else { (dpn, dmn) };
        PIO::REGS.sm(1).pinctrl().modify(|w| w.set_in_base(det_in));
        PIO::REGS
            .sm(1)
            .execctrl()
            .modify(|w| w.set_jmp_pin(det_jmp));

        let dec_pin: u8 = if low { dmn } else { dpn };
        PIO::REGS.sm(2).pinctrl().modify(|w| w.set_in_base(dec_pin));
        PIO::REGS
            .sm(2)
            .execctrl()
            .modify(|w| w.set_jmp_pin(dec_pin));
        unsafe {
            self.dec_sm.exec_instr(DECODER_PRIME_OSR);
            self.dec_sm.exec_instr(DECODER_PRIME_X);
        }

        self.det_sm.set_enable(true);
        self.dec_sm.set_enable(true);
    }

    /// Arm both RX state machines so the next device packet can be captured.
    ///
    /// Called from RAM-resident transaction paths before transmitting an IN token
    /// or OUT/SETUP packet whose handshake must be caught.
    #[inline(always)]
    pub(crate) fn prepare_for_receive(&mut self) {
        ram::pio_sm_disable::<PIO, 1>();
        ram::pio_sm_restart::<PIO, 1>();
        ram::pio_sm_exec_instr::<PIO, 1>(self.det_jmp_eop);
        ram::pio_sm_enable::<PIO, 1>();
        ram::pio_sm_disable::<PIO, 2>();
        ram::pio_sm_clear_fifos::<PIO, 2>();
        ram::pio_sm_restart::<PIO, 2>();
        ram::pio_sm_exec_instr::<PIO, 2>(self.dec_jmp_start);
        ram::pio_sm_exec_instr::<PIO, 2>(DECODER_PRIME_X);
        PIO::REGS
            .irq()
            .write(|w| w.set_irq((1 << IRQ_RX_START) | (1 << DECODER_TRIGGER)));
        ram::pio_sm_enable::<PIO, 2>();
    }

    /// Clear stale RX IRQ flags after the host token is on the wire.
    #[inline(always)]
    pub(crate) fn start_receive(&mut self) {
        PIO::REGS.irq().write(|w| w.set_irq(IRQ_RX_ALL));
    }

    /// Catch and classify the device's reply after [`start_receive`](Self::start_receive).
    ///
    /// Decoded bytes are copied into `out` as `[SYNC, PID, payload..., CRC16]`.
    /// The CRC16 is updated as bytes arrive; when EOP is observed, a valid
    /// DATA0/DATA1 packet can be ACKed immediately by the caller. Overflowed
    /// packets are never reported as valid, preventing ACK of truncated DATA.
    #[inline(always)]
    pub(crate) fn receive(&mut self, out: &mut [u8]) -> (usize, RxPacketStatus) {
        let mut got_start = false;
        let tw = ram::now_us();
        while ram::now_us().wrapping_sub(tw) < RX_SOP_TIMEOUT_US {
            if PIO::REGS.irq().read().irq() & (1 << IRQ_RX_START) != 0 {
                got_start = true;
                break;
            }
        }
        if !got_start {
            return (0, RxPacketStatus::NoReply);
        }

        let mut idx = 0usize;
        let mut overflowed = false;
        let mut crc: u16 = 0xffff; // running USB-CRC16 residual (no final XOR)
        let mut td = ram::now_us();
        loop {
            if let Some(w) = ram::pio_sm_try_pull_rx::<PIO, 2>() {
                let b = (w >> 24) as u8;
                if idx < out.len() {
                    out[idx] = b;
                } else {
                    overflowed = true;
                }
                if idx >= 2 {
                    // Inline table CRC update over the RAM-resident table (no flash call).
                    crc = crate::crc::crc16_update(crc, b);
                }
                idx += 1;
                td = ram::now_us();
            } else if PIO::REGS.irq().read().irq() & (1 << IRQ_RX_EOP) != 0 {
                let n = idx.min(out.len());
                if overflowed {
                    return (n, RxPacketStatus::Overflow);
                }
                // DATA0/DATA1 are the only device-emitted PIDs the host ACKs on LS/FS
                // bulk/interrupt/control-IN. Handshakes (ACK/NAK/STALL) are terminal.
                // Iso IN also uses DATA0/1 but MUST NOT be handshaken — if iso support
                // is added, use a separate no-ACK receive path.
                let valid_data = n >= 4
                    && crc == crate::crc::USB_CRC16_RESIDUE
                    && (out[1] == crate::pid::USB_PID_DATA0 || out[1] == crate::pid::USB_PID_DATA1);
                let status = if valid_data {
                    RxPacketStatus::ValidData
                } else {
                    RxPacketStatus::Invalid
                };
                return (n, status);
            } else if ram::now_us().wrapping_sub(td) > RX_DRAIN_TIMEOUT_US {
                let status = if overflowed {
                    RxPacketStatus::Overflow
                } else {
                    RxPacketStatus::Invalid
                };
                return (idx.min(out.len()), status);
            }
        }
    }
}
