use crate::pio_instance::UsbPioInstance;
use crate::tx_pio::{TX_START_INSTR_J_HIGH_ON_HIGHER, TX_START_INSTR_J_HIGH_ON_LOWER};
use crate::tx_pio::{usb_tx_j_high_on_higher_program, usb_tx_j_high_on_lower_program};
use crate::{chip, ram};
use embassy_rp::pio::{
    Common, Config, FifoJoin, LoadedProgram, Pin, ShiftConfig, ShiftDirection, StateMachine,
};
use embassy_usb_driver::Speed;

/// TX state-machine clock, full speed. The player spends 4 SM cycles per NRZI symbol:
/// 48 MHz / 4 = 12 MHz symbol rate = 12 Mb/s.
const TX_CLOCK_HZ_FS: u32 = 48_000_000;
/// TX state-machine clock, low speed. FS / 8 ⇒ 1.5 Mb/s.
const TX_CLOCK_HZ_LS: u32 = 6_000_000;

/// TX state-machine IRQ bit raised by the EOP instruction.
const IRQ_TX_EOP_BIT: u8 = 1 << 0;

/// Busy-wait timeout for `wait_tx_complete`. Realistically, we should expect about 25µs max
/// for a full-size LS packet.
const TX_WAIT_TIMEOUT_US: u32 = 50;

/// TX FIFO capacity when the state machine uses [`FifoJoin::TxOnly`].
const TX_FIFO_DEPTH: usize = 8;

/// PIO `exec` instruction: drive D+/D- as outputs.
const SET_PINDIRS_OUT: u16 = 0xE083; // set pindirs, 0b11 (both outputs)
/// PIO `exec` instruction: drive SE0 (`D+ = D- = 0`).
const SET_PINS_SE0: u16 = 0xE000; // set pins, 0b00     (both low = SE0)
/// PIO `exec` instruction: release D+/D- to inputs.
const SET_PINDIRS_IN: u16 = 0xE080; // set pindirs, 0b00  (both inputs = release bus)

/// Transmit-side PIO state machine and speed-dependent configuration.
///
/// SM0 is a symbol player: software supplies pre-encoded 2-bit J/K/SE0/COMP
/// symbols, and the PIO program turns them into D+/D- side-set values. Packet
/// encoding, CRC generation, NRZI, and bit-stuffing are handled by
/// [`crate::encoding`].
pub struct TxDriver<'a, PIO: UsbPioInstance> {
    /// PIO state machine 0, used exclusively for transmit.
    tx_sm: StateMachine<'a, PIO, 0>,
    /// Loaded TX program kept alive for embassy-rp's PIO ownership model.
    _tx_loaded: LoadedProgram<'a, PIO>,
    /// Reusable state-machine configuration; updated for the current speed.
    txcfg: Config<'a, PIO>,
    /// Speed-specific `exec` instruction used to jump to the player's start slot.
    tx_start_instr: u16,
    /// Whether logical D+ is the lower GPIO in the adjacent physical pair.
    dp_is_lower: bool,
    /// The pin mask to wait for after transmitting
    ///
    /// This is used to check if the pins are released after transmitting a frame.
    /// The mask is 1 << local_dp_pin | 1 << local_dm_pin
    /// where the pin numbers are based on the GPIO base (pin - 16 for the high bank
    /// on RP235xB).
    tx_pin_mask: u32,
}

impl<'a, PIO: UsbPioInstance> TxDriver<'a, PIO> {
    /// Load the TX program, configure SM0, and leave the bus released.
    pub fn init(
        pio_common: &mut Common<'a, PIO>,
        tx_sm: StateMachine<'a, PIO, 0>,
        dp: &Pin<'a, PIO>,
        dm: &Pin<'a, PIO>,
        gpio_high_window: bool,
    ) -> Self {
        let dp_is_lower = dp.pin() < dm.pin();
        let (lower, higher) = if dp_is_lower { (dp, dm) } else { (dm, dp) };
        let tx_pin_mask = 1 << chip::pio_local_pin(lower.pin(), gpio_high_window)
            | 1 << chip::pio_local_pin(higher.pin(), gpio_high_window);

        let tx_loaded = pio_common.load_program(&usb_tx_j_high_on_lower_program());

        let mut txcfg = Config::default();
        txcfg.use_program(&tx_loaded, &[lower, higher]);
        txcfg.set_set_pins(&[lower, higher]);
        txcfg.set_out_pins(&[lower, higher]);
        txcfg.clock_divider = crate::clock::pio_clkdiv_for(TX_CLOCK_HZ_FS);
        txcfg.fifo_join = FifoJoin::TxOnly;
        txcfg.shift_out = ShiftConfig {
            threshold: 32,
            direction: ShiftDirection::Left,
            auto_fill: true,
        };

        let mut this = Self {
            tx_sm,
            _tx_loaded: tx_loaded,
            txcfg,
            tx_start_instr: TX_START_INSTR_J_HIGH_ON_LOWER,
            dp_is_lower,
            tx_pin_mask,
        };
        this.set_speed(Speed::Full);
        this
    }

    /// Switch the TX player between full-speed and direct-root low-speed signalling.
    ///
    /// Low-speed uses the same physical D+/D- pin order but a different PIO program
    /// whose side-set constants encode low-speed J/K polarity. The state machine is
    /// left enabled and parked with D+/D- released as inputs.
    pub fn set_speed(&mut self, speed: Speed) {
        self.tx_sm.set_enable(false);

        let j_high_on_lower = match speed {
            Speed::Low => !self.dp_is_lower,
            _ => self.dp_is_lower,
        };
        let prog = if j_high_on_lower {
            usb_tx_j_high_on_lower_program()
        } else {
            usb_tx_j_high_on_higher_program()
        };
        for (addr, instr) in prog.code.iter().enumerate() {
            PIO::REGS.instr_mem(addr).write(|w| w.set_instr_mem(*instr));
        }
        self.tx_start_instr = if j_high_on_lower {
            TX_START_INSTR_J_HIGH_ON_LOWER
        } else {
            TX_START_INSTR_J_HIGH_ON_HIGHER
        };

        self.txcfg.clock_divider = crate::clock::pio_clkdiv_for(if speed == Speed::Low {
            TX_CLOCK_HZ_LS
        } else {
            TX_CLOCK_HZ_FS
        });
        self.tx_sm.set_config(&self.txcfg);
        self.tx_sm.set_enable(true);
        unsafe {
            self.tx_sm.exec_instr(SET_PINDIRS_IN); // release the bus to inputs while idle
        }
    }

    /// Start driving root-port reset SE0.
    ///
    /// The caller owns the reset duration. USB 2.0 §7.1.7.5 requires at least
    /// 10 ms of reset SE0 before release.
    pub fn drive_reset_se0(&mut self) {
        self.tx_sm.set_enable(false);
        unsafe {
            self.tx_sm.exec_instr(SET_PINDIRS_OUT);
            self.tx_sm.exec_instr(SET_PINS_SE0);
        }
    }

    /// Release the bus after reset by driving idle J and re-enabling the TX player.
    pub fn release_reset(&mut self) {
        unsafe {
            self.tx_sm.exec_instr(self.tx_start_instr);
        }
        self.tx_sm.set_enable(true);
    }

    /// Release D+/D- to inputs so the device or pull-up can drive the bus.
    #[inline(always)]
    pub fn release_bus(&mut self) {
        ram::pio_sm_exec_instr::<PIO, 0>(SET_PINDIRS_IN);
    }

    /// Load the initial words of a pre-encoded packet without starting SM0.
    ///
    /// `words` must fit in the joined eight-word TX FIFO. This function is called
    /// from RAM-resident paths and therefore only uses crate-local
    /// `#[inline(always)]` PAC helpers.
    #[inline(always)]
    fn prepare_tx_packet(&mut self, words: &[u32]) {
        debug_assert!(words.len() <= TX_FIFO_DEPTH);
        let start_instr = self.tx_start_instr;
        ram::pio_sm_disable::<PIO, 0>();
        ram::pio_sm_clear_fifos::<PIO, 0>();
        ram::pio_sm_restart::<PIO, 0>();
        ram::pio_sm_exec_instr::<PIO, 0>(start_instr);
        for w in words {
            ram::pio_sm_push_tx::<PIO, 0>(*w);
        }
        PIO::REGS.irq().write(|w| w.set_irq(IRQ_TX_EOP_BIT));
    }

    /// Preload an ACK handshake and release the bus for a device DATA packet.
    ///
    /// Used by the IN receive path so the host ACK can be sent immediately at EOP.
    #[inline(always)]
    pub(crate) fn prepare_ack_and_release_bus(&mut self) {
        self.prepare_tx_packet(&crate::encoding::ACK_PACKET);
        self.release_bus();
    }

    /// Enable SM0 to start a packet that was already loaded into the FIFO.
    #[inline(always)]
    pub fn start_tx(&mut self) {
        ram::pio_sm_enable::<PIO, 0>();
    }

    /// Wait until the PIO player has emitted EOP and reached its released-bus slot.
    #[inline(always)]
    pub fn wait(&mut self) {
        let start = ram::now_us();
        while PIO::REGS.irq().read().irq() & IRQ_TX_EOP_BIT == 0 {
            if ram::now_us().wrapping_sub(start) > TX_WAIT_TIMEOUT_US {
                break;
            }
        }
        let start = ram::now_us();
        while PIO::REGS.dbg_padoe().read() & self.tx_pin_mask != 0 {
            if ram::now_us().wrapping_sub(start) > TX_WAIT_TIMEOUT_US {
                break;
            }
        }
    }

    /// Transmit a pre-encoded packet from RAM-resident code.
    #[unsafe(link_section = ".data.ram_func")]
    #[inline(never)]
    pub fn transmit(&mut self, words: &[u32]) {
        let (initial, remaining) = words.split_at(words.len().min(TX_FIFO_DEPTH));
        self.prepare_tx_packet(initial);
        if remaining.is_empty() {
            self.start_tx();
        } else {
            // Once SM0 starts, prevent a long interrupt from draining the FIFO
            // before software has queued the rest of the packet.
            critical_section::with(|_| {
                self.start_tx();
                for word in remaining {
                    while ram::pio_sm_tx_full::<PIO, 0>() {}
                    ram::pio_sm_push_tx::<PIO, 0>(*word);
                }
            });
        }
        self.wait();
    }
}
