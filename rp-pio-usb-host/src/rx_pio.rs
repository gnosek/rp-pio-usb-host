//! RX PIO programs: edge detector plus NRZI decoder.
//!
//! Two state machines cooperate through PIO IRQ flags. The edge detector runs at
//! 8x the USB bit rate, identifies SOP/EOP, and raises [`DECODER_TRIGGER`] once
//! per bit. The decoder samples the inverted line on each trigger, reverses NRZI
//! encoding (USB 2.0 §7.1.8), drops stuffed bits (§7.1.9), and pushes decoded
//! bytes into its RX FIFO.
//!
//! Both D+ and D- GPIO inputs are inverted before reaching PIO. The programs use
//! relative branches and IRQs only, so unlike the TX player they can be loaded at
//! any instruction-memory offset.

use pio::Program;

/// RX end-of-packet IRQ flag raised after detecting SE0.
pub(crate) const IRQ_RX_EOP: u8 = 2;
/// RX packet-start IRQ flag raised after detecting SOP.
pub(crate) const IRQ_RX_START: u8 = 3;
/// Edge-detector → decoder per-bit trigger IRQ flag.
pub(crate) const DECODER_TRIGGER: u8 = 4;

/// Mask of all RX IRQ flags used by the two RX state machines.
pub(crate) const IRQ_RX_ALL: u8 = (1 << IRQ_RX_EOP) | (1 << IRQ_RX_START) | (1 << DECODER_TRIGGER);

/// Build the edge-detector PIO program.
///
/// At full speed it runs at 96 MHz, an 8x oversample of the 12 Mb/s bit rate.
/// `in_pin` and `jmp_pin` are configured by [`crate::rx_driver::RxDriver`] for
/// the current speed; the low-speed configuration swaps the sensed lines.
pub(crate) fn usb_edge_detector_program() -> Program<32> {
    pio::pio_asm!(
        ".define IRQ_RX_EOP 2",
        ".define IRQ_RX_START 3",
        ".define DECODER_TRIGGER 4",
        "eop:",
        "    irq wait IRQ_RX_EOP",
        "start:",
        "    jmp pin start", // wait for the falling edge (SOP)
        "    irq IRQ_RX_START [1]",
        ".wrap_target",
        "pin_still_low:",
        "    irq DECODER_TRIGGER [1]", // clock the NRZI decoder
        "pin_low:",
        "    jmp pin pin_went_high",
        "    jmp pin pin_went_high",
        "pin_went_low:",
        "    jmp pin pin_went_high",
        "    jmp pin pin_went_high",
        "    jmp pin pin_went_high",
        "    jmp pin pin_went_high",
        ".wrap",
        "pin_still_high:",
        "    mov x, isr [1]",
        "    jmp x-- eop", // both inputs high (inverted) ⇒ SE0 ⇒ EOP
        "pin_went_high:",
        "    mov isr, null [1]",
        "    irq DECODER_TRIGGER",
        "    in pins, 1", // capture the pin to check for EOP
        "    jmp pin pin_still_high",
        "    jmp pin_went_low",
    )
    .program
}

/// Build the NRZI decoder PIO program.
///
/// The decoder runs as fast as PIO allows but advances only on
/// [`DECODER_TRIGGER`]. Before enabling, prime it with [`DECODER_PRIME_OSR`] and
/// [`DECODER_PRIME_X`]; configure shift-right autopush with threshold 8.
pub(crate) fn usb_nrzi_decoder_program() -> Program<32> {
    pio::pio_asm!(
        ".define BIT_REPEAT_COUNT 6",
        ".define DECODER_TRIGGER 4",
        "start:",
        // `set x, 0` is done at init via exec, not here.
        ".wrap_target",
        "set_y:",
        "    set y, BIT_REPEAT_COUNT",
        "irq_wait:",
        "    wait 1 irq DECODER_TRIGGER", // wait for the edge detector
        "    jmp !y flip",                // drop the stuff bit (no error check)
        "    jmp pin pin_high",
        "pin_low:",
        "    jmp !x K1",
        "K2:",
        "J1:",
        "    in null, 1",
        "flip:",
        "    mov x, !x",
        ".wrap",
        "pin_high:",
        "    jmp !x J1",
        "J2:",
        "K1:",
        "    in osr, 1",
        "    jmp y-- irq_wait", // y is never 0 here
    )
    .program
}

/// Decoder prime instruction `mov osr, !null` (OSR ← all-ones) — `in osr, 1`
/// then injects a steady `1` for the NRZI "no transition" case. Run once via
/// `exec_instr` before enabling the decoder. Encoding: MOV(101) dest OSR(111)
/// op INVERT(01) src NULL(011) ⇒ `0xA0EB`.
pub(crate) const DECODER_PRIME_OSR: u16 = 0xA0EB;
/// Decoder prime instruction `set x, 0` (NRZI state ← 0). Encoding: SET(111)
/// dest X(001) data 0 ⇒ `0xE020`.
pub(crate) const DECODER_PRIME_X: u16 = 0xE020;
