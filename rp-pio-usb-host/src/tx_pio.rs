use pio::Program;

/// Pre-encoded start instruction for the TX program whose J state has the lower GPIO high.
///
/// The CPU executes this before loading/enabling each packet: `JMP 4` to the
/// `start:` slot (`set pindirs, 0b11`) with side-set `0b01`, which drives the
/// lower of the two GPIOs high. This reasserts D+/D- as outputs because the
/// packet stream's `COMP` symbol releases the bus after EOP.
pub(crate) const TX_START_INSTR_J_HIGH_ON_LOWER: u16 = 0x0804;

/// TX PIO player where logical J is `lower high, higher low`.
///
/// This program is intentionally fixed at origin 0: `out pc, 2` loads absolute
/// symbol addresses 0..3, so it is not relocatable. Address 0 emits the EOP SE0
/// slot; packets therefore start by executing [`TX_START_INSTR_J_HIGH_ON_LOWER`]
/// rather than by jumping to origin 0. USB low-/full-speed EOP is specified in
/// USB 2.0 §7.1.7.2.
pub(crate) fn usb_tx_j_high_on_lower_program() -> Program<32> {
    pio::pio_asm!(
        ".origin 0",
        ".side_set 2",
        // addr 0: EOP — raise IRQ, drive SE0 (both low), then fall through.
        "    irq 0           side 0b00 [7]",
        ".wrap_target",
        // addr 1: fetch next symbol, side 0b01 (J: lower high, higher low).
        "    out pc, 2       side 0b01 [3]",
        // addr 2: COMP — release the bus (pindirs := inputs), side 0b01.
        "    set pindirs, 0  side 0b01 [3]",
        // addr 3: fetch next symbol, side 0b10 (K: lower low, higher high).
        "    out pc, 2       side 0b10 [3]",
        // addr 4: start: drive the bus (pindirs := outputs), J state.
        "    set pindirs, 0b11 side 0b01",
        ".wrap",
    )
    .program
}

/// Pre-encoded start instruction for the TX program whose J state has the higher GPIO high.
///
/// This is the same jump as [`TX_START_INSTR_J_HIGH_ON_LOWER`] but with side-set
/// `0b10`, which drives the higher of the two GPIOs high.
pub(crate) const TX_START_INSTR_J_HIGH_ON_HIGHER: u16 = 0x1004;

/// TX PIO player where logical J is `lower low, higher high`.
///
/// This is the mirror image of [`usb_tx_j_high_on_lower_program`]: target 1 is
/// still logical idle J, but the high line is the higher GPIO. This supports both
/// direct low-speed signalling (where J is D-) and physically reversed D-/D+ wiring.
/// SE0 (`0b00`) is unchanged — both lines low is "single-ended zero" at either
/// speed, so the EOP/COMP framing is untouched.
///
/// Why a whole second program (rather than swapping the D+/D- pin order in the SM
/// config): embassy-rp drives a *consecutive, ascending* pin range (`base..base+n`)
/// and the side-set→line mapping is fixed by the side **values** baked here; and an
/// output-invert override can't be used because it would corrupt SE0 into both-high.
/// Loaded at **origin 0**; start each packet with [`TX_START_INSTR_J_HIGH_ON_HIGHER`].
pub(crate) fn usb_tx_j_high_on_higher_program() -> Program<32> {
    pio::pio_asm!(
        ".origin 0",
        ".side_set 2",
        // addr 0: EOP — raise IRQ, drive SE0 (both low), then fall through.
        "    irq 0           side 0b00 [7]",
        ".wrap_target",
        // addr 1: fetch next symbol, side 0b10 (J: lower low, higher high).
        "    out pc, 2       side 0b10 [3]",
        // addr 2: COMP — release the bus (pindirs := inputs), side 0b10.
        "    set pindirs, 0  side 0b10 [3]",
        // addr 3: fetch next symbol, side 0b01 (K: lower high, higher low).
        "    out pc, 2       side 0b01 [3]",
        // addr 4: start: drive the bus (pindirs := outputs), J state.
        "    set pindirs, 0b11 side 0b10",
        ".wrap",
    )
    .program
}
