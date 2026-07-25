use crate::crc::calc_crc5;

/// 2-bit TX symbols = `out pc` jump targets into the `usb_tx` player
/// (`usb_tx.pio`): SE0=0, idle-J=1, COMP=2, K=3.
/// Single-ended-zero symbol; the player drives both lines low.
const SYM_SE0: u8 = 0;
/// Logical idle-J symbol for the currently selected TX PIO program.
const SYM_J: u8 = 1;
/// Completion symbol; the player releases D+/D- to inputs after EOP.
const SYM_COMP: u8 = 2;
/// Logical K symbol for the currently selected TX PIO program.
const SYM_K: u8 = 3;

/// Encode a raw USB packet into the 2-bit symbols consumed by the TX PIO player.
///
/// USB transmits each byte LSB-first, uses NRZI where zero bits cause a
/// transition (USB 2.0 §7.1.8), and inserts a stuff bit after six consecutive
/// one bits (§7.1.9). The resulting line symbols are packed four per byte,
/// most-significant symbol first, because the PIO player consumes bits from the
/// MSB end of each FIFO word.
///
/// Appends the low-/full-speed EOP sequence (§7.1.7.2) as `SE0` then `COMP`
/// and pads with idle-J symbols to a byte boundary. Returns the number of
/// encoded bytes written to `out`.
///
/// `out` must have room for two encoded bits per input bit, worst-case stuffing,
/// EOP, and up to three alignment symbols. Its initial contents are irrelevant.
fn encode_tx_data(buffer: &[u8], out: &mut [u8]) -> Option<usize> {
    let mut bit_idx: usize = 0;
    let mut state_j = true; // current NRZI line state
    let mut stuffing: i32 = 6;

    // Append a 2-bit symbol at the current bit position.
    #[inline(always)]
    fn put(out: &mut [u8], bit_idx: &mut usize, sym: u8) -> Option<()> {
        let byte_idx = *bit_idx >> 2;
        if byte_idx >= out.len() {
            return None;
        }
        out[byte_idx] = (out[byte_idx] << 2) | sym;
        *bit_idx += 1;
        Some(())
    }

    for &data_byte in buffer {
        for b in 0..8 {
            if data_byte & (1 << b) != 0 {
                // '1' → no transition; emit the symbol for the current state.
                put(out, &mut bit_idx, if state_j { SYM_J } else { SYM_K })?;
                stuffing -= 1;
            } else {
                // '0' → transition.
                if state_j {
                    put(out, &mut bit_idx, SYM_K)?;
                    state_j = false;
                } else {
                    put(out, &mut bit_idx, SYM_J)?;
                    state_j = true;
                }
                stuffing = 6;
            }

            if stuffing == 0 {
                // Insert a stuff bit (forced transition).
                if state_j {
                    put(out, &mut bit_idx, SYM_K)?;
                    state_j = false;
                } else {
                    put(out, &mut bit_idx, SYM_J)?;
                    state_j = true;
                }
                stuffing = 6;
            }
        }
    }

    // EOP marker: SE0 then COMP (the player turns COMP into the real EOP).
    put(out, &mut bit_idx, SYM_SE0)?;
    put(out, &mut bit_idx, SYM_COMP)?;
    // Terminate the buffer with idle-J until byte-aligned.
    loop {
        put(out, &mut bit_idx, SYM_J)?;
        if bit_idx & 0x03 == 0 {
            break;
        }
    }

    Some(bit_idx >> 2)
}

/// Repack the encoder's MSB-first symbol **bytes** into big-endian u32 **words**
/// for the TX FIFO. With `shift_out` = Left + `auto_fill` threshold 32, `out pc,
/// 2` consumes bits \[31:30\] first, so byte 0 (which holds the first 4 symbols in
/// its top bits) must sit in bits \[31:24\]: `word = b0<<24 | b1<<16 | b2<<8 | b3`.
/// The tail is padded with `0x55` (four idle-J symbols) — harmless trailing Js
/// after EOP, since COMP has already released the bus. Returns the word count.
fn repack_for_fifo(encoded: &[u8], words: &mut [u32]) -> Option<usize> {
    let n_words = encoded.len().div_ceil(4);
    if n_words > words.len() {
        return None;
    }
    for (w, slot) in words.iter_mut().enumerate().take(n_words) {
        let mut group = [0x55u8; 4]; // idle-J symbol padding (0b01 ×4) for the partial tail
        let start = w * 4;
        let avail = (encoded.len() - start).min(4);
        group[..avail].copy_from_slice(&encoded[start..start + avail]);
        *slot = u32::from_be_bytes(group);
    }
    Some(n_words)
}

/// Encode one USB packet into FIFO `words`, returning the word count.
fn encode<const N: usize>(pkt: &[u8], scratch: &mut [u8; N], words: &mut [u32]) -> Option<usize> {
    let n = encode_tx_data(pkt, scratch)?;
    repack_for_fifo(&scratch[..n], words)
}

/// FIFO-ready low-speed keep-alive packet.
///
/// The empty packet encodes to the EOP sequence plus idle padding; at the
/// low-speed TX clock the SE0 interval is exactly two low-speed bit times, the
/// keep-alive form required by USB 2.0 §11.8.4.1.
pub(crate) const LS_KEEPALIVE_PACKET: [u32; 1] = [0x25555555];

/// Build a FIFO-ready SOF packet for an 11-bit frame number.
///
/// SOF token format and the 11-bit frame number are defined in USB 2.0 §8.4.3.
/// SOF packets encode to 9 or 10 symbol-bytes after bit-stuffing, so they always
/// fit in exactly three FIFO words.
pub(crate) fn build_sof(frame: u16) -> [u32; 3] {
    let f = frame & 0x7ff;
    let crc5 = calc_crc5(f);
    let pkt = [
        crate::pid::USB_SYNC,
        crate::pid::USB_PID_SOF,
        (f & 0xff) as u8,
        (((f >> 8) & 0x07) as u8) | (crc5 << 3),
    ];
    let mut scratch = [0u8; 12];
    let mut words = [0x55555555u32; 3];
    let n = match encode(&pkt, &mut scratch, &mut words) {
        Some(n) => n,
        None => unreachable!("SOF packet fits fixed TX buffers"),
    };
    debug_assert_eq!(n, words.len());
    words
}
