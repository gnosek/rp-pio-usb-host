/// USB CRC5 lookup table for token and SOF fields.
///
/// USB 2.0 §8.3.5 defines the CRC fields; token and SOF packets use CRC5 over
/// the 11-bit token payload.
pub(crate) const CRC5_TBL: [u8; 32] = [
    0x00, 0x0b, 0x16, 0x1d, 0x05, 0x0e, 0x13, 0x18, 0x0a, 0x01, 0x1c, 0x17, 0x0f, 0x04, 0x19, 0x12,
    0x14, 0x1f, 0x02, 0x09, 0x11, 0x1a, 0x07, 0x0c, 0x1e, 0x15, 0x08, 0x03, 0x1b, 0x10, 0x0d, 0x06,
];

/// USB CRC16 residue after processing a valid `payload + appended CRC16`.
///
/// The USB CRC16 is the reflected form of the polynomial in USB 2.0 §8.3.5,
/// seeded with `0xffff`. The RX path updates the running CRC as bytes arrive and
/// compares against this residue at EOP so no post-packet CRC pass is needed.
pub(crate) const USB_CRC16_RESIDUE: u16 = 0xB001;

/// USB CRC16 lookup table, generated at compile time.
///
/// Kept in RAM because the receive loop indexes it from RAM-resident timing-
/// critical code while deciding whether to ACK a DATA packet. A table update is
/// fast enough to run per received byte; a bitwise CRC pass at EOP would delay
/// the host ACK turnaround.
#[unsafe(link_section = ".data.ram_func")]
pub(crate) static CRC16_TBL: [u16; 256] = {
    let mut tbl = [0u16; 256];
    let mut i = 0usize;
    while i < 256 {
        let mut crc = i as u16;
        let mut j = 0;
        while j < 8 {
            if crc & 1 != 0 {
                crc = (crc >> 1) ^ 0xA001;
            } else {
                crc >>= 1;
            }
            j += 1;
        }
        tbl[i] = crc;
        i += 1;
    }
    tbl
};

/// Calculate USB CRC5 over an 11-bit token/SOF value.
pub(crate) const fn calc_crc5(data: u16) -> u8 {
    let data = data ^ 0x1f;
    let lsb = ((data >> 1) & 0x1f) as u8;
    let msb = ((data >> 6) & 0x1f) as u8;
    let mut crc = if data & 1 != 0 { 0x14 } else { 0x00 };
    crc = CRC5_TBL[(lsb ^ crc) as usize];
    crc = CRC5_TBL[(msb ^ crc) as usize];
    crc ^ 0x1f
}

/// Calculate the USB CRC16 appended to a DATA payload.
pub(crate) const fn calc_crc16(data: &[u8]) -> u16 {
    let mut crc: u16 = 0xffff;
    let mut i = 0;
    while i < data.len() {
        let b = data[i];
        crc = (crc >> 8) ^ CRC16_TBL[((crc ^ b as u16) & 0xff) as usize];
        i += 1;
    }
    crc ^ 0xffff
}

/// Update a running USB CRC16 remainder with one byte.
///
/// Seed with `0xffff` and do not apply the final XOR. This form is used while
/// receiving a packet so validity is known immediately at EOP.
#[inline(always)]
pub(crate) const fn crc16_update(crc: u16, b: u8) -> u16 {
    (crc >> 8) ^ CRC16_TBL[((crc ^ b as u16) & 0xff) as usize]
}
