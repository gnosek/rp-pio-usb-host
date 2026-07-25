/// USB CRC5 lookup table for token and SOF fields.
///
/// USB 2.0 §8.3.5 defines the CRC fields; token and SOF packets use CRC5 over
/// the 11-bit token payload.
pub(crate) const CRC5_TBL: [u8; 32] = [
    0x00, 0x0b, 0x16, 0x1d, 0x05, 0x0e, 0x13, 0x18, 0x0a, 0x01, 0x1c, 0x17, 0x0f, 0x04, 0x19, 0x12,
    0x14, 0x1f, 0x02, 0x09, 0x11, 0x1a, 0x07, 0x0c, 0x1e, 0x15, 0x08, 0x03, 0x1b, 0x10, 0x0d, 0x06,
];

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
