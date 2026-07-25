//! USB packet identifier constants.
//!
//! Values are the on-the-wire PID bytes (low PID nibble plus one's-complement
//! high nibble) defined by USB 2.0 §8.3.1.

/// SYNC byte used at the start of low-/full-speed packets.
pub(crate) const USB_SYNC: u8 = 0x80;
/// Start-of-frame token PID.
pub(crate) const USB_PID_SOF: u8 = 0xA5;
/// SETUP token PID.
pub(crate) const USB_PID_SETUP: u8 = 0x2D;
/// IN token PID.
pub(crate) const USB_PID_IN: u8 = 0x69;
/// OUT token PID.
pub(crate) const USB_PID_OUT: u8 = 0xE1;
/// DATA0 packet PID.
pub(crate) const USB_PID_DATA0: u8 = 0xC3;
/// DATA1 packet PID.
pub(crate) const USB_PID_DATA1: u8 = 0x4B;
/// ACK handshake PID.
pub(crate) const USB_PID_ACK: u8 = 0xD2;
/// NAK handshake PID.
pub(crate) const USB_PID_NAK: u8 = 0x5A;
/// STALL handshake PID.
pub(crate) const USB_PID_STALL: u8 = 0x1E;
