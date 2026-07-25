//! USB packet identifier constants.
//!
//! Values are the on-the-wire PID bytes (low PID nibble plus one's-complement
//! high nibble) defined by USB 2.0 §8.3.1.

/// SYNC byte used at the start of low-/full-speed packets.
pub(crate) const USB_SYNC: u8 = 0x80;
/// Start-of-frame token PID.
pub(crate) const USB_PID_SOF: u8 = 0xA5;
