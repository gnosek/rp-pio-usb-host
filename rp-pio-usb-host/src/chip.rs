//! Chip-family compatibility helpers for the direct PAC accesses that remain.
//!
//! Non-timing-critical pin configuration uses embassy-rp APIs where they exist.
//! These helpers cover the remaining low-level cases: GPIO input inversion, SIO
//! input reads, and RP235x PIO `GPIOBASE` handling for GPIO numbers above the
//! original RP2040 0..31 window.

use crate::pio_instance::UsbPioInstance;
use rp_pac as pac;

#[cfg(any(
    all(feature = "rp2040", feature = "rp235xa"),
    all(feature = "rp2040", feature = "rp235xb"),
    all(feature = "rp235xa", feature = "rp235xb"),
))]
compile_error!("Select exactly one RP chip feature: rp2040, rp235xa, or rp235xb");

#[cfg(not(any(feature = "rp2040", feature = "rp235xa", feature = "rp235xb")))]
compile_error!("Select one RP chip feature: rp2040, rp235xa, or rp235xb");

/// Configure the GPIO input override used by the RX PIO programs.
///
/// The RX edge detector and decoder operate on inverted line sense so the PIO
/// assembly can treat USB SE0 and idle transitions with short branches. embassy-rp
/// does not currently expose this override on PIO pins, so this is a direct
/// `IO_BANK0` write outside the timing-critical path.
pub(crate) fn set_gpio_input_inversion(pin: u8, invert: bool) {
    let inover = if invert {
        pac::io::vals::Inover::INVERT
    } else {
        pac::io::vals::Inover::NORMAL
    };
    pac::IO_BANK0
        .gpio(pin as usize)
        .ctrl()
        .modify(|w| w.set_inover(inover));
}

/// Read a BANK0 GPIO input level through SIO.
///
/// RP2040 exposes one input register for BANK0; RP235x splits BANK0 across
/// 32-bit input registers. The returned level includes any configured GPIO input
/// override, which is what speed detection wants.
pub(crate) fn gpio_input_level(pin: u8) -> bool {
    let bank = sio_gpio_bank(pin);
    let bit = sio_gpio_bit(pin);
    pac::SIO.gpio_in(bank).read() & (1 << bit) != 0
}

#[cfg(feature = "rp2040")]
/// RP2040 has one SIO input register for all exposed BANK0 pins.
const fn sio_gpio_bank(_pin: u8) -> usize {
    0
}

#[cfg(feature = "rp2040")]
/// Bit index within the RP2040 SIO input register.
const fn sio_gpio_bit(pin: u8) -> u8 {
    pin
}

#[cfg(any(feature = "rp235xa", feature = "rp235xb"))]
/// RP235x selects the SIO input register by GPIO number / 32.
const fn sio_gpio_bank(pin: u8) -> usize {
    (pin / 32) as usize
}

#[cfg(any(feature = "rp235xa", feature = "rp235xb"))]
/// Bit index within an RP235x SIO input register.
const fn sio_gpio_bit(pin: u8) -> u8 {
    pin % 32
}

#[cfg(feature = "rp2040")]
/// RP2040 PIO pin selectors are absolute GPIO numbers; no `GPIOBASE` exists.
#[allow(clippy::extra_unused_type_parameters)]
pub(crate) fn configure_pio_gpio_base<PIO: UsbPioInstance>(_dp_pin: u8, _dm_pin: u8) -> bool {
    false
}

#[cfg(any(feature = "rp235xa", feature = "rp235xb"))]
/// Select the RP235x PIO GPIO window that contains the USB D+/D- pair.
///
/// RP235x PIO state machines still use 5-bit pin selectors, but `GPIOBASE`
/// can relocate PIO-local GPIO 0 to system GPIO 16. The pair must therefore fit
/// entirely in either the low window (`<32`) or the high window (`>=16`), matching
/// embassy-rp's `StateMachine::set_config` rule. Returns `true` when the high
/// window is selected.
pub(crate) fn configure_pio_gpio_base<PIO: UsbPioInstance>(dp_pin: u8, dm_pin: u8) -> bool {
    // GPIOBASE is RP235x PIO hardware; only the B package exposes BANK0 GPIOs above 31.
    // On rp235xa, embassy-rp only exposes the lower pin peripherals, so this normally
    // stays in the low window.
    let low_window = dp_pin < 32 && dm_pin < 32;
    let high_window = dp_pin >= 16 && dm_pin >= 16;
    assert!(
        low_window || high_window,
        "PIO USB bus pins must fit one RP235x PIO GPIOBASE window"
    );

    let use_high_window = !low_window;
    PIO::REGS
        .gpiobase()
        .write(|w| w.set_gpiobase(use_high_window));
    use_high_window
}
