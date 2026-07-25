//! RAM-safe replacements for `embassy-rp` helpers used from `.data.ram_func` paths.
//!
//! The corresponding `embassy-rp` methods are ordinary crate functions and may live in
//! flash. Keep these wrappers crate-private + `#[inline(always)]` so hot callers emit only
//! direct PAC MMIO accesses.

use crate::pio_instance::UsbPioInstance;
use rp_pac as pac;

/// RAM-safe microsecond timebase.
///
/// Reads the raw hardware timer low word directly: `TIMER` on RP2040, `TIMER0`
/// on RP235x. This provides the same 1 µs tick used by the embassy time driver
/// without calling `embassy_time::Instant::now()` from timing-critical RAM
/// functions.
#[inline(always)]
pub(crate) fn now_us() -> u32 {
    timer().timerawl().read()
}

#[cfg(feature = "rp2040")]
#[inline(always)]
/// RP2040 hardware timer used by embassy-rp's time driver.
fn timer() -> pac::timer::Timer {
    pac::TIMER
}

#[cfg(any(feature = "rp235xa", feature = "rp235xb"))]
#[inline(always)]
/// RP235x timer instance used for the microsecond timebase.
fn timer() -> pac::timer::Timer {
    pac::TIMER0
}

/// RAM-only replacement for `StateMachine::set_enable(true)`.
#[inline(always)]
pub(crate) fn pio_sm_enable<PIO: UsbPioInstance, const SM: usize>() {
    let mask = 1u8 << SM;
    PIO::REGS
        .ctrl()
        .modify(|w| w.set_sm_enable(w.sm_enable() | mask));
}

/// RAM-only replacement for `StateMachine::set_enable(false)`.
#[inline(always)]
pub(crate) fn pio_sm_disable<PIO: UsbPioInstance, const SM: usize>() {
    let mask = 1u8 << SM;
    PIO::REGS
        .ctrl()
        .modify(|w| w.set_sm_enable(w.sm_enable() & !mask));
}

/// RAM-only replacement for `StateMachine::restart`.
#[inline(always)]
pub(crate) fn pio_sm_restart<PIO: UsbPioInstance, const SM: usize>() {
    PIO::REGS.ctrl().modify(|w| w.set_sm_restart(1u8 << SM));
}

/// RAM-only replacement for `StateMachine::clear_fifos`.
#[inline(always)]
pub(crate) fn pio_sm_clear_fifos<PIO: UsbPioInstance, const SM: usize>() {
    let shiftctrl = PIO::REGS.sm(SM).shiftctrl();
    shiftctrl.modify(|w| {
        w.set_fjoin_rx(!w.fjoin_rx());
    });
    shiftctrl.modify(|w| {
        w.set_fjoin_rx(!w.fjoin_rx());
    });
}

/// RAM-only replacement for `StateMachine::exec_instr`.
#[inline(always)]
pub(crate) fn pio_sm_exec_instr<PIO: UsbPioInstance, const SM: usize>(instr: u16) {
    PIO::REGS.sm(SM).instr().write(|w| w.set_instr(instr));
}

/// RAM-only replacement for `StateMachineTx::push`.
#[inline(always)]
pub(crate) fn pio_sm_push_tx<PIO: UsbPioInstance, const SM: usize>(value: u32) {
    PIO::REGS.txf(SM).write_value(value);
}

/// Test whether a PIO state machine's TX FIFO is full.
#[inline(always)]
pub(crate) fn pio_sm_tx_full<PIO: UsbPioInstance, const SM: usize>() -> bool {
    PIO::REGS.fstat().read().txfull() & (1u8 << SM) != 0
}

/// RAM-only replacement for `StateMachineRx::try_pull`.
#[inline(always)]
pub(crate) fn pio_sm_try_pull_rx<PIO: UsbPioInstance, const SM: usize>() -> Option<u32> {
    if PIO::REGS.fstat().read().rxempty() & (1u8 << SM) != 0 {
        None
    } else {
        Some(PIO::REGS.rxf(SM).read())
    }
}
