use fixed::FixedU32;
use fixed::types::extra::U8;

/// Calculate a PIO state-machine clock divider for `target_hz`.
///
/// RP PIO state machines are clocked from `clk_sys` through a 16.8 fixed-point
/// divider. The value is rounded to the nearest representable divider, which is
/// precise enough for the USB rates used here when `clk_sys` comes from the
/// normal crystal/PLL clock tree. Panics if clocks are not initialized, the target
/// is above `clk_sys`, or the divider would exceed the hardware range.
pub(crate) fn pio_clkdiv_for(target_hz: u32) -> FixedU32<U8> {
    let clk_sys_hz = embassy_rp::clocks::clk_sys_freq();
    assert_ne!(
        clk_sys_hz, 0,
        "embassy_rp::init must configure clocks before Bus::new"
    );
    assert!(
        clk_sys_hz >= target_hz,
        "clk_sys must be at least the requested PIO state-machine clock"
    );

    let bits = ((clk_sys_hz as u64) << 8) + (target_hz as u64 / 2);
    let bits = bits / target_hz as u64;
    assert!(
        bits <= (65536u64 << 8),
        "PIO clock divider exceeds hardware range"
    );
    FixedU32::<U8>::from_bits(bits as u32)
}
