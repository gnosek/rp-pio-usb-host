use core::sync::atomic::AtomicU16;

/// SOF frame counter (11 bit, wrapping)
///
/// Incremented every frame (1ms), even if we don't actually send a SOF because
/// the bus is busy at the time.
pub struct FrameCounter {
    counter: AtomicU16,
}

impl FrameCounter {
    /// Create a new frame counter
    pub fn new() -> Self {
        Self {
            counter: AtomicU16::new(0),
        }
    }

    /// Get the next frame number
    pub fn next(&self) -> u16 {
        // we should never get into situations where we actually race on frame counters
        // and even if we do, a lost update is not the end of the world
        // (not worth taking a mutex for)
        let current = self.counter.load(core::sync::atomic::Ordering::Relaxed);
        let next = (current + 1) & 0x7ff;
        self.counter.store(next, core::sync::atomic::Ordering::Relaxed);
        next
    }
}
