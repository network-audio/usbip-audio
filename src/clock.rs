use std::time::Instant;

/// The clock isochronous transfers complete by, in nanoseconds since an arbitrary epoch.
///
/// It runs at about the rate of real time (the server sleeps for the difference), but needn't be exactly: the
/// audio streams follow it, not the system clock.
pub trait Clock: Send + Sync {
    fn now_ns(&self) -> u64;
}

/// The system's monotonic clock.
pub struct MonotonicClock {
    epoch: Instant,
}

impl MonotonicClock {
    pub fn new() -> Self {
        Self { epoch: Instant::now() }
    }
}

impl Default for MonotonicClock {
    fn default() -> Self {
        Self::new()
    }
}

impl Clock for MonotonicClock {
    fn now_ns(&self) -> u64 {
        self.epoch.elapsed().as_nanos() as u64
    }
}
