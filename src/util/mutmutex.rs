use bare_metal::{CriticalSection, Mutex};
use core::cell::UnsafeCell;

/// A wrapper around the `cortex_m::Mutex` that allows for mutable reference to be retrieved.
///
/// **Warning**: take care to make sure only a single mutable reference exists at the same time.
/// The CriticalSection ensures this for single core processors,
/// as long as they are not nested and no multiple borrows are fetched within the same section.
pub struct MutMutex<T>(Mutex<UnsafeCell<T>>);

impl<T> MutMutex<T> {
    pub fn new(t: T) -> Self {
        Self(Mutex::new(UnsafeCell::new(t)))
    }

    /// Take care to not call this in a nested critical section.
    pub unsafe fn borrow_mut(&self, cs: &CriticalSection) -> &mut T {
        &mut *self.0.borrow(cs).get()
    }
}
