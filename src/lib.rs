//! Async timers for embedded devices in Rust.
//!
//! This crate provides an interface and a generic implemention of a timer that can handle multiple
//! concurrent deadlines using the Future architecture provided by Rust. This crate also provides
//! two reference implementations for this interface for the STM32F103 and STM32L476.
//!
//! Using implementations for the `Timer` trait, you can instance an `AsyncTimer` that provides methods
//! to concurrently wait on a specific deadline or duration with a `Future`-compatible interface.
//! Using this crate you can implement device drivers that require delays. Using HAL crates that are
//! compatible with `embedded-async-timer` you can develop firmware that directly require delays or use
//! these aforementioned drivers.
//! For HAL RTC implementations with alarms it should be trivial to adapt them to implement the `Timer` trait.
//! View the STM32F103 example in the `impls` module for inspiration.
//!
//! **Note:** until `const_generics` lands the capacity for AsyncTimer is hard-coded.
//!
//! **Note:** the current design assumes a `bare_metal`-like single core architecture that supports
//! `interrupt::free`-like blocks. In the future I would like to rewrite everything to no longer require this.
//!
//! **Note:** the priority queue used in this implementation is very likely to not be the most efficient choice of data structure.
//! I intend to experiment with various setups to make an informed performance consideration based on real
//! measurements given a small amount of concurrent timers.
//!
//! **State:** this crate has been tested with trivial examples on two embedded devices, but has not yet
//! been applied in production. It is beyond a simple proof of concept though. Our intention to provide a
//! basis for an async ecosystem. The quality might also be insufficient to run on production.
//! Also the current state of async-await might be such that it is practically unusable for memory constrained embedded devices.
//! We encourage developing async HAL drivers using this crate though.
//!
//! # Example
//! ```rust
//! async {
//!     let timer = AsyncTimer::new(rtc);
//!     timer.wait(Duration::from_millis(500).into()).await;
//!
//!     let mut interval = Interval::new(Duration::from_secs(1).into(), &timer);
//!
//!     loop {
//!         interval.wait(&timer).await;
//!         println!("ping");
//!     }
//! }
//! ```

#![no_std]

pub mod impls;
mod util;

use core::sync::atomic::{AtomicBool, Ordering};
use core::{
    future::Future,
    pin::Pin,
    sync::atomic,
    task::{Context, Poll, Waker},
};

use bare_metal::CriticalSection;

use crate::util::mutmutex::MutMutex;
use crate::util::priorityqueue::{Index, PriorityQueue};

/// The amount of deadlines that can be scheduled for any given Timer.
///
/// Can be parameterized once the unstable `const_generics` features is stabilized.
pub const CAPACITY: usize = 32;

/// Too many timers were scheduled before finishing them.
#[derive(Debug)]
pub struct CapacityError;

struct Handle<TIME: Ord> {
    deadline: TIME,
    awoken: AtomicBool,
    waker: Option<Waker>,
}

impl<TIME: Ord> Handle<TIME> {
    /// Awaken the handle, regardless of time.
    ///
    /// Will return whether it was awoken for the first time.
    fn awaken(&self) -> bool {
        let result = self.awoken.compare_and_swap(false, true, Ordering::SeqCst);

        // Notify the waker, every time this function is called.
        // Notifying the waker extra times cannot hurt.
        if let Some(waker) = &self.waker {
            waker.wake_by_ref();
        }

        result
    }
}

impl<TIME: Ord> PartialEq for Handle<TIME> {
    fn eq(&self, other: &Self) -> bool {
        self.deadline.eq(&other.deadline)
    }
}
impl<TIME: Ord> PartialOrd for Handle<TIME> {
    fn partial_cmp(&self, other: &Self) -> Option<core::cmp::Ordering> {
        self.deadline.partial_cmp(&other.deadline)
    }
}

/// A clock device that supports throwing alarm interrupts at given instant.
///
/// This trait assumes that the clock is not (re)set after initial configuration.
pub trait Timer {
    /// A length of time.
    ///
    /// You will probably want to make this type as close to your Instant type as possible,
    /// with a focus on making addition and storage efficient.
    type Duration: Clone;
    /// A moment in time.
    ///
    /// You will probably make this type as close to the internal representation of an instant
    /// as used by your RTC peripheral.
    type Instant: Clone + Ord + core::ops::AddAssign<Self::Duration>;

    /// A minimal time increment, i.e. the time it takes to execute a handful of instructions.
    ///
    /// Used when scheduling the next deadline, but in the meantime the deadline has passed.
    const DELTA: Self::Duration;

    /// Initialize the clock and start counting.
    ///
    /// You will need to run this sometime after initializing the static memory in which the timer lives,
    /// such that the interrupt handler can access the timer.
    fn reset(&mut self);
    /// Execute the function in an interrupt free critical section.
    ///
    /// Probably you want to directly feed through `cortex_m::interrupt::free` or similar.
    fn interrupt_free<F: FnOnce(&CriticalSection) -> R, R>(f: F) -> R;
    /// Yield the current time.
    fn now(&self) -> Self::Instant;
    /// Disarm the set alarm.
    fn disarm(&mut self);
    /// Set the alarm for a given time.
    fn arm(&mut self, deadline: &Self::Instant);
}

/// Convenience function to easily express a time in the future.
#[inline(always)]
fn future<D, I: Clone + core::ops::AddAssign<D>>(mut i: I, d: D) -> I {
    i += d;
    i
}

/// A Future waiting for a deadline.
///
/// **Note** This future is instantiated by Timer.
pub struct Delay<'a, T: Timer> {
    index: Index,
    timer: &'a AsyncTimer<T>,
}

impl<'a, T: Timer> Future for Delay<'a, T> {
    type Output = ();

    fn poll(self: Pin<&mut Self>, cx: &mut Context<'_>) -> Poll<()> {
        T::interrupt_free(|cs| {
            let inner = unsafe { self.timer.0.borrow_mut(cs) };
            let handle = inner.handles.get_mut(self.index).unwrap(); // Unwrap: only we get to clean up the handle. Logic error otherwise.
            let waker = &mut handle.waker;

            if handle.awoken.load(Ordering::SeqCst) {
                if waker.is_none() {
                    // Uninstall the waker.
                    drop(waker.take());
                }

                // The handle will be removed at Drop.

                Poll::Ready(())
            } else {
                // Always replace the waker, regardless whether it was set or not.
                waker.replace(cx.waker().clone());

                // Drop the mutex.
                drop(inner);

                // Ensure that the waker and dropping was actually done.
                atomic::compiler_fence(Ordering::Release);

                // Arm alarm with earliest (possibly newer) deadline.
                self.timer.wake_and_arm(cs);

                Poll::Pending
            }
        })
    }
}

impl<'a, T: Timer> Drop for Delay<'a, T> {
    fn drop(&mut self) {
        T::interrupt_free(|cs| {
            let inner = unsafe { self.timer.0.borrow_mut(cs) };
            inner.handles.remove(self.index).unwrap(); // Unwrap: should not yet be cleaned up. Logic error otherwise.
        });
    }
}

/// Inner of Timer that might not be Sync.
struct AsyncTimerInner<T: Timer> {
    handles: PriorityQueue<Handle<T::Instant>>,
    timer: T,
}

/// A wrapped timer that can asynchronously wake up for concurrent deadlines.
///
/// The capacity of this timer is determined by the internal PriorityQueue.
pub struct AsyncTimer<T: Timer>(MutMutex<AsyncTimerInner<T>>);

impl<T: Timer> AsyncTimer<T> {
    pub fn new(timer: T) -> Self {
        Self(MutMutex::new(AsyncTimerInner {
            handles: PriorityQueue::new(),
            timer,
        }))
    }

    pub fn reset(&self) {
        T::interrupt_free(|cs| {
            let inner = unsafe { self.0.borrow_mut(cs) };
            inner.timer.reset()
        })
    }

    pub fn now(&self) -> T::Instant {
        T::interrupt_free(|cs| {
            let inner = unsafe { self.0.borrow_mut(cs) };
            inner.timer.now()
        })
    }

    pub unsafe fn get_inner<U>(&self, f: impl FnOnce(&mut T) -> U) -> U {
        T::interrupt_free(|cs| f(&mut self.0.borrow_mut(cs).timer))
    }

    /// Wake up any tasks that can be awoken, and arm the inner RTC with the earliest deadline.
    fn wake_and_arm(&self, cs: &CriticalSection) {
        let inner = unsafe { self.0.borrow_mut(cs) };
        let time = inner.timer.now();
        let earliest = inner.handles.iter().try_fold((), |_, (_index, handle)| {
            if handle.deadline <= time {
                handle.awaken();
                Ok(())
            } else {
                Err(&handle.deadline)
            }
        });

        match earliest {
            Ok(()) => {
                inner.timer.disarm(); // All deadlines have been processed
            }
            Err(earliest) => {
                let min_next = future(inner.timer.now(), T::DELTA);
                if min_next < *earliest {
                    inner.timer.arm(earliest);
                } else {
                    // We were too slow to re-arm the RTC before the next deadline,
                    // hence schedule for the next time increment.
                    inner.timer.arm(&min_next);
                }
            }
        }
    }

    /// Awaken all timers for which the deadline has passed compared to the time.
    ///
    /// Will re-arm the inner RTC with the earliest deadline, if there is one.
    ///
    /// **Note:** call this in the appropriate interrupt handler.
    #[inline(always)]
    pub fn awaken(&self) {
        T::interrupt_free(|cs| {
            self.wake_and_arm(cs);
        });
    }

    /// Wait at least for a specific duration.
    #[inline(always)]
    pub fn wait<'a>(&'a self, dur: T::Duration) -> Result<Delay<'a, T>, CapacityError> {
        self.wait_until_always(future(self.now(), dur))
    }

    /// Wait until some time after a specific deadline.
    ///
    /// Will immediately return (and never yield) if the deadline has already passed.
    #[inline(always)]
    pub async fn wait_until<'a>(&'a self, deadline: T::Instant) -> Result<(), CapacityError> {
        if deadline <= self.now() {
            return Ok(());
        }

        Ok(self.wait_until_always(deadline)?.await)
    }

    /// Wait until some time after a specific deadline.
    ///
    /// Will at least yield once, even if the deadline has already passed.
    #[inline(always)]
    pub fn wait_until_always<'a>(
        &'a self,
        deadline: T::Instant,
    ) -> Result<Delay<'a, T>, CapacityError> {
        // Reading time is a read-only operation, and guaranteed to be consistent,
        // Because the time is not reset after initialisation.
        let handle = Handle {
            deadline,
            awoken: AtomicBool::new(false),
            waker: None,
        };

        let index = T::interrupt_free(|cs| {
            let inner = unsafe { self.0.borrow_mut(cs) };
            inner.handles.insert(handle)
        })?;

        Ok(Delay { index, timer: self })
    }
}

/// A repeating delay with a fixed start and fixed segment duration.
pub struct Interval<CLOCK: Timer> {
    last: CLOCK::Instant,
    duration: CLOCK::Duration,
}

impl<CLOCK: Timer> Interval<CLOCK> {
    /// Create a new interval starting **now**.
    ///
    /// Awaiting this new Interval will yield for the first time after `now + duration`.
    pub fn new(duration: CLOCK::Duration, timer: &AsyncTimer<CLOCK>) -> Self {
        Self {
            last: timer.now(),
            duration,
        }
    }

    /// Await until some time after the end of the next interval segment.
    ///
    /// **Note**: if an interval segment has already completely passed it will return immediately without yielding.
    pub fn wait<'a>(&mut self, timer: &'a AsyncTimer<CLOCK>) -> impl Future + 'a {
        self.last += self.duration.clone();
        timer.wait_until(self.last.clone())
    }
}
