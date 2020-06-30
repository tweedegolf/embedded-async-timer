//! A simple AsyncTimer implementation for the Rtc peripheral in the STM32F103 running at 1kHz.

use bare_metal::CriticalSection;
use core::sync::atomic::{self, Ordering};
use stm32f1xx_hal::{
    pac,
    prelude::*,
    rtc::Rtc,
    stm32::{Interrupt, NVIC},
};

use crate::{AsyncTimer, Timer};

const TICKS_PER_SECOND: u64 = 1000;

/// An instant of time, encoding the RTC instant value exactly as the RTC peripheral does.
#[derive(PartialEq, Eq, Ord, PartialOrd, Clone, Copy, Debug)]
pub struct InstantU32(u32);

/// A segment of time, encoding a duration in such a manner that it compares and adds to a RTC instant efficiently.
#[derive(Clone)]
pub struct DurationU32(u32);

impl core::convert::From<core::time::Duration> for DurationU32 {
    fn from(duration: core::time::Duration) -> Self {
        Self(
            ((duration.as_secs() as u64) * TICKS_PER_SECOND
                + (duration.subsec_nanos() as u64) * TICKS_PER_SECOND / 1_000_000_000)
                as u32,
        )
    }
}

impl core::ops::AddAssign<DurationU32> for InstantU32 {
    fn add_assign(&mut self, duration: DurationU32) {
        self.0 += duration.0;
    }
}

impl core::ops::Add<DurationU32> for InstantU32 {
    type Output = Self;

    fn add(mut self, rhs: DurationU32) -> Self::Output {
        self += rhs;
        self
    }
}

impl Timer for Rtc {
    type Instant = InstantU32;
    type Duration = DurationU32;

    const DELTA: DurationU32 = DurationU32(2);

    fn reset(&mut self) {
        unsafe { NVIC::unmask(Interrupt::RTC) }
        self.select_frequency((TICKS_PER_SECOND as u32).hz());
        self.set_time(0);
    }

    #[inline(always)]
    fn interrupt_free<F: FnOnce(&CriticalSection) -> R, R>(f: F) -> R {
        cortex_m::interrupt::free(f)
    }

    #[inline(always)]
    fn now(&self) -> Self::Instant {
        InstantU32(self.current_time())
    }

    #[inline(always)]
    fn disarm(&mut self) {
        self.unlisten_alarm();
        self.clear_alarm_flag();
    }

    #[inline(always)]
    fn arm(&mut self, deadline: &Self::Instant) {
        // Assumes the alarm was already disarmed.
        self.set_alarm(deadline.0);
        self.clear_alarm_flag();
        self.listen_alarm();
    }
}

/// Handle the RTC interrupt alarm and wake up the appropriate waker.
///
/// Add to your code:
/// ```
/// static mut TIMER: Option<AsyncTimer<Rtc>> = None;
///
/// #[interrupt]
/// #[allow(non_snake_case)]
/// #[no_mangle]
/// fn RTC() {
///     if let Some(timer) = unsafe { TIMER.as_ref() } {
///         handle_interrupt(move || timer)
///     }
/// }
/// ```
#[inline(always)]
pub fn handle_interrupt<'a>(get_timer: impl FnOnce() -> &'a AsyncTimer<Rtc>) {
    if swap_check() {
        get_timer().awaken();
    }
}

#[inline(always)]
fn swap_check() -> bool {
    stm32f1xx_hal::pac::RTC::borrow_unchecked(|rtc| {
        if rtc.crl.read().alrf().bit() {
            rtc.crl.modify(|_, w| w.alrf().clear_bit());
            true
        } else {
            false
        }
    })
}

macro_rules! borrow_unchecked {
    ($($peripheral:ident),*) => {
        $(
            unsafe impl BorrowUnchecked for pac::$peripheral {
                #[inline(always)]
                fn borrow_unchecked<T>(f: impl FnOnce(&Self) -> T) -> T {
                    let p = unsafe { core::mem::transmute(()) };
                    f(&p)
                }
            }
        )*
    }
}

/// Borrows a peripheral without checking if it has already been taken
unsafe trait BorrowUnchecked {
    fn borrow_unchecked<T>(f: impl FnOnce(&Self) -> T) -> T;
}

borrow_unchecked!(RTC);
