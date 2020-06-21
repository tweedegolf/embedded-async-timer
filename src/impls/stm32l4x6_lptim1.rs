use bare_metal::CriticalSection;
use core::sync::atomic::{AtomicU32, Ordering};
use stm32l4xx_hal::stm32::{lptim1, Interrupt, LPTIM1, NVIC, RCC};

use crate::{AsyncTimer, Timer};

type RegisterBlock = lptim1::RegisterBlock;

const ICR_MASK: u32 = 0b111_1111;

/// Set up the heartbeat timer and interrupt.
///
/// Wakes the CPU at a rate of 1khz.
/// Enables casually running sleep or stop2 whilst using async delays.
pub fn setup(tim: &LPTIM1, rcc: &RCC) {
    // Implementation choice: went for LSI in the hope that it would be more efficient to have less ticks.
    // During experiments LSI with x32 prescaler resulted in a ~30% skew.
    // Skew minimized by using a x1 prescaler and counting to 0x20 / 32.

    rcc.apb1enr1.modify(|_, w| w.lptim1en().set_bit());
    rcc.apb1rstr1.modify(|_, w| w.lptim1rst().set_bit());
    rcc.apb1rstr1.modify(|_, w| w.lptim1rst().clear_bit());
    rcc.apb1smenr1.modify(|_, w| w.lptim1smen().set_bit()); // Power LPTIM1 in LP mode

    // Set clock to LSI, which should be 32KHz
    rcc.ccipr.modify(|_, w| unsafe { w.lptim1sel().bits(0b01) });

    start_timer(tim, 0xff00);
}

/// Start the timer for an amount of 32KHz fractions.
fn start_timer(tim: &LPTIM1, fractions: u16) {
    // Pause the clock
    tim.cr.modify(|_, w| w.enable().clear_bit());

    tim.cfgr.modify(|_, w| {
        w.cksel().clear_bit(); // internal clocksource
        w.enc().clear_bit(); // no encoder mode
        w.preload().set_bit();
        w.countmode().clear_bit(); // the counter is incremented following each internal clock pulse
        w.timout().set_bit();
        unsafe {
            w.presc().bits(0b000); // x1
            w.ckpol().bits(0b00);
            w.trigen().bits(0b00)
        }
    });

    // Enable cmp events
    tim.ier.modify(|_, w| w.cmpmie().bit(true));
    tim.cr.modify(|_, w| w.enable().set_bit());

    // Reset waiting events
    tim.icr.write(|w| unsafe { w.bits(ICR_MASK) });
    unsafe { NVIC::unmask(Interrupt::LPTIM1) };

    tim.cr.modify(|_, w| w.cntstrt().set_bit());

    let fractions = fractions as u32;

    // Duty cycle 32 LSI ticks before interrupt, 1KHz
    tim.cmp.write(|w| unsafe { w.bits(fractions - 1) });
    tim.arr.write(|w| unsafe { w.bits(fractions) });
}

/// Stop the heartbeat by disabling LPTIM1.
fn teardown(tim: &RegisterBlock) {
    tim.cr.modify(|_, w| w.enable().clear_bit());
    tim.icr.write(|w| unsafe { w.bits(ICR_MASK) });
}

pub unsafe fn handle_interrupt(tim: &LPTIM1, timer: &AsyncTimer<ExplicitTimer<LPTIM1>>) {
    // Clear pending interrupt flag
    tim.icr.write(|w| w.bits(ICR_MASK));

    ExplicitTimer::<LPTIM1>::interrupt_free(|cs| {
        timer.get_mut_unsafe(cs).update_time();
    });

    timer.awaken();
}

/// How many ticks there are in a second.
pub const TICKS_PER_SECOND: u64 = 32000;

#[derive(PartialEq, Eq, Ord, PartialOrd, Clone, Copy, Debug)]
pub struct InstantU32(u32);
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

/// A timer based on a peripheral that is inadequate to keep
/// the longer term time by itself.
pub struct ExplicitTimer<TIM> {
    inner: TIM,
    time: AtomicU32,
}

impl ExplicitTimer<LPTIM1> {
    pub fn new(inner: LPTIM1) -> Self {
        Self {
            inner,
            time: AtomicU32::new(0),
        }
    }

    #[inline(always)]
    pub fn update_time(&mut self) {
        self.time.fetch_add(self.get_fraction(), Ordering::SeqCst);
    }

    /// Get a fraction from 0x0000 to 0x001F relative to time.
    /// Used by engineering reports to determine amount of time spent sleeping in nicer fractions.
    #[inline(always)]
    fn get_fraction(&self) -> u32 {
        self.inner.cnt.read().bits() & 0xFFFF
    }
}

impl<TIM> ExplicitTimer<TIM> {}

impl Timer for ExplicitTimer<LPTIM1> {
    type Instant = InstantU32;
    type Duration = DurationU32;

    const DELTA: Self::Duration = DurationU32(1);

    fn reset(&mut self) {
        // No-op.
    }

    #[inline(always)]
    fn unmask() {
        unsafe { NVIC::unmask(Interrupt::LPTIM1) }
    }

    #[inline(always)]
    fn mask() {
        NVIC::mask(Interrupt::LPTIM1)
    }

    #[inline(always)]
    fn interrupt_free<F: FnOnce(&CriticalSection) -> R, R>(f: F) -> R {
        cortex_m::interrupt::free(f)
    }

    #[inline(always)]
    fn now(&self) -> Self::Instant {
        InstantU32(self.time.load(Ordering::Acquire))
    }

    #[inline(always)]
    fn disarm(&mut self) {
        Self::mask();
        self.update_time();
        teardown(&self.inner);
    }

    #[inline(always)]
    fn arm(&mut self, deadline: &Self::Instant) {
        let now = self.now();

        assert!(now < *deadline);
        let delta = deadline.0 - now.0;

        // Max duration is now ~2 seconds.
        // TODO use multiple subsequent interrupts for an alarm.
        let delta = if delta < u16::MAX as u32 {
            delta as u16
        } else {
            // Use the max allowed counter value, and the deadline will be within bounds sometime in the future automatically.
            u16::MAX - 1
        };

        Self::unmask();
        start_timer(&self.inner, delta);
    }
}
