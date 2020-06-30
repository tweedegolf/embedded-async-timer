//! A AsyncTimer implementation for the LPTIM1 peripheral in the STM32L4x6 running at 32kHz.
//!
//! This module provides it's own RTC implementation on top of the LPTIM1 which extends its 16 bit counter to 32 bits,
//! and provides alarms.
//!
//! This implementation assumes:
//! * LSI on.
//! * 32kHz granularity.
//! * Interrupts are handled at least every second.
//! * ~1.5 days maximum operation time.
//!
//! **Warning**: time will not observably progress (from `get_time()`) unless interrupts are handled.
//!
//! ```
//! let p = stm32l4xx_hal::stm32::Peripherals::take().unwrap();
//! let timer = AsyncTimer::new(Lptim1Rtc::new(p.LPTIM1, &p.RCC)));
//! let mut flash = p.FLASH.constrain();
//! let mut rcc = p.RCC.constrain();
//! rcc.cfgr.lsi(true).freeze(&mut flash.acr);
//!```
//!
//! Do not forget to call `handle_interrupt` in the interrupt handler for `LPTIM1`.

use crate::{AsyncTimer, Timer};
use core::num::NonZeroU32;
use core::sync::atomic::{compiler_fence, AtomicBool, AtomicU16, Ordering};
use cortex_m::interrupt::CriticalSection;
use stm32l4xx_hal::stm32::{Interrupt, LPTIM1, NVIC, RCC};

const ALARM_TICKS_PER_SECOND: u64 = 32000;

/// Ring size for timer. Timer will overflow during this value.
///
/// You might consider the values for ALARM_SEGMENT and 0 in fragments to be equal.
const ALARM_SEGMENT: u32 = 0xff00;

/// Any part between ALARM_SEGMENT and ALARM_DIRTY_ZONE will yield previous ALARM_SEGMENT as time.
/// The ARRM interrupt **MUST** trigger within this zone to make this logic work.
/// In our case that is within ~1Hz.
const ALARM_DIRTY_ZONE: u32 = ALARM_SEGMENT / 2;

/// Configurated ARR value; timer counter will reset during ALARM_ARR+1 and yield 0 sometimes instead.
const ALARM_ARR: u32 = ALARM_SEGMENT - 1;

/// A low-level Real Time Clock with alarms implemented with the LPTIM1 peripheral.
///
/// *Note:* call `handle_interrupt` in the LPTIM1 interrupt.
///
/// Uses the LSI to yield a 32Khz operating speed. Uses 16 bit time segments to create 32 bit time at 32Khz, yielding a total operating time of 1.5 days.
/// The time will wrap around when saturated, but care must be taken to properly wrap around deadlines as well.
/// Currently wrapped around deadlines will trigger immediately.
/// During the lifetime of this timer 3 interrupts will be triggered:
/// * ARR, once the 16 bit internal counter wraps around.
/// * CMP, to handle deadlines.
/// * CMPOK, when new deadlines have been uploaded to the timer.
///
/// You can extend the 1.5 days operating time by either making time an U64, or by adjusting the prescaler or source clock to a lower operating frequency.
///
/// Operates fine with the STOP2 sleep mode.
pub struct Lptim1Rtc {
    /// The Lower Power Timer 1 register block.
    inner: LPTIM1,
    /// Our time in current amount of fractions.
    time: AtomicU16,
    /// Our most significant bits (i.e. `time`) have not yet been updated.
    dirty: AtomicBool,
    /// Indication whether the CMP register is being written to.
    /// Can only write the CMP register when uploading is not busy.
    uploading: AtomicBool,
    /// Deadline for an alarm, if set.
    deadline: Option<NonZeroU32>,
}

/// Error to indicate that the just scheduled deadline has already passed.
#[derive(Debug)]
pub struct DeadlinePassed;

impl Lptim1Rtc {
    /// Set up the heartbeat timer and interrupt.
    ///
    /// Wakes the CPU at least every 2 seconds to update the time, and more often when necessary.
    /// Enables casually running sleep or stop2 whilst using async delays.
    pub fn new(timer: LPTIM1, rcc: &RCC) -> Self {
        rcc.apb1enr1.modify(|_, w| w.lptim1en().set_bit());
        rcc.apb1rstr1.modify(|_, w| w.lptim1rst().set_bit());
        rcc.apb1rstr1.modify(|_, w| w.lptim1rst().clear_bit());
        rcc.apb1smenr1.modify(|_, w| w.lptim1smen().set_bit()); // Power LPTIM1 in LP mode

        // Set clock to LSI, which should be 32KHz
        rcc.ccipr.modify(|_, w| unsafe { w.lptim1sel().bits(0b01) });

        Self {
            inner: timer,
            time: AtomicU16::new(0),
            // We start with dirty, and will trigger a cmp soon.
            dirty: AtomicBool::new(true),
            uploading: AtomicBool::new(false),
            deadline: None,
        }
    }

    /// Start the timer, starting with fraction `count`.
    ///
    /// We will expect a CMPOK interrupt soon after this call. Hence make sure that the handle_interrupt is already callable
    /// from the interrupt when calling `start_timer`.
    pub fn start_timer(&mut self) {
        // Disable the LPTIM device such that we can configure it & the interrupts.
        self.inner.cr.modify(|_, w| w.enable().clear_bit());
        NVIC::mask(Interrupt::LPTIM1);

        self.inner.cfgr.modify(|_, w| {
            w.cksel().clear_bit(); // internal clocksource
            w.enc().clear_bit(); // no encoder mode
            w.preload().clear_bit(); // load at any register write
            w.countmode().clear_bit(); // the counter is incremented following each internal clock pulse
            w.timout().set_bit();
            unsafe {
                w.presc().bits(0b000); // x1
                w.ckpol().bits(0b00);
                w.trigen().bits(0b00)
            }
        });

        // Enable ARRM, CMPM and CMPOK events, though we only use CMPM when a deadline has been set.
        self.inner
            .ier
            .write(|w| w.arrmie().set_bit().cmpmie().set_bit().cmpokie().set_bit());

        unsafe { NVIC::unmask(Interrupt::LPTIM1) };

        // Enable the LPTIM device.
        self.inner.cr.modify(|_, w| w.enable().set_bit());

        // Set ARR and CMP to default reset values.
        self.inner.arr.write(|w| unsafe { w.bits(ALARM_ARR) });
        self.set_cmp(ALARM_DIRTY_ZONE);

        // Await until it is configured.
        compiler_fence(Ordering::SeqCst);

        self.inner.cr.modify(|_, w| w.cntstrt().set_bit());
    }

    /// Get the current time.
    pub fn get_time(&self) -> u32 {
        // Get the time, but the answer might be unstable because
        // we need to perform multiple (albeit atomic) operations to reach the time.
        let get_time_unstable = || {
            let frac: u32 = self.get_fraction(); // Might take a few cycles.

            let frac = if frac >= ALARM_SEGMENT {
                ALARM_SEGMENT
            } else if frac < ALARM_DIRTY_ZONE {
                frac + ALARM_SEGMENT
            } else if self.dirty.load(Ordering::Acquire) {
                // Implicitly somewhere between ALARM_SEGMENT and ALARM_DIRTY_ZONE + a little.
                frac + ALARM_SEGMENT
            } else {
                frac
            };

            ((self.time.load(Ordering::Acquire) as u32) * ALARM_SEGMENT)
                .wrapping_add(frac)
                .wrapping_sub(ALARM_SEGMENT) // To compensate for our start with a dirty MSB=0 segment.
        };

        loop {
            // We get the time twice, to ensure that we get a consistent answer.
            let x = get_time_unstable();
            let y = get_time_unstable();

            if x == y {
                return x;
            }
        }
    }

    /// Set the alarm for some time on or after a specific deadline.
    pub fn set_alarm(&mut self, deadline: u32) -> Result<(), DeadlinePassed> {
        self.deadline = NonZeroU32::new(deadline);
        if self.update_alarm(false) {
            // Alarm was triggered directly after setting the alarm.
            Err(DeadlinePassed)
        } else {
            Ok(())
        }
    }

    /// Clear the alarm, regardless of being set.
    pub fn clear_alarm(&mut self) {
        self.deadline = None;
        self.reset_cmp();
    }

    /// Handle LPTIM1 interrupt, both to update time and a signal a possible alarm.
    ///
    /// Returns whether an alarm was triggered.
    /// Needs to be run in interrupt-free context.
    pub fn handle_interrupt(&mut self) -> bool {
        let isr = self.inner.isr.read();
        let arrm = isr.arrm().bit();
        let cmpm = isr.cmpm().bit();
        let cmpok = isr.cmpok().bit();

        if cmpok {
            self.uploading.store(false, Ordering::Release);
        }

        if arrm {
            // Update the time as we've moved into the next segment.
            self.dirty.store(true, Ordering::Release);
        }

        // This segment requires interrupt free context.
        let fraction = self.get_fraction();
        if cmpm
            && fraction < ALARM_SEGMENT
            && fraction >= ALARM_DIRTY_ZONE
            && self
                .dirty
                .compare_exchange(true, false, Ordering::SeqCst, Ordering::Acquire)
                .is_ok()
        {
            self.time.fetch_add(1, Ordering::SeqCst);
        }

        if arrm & cmpm {
            panic!("Invariant ARRM and CMPM together violated");
        }

        // Always update alarm to most recent, as upload of CMP may intersect.
        // Returns whether alarm was triggered.
        let u = self.update_alarm(true);

        // Clear pending interrupt flags.
        self.inner
            .icr
            .write(|w| w.arrmcf().bit(arrm).cmpmcf().bit(cmpm).cmpokcf().bit(cmpok));

        compiler_fence(Ordering::SeqCst);

        u
    }

    /// When comparisons are not needed, align cmp with arr such that no extra unnecessary interrupts are triggered.
    #[inline(always)]
    fn reset_cmp(&mut self) {
        self.set_cmp(ALARM_DIRTY_ZONE);
    }

    #[inline(always)]
    fn set_cmp(&mut self, mut fractions: u32) {
        if fractions == ALARM_SEGMENT {
            // Never allow cmp to be set on ALARM_SEGMENT, because the peripheral would glitch out.
            fractions = 0;
        } else if fractions > ALARM_DIRTY_ZONE && self.dirty.load(Ordering::Acquire) {
            // If deadline is in the future, but we're dirty, schedule a CMP first thing after the dirty zone.
            // TODO consider whether this is necessary.
            fractions = ALARM_DIRTY_ZONE;
        }

        // The currently configured deadline is no longer correct.
        if fractions != self.inner.cmp.read().bits() {
            // Claim the upload pipeline if available to update the deadline.
            if self
                .uploading
                .compare_exchange(false, true, Ordering::SeqCst, Ordering::Acquire)
                .is_ok()
            {
                self.inner.cmp.write(|w| unsafe { w.bits(fractions) });
            }
        }
    }

    /// Internally set registers to properly reflect the currently set alarm.
    ///
    /// Returns true if alarm has triggered (i.e. deadline passed).
    /// When triggered will unset alarm, thus consecutive calls will not yield true until a new alarm is set.
    fn update_alarm(&mut self, consume_alarm: bool) -> bool {
        if let Some(deadline) = self.deadline {
            // Note: if deadline && dirty, `set_cmp` will set the CMP alarm to
            // ALARM_DIRTY_ZONE iff fractions >= ALARM_DIRTY_ZONE.
            let deadline = deadline.get();
            let now = self.get_time();
            if deadline <= now {
                // Deadline has passed and throw the alarm.
                if consume_alarm {
                    // Remove the deadline and align the CMP interrupt with ARR, such that we throw one interrupt less for each segment.
                    self.deadline.take();
                    self.reset_cmp();
                } else {
                    // Schedule the interrupt ASAP.
                    self.set_cmp((now + 2) % ALARM_SEGMENT);
                }
                return true;
            } else if deadline < ((now / ALARM_SEGMENT) + 1) * ALARM_SEGMENT {
                // The deadline is within our current segment. Schedule the interrupt.
                self.set_cmp(deadline % ALARM_SEGMENT);
            } else {
                // Deadline is too far into the future, not in this segment.
                self.reset_cmp();
            }
        } else if self.dirty.load(Ordering::Acquire) {
            // If no deadline && dirty always set alarm to end of DIRTY_ZONE.
            self.set_cmp(ALARM_DIRTY_ZONE);
            return false;
        }

        false
    }

    /// Disable the LPTIM1 peripheral.
    fn teardown(&mut self) {
        self.inner.cr.modify(|_, w| w.enable().clear_bit());
        self.inner
            .icr
            .write(|w| w.arrmcf().set_bit().cmpmcf().set_bit());
    }

    /// Get a fraction from 0x0000 to 0xFFFF relative to time.
    pub fn get_fraction(&self) -> u32 {
        // We must get two consecutive consistent reads.
        loop {
            let x = self.inner.cnt.read().bits();
            let y = self.inner.cnt.read().bits();

            if x == y {
                return x & 0xFFFF;
            }
        }
    }
}

impl Drop for Lptim1Rtc {
    fn drop(&mut self) {
        self.teardown();
    }
}

/// The instant as directly used by the `Lptim1Rtc` timer.
#[derive(PartialEq, Eq, Ord, PartialOrd, Clone, Copy, Debug)]
pub struct InstantU32(pub u32);

/// The duration as efficiently usable with instants of the `Lptim1Rtc` timer.
#[derive(Clone)]
pub struct DurationU32(pub u32);

impl core::convert::From<core::time::Duration> for DurationU32 {
    fn from(duration: core::time::Duration) -> Self {
        Self(
            ((duration.as_secs() as u64) * ALARM_TICKS_PER_SECOND
                + (duration.subsec_nanos() as u64) * ALARM_TICKS_PER_SECOND / 1_000_000_000)
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

impl Timer for Lptim1Rtc {
    type Instant = InstantU32;
    type Duration = DurationU32;

    const DELTA: Self::Duration = DurationU32(2);

    #[inline(always)]
    fn reset(&mut self) {
        self.start_timer()
    }

    #[inline(always)]
    fn interrupt_free<F: FnOnce(&CriticalSection) -> R, R>(f: F) -> R {
        cortex_m::interrupt::free(f)
    }

    #[inline(always)]
    fn now(&self) -> Self::Instant {
        InstantU32(self.get_time())
    }

    #[inline(always)]
    fn disarm(&mut self) {
        self.clear_alarm();
    }

    #[inline(always)]
    fn arm(&mut self, deadline: &Self::Instant) {
        // Note we ignore that the deadline has already passed.
        let _ = self.set_alarm(deadline.0);
    }
}

/// Handle the LPTIM1 interrupt.
///
/// Will yield whether the AsyncTimer has been awoken.
///
/// You can call it as such:
/// ```rust
/// static mut TIMER: Option<AsyncTimer<Lptim1Rtc>> = None;
///
/// #[interrupt]
/// #[allow(non_snake_case)]
/// #[no_mangle]
/// fn LPTIM1() {
///     cortex_m::interrupt::free(|_| {
///         if let Some(timer) = unsafe { TIMER.as_ref() } {
///             handle_interrupt(move || timer);
///         }
///     })
/// }
/// ```

#[inline(always)]
pub fn handle_interrupt<'a>(get_timer: impl FnOnce() -> &'a AsyncTimer<Lptim1Rtc>) -> bool {
    let timer = get_timer();
    let triggered = unsafe { timer.get_inner(|t| t.handle_interrupt()) };
    if triggered {
        timer.awaken();
    }
    triggered
}
