//! Peripheral implementations for the Timer trait.
//!
//! You will need to properly instantiate the peripheral yourself, and register the appropriate interrupt.
//! The timer will require a `'static` lifetime to be accessible in the interrupt,
//! if not using the RTFM crate or similar.
//!
//! You can implement Timer for a Peripheral yourself. Please use these implementations
//! for reference on how to do this, and refer to the documentation of the Timer trait.

#[cfg(feature = "stm32f103")]
pub mod stm32f103_rtc;
#[cfg(feature = "stm32l4x6")]
pub mod stm32l4x6_lptim1;
