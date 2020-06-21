# embedded-async-timer
Async timers for embedded devices in Rust.

This crate provides an interface and a generic implemention of a timer that can handle multiple concurrent deadlines using the Future architecture provided by Rust. This crate also provides two reference implementations for this interface for the STM32F103 and STM32L476.

**Note:** until `const_generics` lands the capacity for AsyncTimer is hard-coded.

**State:** this crate has been tested with trivial examples on two embedded devices, but has not yet been applied in production. It is beyond a simple proof of concept though. Our intention to provide a basis for an async ecosystem. The quality might also be insufficient to run on production. We encourage developing async HAL drivers using this crate though.

