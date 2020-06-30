# embedded-async-timer
Async timers for embedded devices in Rust.

This crate provides an interface and a generic implemention of a timer that can handle multiple
concurrent deadlines using the Future architecture provided by Rust. This crate also provides
two reference implementations for this interface for the STM32F103 and STM32L476.

Using implementations for the `Timer` trait, you can instance an `AsyncTimer` that provides methods
to concurrently wait on a specific deadline or duration with a `Future`-compatible interface.
Using this crate you can implement device drivers that require delays. Using HAL crates that are
compatible with `embedded-async-timer` you can develop firmware that directly require delays or use
these aforementioned drivers.
For HAL RTC implementations with alarms it should be trivial to adapt them to implement the `Timer` trait.
View the STM32F103 example in the `impls` module for inspiration.

**Note:** until `const_generics` lands the capacity for AsyncTimer is hard-coded.

**Note:** the current design assumes a `bare_metal`-like single core architecture that supports
`interrupt::free`-like blocks. In the future I would like to rewrite everything to no longer require this.

**Note:** the priority queue used in this implementation is very likely to not be the most efficient choice of data structure.
I intend to experiment with various setups to make an informed performance consideration based on real
measurements given a small amount of concurrent timers.

**State:** this crate has been tested with trivial examples on two embedded devices, but has not yet
been applied in production. It is beyond a simple proof of concept though. Our intention to provide a
basis for an async ecosystem. The quality might also be insufficient to run on production.
Also the current state of async-await might be such that it is practically unusable for memory constrained embedded devices.
We encourage developing async HAL drivers using this crate though.

# Example
```rust
async {
    let timer = AsyncTimer::new(rtc);
    timer.wait(Duration::from_millis(500).into()).await;

    let mut interval = Interval::new(Duration::from_secs(1).into(), &timer);

    loop {
        interval.wait(&timer).await;
        println!("ping");
    }
}
```

