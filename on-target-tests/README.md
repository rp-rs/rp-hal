# Target tests for rp2040-hal

This project is for running tests of rp2040-hal against real hardware via knurling-rs tools

Adding a test:  
- Add a new Rust program to tests (eg tests/my_new_test.rs)
- Add a new [[test]] to the Cargo.toml

Running all tests:  
Linux (and any other Unix-likes where probe-rs are supported):
```system
cd on-target-tests
./run_tests.sh
```
Windows
```system
cd on-target-tests
run_tests.bat
```

To run a specific test (to make developing tests faster)

```system
CARGO_TARGET_THUMBV6M_NONE_EABI_RUNNER="probe-rs run" cargo test -p on-target-tests --test my_new_test -- --chip rp2040
```

## Prerequisites

Some of the tests need connections between specific pins.

Currently, the following connections are required:

- Connect GPIO 4 to GPIO 7 (pins 6 and 10 an a Pico) for the SPI loopback tests
- Connect GPIO 0 to GPIO 2 (pins 1 and 4 on a Pico) and
  connect GPIO 1 to GPIO 3 (pins 2 and 5 on a Pico) for the I2C loopback tests

If you add tests that need some hardware setup, make sure that they are
compatible to the existing on-target tests, so all tests can be run with
a single configuration.

## License

The contents of this repository are dual-licensed under the _MIT OR Apache 2.0_
License. That means you can choose either the MIT license or the Apache 2.0
license when you re-use this code. See [`LICENSE-MIT`](./LICENSE-MIT) or
[`LICENSE-APACHE`](./LICENSE-APACHE) for more information on each specific
license. Our Apache 2.0 notices can be found in [`NOTICE`](./NOTICE).

Unless you explicitly state otherwise, any contribution intentionally submitted
for inclusion in the work by you, as defined in the Apache-2.0 license, shall be
dual licensed as above, without any additional terms or conditions.

