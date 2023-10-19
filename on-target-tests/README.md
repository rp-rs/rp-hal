# Target tests for rp2040-hal

This project is for running tests of rp2040-hal against real hardware via knurling-rs tools

Adding a test:  
- Add a new Rust program to tests (eg tests/my_new_test.rs)
- Add a new [[test]] to the Cargo.toml

Running all tests:  
Linux (and any other Unix-likes where probe-rs are supported):
```system
./run_tests.sh
```
Windows
```system
run_tests.bat
```

To run a specific test (to make developing tests faster)

```system
CARGO_TARGET_THUMBV6M_NONE_EABI_RUNNER="probe-rs run" cargo test --test my_new_test -- --chip rp2040
```

## Prerequisites

Some of the tests need connections between specific pins.

Currently, the following connections are required:

- Connect GPIO 4 to GPIO 7 (pins 6 and 10 an a Pico) for the SPI loopback tests

If you add tests that need some hardware setup, make sure that they are
compatible to the existing on-target tests, so all tests can be run with
a single configuration.
