# Target tests for rp2040-hal

This project is for running tests of rp2040-hal against real hardware via knurling-rs tools

Adding a test:  
- Add a new Rust program to tests (eg tests/my_new_test.rs)
- Add a new [[test]] to the Cargo.toml

Running all tests:  
Linux (and any other Unix-likes where probe-run are supported):
```system
./run_tests.sh
```
Windows
```system
run_tests.bat
```

To run a specific test (to make developing tests faster)

```system
CARGO_TARGET_THUMBV6M_NONE_EABI_RUNNER=probe-run cargo test --test my_new_test -- --chip rp2040
```
