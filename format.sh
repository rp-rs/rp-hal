#!/bin/sh

# Formats all the files in the repo

cargo fmt --manifest-path rp2040-hal/Cargo.toml -- --check
cargo fmt --manifest-path rp2040-hal-macros/Cargo.toml -- --check
cargo fmt --manifest-path rp2040-hal-examples/Cargo.toml -- --check
cargo fmt --manifest-path on-target-tests/Cargo.toml -- --check
