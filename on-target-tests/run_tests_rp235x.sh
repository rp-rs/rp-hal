#!/bin/sh

# Keep running tests even if one of them fails
# We need to specify probe-rs as our runner via environment variables here
# to control build since we aren't able to override them in config.toml
CARGO_TARGET_THUMBV8M_MAIN_NONE_EABIHF_RUNNER="probe-rs run" cargo test --target thumbv8m.main-none-eabihf --no-fail-fast --features rp235x -- --chip rp235x
