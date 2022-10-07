#!/bin/sh

# Keep running tests even if one of them fails
# We need to specify probe-run as our runner via environment variables here 
# to control build since we aren't able to override them in config.toml
CARGO_TARGET_THUMBV6M_NONE_EABI_RUNNER=probe-run cargo test --no-fail-fast -- --chip rp2040
