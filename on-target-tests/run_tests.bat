@rem Keep running tests even if one of them fails
@rem We need to specify environment variables here to control build since we aren't able to override them in Cargo.toml

@SET CARGO_TARGET_THUMBV6M_NONE_EABI_RUNNER=probe-run

cargo test --no-fail-fast -- --chip rp2040
