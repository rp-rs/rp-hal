rem Formats all the files in the repo

cargo fmt --manifest-path rp2040-hal\Cargo.toml
cargo fmt --manifest-path rp2040-hal-macros\Cargo.toml
cargo fmt --manifest-path rp2040-hal-examples\Cargo.toml
cargo fmt --manifest-path on-target-tests\Cargo.toml
cargo fmt --manifest-path rp235x-hal\Cargo.toml
cargo fmt --manifest-path rp235x-hal-macros\Cargo.toml
cargo fmt --manifest-path rp235x-hal-examples\Cargo.toml
cargo fmt --manifest-path rp-hal-common\Cargo.toml
