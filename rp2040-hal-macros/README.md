# `rp2040-hal-macros`

Macros used by rp2040-hal.

## Entry macro

Extension of the `cortex-m-rt` `#[entry]` with rp2040 specific initialization code.

Currently, it just unlocks all spinlocks before calling the entry function.

# License

Licensed under either of

- Apache License, Version 2.0 (`APACHE2.0` or
  http://www.apache.org/licenses/LICENSE-2.0)

- MIT license (`MIT` or http://opensource.org/licenses/MIT)

at your option.

## Contribution

Unless you explicitly state otherwise, any contribution intentionally submitted
for inclusion in the work by you, as defined in the Apache-2.0 license, shall be
dual licensed as above, without any additional terms or conditions.

