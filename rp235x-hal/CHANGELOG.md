# Changelog

All notable changes to this project will be documented in this file.

The format is based on [Keep a Changelog](https://keepachangelog.com/en/1.0.0/),
and this project adheres to [Semantic Versioning](https://semver.org/spec/v2.0.0.html).

## [Unreleased]

## [0.3.0] - 2025-03-02

### MSRV

The Minimum-Supported Rust Version (MSRV) for the next release is 1.79

- Bump MSRV to 1.79 to enable inline\_const, used for static asserts.

### Changed

- Enable co-processors when spawning on core1 - #900 @jannic
- Set EXTEXCLALL when enabling multi-core operation, fixing atomics - #898 @jannic
- Fix implementations of embedded\_io::Write for UartPeripheral - #895 @jannic
- Copy implementation of PinGroup::set\_u32 from rp2040-hal - #893 @jannic
- Derive Debug and defmt::Format for StackAllocation - #889 @jannic
- Implement support for XipCS1 pin function - #873 @Altaflux
- Add a nice wrapper for get\_sys\_info - #877 @nahla-nee
- feat: add support for PIO `in_count` - #867 @allexoll
- implement Debug support for the GPIO structures - #866 @jannic
- Fix clippy warnings reported by rust beta - #865 #887 @jannic
- Update critical-section dependency to version 1.2.0 - #862 @jannic
- spi: port `set_format` changes to rp235x - #842 @jannic

## 0.2.0 - 2024-09-25

This was the initial public release of rp235x-hal. The port from
rp2040-hal was done by Jonathan Pallant (@thejpster).

[Unreleased]: https://github.com/rp-rs/rp-hal/compare/rp235x-hal-0.3.0...HEAD
[0.3.0]: https://github.com/rp-rs/rp-hal/compare/rp235x-hal-0.2.0...rp235x-hal-0.3.0
