# Changelog

All notable changes to this project will be documented in this file.

The format is based on [Keep a Changelog](https://keepachangelog.com/en/1.0.0/),
and this project adheres to [Semantic Versioning](https://semver.org/spec/v2.0.0.html).

## [Unreleased]

## [0.4.0] - 2026-02-01

### MSRV

The Minimum-Supported Rust Version (MSRV) for the next release is 1.82

- Bump MSRV to 1.82 because the crate `indexmap` (an indirect build-depencency) requires it.

### Added
- Implement core::fmt::Display and core::err:Error for ReadErrorType per the requirements of embedded-io 0.7.1

### Changed

- Update `pio` to 0.3.0 - #923 #918 @jannic
- Update `rand_core` to 0.9.3 - #941 @jannic
- Update `embedded-io` to 0.7.1 - #967 @Gip-Gip
- Update `rp235x-pac` to 0.2.0 - #982 @9names
- Update `rp-binary-info` to 0.1.1 - #911 @jannic
- Breaking change: ADC now reports conversion errors - #740 @jannic
- Add additional pins for RP2350B - #920 @cantudo
- Add interrupt support to RP235x riscv - #847 @thejpster
- Add support for PIO2 on RP235x - #931 #933 @horazont @KewlKris
- Add support for PIO PUT and GET FIFO modes - #926 @KewlKris
- Fix `embedded-hal::i2c::transaction` support by removing automatic generation of i2c restart between transactions - #827 @jannic
- Fix handling of flags in `RebootKind::BootSel` reboot -  #957 @iwanders
- Fix `rp_binary_end` - #932 @jannic
- Fix new clippy warnings - #930 @jannic 
- Fix `manual_div_ceil` clippy lints - #902 @jannic
- Allow 4 alarms per timer - #925 @McBandaska and @thejpster
- Implement `SetDutyCycle` generically on PWM ChannelId  - #909 @jannic
- Update number of DMA channels to 16 on RP235x - #919 #927 @KewlKris
- Improve docs by replacing use of "Raspberry Silicon" with "Raspberry Pi", and "RP2350" with "RP235x" - #956 @thejpster
- Improve docs around GPIO proc_int[sef] functions - #908 @jannic

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

[Unreleased]: https://github.com/rp-rs/rp-hal/compare/rp235x-hal-0.4.0...HEAD
[0.4.0]: https://github.com/rp-rs/rp-hal/compare/rp235x-hal-0.3.0...rp235x-hal-0.4.0
[0.3.0]: https://github.com/rp-rs/rp-hal/compare/rp235x-hal-0.2.0...rp235x-hal-0.3.0
