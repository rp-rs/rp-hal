# Changelog

All notable changes to this project will be documented in this file.

The format is based on [Keep a Changelog](https://keepachangelog.com/en/1.0.0/),
and this project adheres to [Semantic Versioning](https://semver.org/spec/v2.0.0.html).

## [Unreleased]

### Added

- None

### Changed

- None

## [0.3.0] - 2021-12-19

### Added

- A README!
- Implementation of the `critical-section` API
- Embedded HAL 1.0-alpha6 support
- I²C Controller and Peripheral support
- Multi-core support, including spin-locks and SIO FIFO
- RTC support
- USB Device support
- Timer support
- PIO support
- Implementation of `rng_core::RngCore` for `RingOscillator`
- ADC example
- GPIO Interrupt support
- Multi-core FIFO example
- PIO LED Blinky example
- ROM Functions example
- SPI example
- Watchdog example
- ADC documentation
- Lots of bug fixes :)

### Changed

- Modified PIO API for better ergonomics
- Updated PAC to 0.2.0
- Exported common driver structs from top-level (e.g. it's now `Sio`, not `sio::Sio`)

## [0.2.0] - 2021-08-14

### Added

- Updated version with support for:
  - Ring Oscillator
  - Crystal Oscillator
  - Plls
  - Watchdog
  - Clocks
  - GPIO
  - PWM
  - ADC
  - SPI
  - I²C
  - Resets
  - UART
  - Hardware divide/modulo

## [0.1.0] - 2021-01-21

- Initial release

[Unreleased]: https://github.com/rp-rs/rp-hal/compare/v0.3.0...HEAD
[0.3.0]: https://github.com/rp-rs/rp-hal/compare/v0.2.0...v0.3.0
[0.2.0]: https://github.com/rp-rs/rp-hal/compare/v0.1.0...v0.2.0
[0.1.0]: https://github.com/rp-rs/rp-hal/releases/tag/v0.1.0
