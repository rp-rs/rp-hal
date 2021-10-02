# Changelog

All notable changes to this project will be documented in this file.

The format is based on [Keep a Changelog](https://keepachangelog.com/en/1.0.0/),
and this project adheres to [Semantic Versioning](https://semver.org/spec/v2.0.0.html).

## [Unreleased]

### Added
- implement `rng_core::RngCore` for `RingOscillator`

### Changed
- Modified PIO API for better ergonomics

## [0.3.0] - 2021-09-20

### Added
- peripheral drivers: Timer(counter) with EH traits, USB, PIO
- examples: Watchdog, GPIO, ADC, i2c, rtic blinky, usb (serial echo, mouse)
- docs: Watchdog
- bsps: adafruit feather
- bsp examples: sparkfun pro micro PIO RGB rainbow LED

### Changed
- i2c fixes
- spi fixes
- pwm fixes

## [0.2.0] - 2021-08-14

### Added

- Initial version with support for Rosc, Xosc, Plls, Watchdog, Clocks, Gpio, Pwm, Adc, Spi, I2C, Resets, Uart

## [0.1.0]

- Initial release

[Unreleased]: https://github.com/rp-rs/rp-hal/compare/v0.3.0...HEAD
[0.3.0]: https://github.com/rp-rs/rp-hal/compare/v0.2.0...v0.3.0
[0.2.0]: https://github.com/rp-rs/rp-hal/compare/v0.1.0...v0.2.0
[0.1.0]: https://github.com/rp-rs/rp-hal/releases/tag/v0.1.0
