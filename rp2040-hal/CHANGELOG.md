# Changelog

All notable changes to this project will be documented in this file.

The format is based on [Keep a Changelog](https://keepachangelog.com/en/1.0.0/),
and this project adheres to [Semantic Versioning](https://semver.org/spec/v2.0.0.html).

## [Unreleased]

### Changed

- Update embedded-hal alpha support to version 1.0.0-alpha.8
- Implement `From<&SomeClock> for Hertz` instead of `From<SomeClock> for Hertz`
  for the clocks in `rp2040_hal::clocks`.

## [0.5.0] - 2022-06-13

### MSRV

The Minimum-Supported Rust Version (MSRV) for this release is 1.61

### Added

- RP2040 specific #[entry] macro that releases spinlocks - @jannic
- Start multiple state machines in sync with each other - @astraw
- Unsafe fn for freeing all spinlocks when you can't use the RP2040 entry macro (eg RTIC) - @9names
- Optional feature for enabling defmt formatting for i2c errors - @ithinuel
- Accessor for getting the offset of an installed PIO program - @fenax

### Changed

- Use thread send safe UART* marker when splitting, improves UART ergonmics - @marius-meissner
- Improve performance for hardware division instrinsics. Internal intrinsics cleanup - @Sizurka
- Provide a better alarm abstraction - @ithinuel
- Update Multicore::spawn to be able to take a closure without requiring alloc.
  Improve Multicore ergonomics and add example for how to use new API - @Liamolucko
- Allow PIO program to be 32 instructions long, was previously limited to 31 - @jannic
- Fix Typos - @mqy, @danbev
- Replace generic pio::Tx::write<T> with write_u8_replicated, write_u16_replicated, and update
  write to take a u32. The generic version was too easy to misuse. - @9names

### Removed

- I2c async driver. Use new one at https://github.com/ithinuel/rp2040-async-i2c/ - @ithinuel
- Unused fields from UartPeripheral and Reader - @jannic

## [0.4.0] - 2022-03-09

### Added

- ROM function caching
- ROM version lookup function
- Compiler intrinsics for ROM functions
- Compiler intrinsics for hardware divider
- Document bsp_pins! macro
- UART IRQ examples
- PIO side-set example
- Stopped PIO state machines can change their clock divider
- Added HAL IRQ example

### Changed

- Rewrite UART driver to own its pins
- Improve UART defaults
- Fix repeated-read in i2c embassy driver
- Fix bug in i2c peripheral state machine
- Fix race condition in alarm
- Fix safety bugs in hardware divider
- Enable watchdog reset trigger bits when watchdog enabled
- Update spinlocks to use new PAC API
- Use generics to improve spinlock implementation
- Update critical_section to use new spinlock implementation
- Update embedded-hal alpha support to version 1.0.0-alpha.7
- Avoid 64-bit division in clock calculations
- Update pio and pio-proc to 0.2.0

## [0.3.0] - 2021-12-19

### MSRV

The Minimum-Supported Rust Version (MSRV) for this release is 1.54.

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

[Unreleased]: https://github.com/rp-rs/rp-hal/compare/v0.5.0...HEAD
[0.5.0]: https://github.com/rp-rs/rp-hal/compare/v0.4.0...v0.5.0
[0.4.0]: https://github.com/rp-rs/rp-hal/compare/v0.3.0...v0.4.0
[0.3.0]: https://github.com/rp-rs/rp-hal/compare/v0.2.0...v0.3.0
[0.2.0]: https://github.com/rp-rs/rp-hal/compare/v0.1.0...v0.2.0
[0.1.0]: https://github.com/rp-rs/rp-hal/releases/tag/v0.1.0
