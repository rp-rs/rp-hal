# Changelog

All notable changes to this project will be documented in this file.

The format is based on [Keep a Changelog](https://keepachangelog.com/en/1.0.0/),
and this project adheres to [Semantic Versioning](https://semver.org/spec/v2.0.0.html).

## [Unreleased]

## [0.8.0] - 2023-02-16

### Added

- Add SPI slave support - @zaksabeast
- Add set_base_1and0 to interpolator - @tomgilligan
- Add DMA support, with implementations for SPI, PIO, UART - @9names, @mgottschlag, @Nashenas88
- Add alarm cancellation feature - @MuratUrsavas
- Add Pin::id() - @jannic
- Add interrupt functions for DynPin - @larsarv

### Changed

- Several documentation improvements - @nmattia, @tianrking
- CI improvements - @ithinuel, @jannic
- Marked several traits as Sealed - @jannic
- Removed `pwm::PwmPinToken` and add PWM IRQ Input example - @dlkj

## [0.7.0] - 2022-12-11

### MSRV

The Minimum-Supported Rust Version (MSRV) for this release is 1.62

### Changed

- Fixed: First frame is getting lost on a USB-CDC device - @MuratUrsavas
- Moved BSP crates to separate repo at https://github.com/rp-rs/rp-hal-boards - @jannic
- Avoid losing USB status events by reading ints rather than sie_status in poll - @ithinuel
- Allow setting clock divisors on running state machines - @jannic
- Remove unnecessary `mut` from `static mut LOCK_OWNER: AtomicU8` in critical section impl - @zachs18
- Update dependency on rp2040-pac to 0.4.0 - @jannic
- Update embedded-hal alpha support to version 1.0.0-alpha.9 - @jannic
  (Non-blocking traits were moved to embedded-hal-nb, which is not yet supported)
- Implement UartConfig::new constructor method - @jannic
- Deprecate uart::common_configs - @jannic
- Fix spelling error in UART error discarded field - @Sizurka
- Fix contents of UART error discarded field - @Sizurka
- Fix watchdog counter load value - @Sizurka
- Fix concurrent accesses to sm_execctrl and sm_instr when sideset isn't optional - @ithinuel
- pio: Move interrupt related (en|dis)abling/forcing methods to the statemachine - @ithinuel
- Mark Timer & Alarm* Send and Sync - @ithinuel
- The feature critical-section-impl is now enabled by default from the BSP crates - @jannic
- Simplify signature of Alarm::schedule, removing generic parameter - @ithinuel
- USB: Use the dedicated write_bitmask_* functions - @ithinuel

### Added

- Add docs.rs metadata - @icedrocket
- Implement embedded-hal aplha SPI traits - @ptpaterson
- Add derive(Debug) and derive(defmt::Format) to error types - @9names
- Add ability to modify installed PIO program wrap bounds - @davidcole1340
- Add rtic-monotonic support for timer & alarms (feature gated) - @ithinuel
- Add SPI is_busy function - @papyDoctor
- Add set_fifos/set_rx_watermark/set_tx_watermark - @papyDoctor
- Add a method to allow setting the PIO's clock divisor without floats - @ithinuel
- Use TimerInstant in Timer::GetCounter & add Alarm::schedule_at - @ithinuel

### Removed

- Removed support for critical-section 0.2 (was already deprecated) - @jannic

## [0.6.1] - 2022-11-30

### Changed

- Upgraded dependency on critical-section 0.2 to 0.2.8 - @jannic
  (There is also a dependency on version 1.0.0)
- Remove critical-section impl for version 0.2 - @jannic
  Both 0.2.8 and 1.x use the same symbols internally to implement the
  critical sections, so one impl is sufficient, and having both causes
  compilation errors

## [0.6.0] - 2022-08-26

### Added

- Documentation Example for the bsp_pin! macro - @ hmvp
- Timer: Documentation & doc examples for Timers - @9names
- Add suspend, resume and remote wakeup support. - @ithinuel & @jannic
- `rp2040-e5` feature enabling the workaround for errata 5 on the USB device peripheral. - @ithinuel
- NonPwmPinMode for gpio::Disabled - @FlorianUekermann
- RAM-based interrupt vector tables - @9names
- Support for critical-section 1.0.0.
  Critical-section 0.2 is still supported (ie. a custom-impl is provided, compatible
  with the 1.0.0 implementation), to avoid a breaking change. It will be removed
  later. - @jannic
- Add support for the Interpolator. @fenax

### Changed

- Update dev dependencies on cortex-m-rtic to 1.1.2 - @jannic
- Use correct interrupt names in `timer::alarms` - @hmvp
- Update embedded-hal alpha support to version 1.0.0-alpha.8 - @jannic
- Fix PIO rx fifo status - @jannic
- Implement `From<&SomeClock> for Hertz` instead of `From<SomeClock> for Hertz`
  for the clocks in `rp2040_hal::clocks`. - @jannic
- Fix i2c example using the wrong clock. - @jannic
- Fix duty cycle handing on disabled pwm channels. - @jannic
- GPIO IRQ example: add check for interrupt source - @9names
- Align USB synchronisation requirements with the manual & pico-sdk - @ithinuel
- Update dependencies on usb-device to 0.2.9 - @ithinuel
- Use wfi in otherwise empty infinite loops in examples. - @jannic
- Use generic bootloader in examples - @jannic & @ithinuel
- Use `rp2040-hal`'s entry function. - @ithinuel
- Migrate from `embedded-time` to `fugit` - @ithinuel
- Fix PIO's `set_pins` and `set_pindirs` when `out_sticky` is set. - @jannic & @ithinuel
- Clarify usage of boot2 section - @a-gavin

### Removed

- Unnecessary cortex_m::interrupt::free in timer.rs - @jannic
- Unused embassy-traits deps - @9names

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

[Unreleased]: https://github.com/rp-rs/rp-hal/compare/v0.8.0...HEAD
[0.8.0]: https://github.com/rp-rs/rp-hal/compare/v0.7.0...v0.8.0
[0.7.0]: https://github.com/rp-rs/rp-hal/compare/v0.6.0...v0.7.0
[0.6.1]: https://github.com/rp-rs/rp-hal/compare/v0.6.0...v0.6.1
[0.6.0]: https://github.com/rp-rs/rp-hal/compare/v0.5.0...v0.6.0
[0.5.0]: https://github.com/rp-rs/rp-hal/compare/v0.4.0...v0.5.0
[0.4.0]: https://github.com/rp-rs/rp-hal/compare/v0.3.0...v0.4.0
[0.3.0]: https://github.com/rp-rs/rp-hal/compare/v0.2.0...v0.3.0
[0.2.0]: https://github.com/rp-rs/rp-hal/compare/v0.1.0...v0.2.0
[0.1.0]: https://github.com/rp-rs/rp-hal/releases/tag/v0.1.0
