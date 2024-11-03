# Changelog

All notable changes to this project will be documented in this file.

The format is based on [Keep a Changelog](https://keepachangelog.com/en/1.0.0/),
and this project adheres to [Semantic Versioning](https://semver.org/spec/v2.0.0.html).

## [Unreleased]

## [0.3.0] - 2024-11-03

### Changed

- feat: add support for PIO `in_count`
- implement Debug support for the GPIO structures (RP235x)
- Fix clippy warnings reported by rust beta
- Merge pull request #862 from jannic/update-critical-section
- Update critical-section dependency to version 1.2.0
- spi: port `set_format` changes to rp235x

## [0.2.0] - 2024-09-25

### Changed

- Don't allow unreachable_patterns
- Disable several warnings that show up with rust 1.82.0(beta)
- Merge pull request #841 from jannic/update-rp2350-uart
- Add uart_loopback example.
- Port SPI changes from rp2040-hal to rp235x-hal
- Port UART updates to rp235x-hal
- Update rp235x-hal/src/timer.rs
- Update rp235x-hal/src/lposc.rs
- RP235x: Formatting
- RP235x: More datasheet fixes.
- RP235x: datasheet link clean-ups
- Use published 2350 PAC.
- Format rp235x-hal/Cargo.toml
- Moved some UART stuff into rp-hal-common.
- Add an RP2350 HAL.
