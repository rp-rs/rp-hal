<!-- PROJECT LOGO -->
<br />
<p align="center">
  <a href="https://github.com/rp-rs/rp-hal">
    <img src="https://www.svgrepo.com/show/281119/microchip.svg" alt="Logo" width="140" height="140">
  </a>

   <h3 align="center">rp-hal</h3>

  <p align="center">
    Rust support for the Raspberry Pi family of microcontrollers
    <br />
    <strong>Explore the API docs for <a href="https://docs.rs/rp2040-hal">RP2040</a> or <a href="https://docs.rs/rp2350-hal">RP2350</a></strong>
    <br />
    <br />
    <a href="https://github.com/rp-rs/rp-hal-boards/tree/main/boards/rp-pico/examples">View Demos</a>
    ·
    <a href="https://github.com/rp-rs/rp-hal/issues">Report a Bug</a>
    ·
    <a href="https://matrix.to/#/#rp-rs:matrix.org">Chat on Matrix</a>
  </p>
</p>


<!-- TABLE OF CONTENTS -->
<details open="open">
  <summary><h2 style="display: inline-block">Table of Contents</h2></summary>
  <ol>
    <li><a href="#getting-started">Getting Started</a></li>
    <li><a href="#programming">Programming</a></li>
    <li><a href="#roadmap">Roadmap</a></li>
    <li><a href="#contributing">Contributing</a></li>
    <li><a href="#license">License</a></li>
    <li><a href="#contact">Contact</a></li>
    <li><a href="#acknowledgements">Acknowledgements</a></li>
  </ol>
</details>

<!-- GETTING STARTED -->

## Getting Started

So, you want to program your new Raspberry Pi microcontroller, using the
Rust programming language. You've come to the right place!

This repository is `rp-hal` - a collection of high-level drivers for the
Raspberry Pi RP2040 and RP2350 microcontrollers.

If you want to write an application for the RP2040, check out our [RP2040
Project Template](https://github.com/rp-rs/rp2040-project-template). If you
want to use the RP2350 family, check out the [RP2350 Project
Template](https://github.com/rp-rs/rp235x-project-template) instead.

If you want to write code that uses the Raspberry Silicon PIO State Machines,
check out [pio-rs](https://github.com/rp-rs/pio-rs). You can even compile PIO
programs at run-time, on the MCU itself!

This repository only includes examples suitable for the Raspberry Pi Pico and
Raspberry Pi Pico 2 boards. We do also have some [*Board Support
Packages*][BSPs] which you can refer to, although note, you can also just
start with the bare HAL if you prefer (or if there is no BSP for your board
yet).

Before trying any of the examples, please ensure you have the latest stable
version of Rust installed, along with the right target support:

```sh
rustup self update
rustup update stable
rustup target add thumbv6m-none-eabi
rustup target add thumbv8m.main-none-eabihf
rustup target add riscv32imac-unknown-none-elf
```

You may also want to install probe-rs, to flash your device over the SWD pins
with a debug probe:

```sh
cargo install --locked probe-rs-tools
```

And also [picotool], if you are not using a debug probe or wish to create UF2
images for the USB bootloader. You can download a [pre-built picotool
binary][picotool-releases] for your system.

[picotool]: https://github.com/raspberrypi/picotool
[picotool-releases]: https://github.com/raspberrypi/pico-sdk-tools/releases

## Packages

There is one _Hardware Abstraction Layer_ (or HAL) crate for the RP2040, and
another for the RP2350. We also have a common HAL, and various examples.

### [rp2040-hal] - The HAL for the [Raspberry Pi RP2040]

You should include this crate in your project if you want to write a driver or
library that runs on the [Raspberry Pi RP2040], or if you are writing a Board
Support Package (see later on).

The crate provides high-level drivers for the RP2040's internal peripherals,
such as the SPI Controller and the I²C Controller. It doesn't know anything
about how your particular board is wired up (such as what each IO pin of the
RP2040 is connected to).

There are examples in this crate to show how to use various peripherals
(GPIO, I²C, SPI, UART, etc) but note that the pin-outs may not match any
particular board.

### [rp2040-hal-examples] - Examples for using [rp2040-hal]

This folder contains various examples for how to use the Rust HAL for the
RP2040. We have examples for the following (plus many more):

* GPIO
* I²C
* SPI
* UART
* Spawning tasks on Core 1
* Blinking an LED with PWM
* Using PIO
* Sleeping and waiting on the RTC

### [rp235x-hal] - The HAL for the [Raspberry Pi RP2350]

You should include this crate in your project if you want to write a driver or
library that runs on the [Raspberry Pi RP2350], or if you are writing a Board
Support Package (see later on). We call it the 'rp235x-hal' because it
supports the RP2350A, the RP2350B and variants which include on-board Flash
such as the RP2354A and RP2354B.

The crate provides high-level drivers for the RP2350's internal peripherals,
such as the SPI Controller and the I²C Controller. It doesn't know anything
about how your particular board is wired up (such as what each IO pin of the
RP2350 is connected to).

There are examples in this crate to show how to use various peripherals
(GPIO, I²C, SPI, UART, etc) but note that the pin-outs may not match any
particular board.

This HAL fully supports the RP2350A, and has partial support for the extra
pins on the RP2350B.

### [rp235x-hal-examples] - Examples for using [rp235x-hal]

This folder contains various examples for how to use the Rust HAL for the
RP2040. We have examples for the following (plus many more):

* GPIO
* I²C
* SPI
* UART
* Spawning tasks on Core 1
* Blinking an LED with PWM
* Using PIO
* Sleeping and waiting on the RTC
* Running in RISC-V mode

### [rp-hal-common] - Shared code for the two HALs

We're in the process of rationalising the two HALs into one (or, mostly into
one) and any common components will get moved here. You will likely never need
to use this library yourself.

### [rp-binary-info] - Library for generating `picotool` compatible metadata

Raspberry Pi's `picotool` program supports marking binaries with certain
metadata - the name of the program, the version number, and so on. We have
implemented support for this on the firmware side, so your Rust binaries can
also have this metadata appear in `picotool`.

### [BSPs] - Board support packages

There are BSPs for various boards based on the RP2040 available in a [separate
repository][BSPs]. They used to be in this repository, but were moved out
because there were so many of them.

[rp2040-hal]: https://github.com/rp-rs/rp-hal/tree/main/rp2040-hal
[rp2040-hal-examples]: https://github.com/rp-rs/rp-hal/tree/main/rp2040-hal-examples
[rp235x-hal]: https://github.com/rp-rs/rp-hal/tree/main/rp235x-hal
[rp235x-hal-examples]: https://github.com/rp-rs/rp-hal/tree/main/rp235x-hal-examples
[rp-binary-info]: https://github.com/rp-rs/rp-hal/tree/main/rp-binary-info
[rp-hal-common]: https://github.com/rp-rs/rp-hal/tree/main/rp-hal-common
[Raspberry Pi RP2040]: https://www.raspberrypi.org/products/rp2040/
[Raspberry Pi RP2350]: https://www.raspberrypi.org/products/rp2350/
[BSPs]: https://github.com/rp-rs/rp-hal-boards/

<!-- PROGRAMMING -->
## Programming

Rust generates standard Arm ELF files, which you can load onto your Raspberry Pi
Silicon device with your favourite Arm flashing/debugging tool. In addition, the
RP2040 contains a ROM bootloader which appears as a Mass Storage Device over USB
that accepts UF2 format images. You can use picotool to flash your device over
USB, or convert the Arm ELF file to a UF2 format image.

The RP2040 contains two Cortex-M0+ processors, which execute Thumb-2 encoded
Armv6-M instructions. There are no operating-specific features in the binaries
produced - they are for 'bare-metal' systems. For compatibility with other Arm
code (e.g. as produced by GCC), Rust uses the *Arm Embedded-Application Binary
Interface* standard or EABI. Therefore, any Rust code for the RP2040 should be
compiled with the target *`thumbv6m-none-eabi`*.

The RP2350 contains two Cortex-M33 processors (each with a single-precision
FPU), which execute Thumb-2 encoded Armv8-M Mainline instructions. There are
no operating-specific features in the binaries produced - they are for
'bare-metal' systems. For compatibility with other Arm code (e.g. as produced
by GCC), Rust uses the *Arm Embedded-Application Binary Interface* standard or
EABI. Therefore, any Rust code for the RP2350 should be compiled with the
target *`thumbv8m.main-none-eabihf`*.

Each core in the RP2350 can optionally be swapped out at run-time for a RISC-V
Hazard3 processor core. Any Rust code for the RP2350 in RISC-V mode should be
compiled with the target *`riscv32imac-unknown-none-elf`*.

More details can be found in the [RP2040 Project Template] and the [RP2350
Project Template].

### Linker flags

Besides the correct target, which mainly defines the instruction set, it's
also necessary to use a certain memory layout compatible with your MCU. To
achieve that, rustc must be called with appropriate linker flags. In the
[RP2040 Project Template], those flags are defined in
[`.cargo/config.toml`](https://github.com/rp-rs/rp2040-project-template/blob/main/.cargo/config.toml).
You must also provide a file called
[`memory.x`](https://github.com/rp-rs/rp2040-project-template/blob/main/memory.x),
which is required by the [`cortex-m-rt`](https://crates.io/crates/cortex-m-rt)
start-up library we use.

More detailed information on how the linker flags work can be found in
[the `cortex-m-rt` docs](https://docs.rs/cortex-m-rt/latest/cortex_m_rt/).

In most cases, it should be sufficient to use the example files from the
[RP2040 Project Template] or [RP2350 Project Template].

### Loading over USB with picotool

*Step 1* - Install a [picotool binary][picotool-releases] for your system.

*Step 2* - Make sure your .cargo/config contains the following (it should by
default if you are working in this repository):

```toml
[target.thumbv6m-none-eabi]
runner = "picotool load --update --verify --execute -t elf"
```

The `thumbv6m-none-eabi` target may be replaced by the all-Arm wildcard
`'cfg(all(target_arch = "arm", target_os = "none"))'`.

*Step 3* - Boot your RP2040 into "USB Bootloader mode", typically by rebooting
whilst holding some kind of "Boot Select" button. On Linux, you will also need
to 'mount' the device, like you would a USB Thumb Drive.

*Step 4* - Use `cargo run`, which will compile the code and start the
specified 'runner'. As the 'runner' is picotool, it will flash your compiled
binary over USB.

```console
$ cargo run --release --features "critical-section-impl,rt,defmt" --example pwm_blink
```

(The `pwm_blink` example doesn't need all these feature flags. They are listed here
so you can use the same command for all examples.)

If you want to create a UF2 file, which is loaded by copying it over to the
RPI-RP2 mass storage device, use the `picotool uf2 convert` command on your
compiled program with the `-t elf` argument.

```console
$ picotool uf2 convert -t elf target/thumbv6m-none-eabi/release/pwm_blink
pwm_blink.uf2
```

Picotool can also read "Binary Info" from a device with `picotool info`. To
enable this in your firmware, see the [rp-binary-info] crate and the
corresponding [binary info example].

[binary info example]: https://github.com/rp-rs/rp-hal/blob/main/rp2040-hal-examples/src/bin/binary_info_demo.rs

### Loading with probe-rs

[probe-rs](https://github.com/probe-rs/probe-rs) is a library and a
command-line tool which can flash a wide variety of microcontrollers
using a wide variety of debug/JTAG probes. Unlike using, say, OpenOCD,
probe-rs can autodetect your debug probe, which can make it easier to use.

*Step 1* - Install `probe-rs`:

```console
$ cargo install --locked probe-rs-tools
```

Alternatively, follow the installation instructions on https://probe.rs/.

*Step 2* - Make sure your .cargo/config contains the following:

```toml
[target.thumbv6m-none-eabi]
runner = "probe-rs run --chip RP2040"
```

*Step 3* - Connect your USB JTAG/debug probe (such as a Raspberry Pi Pico
running [this firmware](https://github.com/majbthrd/DapperMime)) to the SWD
programming pins on your RP2040 board. Check the probe has been found by
running:

```console
$ probe-rs list
The following debug probes were found:
[0]: J-Link (J-Link) (VID: 1366, PID: 0101, Serial: 000099999999, JLink)
```

There is a SEGGER J-Link connected in the example above - the message you see
will reflect the probe you have connected.

*Step 4* - Use `cargo run`, which will compile the code and start the specified
'runner'. As the 'runner' is the `probe-rs` tool, it will connect to the
RP2040 via the first probe it finds, and install your firmware into the Flash
connected to the RP2040.

```console
$ cargo run --release --example pwm_blink
```
[RP2040 Project Template]: https://github.com/rp-rs/rp2040-project-template
[RP2350 Project Template]: https://github.com/rp-rs/rp235x-project-template

<!-- ROADMAP -->
## Roadmap

NOTE These packages are under active development. As such, it is likely to
remain volatile until a 1.0.0 release.

See the [open issues](https://github.com/rp-rs/rp-hal/issues) for a list of
proposed features (and known issues).

<!-- CONTRIBUTING -->
## Contributing

Contributions are what make the open source community such an amazing place to be learn, inspire, and create. Any contributions you make are **greatly appreciated**.

The steps are:

1. Fork the Project by clicking the 'Fork' button at the top of the page.
2. Create your Feature Branch (`git checkout -b feature/AmazingFeature`)
3. Make some changes to the code or documentation.
4. Commit your Changes (`git commit -m 'Add some AmazingFeature'`)
5. Push to the Feature Branch (`git push origin feature/AmazingFeature`)
6. Create a [New Pull Request](https://github.com/rp-rs/rp-hal/pulls)
7. An admin will review the Pull Request and discuss any changes that may be required.
8. Once everyone is happy, the Pull Request can be merged by an admin, and your work is part of our project!

<!-- CODE OF CONDUCT -->
## Code of Conduct

Contribution to this crate is organized under the terms of the [Rust Code of
Conduct][CoC], and the maintainer of this crate, the [rp-rs team], promises
to intervene to uphold that code of conduct.

[CoC]: CODE_OF_CONDUCT.md
[rp-rs team]: https://github.com/orgs/rp-rs/people

<!-- LICENSE -->
## License

The contents of this repository are dual-licensed under the _MIT OR Apache 2.0_
License. That means you can choose either the MIT license or the Apache 2.0
license when you re-use this code. See [`LICENSE-MIT`](./LICENSE-MIT) or
[`LICENSE-APACHE`](./LICENSE-APACHE) for more information on each specific
license. Our Apache 2.0 notices can be found in [`NOTICE`](./NOTICE).

Unless you explicitly state otherwise, any contribution intentionally submitted
for inclusion in the work by you, as defined in the Apache-2.0 license, shall be
dual licensed as above, without any additional terms or conditions.

<!-- CONTACT -->
## Contact

Raise an issue: [https://github.com/rp-rs/rp-hal/issues](https://github.com/rp-rs/rp-hal/issues)
Chat to us on Matrix: [#rp-rs:matrix.org](https://matrix.to/#/#rp-rs:matrix.org)

<!-- ACKNOWLEDGEMENTS -->
## Acknowledgements

* [Othneil Drew's README template](https://github.com/othneildrew)
* [Rust Embedded Working Group](https://github.com/rust-embedded)
* [Raspberry Pi](https://raspberrypi.org) and the [Pico SDK](https://github.com/raspberrypi/pico-sdk)
