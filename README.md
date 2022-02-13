<!-- PROJECT LOGO -->
<br />
<p align="center">
  <a href="https://github.com/rp-rs/rp2040-hal">
    <img src="https://www.svgrepo.com/show/281119/microchip.svg" alt="Logo" width="140" height="140">
  </a>

   <h3 align="center">rp-hal</h3>

  <p align="center">
    Rust support for the "Raspberry Silicon" family of microcontrollers
    <br />
    <a href="https://docs.rs/rp2040-hal"><strong>Explore the API docs »</strong></a>
    <br />
    <br />
    <a href="https://github.com/rp-rs/rp-hal/tree/main/boards/rp-pico/examples">View Demos</a>
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
    <li><a href="#gettting_started">Getting Started</a></li>
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

So, you want to program your new Raspberry Silicon microcontroller, using the
Rust programming language. You've come to the right place!

This repository is `rp-hal` - a collection of high-level drivers for the
Raspberry Silicon RP2040 microcontroller and various associated boards, like
the Raspberry Pi Pico and the Adafruit Feather RP2040.

If you want to write an application for Raspberry Silicon, check out our
[RP2040 Project Template](https://github.com/rp-rs/rp2040-project-template).

If you want to write code that uses the Raspberry Silicon PIO State Machines,
check out [pio-rs](https://github.com/rp-rs/pio-rs). You can even compile PIO
programs at run-time, on the RP2040 itself!

If you want to try out some examples on one of our supported boards, check out
the list of *Board Support Packages* below, and click through to see the various
examples for each board.

Before trying any of the examples, please ensure you have the latest stable
version of Rust installed, along with the right target support:

```sh
rustup self update
rustup update stable
rustup target add thumbv6m-none-eabi
```

You may also want to install these helpful tools:

```sh
# Useful to creating UF2 images for the RP2040 USB Bootloader
cargo install elf2uf2-rs --locked
# Useful for flashing over the SWD pins using a supported JTAG probe
cargo install probe-run
```

## Packages

This git repository is organised as a [Cargo Workspace].

There is a _Hardware Abstraction Layer_ (or HAL) crate for the RP2040 chip,
and _Board Support Package_ crates for a number of RP2040 based PCBs. If you
are writing code that should run on any microcontroller, consider using the
generic Rust Embedded Working Group's [Embedded HAL].

If you are writing code that should work on any RP2040 device, use the _HAL_
crate. If you are running code on a specific board, use the appropriate _BSP_
crate (which will include the _HAL_ crate for you). Please note, you cannot
depend on multiple _BSP_ crates; you have to pick one, or use [Cargo Features]
to select one at build time.

Each BSP includes some examples to show off the features of that particular board.

[Cargo Workspace]: https://doc.rust-lang.org/cargo/reference/workspaces.html
[Embedded HAL]: https://github.com/rust-embedded/embedded-hal
[Cargo Features]: https://doc.rust-lang.org/cargo/reference/features.html

### [rp2040-hal] - The HAL for the [Raspberry Silicon RP2040]

You should include this crate in your project if you want to write a driver or
library that runs on the [Raspberry Silicon RP2040], or if you are writing a Board
Support Package (see later on).

The crate provides high-level drivers for the RP2040's internal peripherals,
such as the SPI Controller and the I²C Controller. It doesn't know anything
about how your particular board is wired up (such as what each IO pin of the
RP2040 is connected to).

There are examples in this crate to show how to use various peripherals
(GPIO, I²C, SPI, UART, etc) but note that the pin-outs may not match any
particular board.

[rp2040-hal]: https://github.com/rp-rs/rp-hal/tree/main/rp2040-hal
[Raspberry Silicon RP2040]: https://www.raspberrypi.org/products/rp2040/

### [rp-pico] - Board Support for the [Raspberry Pi Pico]

You should include this crate if you are writing code that you want to run on
a [Raspberry Pi Pico] - the original launch PCB for the RP2040 chip.  

This crate includes the [rp2040-hal], but also configures each pin of the
RP2040 chip according to how it is connected up on the Pico.

[Raspberry Pi Pico]: https://www.raspberrypi.org/products/raspberry-pi-pico/
[rp-pico]: https://github.com/rp-rs/rp-hal/tree/main/boards/rp-pico

### [adafruit-feather-rp2040] - Board Support for the [Adafruit Feather RP2040]

You should include this crate if you are writing code that you want to run on
an [Adafruit Feather RP2040] - a Feather form-factor RP2040 board from Adafruit.

This crate includes the [rp2040-hal], but also configures each pin of the
RP2040 chip according to how it is connected up on the Feather RP2040.

[Adafruit Feather RP2040]: https://www.adafruit.com/product/4884
[adafruit-feather-rp2040]: https://github.com/rp-rs/rp-hal/tree/main/boards/adafruit-feather-rp2040

### [adafruit-itsy-bitsy-rp2040] - Board Support for the [Adafruit ItsyBitsy RP2040]
You should include this crate if you are writing code that you want to run on
an [Adafruit ItsyBitsy RP2040] - an RP2040 board in the ItsyBitsy family.

This crate includes the [rp2040-hal], but also configures each pin of the
RP2040 chip according to how it is connected up on the ItsyBitsy RP2040.

[Adafruit ItsyBitsy RP2040]: https://www.adafruit.com/product/4888
[adafruit-itsy-bitsy-rp2040]: https://github.com/rp-rs/rp-hal/tree/main/boards/adafruit-itsy-bitsy-rp2040

### [adafruit-kb2040] - Board Support for the [Adafruit KB2040]
You should include this crate if you are writing code that you want to run on
an [Adafruit KB2040] - an Arduino Pro Micro-shaped board for keyboards.

This crate includes the [rp2040-hal], but also configures each pin of the
RP2040 chip according to how it is connected up on the KB2040.

[Adafruit KB2040]: https://www.adafruit.com/product/5302
[adafruit-kb2040]: https://github.com/rp-rs/rp-hal/tree/main/boards/adafruit-kb2040

### [adafruit-macropad] - Board Support for the [Adafruit Macropad]

You should include this crate if you are writing code that you want to run on
an [Adafruit Macropad] - a 3x4 keyboard and OLED combo board from Adafruit.

This crate includes the [rp2040-hal], but also configures each pin of the
RP2040 chip according to how it is connected up on the Macropad.

[adafruit-macropad]: https://github.com/rp-rs/rp-hal/tree/main/boards/adafruit-macropad
[Adafruit Macropad]: https://www.adafruit.com/product/5128

### [adafruit-qt-py-rp2040] - Board Support for the [Adafruit QT Py RP2040]

You should include this crate if you are writing code that you want to run on
an [Adafruit QT Py RP2040] - an extremely small form-factor RP2040 board from Adafruit.

This crate includes the [rp2040-hal], but also configures each pin of the
RP2040 chip according to how it is connected up on the Feather RP2040.

[Adafruit QT Py RP2040]: https://www.adafruit.com/product/4900
[adafruit-qt-py-rp2040]: https://github.com/rp-rs/rp-hal/tree/main/boards/adafruit-qt-py-rp2040

### [adafruit-trinkey-qt2040] - Board Support for the [Adafruit Trinkey QT2040]

You should include this crate if you are writing code that you want to run on
an [Adafruit Trinkey QT2040] - a 3x4 keyboard and OLED combo board from Adafruit.

This crate includes the [rp2040-hal], but also configures each pin of the
RP2040 chip according to how it is connected up on the Trinkey.

[Adafruit-Trinkey-QT2040]: https://github.com/rp-rs/rp-hal/tree/main/boards/adafruit-trinkey-qt2040
[adafruit trinkey qt2040]: https://www.adafruit.com/product/5056

### [pimoroni-pico-explorer] - Board Support for the [Pimoroni Pico Explorer]

You should include this crate if you are writing code that you want to run on
a [Pimoroni Pico Explorer] - a board featuring a small LCD screen, a
breadboard and some breakout headers.

This crate includes the [rp2040-hal], but also configures each pin of the
RP2040 chip according to how it is connected up on the Pico Explorer.

[Pimoroni Pico Explorer]: https://shop.pimoroni.com/products/pimoroni-pico-explorer-base
[pimoroni-pico-explorer]: https://github.com/rp-rs/rp-hal/tree/main/boards/pimoroni-pico-explorer

### [pimoroni-pico-lipo-16mb] - Board Support for the [Pimoroni Pico Lipo 16MB]

You should include this crate if you are writing code that you want to run on
a [Pimoroni Pico Lipo 16MB] - a board with USB-C, STEMMA QT/Qwiic connectors,
plus a Li-Po battery charging circuit.

This crate includes the [rp2040-hal], but also configures each pin of the
RP2040 chip according to how it is connected up on the Pico Lipo.

Note that if you use this crate the compiler will expect the full 16MB flash
space, and so it may not work if you only have the 4MB variant. 

[Pimoroni Pico Lipo 16MB]: https://shop.pimoroni.com/products/pimoroni-pico-lipo?variant=39335427080275
[pimoroni-pico-lipo-16mb]: https://github.com/rp-rs/rp-hal/tree/main/boards/pimoroni-pico-lipo-16mb

### [solderparty-rp2040-stamp] - Board Support for the [SolderParty RP2040 Stamp]

You should include this crate if you are writing code that you want to run on
a [SolderParty RP2040 Stamp] - a square RP2040 board with castellated edges.

This crate includes the [rp2040-hal], but also configures each pin of the
RP2040 chip according to how it is connected up on the Stamp.

[SolderParty RP2040 Stamp]: https://www.solder.party/docs/rp2040-stamp/
[solderparty-rp2040-stamp]: https://github.com/rp-rs/rp-hal/tree/main/boards/solderparty-rp2040-stamp

### [sparkfun-pro-micro-rp2040] - Board Support for the [Sparkfun Pro Micro RP2040]

You should include this crate if you are writing code that you want to run on
a [Sparkfun Pro Micro RP2040] - a smaller RP2040 board with USB-C and a WS2812B addressable LED.

This crate includes the [rp2040-hal], but also configures each pin of the
RP2040 chip according to how it is connected up on the Pro Micro RP2040.

[Sparkfun Pro Micro RP2040]: https://www.sparkfun.com/products/18288
[sparkfun-pro-micro-rp2040]: https://github.com/rp-rs/rp-hal/tree/main/boards/sparkfun-pro-micro-rp2040

<!-- PROGRAMMING -->
## Programming

Rust generates standard Arm ELF files, which you can load onto your Raspberry Pi
Silicon device with your favourite Arm flashing/debugging tool. In addition, the
RP2040 contains a ROM bootloader which appears as a Mass Storage Device over USB
that accepts UF2 format images. You can use the `elf2uf2-rs` package to convert
the Arm ELF file to a UF2 format image.

For boards with USB Device support like the Raspberry Pi Pico, we recommend you
use the UF2 process.

The RP2040 contains two Cortex-M0+ processors, which execute Thumb-2 encoded
ARMv6-M instructions. There are no operating-specific features in the binaries
produced - they are for 'bare-metal' systems. For compatibilty with other Arm
code (e.g. as produced by GCC), Rust uses the *Arm Embedded-Application Binary
Interface* standard or EABI. Therefore, any Rust code for the RP2040 should be
compiled with the target *`thumbv6m-none-eabi`*.

More details can be found in the [Project Template](https://github.com/rp-rs/rp2040-project-template).

### Loading a UF2 over USB

*Step 1* - Install [`elf2uf2-rs`](https://github.com/JoNil/elf2uf2-rs):

```console
$ cargo install elf2uf2-rs --locked
```

*Step 2* - Make sure your .cargo/config contains the following (it should by
default if you are working in this repository):

```toml
[target.thumbv6m-none-eabi]
runner = "elf2uf2-rs -d"
```

The `thumbv6m-none-eabi` target may be replaced by the all-Arm wildcard
`'cfg(all(target_arch = "arm", target_os = "none"))'`.

*Step 3* - Boot your RP2040 into "USB Bootloader mode", typically by rebooting
whilst holding some kind of "Boot Select" button. On Linux, you will also need
to 'mount' the device, like you would a USB Thumb Drive.

*Step 4* - Use `cargo run`, which will compile the code and started the
specified 'runner'. As the 'runner' is the elf2uf2-rs tool, it will build a UF2
file and copy it to your RP2040.

```console
$ cargo run --release --example pico_pwm_blink
```

### Loading with probe-run

The Knurling project has a tool called
[probe-run](https://github.com/knurling-rs/probe-run). This is a command-line
tool which can flash a wide variety of microcontrollers using a wide variety of
debug/JTAG probes. It is based on a library called
[probe-rs](https://github.com/probe-rs/probe-rs). Unlike using, say, OpenOCD,
probe-rs can autodetect your debug probe, which can make it easier to use.

*Step 1* - Install `probe-run`:

```console
$ cargo install probe-run
```

*Step 2* - Make sure your .cargo/config contains the following:

```toml
[target.thumbv6m-none-eabi]
runner = "probe-run --chip RP2040"
```

*Step 3* - Connect your USB JTAG/debug probe (such as a Raspberry Pi Pico
running [this firmware](https://github.com/majbthrd/DapperMime)) to the SWD
programming pins on your RP2040 board. Check the probe has been found by
running:

```console
$ probe-run --chip RP2040 --list-probes
The following devices were found:
[0]: J-Link (J-Link) (VID: 1366, PID: 0101, Serial: 000099999999, JLink)
```

There is a SEGGER J-Link connected in the example above - the mesage you see
will reflect the probe you have connected.

*Step 4* - Use `cargo run`, which will compile the code and start the specified
'runner'. As the 'runner' is the `probe-run` tool, it will connect to the
RP2040 via the first probe it finds, and install your firmware into the Flash
connected to the RP2040.

```console
$ cargo run --release --example pico_pwm_blink
```

### Loading with picotool

As ELF files produced by compiling Rust code are completely compatible with ELF
files produced by compiling C or C++ code, you can also use the Raspberry Pi
tool [picotool](https://github.com/raspberrypi/picotool). The only thing to be
aware of is that picotool expects your ELF files to have a `.elf` extension, and
by default Rust does not give the ELF files any extension. You can fix this by
simply renaming the file.

Also of note is that the special
[pico-sdk](https://github.com/raspberrypi/pico-sdk) macros which hide
information in the ELF file in a way that `picotool info` can read it out, are
not supported in Rust. An alternative is TBC.

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
[rp-rs team]: https://github.com/orgs/rp-rs/teams/rp-rs

<!-- LICENSE -->
## License

The contents of this repository are dual-licensed under the _MIT OR Apache
2.0_ License. That means you can chose either the MIT licence or the
Apache-2.0 licence when you re-use this code. See `MIT` or `APACHE2.0` for more
information on each specific licence.

Any submissions to this project (e.g. as Pull Requests) must be made available
under these terms.

<!-- CONTACT -->
## Contact

Raise an issue: [https://github.com/rp-rs/rp-hal/issues](https://github.com/rp-rs/rp-hal/issues)
Chat to us on Matrix: [#rp-rs:matrix.org](https://matrix.to/#/#rp-rs:matrix.org)

<!-- ACKNOWLEDGEMENTS -->
## Acknowledgements

* [Othneil Drew's README template](https://github.com/othneildrew)
* [Rust Embedded Working Group](https://github.com/rust-embedded)
* [Raspberry Pi](https://raspberrypi.org) and the [Pico SDK](https://github.com/raspberrypi/pico-sdk)
