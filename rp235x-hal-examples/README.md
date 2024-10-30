<!-- PROJECT LOGO -->
<br />
<p align="center">
  <a href="https://github.com/rp-rs/rp-hal">
    <img src="https://www.svgrepo.com/show/281119/microchip.svg" alt="Logo" width="140" height="140">
  </a>

   <h3 align="center">rp-hal</h3>

  <p align="center">
    Rust Examples for the Raspberry Silicon RP235x family Microcontrollers
    <br />
    <a href="https://github.com/rp-rs/rp-hal/tree/main/rp235x-hal-examples"><strong>View the examples  »</strong></a>
    <br />
    <br />
    <a href="https://docs.rs/rp235x-hal">Explore the API docs</a>
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
    <li><a href="#introduction">Introduction</a></li>
    <li><a href="#getting-started">Getting Started</a></li>
    <li><a href="#roadmap">Roadmap</a></li>
    <li><a href="#contributing">Contributing</a></li>
    <li><a href="#license">License</a></li>
    <li><a href="#contact">Contact</a></li>
    <li><a href="#acknowledgements">Acknowledgements</a></li>
  </ol>
</details>

<!-- INTRODUCTION -->
## Introduction

The `rp235x-hal` package is a library crate of high-level Rust drivers for the
Raspberry Silicon RP235x family of microcontrollers. This folder contains a
collection of non-board specific example programs for you to study.

We also provide a series of [*Board Support Package* (BSP) crates][BSPs], which
take the HAL crate and pre-configure the pins according to a specific PCB
design. If you are using one of the supported boards, you should use one of
those crates in preference, and return here to see documentation about specific
peripherals on the RP235x and how to use them. See the `boards` folder in
https://github.com/rp-rs/rp-hal-boards/ for more details.

[BSPs]: https://github.com/rp-rs/rp-hal-boards/

<!-- GETTING STARTED -->
## Getting Started

To build the examples, first grab a copy of the source code:

```console
$ git clone https://github.com/rp-rs/rp-hal.git
```

Then use `rustup` to grab the Rust Standard Library for the appropriate targets.
RP2350 has two possible targets: `thumbv8m.main-none-eabihf` for the Arm mode, and
`riscv32imac-unknown-none-elf` for the RISC-V mode.

```console
$ rustup target add thumbv8m.main-none-eabihf
$ rustup target add riscv32imac-unknown-none-elf
```

**Note: all examples assume the current directory is `<repo root>/rp235x-hal-examples`.**
```
cd rp235x-hal-examples
```

The most basic method is to use `cargo build` with the `--bin` flag to specify the example you want to
build. For example, to build the `blinky` example:

```console
$ cargo build --bin blinky
   Compiling proc-macro2 v1.0.89
   Compiling unicode-ident v1.0.13
   Compiling syn v1.0.109
...
   Compiling pio-parser v0.2.2
   Compiling rp235x-hal v0.2.0
   Compiling pio-proc v0.2.2
    Finished `dev` profile [unoptimized + debuginfo] target(s) in 14.97s
```

This builds the default target, which is Arm mode and the ELF file
is located at `./target/thumbv8m.main-none-eabihf/debug/blinky`:

```console
$ file ./target/thumbv8m.main-none-eabihf/debug/blinky
./target/thumbv8m.main-none-eabihf/debug/blinky: ELF 32-bit LSB executable, ARM, EABI5 version 1 (SYSV), statically linked, with debug_info, not stripped
```

If you want to build a binary that runs in RISC-V mode, then you must specify the RISC-V target to override the default:

```console
$ cargo build --target=riscv32imac-unknown-none-elf --bin blinky
   Compiling nb v1.1.0
   Compiling byteorder v1.5.0
   Compiling stable_deref_trait v1.2.0
..
   Compiling futures v0.3.31
   Compiling frunk v0.4.3
   Compiling rp235x-hal v0.2.0
    Finished `dev` profile [unoptimized + debuginfo] target(s) in 6.23s```
```

And we see that the RISC-V mode ELF file is now present at `./target/riscv32imac-unknown-none-elf/debug/blinky`:

```console
$ file ./target/riscv32imac-unknown-none-elf/debug/blinky
./target/riscv32imac-unknown-none-elf/debug/blinky: ELF 32-bit LSB executable, UCB RISC-V, RVC, soft-float ABI, version 1 (SYSV), statically linked, with debug_info, not stripped
```

You can also specify the Arm mode target directly by using
`--target thumbv8m.main-none-eabihf` instead of `--target riscv32imac-unknown-none-elf`.

To build, flash and start the application on the RP235x
you use `cargo run` with one of the following commands. Note: be sure the RP235x is in BOOTSEL mode before using the `run` command because we use `picotool` to flash and run the binary:

```console
$ cargo run --target thumbv8m.main-none-eabihf --bin blinky
$ cargo run --target riscv32imac-unknown-none-elf --bin blinky
```

For the release profile build pass `--release` to the `cargo build`
or `cargo run` commands. This will build the example with optimizations enabled:

```console
$ cargo run --target thumbv8m.main-none-eabihf --release --bin blinky
$ cargo run --target riscv32imac-unknown-none-elf --release --bin blinky
```

For the Arm mode target all of the examples are built if no `--bin` is specified:

```console
$ cargo clean
     Removed 1488 files, 398.6MiB total
$ cargo build --target thumbv8m.main-none-eabihf
   Compiling proc-macro2 v1.0.89
   Compiling unicode-ident v1.0.13
..
   Compiling rp235x-hal v0.2.0
   Compiling pio-proc v0.2.2
    Finished `dev` profile [unoptimized + debuginfo] target(s) in 16.08s
$ find target/thumbv8m.main-none-eabihf/debug/ -maxdepth 1 -type f -executable | sort
target/thumbv8m.main-none-eabihf/debug/adc
target/thumbv8m.main-none-eabihf/debug/adc_fifo_dma
..
target/thumbv8m.main-none-eabihf/debug/blinky
..
target/thumbv8m.main-none-eabihf/debug/vector_table
target/thumbv8m.main-none-eabihf/debug/watchdog
```

For the RISC-V mode it is currently possible to build only *some* of the examples. See
[`riscv_examples.txt`](./riscv_examples.txt) for a list of known working examples.
The missing ones probably rely on interrupts, or some other thing we
haven't ported to work in RISC-V mode yet.

Here is an example using the `blinky` example in RISC-V mode. We'll build and
run it first using the default dev profile, emulating the development cycle **Note:** be sure
the RP235x is in BOOTSEL mode before using the `run` command because we use `picotool` to flash and run the binary:

```console
$ cargo build --bin blinky --target=riscv32imac-unknown-none-elf
   Compiling rp235x-hal-examples v0.1.0
    Finished `dev` profile [unoptimized + debuginfo] target(s) in 0.15s
$ cargo run --bin blinky --target=riscv32imac-unknown-none-elf
    Finished `dev` profile [unoptimized + debuginfo] target(s) in 0.07s
     Running `picotool load -u -v -x -t elf target/riscv32imac-unknown-none-elf/debug/blinky`
Family id 'rp2350-riscv' can be downloaded in absolute space:
  00000000->02000000
Loading into Flash: [==============================]  100%
Verifying Flash:    [==============================]  100%
  OK

The device was rebooted to start the application.
```

At this point the development version is running on the RP2350 and the LED is blinking.
When we look at `blinky` using `file` we see we have generated a RISC-V mode ELF file:

```console
$ file ./target/riscv32imac-unknown-none-elf/debug/blinky
./target/riscv32imac-unknown-none-elf/debug/blinky: ELF 32-bit LSB executable, UCB RISC-V, RVC, soft-float ABI, version 1 (SYSV), statically linked, with debug_info, not stripped
```

Next we'll build and run it using the release profile for "final" testing,
again be sure the RP235x is in BOOTSEL mode:

```console
$ cargo run --bin blinky --target=riscv32imac-unknown-none-elf --release
   Compiling proc-macro2 v1.0.89
   Compiling unicode-ident v1.0.13
..
   Compiling pio-parser v0.2.2
   Compiling pio-proc v0.2.2
    Finished `release` profile [optimized] target(s) in 17.05s
     Running `picotool load -u -v -x -t elf target/riscv32imac-unknown-none-elf/release/blinky`
Family id 'rp2350-riscv' can be downloaded in absolute space:
  00000000->02000000
Loading into Flash: [==============================]  100%
Verifying Flash:    [==============================]  100%
  OK

The device was rebooted to start the application.
```

The LED should be blinking as we are now running this binary:

```console
$ file ./target/riscv32imac-unknown-none-elf/release/blinky
./target/riscv32imac-unknown-none-elf/release/blinky: ELF 32-bit LSB executable, UCB RISC-V, RVC, soft-float ABI, version 1 (SYSV), statically linked, not stripped
```

The above commands work well, but the commands are somewhat verbose.
To make building and running commands more succinct an `[alias]` section
has been added to [.cargo/config.toml](../.cargo/config.toml) that define:
| Command Alias   | Description |
|---|---|
| build-arm   | build for ARM |
| build-riscv | build for RISC-V |
| run-arm   | run on ARM |
| run-riscv | run on RISC-V |
| rrr-blinky | run release on RISC-V blinky |

When using these aliases your build and run commands are much shorter.
Below we see the development cycle using `build-riscv` and `run-riscv`:

```console
$ cargo build-riscv --bin blinky
    Finished `dev` profile [unoptimized + debuginfo] target(s) in 0.05s
$ cargo run-riscv --bin blinky
    Finished `dev` profile [unoptimized + debuginfo] target(s) in 0.05s
     Running `picotool load -u -v -x -t elf target/riscv32imac-unknown-none-elf/debug/blinky`
Family id 'rp2350-riscv' can be downloaded in absolute space:
  00000000->02000000
Loading into Flash: [==============================]  100%
Verifying Flash:    [==============================]  100%
  OK

The device was rebooted to start the application.
```

And for the `run` command in `--release` profile and a RISC-V mode we added the `rrr-blinky` alias
as an example of customization. You might want to add others as you see fit:

```console
$ cargo rrr-blinky
    Finished `release` profile [optimized] target(s) in 0.05s
     Running `picotool load -u -v -x -t elf target/riscv32imac-unknown-none-elf/release/blinky`
Family id 'rp2350-riscv' can be downloaded in absolute space:
  00000000->02000000
Loading into Flash: [==============================]  100%
Verifying Flash:    [==============================]  100%
  OK

The device was rebooted to start the application.
```

<!-- ROADMAP -->
## Roadmap

NOTE The HAL is under active development, and so are these examples. As such, it
is likely to remain volatile until a 1.0.0 release.

See the [open issues](https://github.com/rp-rs/rp-hal/issues) for a list of
proposed features (and known issues).

<!-- CONTRIBUTING -->
## Contributing

Contributions are what make the open source community such an amazing place to
be learn, inspire, and create. Any contributions you make are **greatly
appreciated**.

1. Fork the Project
2. Create your Feature Branch (`git checkout -b feature/AmazingFeature`)
3. Commit your Changes (`git commit -m 'Add some AmazingFeature'`)
4. Push to the Branch (`git push origin feature/AmazingFeature`)
5. Open a Pull Request


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

* Project Link: [https://github.com/rp-rs/rp-hal/issues](https://github.com/rp-rs/rp-hal/issues)
* Matrix: [#rp-rs:matrix.org](https://matrix.to/#/#rp-rs:matrix.org)

<!-- ACKNOWLEDGEMENTS -->
## Acknowledgements

* [Othneil Drew's README template](https://github.com/othneildrew)
