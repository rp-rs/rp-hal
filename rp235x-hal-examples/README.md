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

Then use `rustup` to grab the Rust Standard Library for the appropriate targets. the RP2350
has two possible targets, thumbv8m.main-none-eabihf for the ARM cpu. And riscv32imac-unknown-none-elf
for the RISC-V cpu.

```console
$ rustup target add thumbv8m.main-none-eabihf
$ rustup target add riscv32imac-unknown-none-elf
```

**Note: all examples assume the current directory is <repo root>/rp235x-hal-examples.**
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
   Compiling rp235x-hal v0.2.0 (/home/wink/prgs/rpi-pico/myrepos/rp-hal-clone/rp235x-hal)
   Compiling pio-proc v0.2.2
    Finished `dev` profile [unoptimized + debuginfo] target(s) in 14.97s
```

This builds the default target, which is the for the ARM at `./target/thumbv8m.main-none-eabihf/debug/blinky`:
```console
$ file ./target/thumbv8m.main-none-eabihf/debug/blinky
./target/thumbv8m.main-none-eabihf/debug/blinky: ELF 32-bit LSB executable, ARM, EABI5 version 1 (SYSV), statically linked, with debug_info, not stripped
```

If you want to build the RISC-V you can specify the target directly to override the default:
```console
$ cargo build --target=riscv32imac-unknown-none-elf --bin blinky
   Compiling nb v1.1.0
   Compiling byteorder v1.5.0
   Compiling stable_deref_trait v1.2.0
..
   Compiling futures v0.3.31
   Compiling frunk v0.4.3
   Compiling rp235x-hal v0.2.0 (/home/wink/prgs/rpi-pico/myrepos/rp-hal-clone/rp235x-hal)
    Finished `dev` profile [unoptimized + debuginfo] target(s) in 6.23s```
```

And we see the RISC-V at `./target/riscv32imac-unknown-none-elf/debug/blinky`:
```console
$ file ./target/riscv32imac-unknown-none-elf/debug/blinky
./target/riscv32imac-unknown-none-elf/debug/blinky: ELF 32-bit LSB executable, UCB RISC-V, RVC, soft-float ABI, version 1 (SYSV), statically linked, with debug_info, not stripped
```

You can also specify the ARM target directly just pass
`--target thumbv8m.main-none-eabihf` instead of `--target riscv32imac-unknown-none-elf`.

You can also easily build, flash and start the application on the RP235x
by using the `cargo run` using command:
```console
$ cargo run --target thumbv8m.main-none-eabihf --bin blinky
$ cargo run --target riscv32imac-unknown-none-elf --bin blinky
```

You can also specify a release build by passing `--release` to the `cargo build`
or `cargo run` commands. This will build the example with optimizations enabled:
```console
$ cargo run --target thumbv8m.main-none-eabihf --release --bin blinky
$ cargo run --target riscv32imac-unknown-none-elf --release --bin blinky
```

The above work well but the commands are somewhat verbose. To make building and running
more succinct we have added some aliases in .carog/config.toml:

```toml
# Add aliases for building and running for the ARM and RISC-V targets.
[alias]

# Build arm or riscv using defaults or other options
bld-arm = "build --target=thumbv8m.main-none-eabihf"
bld-riscv = "build --target=riscv32imac-unknown-none-elf"

# Run arm or riscv using defaults or other options
run-arm = "run --target=thumbv8m.main-none-eabihf"
run-riscv = "run --target=riscv32imac-unknown-none-elf"

# Build dev and release profiles for arm
bda = "bld-arm"
bra = "bld-arm --release"

# Build dev and release for profiles risc-v
bdr = "bld-riscv"
brr = "bld-riscv --release"

# Run dev and release for profiles arm
rda = "run-arm"
rra = "run-arm --release"

# Run dev and release for profiles risc-v
rdr = "run-riscv"
rrr = "run-riscv --release"
```

Using the aliases the commands are much shorter the first is for
running on ARM in rlease mode and the second is for running on RISC-V in release mode:
```console
$ cargo rra --bin blinky
$ cargo rrr --bin blinky
```

These are exactly the same as:
```console
$ cargo run --target thumbv8m.main-none-eabihf --release --bin blinky
$ cargo run --target riscv32imac-unknown-none-elf --release --bin blinky
```

Here is the output of running in release mode the `blinky` example on ARM
after cleaning the build **Note:** have the pico 2 in BOOTSEL mode:
```console
$ cargo clean
     Removed 3565 files, 1.4GiB total
$ cargo rra --bin blinky
   Compiling proc-macro2 v1.0.89
   Compiling unicode-ident v1.0.13
..
   Compiling rp235x-hal v0.2.0 (/home/wink/prgs/rpi-pico/myrepos/rp-hal/rp235x-hal)
   Compiling pio-proc v0.2.2
    Finished `release` profile [optimized] target(s) in 11.72s
     Running `picotool load -u -v -x -t elf target/thumbv8m.main-none-eabihf/release/blinky`
Family id 'rp2350-arm-s' can be downloaded in absolute space:
  00000000->02000000
Loading into Flash: [==============================]  100%
Verifying Flash:    [==============================]  100%
  OK

The device was rebooted to start the application.
```

And you can build first and then run, in this case the `blinky` example was already built:
```console
$ cargo bra --bin blinky
    Finished `release` profile [optimized] target(s) in 0.05s
$ cargo rra --bin blinky
    Finished `release` profile [optimized] target(s) in 0.05s
     Running `picotool load -u -v -x -t elf target/thumbv8m.main-none-eabihf/release/blinky`
Family id 'rp2350-arm-s' can be downloaded in absolute space:
  00000000->02000000
Loading into Flash: [==============================]  100%
Verifying Flash:    [==============================]  100%
  OK

The device was rebooted to start the application.
```

It is currently possible to build *some* of the examples in RISC-V mode. See
[`riscv_examples.txt`](./riscv_examples.txt) for a list of the examples known to
work. The missing ones probably rely on interrupts, or some other thing we
haven't ported to work in RISC-V mode yet.

Here is an example using the `blinky` example in RISC-V mode:
```console
$ cargo brr --bin blinky
    Finished `release` profile [optimized] target(s) in 0.06s
$ cargo rrr --bin blinky
    Finished `release` profile [optimized] target(s) in 0.06s
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
