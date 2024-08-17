<!-- PROJECT LOGO -->
<br />
<p align="center">
  <a href="https://github.com/rp-rs/rp-hal">
    <img src="https://www.svgrepo.com/show/281119/microchip.svg" alt="Logo" width="140" height="140">
  </a>

   <h3 align="center">rp-hal</h3>

  <p align="center">
    Rust Examples for the Raspberry Silicon RP2040 Microcontroller
    <br />
    <a href="https://docs.rs/rp2040-hal"><strong>Explore the API docs »</strong></a>
    <br />
    <br />
    <a href="https://github.com/rp-rs/rp-hal-boards/tree/main/boards/rp-pico/examples">View more Demos</a>
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

The `rp2040-hal` package is a library crate of high-level Rust drivers for the
Raspberry Silicon RP2040 microcontroller. This folder contains a collection of
non-board specific example programs for you to study.

We also provide a series of [*Board Support Package* (BSP) crates][BSPs], which
take the HAL crate and pre-configure the pins according to a specific PCB
design. If you are using one of the supported boards, you should use one of
those crates in preference, and return here to see documentation about specific
peripherals on the RP2040 and how to use them. See the `boards` folder in
https://github.com/rp-rs/rp-hal-boards/ for more details.

[BSPs]: https://github.com/rp-rs/rp-hal-boards/

<!-- GETTING STARTED -->
## Getting Started

To build all the examples, first grab a copy of the source code:

```console
$ git clone https://github.com/rp-rs/rp-hal.git
```

Then use `rustup` to grab the Rust Standard Library for the appropriate target and our preferred flashing tool:

```console
$ rustup target add thumbv6m-none-eabi
```

Then you can build the examples:

```console
$ cd rp2040-hal-examples
$ cargo build
   Compiling rp2040-hal-examples v0.1.0 (/home/user/rp-hal/rp2040-hal-examples)
    Finished `dev` profile [unoptimized + debuginfo] target(s) in 4.53s
$ cargo build --bin dormant_sleep
    Finished `dev` profile [unoptimized + debuginfo] target(s) in 0.06s
$ file ./target/thumbv6m-none-eabi/debug/dormant_sleep
./target/thumbv6m-none-eabi/debug/dormant_sleep: ELF 32-bit LSB executable, ARM, EABI5 version 1 (SYSV), statically linked, with debug_info, not stripped
```

You can also 'run' an example, which will invoke [`elf2uf2-rs`] to copy it to an
RP2040's virtual USB Mass Storage Device (which it puts over the USB port when
in its ROM bootloader). You should install that if you don't have it already:

```console
$ cargo install elf2uf2-rs --locked
$ cd rp2040-hal-examples
$ cargo run --bin dormant_sleep
   Compiling rp2040-hal v0.10.0 (/home/user/rp-hal/rp2040-hal)
   Compiling rp2040-hal-examples v0.1.0 (/home/user/rp-hal/rp2040-hal-examples)
    Finished `dev` profile [unoptimized + debuginfo] target(s) in 1.62s
     Running `elf2uf2-rs -d target/thumbv6m-none-eabi/debug/dormant_sleep`
Found pico uf2 disk /media/user/RP2040
Transfering program to pico
88.50 KB / 88.50 KB [=====================================] 100.00 % 430.77 KB/s
$
```

[`elf2uf2-rs`]: https://github.com/JoNil/elf2uf2-rs

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
