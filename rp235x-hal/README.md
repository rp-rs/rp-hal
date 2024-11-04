<!-- PROJECT LOGO -->
<br />
<p align="center">
  <a href="https://github.com/rp-rs/rp-hal">
    <img src="https://www.svgrepo.com/show/281119/microchip.svg" alt="Logo" width="140" height="140">
  </a>

   <h3 align="center">rp-hal</h3>

  <p align="center">
    High-level Rust drivers for the Raspberry Silicon RP2350 Microcontroller
    <br />
    <a href="https://docs.rs/rp235x-hal"><strong>Explore the API docs »</strong></a>
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

This is the `rp235x-hal` package - a library crate of high-level Rust drivers
for the Raspberry Silicon RP2350 microcontroller, along with a collection of
non-board specific example programs for you to study. You should use this crate
in your application if you want to write code for the RP2350 microcontroller.
The *HAL* in the name standards for *Hardware Abstraction Layer*, and comes from
the fact that many of the drivers included implement the generic
hardware-abstraction interfaces defined in the Rust Embedded Working Group's
[embedded-hal](https://github.com/rust-embedded/embedded-hal) crate.

We also provide a series of [*Board Support Package* (BSP) crates][BSPs], which take
this HAL crate and pre-configure the pins according to a specific PCB design. If
you are using one of the supported boards, you should use one of those crates in
preference, and return here to see documentation about specific peripherals on
the RP2350 and how to use them. See the `boards` folder in
https://github.com/rp-rs/rp-hal-boards/ for more details.

[BSPs]: https://github.com/rp-rs/rp-hal-boards/

Some of the source code herein refers to the "RP2350 Datasheet". This can be
found at <https://rptl.io/rp2350-datasheet>.

<!-- GETTING STARTED -->
## Getting Started

To include this crate in your project, amend your `Cargo.toml` file to include

```toml
rp235x-hal = "*"
```

Or to include this version specifically:

```toml
rp235x-hal = "0.3.0"
```

To obtain a copy of the source code (e.g. if you want to propose a bug-fix or
new feature, or simply to study the code), run:

```console
$ git clone https://github.com/rp-rs/rp-hal.git
```

For details on how to program an RP2350 microcontroller, see the [top-level
rp-hal README](https://github.com/rp-rs/rp-hal/).

<!-- ROADMAP -->
## Roadmap

NOTE This HAL is under active development. As such, it is likely to remain
volatile until a 1.0.0 release.

See the [open issues](https://github.com/rp-rs/rp-hal/issues) for a list of
proposed features (and known issues).

### Implemented traits

This crate aims to implement all traits from embedded-hal, both version
0.2 and 1.0. They can be used at the same time, so you can upgrade drivers
incrementally.

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
