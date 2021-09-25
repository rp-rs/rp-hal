<!-- PROJECT LOGO -->
<br />
<p align="center">
  <a href="https://github.com/rp-rs/rp2040-hal">
    <img src="https://www.svgrepo.com/show/281119/microchip.svg" alt="Logo" width="140" height="140">
  </a>

   <h3 align="center">rp-hal</h3>

  <p align="center">
    A Rust HAL impl for the RP family of microcontrollers from the Raspberry Pi Foundation
    <br />
    <a href="https://github.com/rp-rs/rp-hal"><strong>Explore the docs »</strong></a>
    <br />
    <br />
    <a href="https://github.com/rp-rs/rp-hal">View Demo</a>
    ·
    <a href="https://github.com/rp-rs/rp-hal/issues">Report Bug</a>
    ·
    <a href="https://github.com/rp-rs/rp-hal/issues">Request Feature</a>
  </p>
</p>



<!-- TABLE OF CONTENTS -->
<details open="open">
  <summary><h2 style="display: inline-block">Table of Contents</h2></summary>
  <ol>
   <li>
      <a href="#getting-started">Getting Started</a>
      <ul>
        <li><a href="#prerequisites">Prerequisites</a></li>
        <li><a href="#installation">Installation</a></li>
      </ul>
    </li>
    <li><a href="#usage">Usage</a></li>
    <li><a href="#roadmap">Roadmap</a></li>
    <li><a href="#contributing">Contributing</a></li>
    <li><a href="#license">License</a></li>
    <li><a href="#contact">Contact</a></li>
    <li><a href="#acknowledgements">Acknowledgements</a></li>
  </ol>
</details>

<!-- GETTING STARTED -->
## Getting Started

To get a local copy up and running follow these simple steps.

### Prerequisites

* A [Rust](https://www.rust-lang.org/tools/install) toolchain

### Installation

1. Clone the repo or use the crate

   ```sh
   git clone https://github.com/rp-rs/rp-hal
   ```

   or

   ```sh
   cargo install rp2040-hal
   ```

<!-- USAGE EXAMPLES -->
## Usage

Use this space to show useful examples of how a project can be used. Additional screenshots, code examples and demos work well in this space. You may also link to more resources.

For more examples, please refer to the [Documentation](https://github.com/rp-rs/rp-hal)

### Run examples

#### UF2

For boards with uf2 flashloaders like the raspberry pi pico. Install [`elf2uf2-rs`](https://github.com/JoNil/elf2uf2-rs):

```sh
cargo install elf2uf2-rs
```

Make sure .cargo/config contains the following (it should by default):

```toml
runner = "elf2uf2-rs -d"
```

**IMPORTANT: Make sure you've put your device into bootloader mode and the drive is showing as mounted before executing the next command.**

```sh
cargo run --example pico_pwm_blink # Run `cargo run --example` for more examples
```

<!-- ROADMAP -->
## Roadmap

NOTE This HAL is under active development. As such, it is likely to remain volatile until a 1.0.0 release.

See the [open issues](https://github.com/rp-rs/rp-hal/issues) for a list of proposed features (and known issues).

### Support for embedded-hal 1.0

We plan to support embedded-hal 1.0 soon after it is released.

For now, there is preliminary support for alpha versions of embedded-hal, which can
be enabled with the feature `eh1_0_alpha`. Please note that this support does not
provide any semver compatibility guarantees: With that feature activated, there
will be breaking changes even in minor versions of rp2040-hal.

Support for embedded-hal 1.0(-alpha) exists in parallel to support for
embedded-hal 0.2: Traits of both versions are implemented and can be used
at the same time.

<!-- CONTRIBUTING -->
## Contributing

Contributions are what make the open source community such an amazing place to be learn, inspire, and create. Any contributions you make are **greatly appreciated**.

1. Fork the Project
2. Create your Feature Branch (`git checkout -b feature/AmazingFeature`)
3. Commit your Changes (`git commit -m 'Add some AmazingFeature'`)
4. Push to the Branch (`git push origin feature/AmazingFeature`)
5. Open a Pull Request



<!-- LICENSE -->
## License

Distributed under the MIT OR Apache2.0 License. See `MIT` or `APACHE2.0` for more information.



<!-- CONTACT -->
## Contact

Project Link: [https://github.com/rp-rs/rp-hal/issues](https://github.com/rp-rs/rp-hal/issues)
Matrix: [#rp-rs:matrix.org](https://matrix.to/#/#rp-rs:matrix.org)


<!-- ACKNOWLEDGEMENTS -->
## Acknowledgements

* [Othneil Drew's README template](https://github.com/othneildrew)
