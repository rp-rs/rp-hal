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
   cargo install rpXXXX-hal
   ```

<!-- USAGE EXAMPLES -->
## Usage

Use this space to show useful examples of how a project can be used. Additional screenshots, code examples and demos work well in this space. You may also link to more resources.

For more examples, please refer to the [Documentation](https://github.com/rp-rs/rp-hal)

### Run examples

Install [`uf2conv`](https://github.com/sajattack/uf2conv-rs) and [`cargo-binutils`](https://github.com/rust-embedded/cargo-binutils) as well as the `llvm-tools-preview` component:

```sh
cargo install uf2conv cargo-binutils
rustup component add llvm-tools-preview
```

For boards with uf2 flashloaders you can use the following lines to run the examples:

```sh
export RPI_MOUNT_FOLDER=/media/RPI-RP2/
export EXAMPLE=pico_blinky; # See `cargo check --example` for valid values
cargo check --example $EXAMPLE && \
cargo objcopy --release --example $EXAMPLE -- -O binary target/$EXAMPLE.bin && \
uf2conv target/$EXAMPLE.bin --base 0x10000000 --family 0xe48bff56 --output target/$EXAMPLE.uf2 && \
cp target/$EXAMPLE.uf2 $RPI_MOUNT_FOLDER
```

<!-- ROADMAP -->
## Roadmap

NOTE This HAL is under active development. As such, it is likely to remain volatile until a 1.0.0 release.

See the [open issues](https://github.com/rp-rs/rp-hal/issues) for a list of proposed features (and known issues).


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



<!-- ACKNOWLEDGEMENTS -->
## Acknowledgements

* [Othneil Drew's README template](https://github.com/othneildrew)
