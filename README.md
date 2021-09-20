<!-- PROJECT LOGO -->
<br />
<p align="center">
  <a href="https://github.com/rp-rs/rp2040-hal">
    <img src="https://www.svgrepo.com/show/281119/microchip.svg" alt="Logo" width="140" height="140">
  </a>

   <h3 align="center">rp-hal</h3>

  <p align="center">
    A Rust HAL and board support packages for the RP family of microcontrollers from the Raspberry Pi Foundation
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
    <li><a href="#packages">Packages</a></li>
    <li><a href="#roadmap">Roadmap</a></li>
    <li><a href="#contributing">Contributing</a></li>
    <li><a href="#license">License</a></li>
    <li><a href="#contact">Contact</a></li>
    <li><a href="#acknowledgements">Acknowledgements</a></li>
  </ol>
</details>

<!-- GETTING STARTED -->
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

Each BSP will include some examples to show off the features of that particular board.

[Cargo Workspace]: https://doc.rust-lang.org/cargo/reference/workspaces.html]
[Embedded HAL]: https://github.com/rust-embedded/embedded-hal
[Cargo Features]: https://doc.rust-lang.org/cargo/reference/features.html

### [rp2040-hal] - The HAL for the [Raspberry Pi Silicon RP2040]

You should include this crate in your project if you want to write a driver or
library that runs on the [Raspberry Pi Silicon RP2040], or if you are writing a Board
Support Package (see later on).

The crate provides high-level drivers for the RP2040's internal peripherals,
such as the SPI Controller and the I²C Controller. It doesn't know anything
about how your particular board is wired up (such as what each IO pin of the
RP2040 is connected to).

There are examples in this crate to show how to use various peripherals
(GPIO, I²C, SPI, UART, etc) but note that the pin-outs may not match any
particular board.

[rp2040-hal]: https://github.com/rp-rs/rp-hal/tree/main/rp2040-hal
[Raspberry Pi Silicon RP2040]: https://www.raspberrypi.org/products/rp2040/

### [pico] - Board Support for the [Raspberry Pi Pico]

You should include this crate if you are writing code that you want to run on
a [Raspberry Pi Pico] - the original launch PCB for the RP2040 chip.  

This crate includes the [rp2040-hal], but also configures each pin of the
RP2040 chip according to how it is connected up on the Pico.

[Raspberry Pi Pico]: https://www.raspberrypi.org/products/raspberry-pi-pico/
[pico]: https://github.com/rp-rs/rp-hal/tree/main/boards/pico

### [adafruit_macropad] - Board Support for the [Adafruit Macropad]

You should include this crate if you are writing code that you want to run on
an [Adafruit Macropad] - a 3x4 keyboard and OLED combo board from Adafruit.

This crate includes the [rp2040-hal], but also configures each pin of the
RP2040 chip according to how it is connected up on the Macropad.

[adafruit_macropad]: https://github.com/rp-rs/rp-hal/tree/main/boards/adafruit_macropad
[Adafruit Macropad]: https://www.adafruit.com/product/5128

### [feather_rp2040] - Board Support for the [Adafruit Feather RP2040]

You should include this crate if you are writing code that you want to run on
an [Adafruit Feather RP2040] - a Feather form-factor RP2040 board from Adafruit.

This crate includes the [rp2040-hal], but also configures each pin of the
RP2040 chip according to how it is connected up on the Feather RP2040.

[Adafruit Feather RP2040]: https://www.adafruit.com/product/4884
[feather_rp2040]: https://github.com/rp-rs/rp-hal/tree/main/boards/feather_rp2040

### [pico_explorer] - Board Support for the [Pimoroni Pico Explorer]

You should include this crate if you are writing code that you want to run on
a [Pimoroni Pico Explorer] - a board featuring a small LCD screen, a
breadboard and some breakout headers.

This crate includes the [rp2040-hal], but also configures each pin of the
RP2040 chip according to how it is connected up on the Pico Explorer.

[Pimoroni Pico Explorer]: https://shop.pimoroni.com/products/pico-explorer-base
[pico_explorer]: https://github.com/rp-rs/rp-hal/tree/main/boards/pico_explorer

### [pico_lipo_16mb] - Board Support for the [Pimoroni Pico Lipo 16MB]

You should include this crate if you are writing code that you want to run on
a [Pimoroni Pico Lipo 16MB] - a board with USB-C, STEMMA QT/Qwiic connectors,
plus a Li-Po battery charging circuit.

This crate includes the [rp2040-hal], but also configures each pin of the
RP2040 chip according to how it is connected up on the Pico Liop.

Note that if you use this crate the compiler will expect the full 16MB flash
space, and so it may not work if you only have the 4MB variant. 

[Pimoroni Pico Lipo 16MB]: https://shop.pimoroni.com/products/pimoroni-pico-lipo?variant=39335427080275
[pico_lipo_16mb]: https://github.com/rp-rs/rp-hal/tree/main/boards/pico_lipo_16mb

<!-- ROADMAP -->
## Roadmap

NOTE These packages are under active development. As such, it is likely to remain volatile until a 1.0.0 release.

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
Matrix: [#rp-rs:matrix.org](https://matrix.to/#/#rp-rs:matrix.org)


<!-- ACKNOWLEDGEMENTS -->
## Acknowledgements

* [Othneil Drew's README template](https://github.com/othneildrew)
