# [pimoroni-pico-lipo-16mb] - Board Support for the [Pimoroni Pico Lipo 16MB]

You should include this crate if you are writing code that you want to run on
a [Pimoroni Pico Lipo 16MB] - a board with USB-C, STEMMA QT/Qwiic connectors,
plus a Li-Po battery charging circuit.

This crate includes the [rp2040-hal], but also configures each pin of the
RP2040 chip according to how it is connected up on the Pico Lipo.

Note that if you use this crate the compiler will expect the full 16MB flash
space, and so it may not work if you only have the 4MB variant. 

[Pimoroni Pico Lipo 16MB]: https://shop.pimoroni.com/products/pimoroni-pico-lipo?variant=39335427080275
[pimoroni-pico-lipo-16mb]: https://github.com/rp-rs/rp-hal/tree/main/boards/pimoroni-pico-lipo-16mb

## Using

To use this crate, your `Cargo.toml` file should contain:

```toml
pimoroni-pico-lipo-16mb = "0.2.0"
```

In your program, you will need to call `pimoroni_pico_lipo_16mb::Pins::new` to create
a new `Pins` structure. This will set up all the GPIOs for any on-board
devices. See the [examples](./examples) folder for more details.

## Examples

### General Instructions

To compile an example, clone the _rp-hal_ repository and run:

```console
rp-hal/boards/pimoroni-pico-lipo-16mb $ cargo build --release --example <name>
```

You will get an ELF file called
`./target/thumbv6m-none-eabi/release/examples/<name>`, where the `target`
folder is located at the top of the _rp-hal_ repository checkout. Normally
you would also need to specify `--target=thumbv6m-none-eabi` but when
building examples from this git repository, that is set as the default.

If you want to convert the ELF file to a UF2 and automatically copy it to the
USB drive exported by the RP2040 bootloader, simply boot your board into
bootloader mode and run:

```console
rp-hal/boards/pimoroni-pico-lipo-16mb $ cargo run --release --example <name>
```

If you get an error about not being able to find `elf2uf2-rs`, try:

```console
$ cargo install elf2uf2-rs, then repeating the `cargo run` command above.
```

### [pimoroni_pico_lipo_16mb_blinky](./examples/pimoroni_pico_lipo_16mb_blinky.rs)

Flashes the Pico Lipo's on-board LED on and off.

## Contributing

Contributions are what make the open source community such an amazing place to
be learn, inspire, and create. Any contributions you make are **greatly
appreciated**.

The steps are:

1. Fork the Project by clicking the 'Fork' button at the top of the page.
2. Create your Feature Branch (`git checkout -b feature/AmazingFeature`)
3. Make some changes to the code or documentation.
4. Commit your Changes (`git commit -m 'Add some AmazingFeature'`)
5. Push to the Feature Branch (`git push origin feature/AmazingFeature`)
6. Create a [New Pull Request](https://github.com/rp-rs/rp-hal/pulls)
7. An admin will review the Pull Request and discuss any changes that may be required.
8. Once everyone is happy, the Pull Request can be merged by an admin, and your work is part of our project!

## Code of Conduct

Contribution to this crate is organized under the terms of the [Rust Code of
Conduct][CoC], and the maintainer of this crate, the [rp-rs team], promises
to intervene to uphold that code of conduct.

[CoC]: CODE_OF_CONDUCT.md
[rp-rs team]: https://github.com/orgs/rp-rs/teams/rp-rs

## License

The contents of this repository are dual-licensed under the _MIT OR Apache
2.0_ License. That means you can chose either the MIT licence or the
Apache-2.0 licence when you re-use this code. See `MIT` or `APACHE2.0` for more
information on each specific licence.

Any submissions to this project (e.g. as Pull Requests) must be made available
under these terms.
