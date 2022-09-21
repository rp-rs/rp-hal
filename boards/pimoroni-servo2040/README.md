# [pimoroni-servo2040] - Board Support for the [Pimoroni Servo2040]

You should include this crate if you are writing code that you want to run on
a [Pimoroni Servo2040] - a standalone servo motor controller for up to 18 servos
and 6 sensors.

This crate includes the [rp2040-hal], but also configures each pin of the
RP2040 chip according to how it is connected up on the Servo2040.

[Pimoroni Servo2040]: https://shop.pimoroni.com/products/servo-2040
[pimoroni-servo2040]: https://github.com/rp-rs/rp-hal/tree/main/boards/pimoroni-servo2040
[rp2040-hal]: https://github.com/rp-rs/rp-hal/tree/main/rp2040-hal
[Raspberry Silicon RP2040]: https://www.raspberrypi.org/products/rp2040/

## Using

To use this crate, your `Cargo.toml` file should contain:

```toml
pimoroni-servo2040 = "0.1.0"
```

In your program, you will need to call `pimoroni_servo2040::Pins::new` to create
a new `Pins` structure. This will set up all the GPIOs for any on-board
devices. See the [examples](./examples) folder for more details.

## Examples

### General Instructions

To compile an example, clone the _rp-hal_ repository and run:

```console
rp-hal/boards/pimoroni-servo2040 $ cargo build --release --example <name>
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
rp-hal/boards/pimoroni-servo2040 $ cargo run --release --example <name>
```

If you get an error about not being able to find `elf2uf2-rs`, try:

```console
$ cargo install elf2uf2-rs
```
then try repeating the `cargo run` command above.

### [pimoroni_servo2040_rainbow](./examples/pimoroni_servo2040_rainbow.rs)

Animates a rainbow wheel on the Servo2040's six on-board LEDs in sequence.

![Servo 2040 Rainbow LED Animation](
    https://user-images.githubusercontent.com/1673130/190923083-4cd9794a-6775-4f9e-92c3-64d6d25c61dd.gif)

### [pimoroni_servo2040_pwm_servo](./examples/pimoroni_servo2040_pwm_servo.rs)

Moves a micro servo connected to pin Servo 1 on a Servo2040 board using the PWM
peripheral.

![Servo 2040 Servo Animation](
    https://user-images.githubusercontent.com/1673130/190923094-1e7dc85d-2742-4d2f-9b9d-f3337b03cda2.gif)
![Servo 2040 Oscilloscope Waveform Animation](
    https://user-images.githubusercontent.com/1673130/190923098-f725bf4a-541b-4e4c-9f79-eb435e8d2d27.gif)
![Servo 2040 Oscilloscope Waveform Image with Frequency](
    https://user-images.githubusercontent.com/1673130/190923130-43bab8aa-8773-42f8-90cb-9bc684368027.jpg)

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
2.0_ License. That means you can choose either the MIT license or the
Apache-2.0 license when you re-use this code. See `MIT` or `APACHE2.0` for more
information on each specific license.

Any submissions to this project (e.g. as Pull Requests) must be made available
under these terms.
