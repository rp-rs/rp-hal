# [rp-pico] - Board Support for the [Raspberry Pi Pico]

You should include this crate if you are writing code that you want to run on
a [Raspberry Pi Pico] - the original launch PCB for the RP2040 chip.

This crate includes the [rp2040-hal], but also configures each pin of the
RP2040 chip according to how it is connected up on the Pico.

[Raspberry Pi Pico]: https://www.raspberrypi.org/products/raspberry-pi-pico/
[rp-pico]: https://github.com/rp-rs/rp-hal/tree/main/boards/rp-pico
[rp2040-hal]: https://github.com/rp-rs/rp-hal/tree/main/rp2040-hal
[Raspberry Silicon RP2040]: https://www.raspberrypi.org/products/rp2040/

## Using

To use this crate, your `Cargo.toml` file should contain:

```toml
rp-pico = "0.2.0"
```

In your program, you will need to call `rp_pico::Pins::new` to create
a new `Pins` structure. This will set up all the GPIOs for any on-board
devices. See the [examples](./examples) folder for more details.

## Examples

### General Instructions

To compile an example, clone the _rp-hal_ repository and run:

```console
rp-hal/boards/rp-pico $ cargo build --release --example <name>
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
rp-hal/boards/rp-pico $ cargo run --release --example <name>
```

If you get an error about not being able to find `elf2uf2-rs`, try:

```console
$ cargo install elf2uf2-rs
```
then try repeating the `cargo run` command above.

### [pico_blinky](./examples/pico_blinky.rs)

Flashes the Pico's on-board LED on and off.

### [pico_gpio_in_out](./examples/pico_gpio_in_out.rs)

Reads the 'Boot Select' pin and drives the on-board LED to match it (i.e. on when pressed, off when not pressed).

### [pico_rtic](./examples/pico_rtic.rs)

Demonstrates the use of the [Real-Time Interrupt-driven Concurrency Framework] on the Raspberry Pi Pico.

[Real-Time Interrupt-driven Concurrency Framework]: https://rtic.rs

### [pico_countdown_blinky](./examples/pico_countdown_blinky.rs)

Another LED blinking example, but using a Timer in count-down mode.

### [pico_pwm_blink](./examples/pico_pwm_blink.rs)

Puts out an analog 'triangle wave' on GPIO 25, using the PWM hardware.

### [pico_usb_serial](./examples/pico_usb_serial.rs)

Creates a USB Serial device on a Pico board.

The USB Serial device will print `HelloWorld` on start-up, and then echo any
incoming characters - except that any lower-case ASCII characters are
converted to the upper-case equivalent.

### [pico_usb_serial_interrupt](./examples/pico_usb_serial_interrupt.rs)

Creates a USB Serial device on a Pico board, but demonstrating handling
interrupts when USB data arrives.

### [pico_usb_twitchy_mouse](./examples/pico_usb_twitchy_mouse.rs)

Demonstrates emulating a USB Human Input Device (HID) Mouse. The mouse
cursor will jiggle up and down.

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
