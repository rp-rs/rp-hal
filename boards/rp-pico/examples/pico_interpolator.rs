//! # Pico Interpolator Example
//!
//! Example demonstrating the usage of the hardware interpolator.
//!
//! Runs several test programs, outputs the result on LEDs.
//! Green led for successful test connects to GPIO3.
//! Red led for unsuccessful test connects to GPIO4.
//! In case of failure, the system LED blinks the number of the test.
//! In case of success, the system LED stays lit.
//!
//! See the `Cargo.toml` file for Copyright and license details.

#![no_std]
#![no_main]

// The macro for our start-up function
use rp_pico::entry;

// GPIO traits
use embedded_hal::digital::v2::OutputPin;

// Ensure we halt the program on panic (if we don't mention this crate it won't
// be linked)
use panic_halt as _;

// A shorter alias for the Peripheral Access Crate, which provides low-level
// register access
use rp_pico::hal::pac;

// A shorter alias for the Hardware Abstraction Layer, which provides
// higher-level drivers.
use rp_pico::hal;

// Pull in any important traits
use rp_pico::hal::prelude::*;

use rp_pico::hal::sio::{Interp, Interp0, Interp1, Lane, LaneCtrl};

/// Entry point to our bare-metal application.
///
/// The `#[entry]` macro ensures the Cortex-M start-up code calls this function
/// as soon as all global variables are initialised.
///
/// The function configures the RP2040 peripherals, then just reads the button
/// and sets the LED appropriately.
#[entry]
fn main() -> ! {
    // Grab our singleton objects
    let mut pac = pac::Peripherals::take().unwrap();
    let core = pac::CorePeripherals::take().unwrap();

    // Set up the watchdog driver - needed by the clock setup code
    let mut watchdog = hal::Watchdog::new(pac.WATCHDOG);

    // Configure the clocks
    //
    // The default is to generate a 125 MHz system clock
    let clocks = hal::clocks::init_clocks_and_plls(
        rp_pico::XOSC_CRYSTAL_FREQ,
        pac.XOSC,
        pac.CLOCKS,
        pac.PLL_SYS,
        pac.PLL_USB,
        &mut pac.RESETS,
        &mut watchdog,
    )
    .ok()
    .unwrap();

    // The delay object lets us wait for specified amounts of time (in
    // milliseconds)
    let mut delay = cortex_m::delay::Delay::new(core.SYST, clocks.system_clock.freq().to_Hz());

    // The single-cycle I/O block controls our GPIO pins
    let mut sio = hal::Sio::new(pac.SIO);

    // Set the pins up according to their function on this particular board
    let pins = rp_pico::Pins::new(
        pac.IO_BANK0,
        pac.PADS_BANK0,
        sio.gpio_bank0,
        &mut pac.RESETS,
    );

    // Our LED outputs
    let mut system_led_pin = pins.led.into_push_pull_output();
    let mut green_led_pin = pins.gpio3.into_push_pull_output();
    let mut red_led_pin = pins.gpio4.into_push_pull_output();

    system_led_pin.set_low().unwrap();
    green_led_pin.set_low().unwrap();
    red_led_pin.set_low().unwrap();

    let mut choose_led = |index: u32, result: bool| {
        if result {
            // blink the green led once to indicate success
            green_led_pin.set_high().unwrap();
            delay.delay_ms(500);
            green_led_pin.set_low().unwrap();
            delay.delay_ms(500);
        } else {
            // turn the red led on to indicate failure
            // and blink the on board led to indicate which test failed, looping forever
            red_led_pin.set_high().unwrap();
            loop {
                for _ in 0..index {
                    system_led_pin.set_high().unwrap();
                    delay.delay_ms(200);
                    system_led_pin.set_low().unwrap();
                    delay.delay_ms(200);
                }
                delay.delay_ms(1000);
            }
        }
    };

    // Run forever, setting the LED according to the button

    choose_led(1, multiplication_table(&mut sio.interp0));
    choose_led(2, moving_mask(&mut sio.interp0));
    choose_led(3, cross_lanes(&mut sio.interp0));
    choose_led(4, simple_blend1(&mut sio.interp0));
    choose_led(5, simple_blend2(&mut sio.interp0));
    choose_led(6, clamp(&mut sio.interp1));
    choose_led(7, texture_mapping(&mut sio.interp0));

    // turn the on board led on to indicate testing is done
    system_led_pin.set_high().unwrap();
    loop {
        delay.delay_ms(1000);
    }
}

fn multiplication_table(interp: &mut Interp0) -> bool {
    //get the default configuration that just keep adding base into accum
    let config = LaneCtrl::new();

    //write the configuration to the hardware.
    interp.get_lane0().set_ctrl(config.encode());

    //set the accumulator to 0 and the base to 9
    interp.get_lane0().set_accum(0);
    interp.get_lane0().set_base(9);

    //the expected output for comparison
    let expected = [9, 18, 27, 36, 45, 54, 63, 72, 81, 90];

    for i in expected {
        //returns the value of accum + base and sets accum to the same value
        let value = interp.get_lane0().pop();

        if value != i {
            return false; //inform that the interpolator did not return the expected value
        }
    }
    true
}

fn moving_mask(interp: &mut Interp0) -> bool {
    //get the default configuration that just keep adding base into accum
    let mut config = LaneCtrl::new();

    interp.get_lane0().set_accum(0x1234ABCD);

    let expected = [
        0x0000_000D,
        0x0000_00C0,
        0x0000_0B00,
        0x0000_A000,
        0x0004_0000,
        0x0030_0000,
        0x0200_0000,
        0x1000_0000,
    ];
    for i in 0..8 {
        // LSB, then MSB. These are inclusive, so 0,31 means "the entire 32 bit register"
        config.mask_lsb = i * 4;
        config.mask_msb = i * 4 + 3;
        interp.get_lane0().set_ctrl(config.encode());

        // Reading read_raw() returns the lane data
        //   after shifting, masking and sign extending, without adding base
        if interp.get_lane0().read_raw() != expected[i as usize] {
            return false;
        }
    }

    let signed_expected = [
        0xFFFF_FFFD,
        0xFFFF_FFC0,
        0xFFFF_FB00,
        0xFFFF_A000,
        0x0004_0000,
        0x0030_0000,
        0x0200_0000,
        0x1000_0000,
    ];

    config.signed = true;
    for i in 0..8 {
        config.mask_lsb = i * 4;
        config.mask_msb = i * 4 + 3;
        interp.get_lane0().set_ctrl(config.encode());

        if interp.get_lane0().read_raw() != signed_expected[i as usize] {
            return false;
        }
    }
    true
}

fn cross_lanes(interp: &mut Interp0) -> bool {
    // this configuration will at the time of pop()
    // when applied to lane0 : set lane0 accumulator to the result from lane1
    // when applied to lane1 : set lane1 accumulator to the result from lane0
    let config = LaneCtrl {
        cross_result: true,
        ..LaneCtrl::new()
    };
    let encoded_config = config.encode();

    // each lane is used through an accessor,
    // as lanes mutate each other, they can not be borrowed at the same time
    interp.get_lane0().set_ctrl(encoded_config);
    interp.get_lane1().set_ctrl(encoded_config);

    interp.get_lane0().set_accum(123);
    interp.get_lane1().set_accum(456);

    // lane0 will add 1 to its result, lane1 will add nothing
    interp.get_lane0().set_base(1);
    interp.get_lane1().set_base(0);

    let expected = [
        (124, 456),
        (457, 124),
        (125, 457),
        (458, 125),
        (126, 458),
        (459, 126),
        (127, 459),
        (460, 127),
        (128, 460),
        (461, 128),
    ];

    for i in expected {
        if i != (interp.get_lane0().peek(), interp.get_lane1().pop()) {
            return false;
        }
    }
    true
}

fn simple_blend1(interp: &mut Interp0) -> bool {
    let config = LaneCtrl {
        blend: true,
        ..LaneCtrl::new()
    };

    //enable blend mode
    interp.get_lane0().set_ctrl(config.encode());
    //make sure the default configuration is in lane1 as the value may be shifted and masked.
    interp.get_lane1().set_ctrl(LaneCtrl::new().encode());

    //set the minimum value for interp.get_lane0().set_accum(0) 0/256
    interp.get_lane0().set_base(500);
    //set the maximum value which is inaccessible
    // as the blend is done between 0/256 and 255/256
    interp.get_lane1().set_base(1000);

    let expected = [500, 582, 666, 748, 832, 914, 998];
    for i in 0..=6 {
        interp.get_lane1().set_accum(255 * i / 6);
        if expected[i as usize] != interp.get_lane1().peek() {
            return false;
        }
    }
    true
}

fn simple_blend2(interp: &mut Interp0) -> bool {
    let config = LaneCtrl {
        blend: true,
        ..LaneCtrl::new()
    };
    //enable blend mode
    interp.get_lane0().set_ctrl(config.encode());

    interp.get_lane0().set_base((-1000i32) as u32);
    interp.get_lane1().set_base(1000);

    let mut config1 = LaneCtrl {
        signed: true,
        ..LaneCtrl::new()
    };
    interp.get_lane1().set_ctrl(config1.encode());
    let expected_signed = [-1000, -672, -336, -8, 328, 656, 992];
    for i in 0..=6 {
        // write a value between 0 and 256 (exclusive)
        interp.get_lane1().set_accum(255 * i / 6);
        // reads it as a value between -1000 and 1000 (exclusive)
        if interp.get_lane1().peek() as i32 != expected_signed[i as usize] {
            return false;
        }
    }
    config1.signed = false;
    interp.get_lane1().set_ctrl(config1.encode());
    let expected_unsigned = [
        0xfffffc18, 0xd5fffd60, 0xaafffeb0, 0x80fffff8, 0x56000148, 0x2c000290, 0x010003e0,
    ];
    for i in 0..=6 {
        interp.get_lane1().set_accum(255 * i / 6);
        // reads a value between 4294966296 and 1000
        if interp.get_lane1().peek() != expected_unsigned[i as usize] {
            return false;
        }
    }
    true
}

///Divides by 4 and clamp the value between 0 and 255 inclusive
fn clamp(interp: &mut Interp1) -> bool {
    // Enables Clamp ONLY AVAILABLE ON Interp1
    // shift two bits to the right and mask the two most significant bits
    // because sign extension is made after the mask
    let config = LaneCtrl {
        clamp: true,
        shift: 2,
        mask_lsb: 0,
        mask_msb: 29,
        signed: true,
        ..LaneCtrl::new()
    };
    interp.get_lane0().set_ctrl(config.encode());
    //set minimum value of result
    interp.get_lane0().set_base(0);
    //set maximum value of result
    interp.get_lane1().set_base(255);
    let values: [(i32, i32); 9] = [
        (-1024, 0),
        (-768, 0),
        (-512, 0),
        (-256, 0),
        (0, 0),
        (256, 64),
        (512, 128),
        (768, 192),
        (1024, 255),
    ];
    for (arg, result) in values {
        interp.get_lane0().set_accum(arg as u32);
        if result != interp.get_lane0().peek() as i32 {
            return false;
        }
    }
    true
}

fn texture_mapping(interp: &mut Interp0) -> bool {
    #[rustfmt::skip]
    let texture: [u8;16] = [
        0x00, 0x01, 0x02, 0x03,
        0x10, 0x11, 0x12, 0x13,
        0x20, 0x21, 0x22, 0x23,
        0x30, 0x31, 0x32, 0x33,
    ];

    // the position will be given in fixed point with 16 bits
    // fractional part
    let uv_fractional_bits = 16;
    let texture_width_bits = 2;
    let texture_height_bits = 2;

    // bits
    //                       3322222222221111 1111110000000000
    //                       1098765432109876 5432109876543210
    // accum0 u axis coordinate            xx xxxxxxxxxxxxxxxx  18 bits
    // after shift and mask                                 xx
    // accum1 v axis                       xx xxxxxxxxxxxxxxxx  18 bits
    // after shift and mask                               xx

    // add_raw make the interpolator increment the accumulator
    // with the base value without masking or shifting
    let config0 = LaneCtrl {
        add_raw: true,
        shift: uv_fractional_bits,
        mask_lsb: 0,
        mask_msb: texture_width_bits - 1,
        ..LaneCtrl::new()
    };
    interp.get_lane0().set_ctrl(config0.encode());
    let config1 = LaneCtrl {
        add_raw: true,
        shift: uv_fractional_bits - texture_width_bits,
        mask_lsb: texture_width_bits,
        mask_msb: texture_width_bits + texture_height_bits - 1,
        ..LaneCtrl::new()
    };
    interp.get_lane1().set_ctrl(config1.encode());

    interp.set_base(0);

    // set starting position to 0x0
    // will move 1/2 a pixel horizontally
    // and 1/3 a pixel vertically per call to pop()
    interp.get_lane0().set_accum(0);
    interp.get_lane0().set_base(65536 / 2);
    interp.get_lane1().set_accum(0);
    interp.get_lane1().set_base(65536 / 3);

    let expected = [
        0x00, 0x00, 0x01, 0x01, 0x12, 0x12, 0x13, 0x23, 0x20, 0x20, 0x31, 0x31,
    ];

    for i in expected {
        if i != texture[interp.pop() as usize] {
            return false;
        }
    }

    // reset the starting position
    interp.get_lane0().set_accum(0);
    interp.get_lane1().set_accum(0);
    interp.set_base(texture.as_ptr() as u32);

    for i in expected {
        // This is unsafe and should be done extremely carefully
        // remember to follow memory alignment,
        // reading or writing an unaligned address will crash
        if i != unsafe { *(interp.pop() as *const u8) } {
            return false;
        }
    }

    true
}
// End of file
