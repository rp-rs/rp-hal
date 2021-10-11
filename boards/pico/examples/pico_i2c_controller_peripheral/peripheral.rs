//! I2C Peripheral demo
//!
//! This module implements a state machine serving the I2C requests from the controller in this
//! demo. In a real-life application the state machine may not need to be validated as thoroughly
//! demonstrated here.

use core::ops::Deref;
use rp2040_hal::i2c::peripheral::I2CAsyncPeripheral;
use rp2040_hal::i2c::peripheral::I2CEvent;
use rp2040_hal::pac::i2c0::RegisterBlock as I2CBlock;

pub async fn run_demo<Block, Pins>(
    i2c: &mut I2CAsyncPeripheral<Block, Pins>,
) -> Result<(), rp2040_hal::i2c::Error>
where
    Block: Deref<Target = I2CBlock>,
{
    let mut expected_value = 0..;
    let mut output = 128;

    #[derive(Debug, PartialEq)]
    enum Stage {
        Idle0,
        FirstRead,
        Idle1,
        FirstWrite,
        SecondRead,
        Idle2,
        SecondWrite,
        Done,
    }
    let mut stage = Stage::Idle0;

    while stage != Stage::Done {
        let ev = i2c.next_event().await?;
        match ev {
            I2CEvent::Start => {
                stage = match stage {
                    Stage::Idle0 => Stage::FirstRead,
                    Stage::Idle1 => Stage::FirstWrite,
                    Stage::Idle2 => Stage::SecondWrite,
                    _ => panic!("Unexpected {:?} while in {:?}", ev, stage),
                }
            }
            I2CEvent::TransferRead => {
                if stage != Stage::FirstRead && stage != Stage::SecondRead {
                    panic!("Unexpected {:?} while in {:?}", ev, stage);
                }

                i2c.write(&[output, output + 1, output + 2, output + 3]);
                output += 4;
            }
            I2CEvent::TransferWrite => {
                if stage != Stage::FirstWrite && stage != Stage::SecondWrite {
                    panic!("Unexpected {:?} while in {:?}", ev, stage);
                }

                let mut buf = [0; 16];
                loop {
                    let read = i2c.read(&mut buf);
                    if read == 0 {
                        break;
                    }

                    buf.iter()
                        .take(read)
                        .cloned()
                        .zip(&mut expected_value)
                        .for_each(|(a, b)| assert_eq!(a, b));
                }
            }
            I2CEvent::Stop => {
                stage = match stage {
                    Stage::FirstRead => Stage::Idle1,
                    Stage::SecondRead => Stage::Idle2,
                    Stage::SecondWrite => Stage::Done,
                    _ => panic!("Unexpected {:?} while in {:?}", ev, stage),
                }
            }
            I2CEvent::Restart => {
                stage = match stage {
                    Stage::FirstWrite => Stage::SecondRead,
                    _ => panic!("Unexpected {:?} while in {:?}", ev, stage),
                }
            }
        }
    }
    Ok(())
}
