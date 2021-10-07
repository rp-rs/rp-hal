use super::ADDRESS;
use core::ops::Deref;
use rp2040_hal::i2c::I2C;
use rp2040_hal::pac::i2c0::RegisterBlock as I2CBlock;

/// Controller demo
pub async fn run_demo<Block, Pins>(i2c: &mut I2C<Block, Pins>) -> Result<(), rp2040_hal::i2c::Error>
where
    Block: Deref<Target = I2CBlock>,
{
    let mut tx_filler = 0;
    let mut tx = [0u8; 24];
    let mut rx = [0u8; 24];

    i2c.non_blocking_read(ADDRESS, &mut rx).await?;
    rx.iter()
        .cloned()
        .zip(0x80..)
        .for_each(|(a, b)| assert_eq!(a, b));

    tx.iter_mut().for_each(|b| {
        *b = tx_filler;
        tx_filler += 1;
    });

    i2c.non_blocking_write_read(ADDRESS, &tx, &mut rx).await?;
    rx.iter()
        .cloned()
        .zip(0x80 + 24..) // follows the inital read
        .for_each(|(a, b)| assert_eq!(a, b));

    tx.iter_mut().for_each(|b| {
        *b = tx_filler;
        tx_filler += 1;
    });
    i2c.non_blocking_write(ADDRESS, &tx).await?;
    Ok(())
}
