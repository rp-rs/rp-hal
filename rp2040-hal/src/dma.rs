//! # DMA
//!
//! This is the start of a DMA driver.

/// The DREQ value for PIO0's TX FIFO 0
pub const DREQ_PIO0_TX0: u8 = 0;
/// The DREQ value for PIO0's TX FIFO 1
pub const DREQ_PIO0_TX1: u8 = 1;
/// The DREQ value for PIO0's TX FIFO 2
pub const DREQ_PIO0_TX2: u8 = 2;
/// The DREQ value for PIO0's TX FIFO 3
pub const DREQ_PIO0_TX3: u8 = 3;
/// The DREQ value for PIO0's RX FIFO 0
pub const DREQ_PIO0_RX0: u8 = 4;
/// The DREQ value for PIO0's RX FIFO 1
pub const DREQ_PIO0_RX1: u8 = 5;
/// The DREQ value for PIO0's RX FIFO 2
pub const DREQ_PIO0_RX2: u8 = 6;
/// The DREQ value for PIO0's RX FIFO 3
pub const DREQ_PIO0_RX3: u8 = 7;
/// The DREQ value for PIO1's TX FIFO 0
pub const DREQ_PIO1_TX0: u8 = 8;
/// The DREQ value for PIO1's TX FIFO 1
pub const DREQ_PIO1_TX1: u8 = 9;
/// The DREQ value for PIO1's TX FIFO 2
pub const DREQ_PIO1_TX2: u8 = 10;
/// The DREQ value for PIO1's TX FIFO 3
pub const DREQ_PIO1_TX3: u8 = 11;
/// The DREQ value for PIO1's RX FIFO 0
pub const DREQ_PIO1_RX0: u8 = 12;
/// The DREQ value for PIO1's RX FIFO 1
pub const DREQ_PIO1_RX1: u8 = 13;
/// The DREQ value for PIO1's RX FIFO 2
pub const DREQ_PIO1_RX2: u8 = 14;
/// The DREQ value for PIO1's RX FIFO 3
pub const DREQ_PIO1_RX3: u8 = 15;
/// The DREQ value for SPI0's TX FIFO
pub const DREQ_SPI0_TX: u8 = 16;
/// The DREQ value for SPI0's RX FIFO
pub const DREQ_SPI0_RX: u8 = 17;
/// The DREQ value for SPI1's TX FIFO
pub const DREQ_SPI1_TX: u8 = 18;
/// The DREQ value for SPI1's RX FIFO
pub const DREQ_SPI1_RX: u8 = 19;
/// The DREQ value for UART0's TX FIFO
pub const DREQ_UART0_TX: u8 = 20;
/// The DREQ value for UART0's RX FIFO
pub const DREQ_UART0_RX: u8 = 21;
/// The DREQ value for UART1's TX FIFO
pub const DREQ_UART1_TX: u8 = 22;
/// The DREQ value for UART1's RX FIFO
pub const DREQ_UART1_RX: u8 = 23;
/// The DREQ value for PWM Counter 0's Wrap Value
pub const DREQ_PWM_WRAP0: u8 = 24;
/// The DREQ value for PWM Counter 1's Wrap Value
pub const DREQ_PWM_WRAP1: u8 = 25;
/// The DREQ value for PWM Counter 2's Wrap Value
pub const DREQ_PWM_WRAP2: u8 = 26;
/// The DREQ value for PWM Counter 3's Wrap Value
pub const DREQ_PWM_WRAP3: u8 = 27;
/// The DREQ value for PWM Counter 4's Wrap Value
pub const DREQ_PWM_WRAP4: u8 = 28;
/// The DREQ value for PWM Counter 5's Wrap Value
pub const DREQ_PWM_WRAP5: u8 = 29;
/// The DREQ value for PWM Counter 6's Wrap Value
pub const DREQ_PWM_WRAP6: u8 = 30;
/// The DREQ value for PWM Counter 7's Wrap Value
pub const DREQ_PWM_WRAP7: u8 = 31;
/// The DREQ value for I2C0's TX FIFO
pub const DREQ_I2C0_TX: u8 = 32;
/// The DREQ value for I2C0's RX FIFO
pub const DREQ_I2C0_RX: u8 = 33;
/// The DREQ value for I2C1's TX FIFO
pub const DREQ_I2C1_TX: u8 = 34;
/// The DREQ value for I2C1's RX FIFO
pub const DREQ_I2C1_RX: u8 = 35;
/// The DREQ value for the ADC
pub const DREQ_ADC: u8 = 36;
/// The DREQ value for the XIP Streaming FIFO
pub const DREQ_XIP_STREAM: u8 = 37;
/// The DREQ value for the XIP SSI TX FIFO
pub const DREQ_XIP_SSITX: u8 = 38;
/// The DREQ value for the XIP SSI RX FIFO
pub const DREQ_XIP_SSIRX: u8 = 39;
