use fugit::HertzU32;

use super::{DataBits, StopBits, UartConfig};

/// 9600 baud, 8 data bits, no parity, 1 stop bit
pub const _9600_8_N_1: UartConfig = UartConfig {
    baudrate: HertzU32::from_raw(9600),
    data_bits: DataBits::Eight,
    stop_bits: StopBits::One,
    parity: None,
};

/// 19200 baud, 8 data bits, no parity, 1 stop bit
pub const _19200_8_N_1: UartConfig = UartConfig {
    baudrate: HertzU32::from_raw(19200),
    data_bits: DataBits::Eight,
    stop_bits: StopBits::One,
    parity: None,
};

/// 38400 baud, 8 data bits, no parity, 1 stop bit
pub const _38400_8_N_1: UartConfig = UartConfig {
    baudrate: HertzU32::from_raw(38400),
    data_bits: DataBits::Eight,
    stop_bits: StopBits::One,
    parity: None,
};

/// 57600 baud, 8 data bits, no parity, 1 stop bit
pub const _57600_8_N_1: UartConfig = UartConfig {
    baudrate: HertzU32::from_raw(57600),
    data_bits: DataBits::Eight,
    stop_bits: StopBits::One,
    parity: None,
};

/// 115200 baud, 8 data bits, no parity, 1 stop bit
pub const _115200_8_N_1: UartConfig = UartConfig {
    baudrate: HertzU32::from_raw(115200),
    data_bits: DataBits::Eight,
    stop_bits: StopBits::One,
    parity: None,
};
