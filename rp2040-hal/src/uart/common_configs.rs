use super::{DataBits, StopBits, UartConfig};
use embedded_time::rate::Baud;

/// 9600 baud, 8 data bits, no parity, 1 stop bit
pub const _9600_8_N_1: UartConfig = UartConfig {
    baudrate: Baud(9600),
    data_bits: DataBits::Eight,
    stop_bits: StopBits::One,
    parity: None,
};

/// 19200 baud, 8 data bits, no parity, 1 stop bit
pub const _19200_8_N_1: UartConfig = UartConfig {
    baudrate: Baud(19200),
    data_bits: DataBits::Eight,
    stop_bits: StopBits::One,
    parity: None,
};

/// 38400 baud, 8 data bits, no parity, 1 stop bit
pub const _38400_8_N_1: UartConfig = UartConfig {
    baudrate: Baud(38400),
    data_bits: DataBits::Eight,
    stop_bits: StopBits::One,
    parity: None,
};

/// 57600 baud, 8 data bits, no parity, 1 stop bit
pub const _57600_8_N_1: UartConfig = UartConfig {
    baudrate: Baud(57600),
    data_bits: DataBits::Eight,
    stop_bits: StopBits::One,
    parity: None,
};

/// 115200 baud, 8 data bits, no parity, 1 stop bit
pub const _115200_8_N_1: UartConfig = UartConfig {
    baudrate: Baud(115200),
    data_bits: DataBits::Eight,
    stop_bits: StopBits::One,
    parity: None,
};
