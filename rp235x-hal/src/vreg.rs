//! VREG voltage control types.

/// Voltage selection for the on-chip VREG.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum VRegVoltage {
    /// 0.55V
    V550mV = 0b00000,
    /// 0.60V
    V600mV = 0b00001,
    /// 0.65V
    V650mV = 0b00010,
    /// 0.70V
    V700mV = 0b00011,
    /// 0.75V
    V750mV = 0b00100,
    /// 0.80V
    V800mV = 0b00101,
    /// 0.85V
    V850mV = 0b00110,
    /// 0.90V
    V900mV = 0b00111,
    /// 0.95V
    V950mV = 0b01000,
    /// 1.00V
    V1000mV = 0b01001,
    /// 1.05V
    V1050mV = 0b01010,
    /// 1.10V (default)
    V1100mV = 0b01011,
    /// 1.15V
    V1150mV = 0b01100,
    /// 1.20V
    V1200mV = 0b01101,
    /// 1.25V
    V1250mV = 0b01110,
    /// 1.30V
    V1300mV = 0b01111,
    /// 1.35V
    V1350mV = 0b10000,
    /// 1.40V
    V1400mV = 0b10001,
    /// 1.50V
    V1500mV = 0b10010,
    /// 1.60V
    V1600mV = 0b10011,
    /// 1.65V
    V1650mV = 0b10100,
    /// 1.70V
    V1700mV = 0b10101,
    /// 1.80V
    V1800mV = 0b10110,
    /// 1.90V
    V1900mV = 0b10111,
    /// 2.00V
    V2000mV = 0b11000,
    /// 2.35V
    V2350mV = 0b11001,
    /// 2.50V
    V2500mV = 0b11010,
    /// 2.65V
    V2650mV = 0b11011,
    /// 2.80V
    V2800mV = 0b11100,
    /// 3.00V
    V3000mV = 0b11101,
    /// 3.15V
    V3150mV = 0b11110,
    /// 3.30V
    V3300mV = 0b11111,
}

/// Only binary values representable in five bits or less correspond
/// to possible voltage settings of VSEL on RP235x.
pub struct InvalidVRegVoltageError;

impl TryFrom<u8> for VRegVoltage {
    type Error = InvalidVRegVoltageError;
    /// Convert from raw register value to [`VRegVoltage`].
    fn try_from(bits: u8) -> Result<Self, InvalidVRegVoltageError> {
        match bits {
            0b00000 => Ok(VRegVoltage::V550mV),
            0b00001 => Ok(VRegVoltage::V600mV),
            0b00010 => Ok(VRegVoltage::V650mV),
            0b00011 => Ok(VRegVoltage::V700mV),
            0b00100 => Ok(VRegVoltage::V750mV),
            0b00101 => Ok(VRegVoltage::V800mV),
            0b00110 => Ok(VRegVoltage::V850mV),
            0b00111 => Ok(VRegVoltage::V900mV),
            0b01000 => Ok(VRegVoltage::V950mV),
            0b01001 => Ok(VRegVoltage::V1000mV),
            0b01010 => Ok(VRegVoltage::V1050mV),
            0b01011 => Ok(VRegVoltage::V1100mV),
            0b01100 => Ok(VRegVoltage::V1150mV),
            0b01101 => Ok(VRegVoltage::V1200mV),
            0b01110 => Ok(VRegVoltage::V1250mV),
            0b01111 => Ok(VRegVoltage::V1300mV),
            0b10000 => Ok(VRegVoltage::V1350mV),
            0b10001 => Ok(VRegVoltage::V1400mV),
            0b10010 => Ok(VRegVoltage::V1500mV),
            0b10011 => Ok(VRegVoltage::V1600mV),
            0b10100 => Ok(VRegVoltage::V1650mV),
            0b10101 => Ok(VRegVoltage::V1700mV),
            0b10110 => Ok(VRegVoltage::V1800mV),
            0b10111 => Ok(VRegVoltage::V1900mV),
            0b11000 => Ok(VRegVoltage::V2000mV),
            0b11001 => Ok(VRegVoltage::V2350mV),
            0b11010 => Ok(VRegVoltage::V2500mV),
            0b11011 => Ok(VRegVoltage::V2650mV),
            0b11100 => Ok(VRegVoltage::V2800mV),
            0b11101 => Ok(VRegVoltage::V3000mV),
            0b11110 => Ok(VRegVoltage::V3150mV),
            0b11111 => Ok(VRegVoltage::V3300mV),
            _ => Err(InvalidVRegVoltageError),
        }
    }
}
