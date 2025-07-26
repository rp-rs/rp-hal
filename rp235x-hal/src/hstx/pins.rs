use crate::{
    gpio::{bank0::*, AnyPin, FunctionHstx},
    typelevel::{OptionT, OptionTNone, OptionTSome, Sealed},
};

macro_rules! hstx_pins {
    ( $( $bit:expr => $pin:expr ),* ) => {
        paste::paste!{
            $(
                #[doc = "Indicates a valid bit " $bit " pin for HSTX"]
                pub trait [<ValidHstxBit $bit Pin>] : Sealed {}

                impl<T> [<ValidHstxBit $bit Pin>] for T
                where
                    T: AnyPin<Function = FunctionHstx, Id = [<Gpio $pin>]>
                {
                }

                #[doc = "Indicates a valid optional bit " $bit " pin for HSTX"]
                pub trait [<ValidOptionHstxBit $bit>]: OptionT {}

                impl [<ValidOptionHstxBit $bit>] for OptionTNone {}
                impl<T> [<ValidOptionHstxBit $bit>] for OptionTSome<T>
                where
                    T: [<ValidHstxBit $bit Pin>],
                {
                }
            )*
        }
    }
}

hstx_pins!(0 => 12, 1 => 13, 2 => 14, 3 => 15, 4 => 16, 5 => 17, 6 => 18, 7 => 19);

/// Declares a valid HSTX pinout
pub trait ValidHstxPinout: Sealed {
    #[allow(missing_docs)]
    type Bit0: ValidOptionHstxBit0;
    #[allow(missing_docs)]
    type Bit1: ValidOptionHstxBit1;
    #[allow(missing_docs)]
    type Bit2: ValidOptionHstxBit2;
    #[allow(missing_docs)]
    type Bit3: ValidOptionHstxBit3;
    #[allow(missing_docs)]
    type Bit4: ValidOptionHstxBit4;
    #[allow(missing_docs)]
    type Bit5: ValidOptionHstxBit5;
    #[allow(missing_docs)]
    type Bit6: ValidOptionHstxBit6;
    #[allow(missing_docs)]
    type Bit7: ValidOptionHstxBit7;
}

/// Set of valid HSTX pins
pub struct HstxPins<Opt0, Opt1, Opt2, Opt3, Opt4, Opt5, Opt6, Opt7>
where
    Opt0: ValidOptionHstxBit0,
    Opt1: ValidOptionHstxBit1,
    Opt2: ValidOptionHstxBit2,
    Opt3: ValidOptionHstxBit3,
    Opt4: ValidOptionHstxBit4,
    Opt5: ValidOptionHstxBit5,
    Opt6: ValidOptionHstxBit6,
    Opt7: ValidOptionHstxBit7,
{
    /// Optional bit 0 pin
    pub bit0: Opt0,
    /// Optional bit 1 pin
    pub bit1: Opt1,
    /// Optional bit 2 pin
    pub bit2: Opt2,
    /// Optional bit 3 pin
    pub bit3: Opt3,
    /// Optional bit 4 pin
    pub bit4: Opt4,
    /// Optional bit 5 pin
    pub bit5: Opt5,
    /// Optional bit 6 pin
    pub bit6: Opt6,
    /// Optional bit 7 pin
    pub bit7: Opt7,
}

impl<
        Opt0: ValidOptionHstxBit0,
        Opt1: ValidOptionHstxBit1,
        Opt2: ValidOptionHstxBit2,
        Opt3: ValidOptionHstxBit3,
        Opt4: ValidOptionHstxBit4,
        Opt5: ValidOptionHstxBit5,
        Opt6: ValidOptionHstxBit6,
        Opt7: ValidOptionHstxBit7,
    > Sealed for HstxPins<Opt0, Opt1, Opt2, Opt3, Opt4, Opt5, Opt6, Opt7>
{
}

impl<
        Opt0: ValidOptionHstxBit0,
        Opt1: ValidOptionHstxBit1,
        Opt2: ValidOptionHstxBit2,
        Opt3: ValidOptionHstxBit3,
        Opt4: ValidOptionHstxBit4,
        Opt5: ValidOptionHstxBit5,
        Opt6: ValidOptionHstxBit6,
        Opt7: ValidOptionHstxBit7,
    > ValidHstxPinout for HstxPins<Opt0, Opt1, Opt2, Opt3, Opt4, Opt5, Opt6, Opt7>
{
    type Bit0 = Opt0;
    type Bit1 = Opt1;
    type Bit2 = Opt2;
    type Bit3 = Opt3;
    type Bit4 = Opt4;
    type Bit5 = Opt5;
    type Bit6 = Opt6;
    type Bit7 = Opt7;
}

impl
    HstxPins<
        OptionTNone,
        OptionTNone,
        OptionTNone,
        OptionTNone,
        OptionTNone,
        OptionTNone,
        OptionTNone,
        OptionTNone,
    >
{
    /// Create an empty set of HstxPins
    #[allow(clippy::new_without_default)]
    pub fn new() -> HstxPins<
        OptionTNone,
        OptionTNone,
        OptionTNone,
        OptionTNone,
        OptionTNone,
        OptionTNone,
        OptionTNone,
        OptionTNone,
    > {
        HstxPins {
            bit0: OptionTNone {},
            bit1: OptionTNone {},
            bit2: OptionTNone {},
            bit3: OptionTNone {},
            bit4: OptionTNone {},
            bit5: OptionTNone {},
            bit6: OptionTNone {},
            bit7: OptionTNone {},
        }
    }
}

impl<
        Opt1: ValidOptionHstxBit1,
        Opt2: ValidOptionHstxBit2,
        Opt3: ValidOptionHstxBit3,
        Opt4: ValidOptionHstxBit4,
        Opt5: ValidOptionHstxBit5,
        Opt6: ValidOptionHstxBit6,
        Opt7: ValidOptionHstxBit7,
    > HstxPins<OptionTNone, Opt1, Opt2, Opt3, Opt4, Opt5, Opt6, Opt7>
{
    /// Add bit0 pin to HSTX pin set
    pub fn add_bit0_pin<Bit0: ValidHstxBit0Pin>(
        self,
        bit0: Bit0,
    ) -> HstxPins<OptionTSome<Bit0>, Opt1, Opt2, Opt3, Opt4, Opt5, Opt6, Opt7> {
        HstxPins {
            bit0: OptionTSome(bit0),
            bit1: self.bit1,
            bit2: self.bit2,
            bit3: self.bit3,
            bit4: self.bit4,
            bit5: self.bit5,
            bit6: self.bit6,
            bit7: self.bit7,
        }
    }
}

impl<
        Opt0: ValidOptionHstxBit0,
        Opt2: ValidOptionHstxBit2,
        Opt3: ValidOptionHstxBit3,
        Opt4: ValidOptionHstxBit4,
        Opt5: ValidOptionHstxBit5,
        Opt6: ValidOptionHstxBit6,
        Opt7: ValidOptionHstxBit7,
    > HstxPins<Opt0, OptionTNone, Opt2, Opt3, Opt4, Opt5, Opt6, Opt7>
{
    /// Add bit1 pin to HSTX pin set
    pub fn add_bit1_pin<Bit1: ValidHstxBit1Pin>(
        self,
        bit1: Bit1,
    ) -> HstxPins<Opt0, OptionTSome<Bit1>, Opt2, Opt3, Opt4, Opt5, Opt6, Opt7> {
        HstxPins {
            bit0: self.bit0,
            bit1: OptionTSome(bit1),
            bit2: self.bit2,
            bit3: self.bit3,
            bit4: self.bit4,
            bit5: self.bit5,
            bit6: self.bit6,
            bit7: self.bit7,
        }
    }
}

impl<
        Opt0: ValidOptionHstxBit0,
        Opt1: ValidOptionHstxBit1,
        Opt3: ValidOptionHstxBit3,
        Opt4: ValidOptionHstxBit4,
        Opt5: ValidOptionHstxBit5,
        Opt6: ValidOptionHstxBit6,
        Opt7: ValidOptionHstxBit7,
    > HstxPins<Opt0, Opt1, OptionTNone, Opt3, Opt4, Opt5, Opt6, Opt7>
{
    /// Add bit2 pin to HSTX pin set
    pub fn add_bit2_pin<Bit2: ValidHstxBit2Pin>(
        self,
        bit2: Bit2,
    ) -> HstxPins<Opt0, Opt1, OptionTSome<Bit2>, Opt3, Opt4, Opt5, Opt6, Opt7> {
        HstxPins {
            bit0: self.bit0,
            bit1: self.bit1,
            bit2: OptionTSome(bit2),
            bit3: self.bit3,
            bit4: self.bit4,
            bit5: self.bit5,
            bit6: self.bit6,
            bit7: self.bit7,
        }
    }
}

impl<
        Opt0: ValidOptionHstxBit0,
        Opt1: ValidOptionHstxBit1,
        Opt2: ValidOptionHstxBit2,
        Opt4: ValidOptionHstxBit4,
        Opt5: ValidOptionHstxBit5,
        Opt6: ValidOptionHstxBit6,
        Opt7: ValidOptionHstxBit7,
    > HstxPins<Opt0, Opt1, Opt2, OptionTNone, Opt4, Opt5, Opt6, Opt7>
{
    /// Add bit3 pin to HSTX pin set
    pub fn add_bit3_pin<Bit3: ValidHstxBit3Pin>(
        self,
        bit3: Bit3,
    ) -> HstxPins<Opt0, Opt1, Opt2, OptionTSome<Bit3>, Opt4, Opt5, Opt6, Opt7> {
        HstxPins {
            bit0: self.bit0,
            bit1: self.bit1,
            bit2: self.bit2,
            bit3: OptionTSome(bit3),
            bit4: self.bit4,
            bit5: self.bit5,
            bit6: self.bit6,
            bit7: self.bit7,
        }
    }
}

impl<
        Opt0: ValidOptionHstxBit0,
        Opt1: ValidOptionHstxBit1,
        Opt2: ValidOptionHstxBit2,
        Opt3: ValidOptionHstxBit3,
        Opt5: ValidOptionHstxBit5,
        Opt6: ValidOptionHstxBit6,
        Opt7: ValidOptionHstxBit7,
    > HstxPins<Opt0, Opt1, Opt2, Opt3, OptionTNone, Opt5, Opt6, Opt7>
{
    /// Add bit4 pin to HSTX pin set
    pub fn add_bit4_pin<Bit4: ValidHstxBit4Pin>(
        self,
        bit4: Bit4,
    ) -> HstxPins<Opt0, Opt1, Opt2, Opt3, OptionTSome<Bit4>, Opt5, Opt6, Opt7> {
        HstxPins {
            bit0: self.bit0,
            bit1: self.bit1,
            bit2: self.bit2,
            bit3: self.bit3,
            bit4: OptionTSome(bit4),
            bit5: self.bit5,
            bit6: self.bit6,
            bit7: self.bit7,
        }
    }
}

impl<
        Opt0: ValidOptionHstxBit0,
        Opt1: ValidOptionHstxBit1,
        Opt2: ValidOptionHstxBit2,
        Opt3: ValidOptionHstxBit3,
        Opt4: ValidOptionHstxBit4,
        Opt6: ValidOptionHstxBit6,
        Opt7: ValidOptionHstxBit7,
    > HstxPins<Opt0, Opt1, Opt2, Opt3, Opt4, OptionTNone, Opt6, Opt7>
{
    /// Add bit5 pin to HSTX pin set
    pub fn add_bit5_pin<Bit5: ValidHstxBit5Pin>(
        self,
        bit5: Bit5,
    ) -> HstxPins<Opt0, Opt1, Opt2, Opt3, Opt4, OptionTSome<Bit5>, Opt6, Opt7> {
        HstxPins {
            bit0: self.bit0,
            bit1: self.bit1,
            bit2: self.bit2,
            bit3: self.bit3,
            bit4: self.bit4,
            bit5: OptionTSome(bit5),
            bit6: self.bit6,
            bit7: self.bit7,
        }
    }
}

impl<
        Opt0: ValidOptionHstxBit0,
        Opt1: ValidOptionHstxBit1,
        Opt2: ValidOptionHstxBit2,
        Opt3: ValidOptionHstxBit3,
        Opt4: ValidOptionHstxBit4,
        Opt5: ValidOptionHstxBit5,
        Opt7: ValidOptionHstxBit7,
    > HstxPins<Opt0, Opt1, Opt2, Opt3, Opt4, Opt5, OptionTNone, Opt7>
{
    /// Add bit6 pin to HSTX pin set
    pub fn add_bit6_pin<Bit6: ValidHstxBit6Pin>(
        self,
        bit6: Bit6,
    ) -> HstxPins<Opt0, Opt1, Opt2, Opt3, Opt4, Opt5, OptionTSome<Bit6>, Opt7> {
        HstxPins {
            bit0: self.bit0,
            bit1: self.bit1,
            bit2: self.bit2,
            bit3: self.bit3,
            bit4: self.bit4,
            bit5: self.bit5,
            bit6: OptionTSome(bit6),
            bit7: self.bit7,
        }
    }
}

impl<
        Opt0: ValidOptionHstxBit0,
        Opt1: ValidOptionHstxBit1,
        Opt2: ValidOptionHstxBit2,
        Opt3: ValidOptionHstxBit3,
        Opt4: ValidOptionHstxBit4,
        Opt5: ValidOptionHstxBit5,
        Opt6: ValidOptionHstxBit6,
    > HstxPins<Opt0, Opt1, Opt2, Opt3, Opt4, Opt5, Opt6, OptionTNone>
{
    /// Add bit7 pin to HSTX pin set
    pub fn add_bit7_pin<Bit7: ValidHstxBit7Pin>(
        self,
        bit7: Bit7,
    ) -> HstxPins<Opt0, Opt1, Opt2, Opt3, Opt4, Opt5, Opt6, OptionTSome<Bit7>> {
        HstxPins {
            bit0: self.bit0,
            bit1: self.bit1,
            bit2: self.bit2,
            bit3: self.bit3,
            bit4: self.bit4,
            bit5: self.bit5,
            bit6: self.bit6,
            bit7: OptionTSome(bit7),
        }
    }
}
