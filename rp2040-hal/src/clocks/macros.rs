macro_rules! clocks {
    (
        $(
            $(#[$attr:meta])*
            struct $name:ident {
                init_freq: $init_freq:expr,
                reg: $reg:ident,
                $(src: {$($src:ident: $src_variant:ident),*},)?
                auxsrc: {$($auxsrc:ident: $aux_variant:ident),*}
                $(, div: $div:tt)?
            }
        )*

    ) => {

        $crate::paste::paste!{
            /// Abstraction layer providing Clock Management.
            pub struct ClocksManager {
                clocks: CLOCKS,
                $(
                    #[doc = "`" $name "` field"]
                    pub [<$name:snake>]: $name,
                )*
            }

            impl ClocksManager {
                /// Exchanges CLOCKS block against Self.
                pub fn new(mut clocks_block: CLOCKS) -> Self {
                    // Disable resus that may be enabled from previous software
                    unsafe {
                        clocks_block.clk_sys_resus_ctrl.write_with_zero(|w| w);
                    }

                    let shared_clocks = ShareableClocks::new(&mut clocks_block);
                    ClocksManager {
                        clocks: clocks_block,
                        $(
                            [<$name:snake>]: $name {
                                shared_dev: shared_clocks,
                                frequency: $init_freq.Hz(),
                            },
                        )*
                    }
                }
            }
        }

        $(
            clock!(
                $(#[$attr])*
                struct $name {
                    reg: $reg,
                    $(src: {$($src: $src_variant),*},)?
                    auxsrc: {$($auxsrc: $aux_variant),*}
                    $(, div: $div )?
                }
            );
        )*
    };
}

macro_rules! clock {
    {
        $(#[$attr:meta])*
        struct $name:ident {
            reg: $reg:ident,
            src: {$($src:ident: $src_variant:ident),*},
            auxsrc: {$($auxsrc:ident: $aux_variant:ident),*}
        }
     } => {
        base_clock!{
            $(#[$attr])*
            ($name, $reg, auxsrc={$($auxsrc: $aux_variant),*})
        }

        divisable_clock!($name, $reg);

        $crate::paste::paste!{
            $(impl ValidSrc<$name> for $src {
                fn is_aux(&self) -> bool{
                    false
                }
                fn variant(&self) -> [<$reg:camel SrcType>] {
                    [<$reg:camel SrcType>]::Src(pac::clocks::[<$reg _ctrl>]::SRC_A::$src_variant)
                }
            })*

            impl GlitchlessClock for $name {
                type Clock = Self;

                fn await_select(&self, clock_token: &ChangingClockToken<Self>) -> nb::Result<(), Infallible> {
                    let shared_dev = unsafe { self.shared_dev.get() };

                    let selected = shared_dev.[<$reg _selected>].read().bits();
                    if selected != 1 << clock_token.clock_nr {
                        return Err(nb::Error::WouldBlock);
                    }

                    Ok(())
                }
            }

            #[doc = "Holds register value for ClockSource for `"$name"`"]
            pub enum [<$reg:camel SrcType>] {
                #[doc = "Contains a valid clock source register value that is to be used to set a clock as glitchless source for `"$name"`"]
                Src(pac::clocks::[<$reg _ctrl>]::SRC_A),
                #[doc = "Contains a valid clock source register value that is to be used to set a clock as aux source for `"$name"`"]
                Aux(pac::clocks::[<$reg _ctrl>]::AUXSRC_A)
            }

            impl [<$reg:camel SrcType>] {
                fn get_clock_id(&self) -> u8 {
                    match self {
                        Self::Src(v) => *v as u8,
                        Self::Aux(v) => *v as u8,
                    }
                }

                fn unwrap_src(&self) -> pac::clocks::[<$reg _ctrl>]::SRC_A{
                    match self {
                        Self::Src(v) => *v,
                        Self::Aux(_) => panic!(),
                    }
                }

                fn unwrap_aux(&self) -> pac::clocks::[<$reg _ctrl>]::AUXSRC_A {
                    match self {
                        Self::Src(_) => panic!(),
                        Self::Aux(v) => *v
                    }
                }
            }

            impl $name {
                /// Reset clock back to its reset source
                pub fn reset_source_await(&mut self) -> nb::Result<(), Infallible> {
                    let shared_dev = unsafe { self.shared_dev.get() };

                    shared_dev.[<$reg _ctrl>].modify(|_, w| {
                        w.src().variant(self.get_default_clock_source())
                    });

                    self.frequency = 12_000_000.Hz(); //TODO Get actual clock source.. Most likely 12 MHz though

                    self.await_select(&ChangingClockToken{clock_nr:0, clock: PhantomData::<Self>})
                }

                fn set_src<S: ValidSrc<$name>>(&mut self, src: &S)-> ChangingClockToken<$name> {
                    let shared_dev = unsafe { self.shared_dev.get() };

                    shared_dev.[<$reg _ctrl>].modify(|_,w| {
                        w.src().variant(src.variant().unwrap_src())
                    });

                    ChangingClockToken {
                        clock: PhantomData::<$name>,
                        clock_nr: src.variant().get_clock_id(),
                    }
                }

                fn set_self_aux_src(&mut self) -> ChangingClockToken<$name> {
                    unsafe { self.shared_dev.get() }.[<$reg _ctrl>].modify(|_, w| {
                        w.src().variant(self.get_aux_source())
                    });

                    ChangingClockToken{
                        clock: PhantomData::<$name>,
                        clock_nr: pac::clocks::clk_ref_ctrl::SRC_A::CLKSRC_CLK_REF_AUX as u8,
                    }
                }
            }

            impl Clock for $name {
                type Variant = [<$reg:camel SrcType>];

                #[doc = "Get operating frequency for `"$name"`"]
                fn freq(&self) -> Hertz {
                    self.frequency
                }

                #[doc = "Configure `"$name"`"]
                fn configure_clock<S: ValidSrc<$name>>(&mut self, src: &S, freq: Hertz) -> Result<(), ClockError>{
                    let src_freq: Hertz<u32> = src.get_freq().into();

                    if freq.gt(&src_freq){
                        return Err(ClockError::CantIncreaseFreq);
                    }

                    let div = fractional_div(src_freq.integer(), freq.integer()).ok_or(ClockError::FrequencyTooLow)?;

                    // If increasing divisor, set divisor before source. Otherwise set source
                    // before divisor. This avoids a momentary overspeed when e.g. switching
                    // to a faster source and increasing divisor to compensate.
                    if div > self.get_div() {
                        self.set_div(div);
                    }

                    // If switching a glitchless slice (ref or sys) to an aux source, switch
                    // away from aux *first* to avoid passing glitches when changing aux mux.
                    // Assume (!!!) glitchless source 0 is no faster than the aux source.
                    nb::block!(self.reset_source_await()).unwrap();


                    // Set aux mux first, and then glitchless mux if this self has one
                    let token = if src.is_aux() {
                        self.set_aux(src);
                        self.set_self_aux_src()
                    } else {
                        self.set_src(src)
                    };

                    nb::block!(self.await_select(&token)).unwrap();


                    // Now that the source is configured, we can trust that the user-supplied
                    // divisor is a safe value.
                    self.set_div(div);

                    // Store the configured frequency
                    self.frequency = fractional_div(src_freq.integer(), div).ok_or(ClockError::FrequencyTooHigh)?.Hz();

                    Ok(())
                }
            }
        }
    };
    {
        $( #[$attr:meta])*
        struct $name:ident {
            reg: $reg:ident,
            auxsrc: {$($auxsrc:ident: $variant:ident),*},
            div: false
        }
    } => {
        base_clock!{
            $(#[$attr])*
            ($name, $reg, auxsrc={$($auxsrc: $variant),*})
        }

        // Just to match proper divisible clocks so we don't have to do something special in configure function
        impl ClockDivision for $name {
            fn set_div(&mut self, _: u32) {}
            fn get_div(&self) -> u32 {1}
        }

        stoppable_clock!($name, $reg);
    };
    {
        $( #[$attr:meta])*
        struct $name:ident {
            reg: $reg:ident,
            auxsrc: {$($auxsrc:ident: $variant:ident),*}
        }
    } => {
        base_clock!{
            $(#[$attr])*
            ($name, $reg, auxsrc={$($auxsrc: $variant),*})
        }

        divisable_clock!($name, $reg);
        stoppable_clock!($name, $reg);
    };
}

macro_rules! divisable_clock {
    ($name:ident, $reg:ident) => {
        $crate::paste::paste! {
            impl ClockDivision for $name {
                fn set_div(&mut self, div: u32) {
                    unsafe { self.shared_dev.get() }.[<$reg _div>].modify(|_, w| unsafe {
                        w.bits(div);
                        w
                    });
                }
                fn get_div(&self) -> u32 {
                    unsafe { self.shared_dev.get() }.[<$reg _div>].read().bits()
                }
            }
        }
    };
}

macro_rules! stoppable_clock {
    ($name:ident, $reg:ident) => {
        $crate::paste::paste!{
            #[doc = "Holds register value for ClockSource for `"$name"`"]
            pub enum [<$reg:camel SrcType>] {
                #[doc = "Contains a valid clock source register value that is to be used to set a clock as aux source for `"$name"`"]
                Aux(pac::clocks::[<$reg _ctrl>]::AUXSRC_A)
            }

            impl [<$reg:camel SrcType>] {
                fn unwrap_aux(&self) -> pac::clocks::[<$reg _ctrl>]::AUXSRC_A {
                   match self {
                       Self::Aux(v) => *v
                   }
                }
            }

            impl StoppableClock for $name {
                /// Enable the clock
                fn enable(&mut self) {
                    unsafe { self.shared_dev.get() }.[<$reg _ctrl>].modify(|_, w| {
                        w.enable().set_bit()
                    });
                }

                /// Disable the clock cleanly
                fn disable(&mut self) {
                    unsafe { self.shared_dev.get() }.[<$reg _ctrl>].modify(|_, w| {
                        w.enable().clear_bit()
                    });
                }

                /// Disable the clock asynchronously
                fn kill(&mut self) {
                    unsafe { self.shared_dev.get() }.[<$reg _ctrl>].modify(|_, w| {
                        w.kill().set_bit()
                    });
                }
            }

            impl Clock for $name {
                type Variant = [<$reg:camel SrcType>];

                #[doc = "Get operating frequency for `"$name"`"]
                fn freq(&self) -> Hertz {
                    self.frequency
                }

                #[doc = "Configure `"$name"`"]
                fn configure_clock<S: ValidSrc<$name>>(&mut self, src: &S, freq: Hertz) -> Result<(), ClockError>{
                    let src_freq: Hertz<u32> = src.get_freq().into();

                    if freq.gt(&src_freq){
                        return Err(ClockError::CantIncreaseFreq);
                    }

                    let div = fractional_div(src_freq.integer(), freq.integer()).ok_or(ClockError::FrequencyTooLow)?;

                    // If increasing divisor, set divisor before source. Otherwise set source
                    // before divisor. This avoids a momentary overspeed when e.g. switching
                    // to a faster source and increasing divisor to compensate.
                    if div > self.get_div() {
                        self.set_div(div);
                    }

                    // If no glitchless mux, cleanly stop the clock to avoid glitches
                    // propagating when changing aux mux. Note it would be a really bad idea
                    // to do this on one of the glitchless clocks (clk_sys, clk_ref).

                    // Disable clock. On clk_ref and clk_sys this does nothing,
                    // all other clocks have the ENABLE bit in the same position.
                    self.disable();
                    if (self.frequency > 0u32.Hz()) {
                        // Delay for 3 cycles of the target clock, for ENABLE propagation.
                        // Note XOSC_COUNT is not helpful here because XOSC is not
                        // necessarily running, nor is timer... so, 3 cycles per loop:
                        let sys_freq = 125_000_000; // TODO get actual sys_clk frequency
                        let delay_cyc = sys_freq / self.frequency.integer() + 1u32;
                        cortex_m::asm::delay(delay_cyc);
                    }

                    // Set aux mux first, and then glitchless mux if this self has one
                    self.set_aux(src);

                    // Enable clock. On clk_ref and clk_sys this does nothing,
                    // all other clocks have the ENABLE bit in the same posi
                    self.enable();

                    // Now that the source is configured, we can trust that the user-supplied
                    // divisor is a safe value.
                    self.set_div(div);

                    // Store the configured frequency
                    self.frequency = fractional_div(src_freq.integer(), div).ok_or(ClockError::FrequencyTooHigh)?.Hz();

                    Ok(())
                }
            }
        }
    };
}

macro_rules! base_clock {
    {
        $(#[$attr:meta])*
        ($name:ident, $reg:ident, auxsrc={$($auxsrc:ident: $variant:ident),*})
    } => {
        $crate::paste::paste!{

            $(impl ValidSrc<$name> for $auxsrc {

                fn is_aux(&self) -> bool{
                    true
                }
                fn variant(&self) -> [<$reg:camel SrcType>] {
                    [<$reg:camel SrcType>]::Aux(pac::clocks::[<$reg _ctrl>]::AUXSRC_A::$variant)
                }
            })*

            $(#[$attr])*
            pub struct $name {
                shared_dev: ShareableClocks,
                frequency: Hertz,
            }

            impl $name {
                fn set_aux<S: ValidSrc<$name>>(&mut self, src: &S) {
                    let shared_dev = unsafe { self.shared_dev.get() };

                    shared_dev.[<$reg _ctrl>].modify(|_,w| {
                        w.auxsrc().variant(src.variant().unwrap_aux())
                    });
                }
            }

            impl Sealed for $name {}

            impl From<$name> for Hertz
             {
                fn from(value: $name) -> Hertz {
                    value.frequency
                }
            }
        }
    };
}
