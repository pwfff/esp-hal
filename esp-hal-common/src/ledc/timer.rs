use fugit::HertzU32;

#[cfg(esp32)]
use super::HighSpeed;
use super::{LowSpeed, Speed};
use crate::{clock::Clocks, peripherals::ledc};

const LEDC_TIMER_DIV_NUM_MAX: u64 = 0x3FFFF;

/// Timer errors
#[derive(Debug)]
pub enum Error {
    /// Invalid Divisor
    Divisor,
}

#[cfg(esp32)]
/// Clock source for HS Timers
#[derive(PartialEq, Eq, Copy, Clone, Debug)]
pub enum HSClockSource {
    APBClk,
    // TODO RefTick,
}

/// Clock source for LS Timers
#[derive(PartialEq, Eq, Copy, Clone, Debug)]
pub enum LSClockSource {
    APBClk,
    // TODO SLOWClk
}

/// Timer number
#[derive(PartialEq, Eq, Copy, Clone, Debug)]
pub enum Number {
    Timer0,
    Timer1,
    Timer2,
    Timer3,
}

/// Timer configuration
pub mod config {
    use fugit::HertzU32;

    /// Number of bits reserved for duty cycle adjustment
    #[derive(PartialEq, Eq, Copy, Clone, Debug)]
    pub enum Duty {
        Duty1Bit = 1,
        Duty2Bit,
        Duty3Bit,
        Duty4Bit,
        Duty5Bit,
        Duty6Bit,
        Duty7Bit,
        Duty8Bit,
        Duty9Bit,
        Duty10Bit,
        Duty11Bit,
        Duty12Bit,
        Duty13Bit,
        Duty14Bit,
        #[cfg(esp32)]
        Duty15Bit,
        #[cfg(esp32)]
        Duty16Bit,
        #[cfg(esp32)]
        Duty17Bit,
        #[cfg(esp32)]
        Duty18Bit,
        #[cfg(esp32)]
        Duty19Bit,
        #[cfg(esp32)]
        Duty20Bit,
    }

    /// Timer configuration
    #[derive(Copy, Clone)]
    pub struct Config<CS> {
        pub duty: Duty,
        pub clock_source: CS,
        pub frequency: HertzU32,
    }
}

/// Trait defining the type of timer source
pub trait TimerSpeed: Speed {
    type ClockSourceType: Sync;
}

/// Timer source type for LowSpeed timers
impl TimerSpeed for LowSpeed {
    type ClockSourceType = LSClockSource;
}

#[cfg(esp32)]
/// Timer source type for HighSpeed timers
impl TimerSpeed for HighSpeed {
    type ClockSourceType = HSClockSource;
}

/// Interface for Timers
pub trait TimerIFace<S: TimerSpeed>: Sync {
    /// Return the frequency of the timer
    fn get_freq(&self, clocks: &Clocks) -> Option<HertzU32>;

    /// Configure the timer
    fn configure(&mut self, clocks: &Clocks, config: config::Config<S::ClockSourceType>) -> Result<(), Error>;

    /// Check if the timer has been configured
    fn is_configured(&self) -> bool;

    /// Return the duty resolution of the timer
    fn get_duty(&self) -> Option<config::Duty>;

    /// Return the timer number
    fn get_number(&self) -> Number;

    /// Return the timer frequency, or 0 if not configured
    fn get_frequency(&self) -> u32;
}

/// Interface for HW configuration of timer
pub trait TimerHW<S: TimerSpeed> {
    /// Get the current source timer frequency from the HW
    fn get_freq_hw(&self, clocks: &Clocks) -> Option<HertzU32>;

    /// Configure the HW for the timer
    fn configure_hw(&self, divisor: u32);

    /// Update the timer in HW
    fn update_hw(&self);
}

/// Timer struct
pub struct Timer<S: TimerSpeed> {
    number: Number,
    duty: Option<config::Duty>,
    frequency: u32,
    configured: bool,
    use_ref_tick: bool,
    clock_source: Option<S::ClockSourceType>,
}

impl<S: TimerSpeed> TimerIFace<S> for Timer<S>
where
    Timer<S>: TimerHW<S>,
{
    /// Return the frequency of the timer
    fn get_freq(&self, clocks: &Clocks) -> Option<HertzU32> {
        self.get_freq_hw(clocks)
    }

    /// Configure the timer
    fn configure(&mut self, clocks: &Clocks, config: config::Config<S::ClockSourceType>) -> Result<(), Error> {
        self.duty = Some(config.duty);
        self.clock_source = Some(config.clock_source);

        // TODO: we should return some error here if `unwrap()` fails
        let src_freq: u32 = self.get_freq(clocks).unwrap().to_Hz();
        let precision = 1 << config.duty as u32;
        let frequency: u32 = config.frequency.raw();
        self.frequency = frequency;

        let mut divisor = ((src_freq as u64) << 8) / frequency as u64 / precision as u64;

        if divisor > LEDC_TIMER_DIV_NUM_MAX {
            // APB_CLK results in divisor which too high. Try using REF_TICK as clock
            // source.
            self.use_ref_tick = true;
            divisor = ((1_000_000 as u64) << 8) / frequency as u64 / precision as u64;
        }

        if divisor >= LEDC_TIMER_DIV_NUM_MAX || divisor < 256 {
            return Err(Error::Divisor);
        }

        self.configure_hw(divisor as u32);
        self.update_hw();

        self.configured = true;

        Ok(())
    }

    /// Check if the timer has been configured
    fn is_configured(&self) -> bool {
        self.configured
    }

    /// Return the duty resolution of the timer
    fn get_duty(&self) -> Option<config::Duty> {
        self.duty
    }

    /// Return the timer number
    fn get_number(&self) -> Number {
        self.number
    }

    /// Return the timer frequency
    fn get_frequency(&self) -> u32 {
        self.frequency
    }
}

impl<S: TimerSpeed> Timer<S> {
    /// Create a new intance of a timer
    pub fn new(
        number: Number,
    ) -> Self {
        Timer {
            number,
            duty: None,
            frequency: 0u32,
            configured: false,
            use_ref_tick: false,
            clock_source: None,
        }
    }
}

/// Timer HW implementation for LowSpeed timers
impl TimerHW<LowSpeed> for Timer<LowSpeed> {
    /// Get the current source timer frequency from the HW
    fn get_freq_hw(&self, clocks: &Clocks) -> Option<fugit::HertzU32> {
        self.clock_source.map(|cs| match cs {
            LSClockSource::APBClk => clocks.apb_clock,
        })
    }

    #[cfg(esp32)]
    /// Configure the HW for the timer
    fn configure_hw(&self, divisor: u32) {
        let duty = self.duty.unwrap() as u8;
        let use_apb = !self.use_ref_tick;
        let ledc = unsafe { &*crate::peripherals::LEDC::PTR };

        match self.number {
            Number::Timer0 => ledc.lstimer0_conf.modify(|_, w| unsafe {
                w.tick_sel()
                    .bit(use_apb)
                    .rst()
                    .clear_bit()
                    .pause()
                    .clear_bit()
                    .div_num()
                    .bits(divisor)
                    .duty_res()
                    .bits(duty)
            }),
            Number::Timer1 => ledc.lstimer1_conf.modify(|_, w| unsafe {
                w.tick_sel()
                    .bit(use_apb)
                    .rst()
                    .clear_bit()
                    .pause()
                    .clear_bit()
                    .div_num()
                    .bits(divisor)
                    .duty_res()
                    .bits(duty)
            }),
            Number::Timer2 => ledc.lstimer2_conf.modify(|_, w| unsafe {
                w.tick_sel()
                    .bit(use_apb)
                    .rst()
                    .clear_bit()
                    .pause()
                    .clear_bit()
                    .div_num()
                    .bits(divisor)
                    .duty_res()
                    .bits(duty)
            }),
            Number::Timer3 => ledc.lstimer3_conf.modify(|_, w| unsafe {
                w.tick_sel()
                    .bit(use_apb)
                    .rst()
                    .clear_bit()
                    .pause()
                    .clear_bit()
                    .div_num()
                    .bits(divisor)
                    .duty_res()
                    .bits(duty)
            }),
        };
    }

    #[cfg(not(esp32))]
    /// Configure the HW for the timer
    fn configure_hw(&self, divisor: u32) {
        let duty = self.duty.unwrap() as u8;
        let use_ref_tick = self.use_ref_tick;
        let ledc = unsafe { &*crate::peripherals::LEDC::PTR };

        match self.number {
            Number::Timer0 => ledc.timer0_conf.modify(|_, w| unsafe {
                w.tick_sel()
                    .bit(use_ref_tick)
                    .rst()
                    .clear_bit()
                    .pause()
                    .clear_bit()
                    .clk_div()
                    .bits(divisor)
                    .duty_res()
                    .bits(duty)
            }),
            Number::Timer1 => ledc.timer1_conf.modify(|_, w| unsafe {
                w.tick_sel()
                    .bit(use_ref_tick)
                    .rst()
                    .clear_bit()
                    .pause()
                    .clear_bit()
                    .clk_div()
                    .bits(divisor)
                    .duty_res()
                    .bits(duty)
            }),
            Number::Timer2 => ledc.timer2_conf.modify(|_, w| unsafe {
                w.tick_sel()
                    .bit(use_ref_tick)
                    .rst()
                    .clear_bit()
                    .pause()
                    .clear_bit()
                    .clk_div()
                    .bits(divisor)
                    .duty_res()
                    .bits(duty)
            }),
            Number::Timer3 => ledc.timer3_conf.modify(|_, w| unsafe {
                w.tick_sel()
                    .bit(use_ref_tick)
                    .rst()
                    .clear_bit()
                    .pause()
                    .clear_bit()
                    .clk_div()
                    .bits(divisor)
                    .duty_res()
                    .bits(duty)
            }),
        };
    }

    #[cfg(esp32)]
    /// Update the timer in HW
    fn update_hw(&self) {
        let ledc = unsafe { &*crate::peripherals::LEDC::PTR };
        match self.number {
            Number::Timer0 => ledc.lstimer0_conf.modify(|_, w| w.para_up().set_bit()),
            Number::Timer1 => ledc.lstimer1_conf.modify(|_, w| w.para_up().set_bit()),
            Number::Timer2 => ledc.lstimer2_conf.modify(|_, w| w.para_up().set_bit()),
            Number::Timer3 => ledc.lstimer3_conf.modify(|_, w| w.para_up().set_bit()),
        };
    }

    #[cfg(not(esp32))]
    /// Update the timer in HW
    fn update_hw(&self) {
        let ledc = unsafe { &*crate::peripherals::LEDC::PTR };
        match self.number {
            Number::Timer0 => ledc.timer0_conf.modify(|_, w| w.para_up().set_bit()),
            Number::Timer1 => ledc.timer1_conf.modify(|_, w| w.para_up().set_bit()),
            Number::Timer2 => ledc.timer2_conf.modify(|_, w| w.para_up().set_bit()),
            Number::Timer3 => ledc.timer3_conf.modify(|_, w| w.para_up().set_bit()),
        };
    }
}

#[cfg(esp32)]
/// Timer HW implementation for HighSpeed timers
impl TimerHW<HighSpeed> for Timer<HighSpeed> {
    /// Get the current source timer frequency from the HW
    fn get_freq_hw(&self, clocks: &Clocks) -> Option<HertzU32> {
        self.clock_source.map(|cs| match cs {
            // TODO RefTick HSClockSource::RefTick => clocks.apb_clock,
            HSClockSource::APBClk => clocks.apb_clock,
        })
    }

    /// Configure the HW for the timer
    fn configure_hw(&self, divisor: u32) {
        let duty = self.duty.unwrap() as u8;
        let sel_hstimer = self.clock_source == Some(HSClockSource::APBClk);
        let ledc = unsafe { &*crate::peripherals::LEDC::PTR };

        match self.number {
            Number::Timer0 => ledc.hstimer0_conf.modify(|_, w| unsafe {
                w.tick_sel()
                    .bit(sel_hstimer)
                    .rst()
                    .clear_bit()
                    .pause()
                    .clear_bit()
                    .div_num()
                    .bits(divisor)
                    .duty_res()
                    .bits(duty)
            }),
            Number::Timer1 => ledc.hstimer1_conf.modify(|_, w| unsafe {
                w.tick_sel()
                    .bit(sel_hstimer)
                    .rst()
                    .clear_bit()
                    .pause()
                    .clear_bit()
                    .div_num()
                    .bits(divisor)
                    .duty_res()
                    .bits(duty)
            }),
            Number::Timer2 => ledc.hstimer2_conf.modify(|_, w| unsafe {
                w.tick_sel()
                    .bit(sel_hstimer)
                    .rst()
                    .clear_bit()
                    .pause()
                    .clear_bit()
                    .div_num()
                    .bits(divisor)
                    .duty_res()
                    .bits(duty)
            }),
            Number::Timer3 => ledc.hstimer3_conf.modify(|_, w| unsafe {
                w.tick_sel()
                    .bit(sel_hstimer)
                    .rst()
                    .clear_bit()
                    .pause()
                    .clear_bit()
                    .div_num()
                    .bits(divisor)
                    .duty_res()
                    .bits(duty)
            }),
        };
    }

    /// Update the timer in HW
    fn update_hw(&self) {
        // Nothing to do for HS timers
    }
}
