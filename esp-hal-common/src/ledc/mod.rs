//! LEDC (LED PWM Controller) peripheral control
//!
//! Currently only supports fixed-frequency output. Interrupts are not currently
//! implemented. High Speed channels are available for the ESP32 only, while Low
//! Speed channels are available for all supported chips.
//!
//! # LowSpeed Example:
//!
//! The following will configure the Low Speed Channel0 to 24kHz output with
//! 10% duty using the ABPClock
//!
//! ```rust,ignore
//! let mut ledc = LEDC::new(peripherals.LEDC, &mut system.peripheral_clock_control);
//! ledc.set_global_slow_clock(LSGlobalClkSource::APBClk);
//!
//! let mut lstimer0 = ledc.get_timer::<LowSpeed>(timer::Number::Timer0);
//! lstimer0
//! .configure(timer::config::Config {
//!            duty: timer::config::Duty::Duty5Bit,
//!            clock_source: timer::LSClockSource::APBClk,
//!            frequency: 24u32.kHz(),
//!        })
//!        .unwrap();
//!
//! let mut channel0 = ledc.get_channel(channel::Number::Channel0, led);
//! channel0
//!     .configure(channel::config::Config {
//!         timer: &lstimer0,
//!         duty: 10,
//!     })
//!     .unwrap();
//! ```
//!
//! # HighSpeed Example (ESP32 only):
//!
//! The following will configure the High Speed Channel0 to 24kHz output with
//! 10% duty using the ABPClock
//!
//! ```rust,ignore
//! let ledc = LEDC::new(peripherals.LEDC, &mut system.peripheral_clock_control);
//!
//! let mut hstimer0 = ledc.get_timer::<HighSpeed>(timer::Number::Timer0);
//! hstimer0
//! .configure(timer::config::Config {
//!            duty: timer::config::Duty::Duty5Bit,
//!            clock_source: timer::HSClockSource::APBClk,
//!            frequency: 24u32.kHz(),
//!        })
//!        .unwrap();
//!
//! let mut channel0 = ledc.get_channel(channel::Number::Channel0, led);
//! channel0
//!     .configure(channel::config::Config {
//!         timer: &hstimer0,
//!         duty: 10,
//!     })
//!     .unwrap();
//! ```
//!
//! # TODO
//!
//! - Source clock selection
//! - Interrupts

use self::{
    channel::Channel,
    timer::{Timer, TimerSpeed},
};
use crate::{
    clock::Clocks,
    gpio::OutputPin,
    peripheral::{Peripheral, PeripheralRef},
    system::{Peripheral as PeripheralEnable, PeripheralClockControl},
};

pub mod channel;
pub mod timer;

/// Global slow clock source
#[derive(PartialEq, Eq, Copy, Clone, Debug)]
pub enum LSGlobalClkSource {
    APBClk,
}

/// LEDC (LED PWM Controller)
pub struct LEDC<'d> {
    _instance: PeripheralRef<'d, crate::peripherals::LEDC>,
}

#[cfg(esp32)]
/// Used to specify HighSpeed Timer/Channel
pub struct HighSpeed {}

/// Used to specify LowSpeed Timer/Channel
pub struct LowSpeed {}

pub trait Speed {}

#[cfg(esp32)]
impl Speed for HighSpeed {}

impl Speed for LowSpeed {}

impl<'d> LEDC<'d> {
    /// Return a new LEDC
    pub fn new(
        _instance: impl Peripheral<P = crate::peripherals::LEDC> + 'd,
        system: &mut PeripheralClockControl,
    ) -> Self {
        crate::into_ref!(_instance);
        system.enable(PeripheralEnable::Ledc);

        LEDC {
            _instance,
        }
    }

    /// Set global slow clock source
    #[cfg(esp32)]
    pub fn set_global_slow_clock(&mut self, _clock_source: LSGlobalClkSource) {
        let ledc = unsafe { &*crate::peripherals::LEDC::PTR };
        ledc.conf.write(|w| w.apb_clk_sel().set_bit());
        ledc.lstimer0_conf.modify(|_, w| w.para_up().set_bit());
    }

    #[cfg(not(esp32))]
    /// Set global slow clock source
    pub fn set_global_slow_clock(&mut self, clock_source: LSGlobalClkSource) {
        let ledc = unsafe { &*crate::peripherals::LEDC::PTR };

        #[cfg(any(esp32c6, esp32h2))]
        let pcr = unsafe { &*crate::peripherals::PCR::ptr() };

        #[cfg(any(esp32c6, esp32h2))]
        pcr.ledc_sclk_conf.write(|w| w.ledc_sclk_en().set_bit());

        match clock_source {
            LSGlobalClkSource::APBClk => {
                #[cfg(not(any(esp32c6, esp32h2)))]
                ledc.conf.write(|w| unsafe { w.apb_clk_sel().bits(1) });
                #[cfg(esp32c6)]
                pcr.ledc_sclk_conf
                    .write(|w| unsafe { w.ledc_sclk_sel().bits(1) });
                #[cfg(esp32h2)]
                pcr.ledc_sclk_conf
                    .write(|w| unsafe { w.ledc_sclk_sel().bits(0) });
            }
        }
        ledc.timer0_conf.modify(|_, w| w.para_up().set_bit());
    }

    /// Return a new timer
    pub fn get_timer<S: TimerSpeed>(&self, number: timer::Number) -> Timer<S> {
        Timer::new(number)
    }

    /// Return a new channel
    pub fn get_channel<S: TimerSpeed, O: OutputPin>(
        &self,
        number: channel::Number,
        output_pin: impl Peripheral<P = O> + 'd,
    ) -> Channel<S, O> {
        Channel::new(number, output_pin)
    }
}
