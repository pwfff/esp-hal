//! Uses timer0 and operator0 of the MCPWM0
//!
//!  to output a 50% duty
//! signal at 16 kHz.
//!
//! The signal will be output to the pin assigned to `pin`. (GPIO4)

#![no_std]
#![no_main]

use esp32h2_hal::{
    clock::ClockControl,
    gpio::IO,
    mcpwm::{operator::PwmPinConfig, timer::PwmWorkingMode, PeripheralClockConfig, MCPWM},
    peripherals::Peripherals,
    prelude::*,
    timer::TimerGroup,
    Rtc,
};
use esp_backtrace as _;

#[entry]
fn main() -> ! {
    let peripherals = Peripherals::take();
    let mut system = peripherals.PCR.split();
    let clocks = ClockControl::boot_defaults(system.clock_control).freeze();

    // Disable the watchdog timers. For the ESP32-H2, this includes the Super WDT,
    // and the TIMG WDTs.
    let mut rtc = Rtc::new(peripherals.LP_CLKRST);
    let timer_group0 = TimerGroup::new(
        peripherals.TIMG0,
        &clocks,
        &mut system.peripheral_clock_control,
    );
    let mut wdt0 = timer_group0.wdt;
    let timer_group1 = TimerGroup::new(
        peripherals.TIMG1,
        &clocks,
        &mut system.peripheral_clock_control,
    );
    let mut wdt1 = timer_group1.wdt;

    // Disable watchdog timers
    rtc.swd.disable();
    rtc.rwdt.disable();
    wdt0.disable();
    wdt1.disable();

    let io = IO::new(peripherals.GPIO, peripherals.IO_MUX);
    let pin = io.pins.gpio4;

    // initialize peripheral
    let clock_cfg = PeripheralClockConfig::with_frequency(&clocks, 32u32.MHz()).unwrap();
    let mut mcpwm = MCPWM::new(
        peripherals.MCPWM0,
        clock_cfg,
        &mut system.peripheral_clock_control,
    );

    // connect operator0 to timer0
    mcpwm.operator0.set_timer(&mcpwm.timer0);
    // connect operator0 to pin
    let mut pwm_pin = mcpwm
        .operator0
        .with_pin_a(pin, PwmPinConfig::UP_ACTIVE_HIGH);

    // start timer with timestamp values in the range of 0..=99 and a frequency of
    // 20 kHz
    let timer_clock_cfg = clock_cfg
        .timer_clock_with_frequency(99, PwmWorkingMode::Increase, 20u32.kHz())
        .unwrap();
    mcpwm.timer0.start(timer_clock_cfg);

    // pin will be high 50% of the time
    pwm_pin.set_timestamp(50);

    loop {}
}
