#![deny(unsafe_code)]
#![deny(warnings)]
#![no_main]
#![no_std]

extern crate panic_itm;

use cortex_m_rt::entry;
use stm32f7::stm32f730 as pac;
use stm32f7xx_hal::{delay::Delay, prelude::*};

#[entry]
fn main() -> ! {
    // Take control of the peripherals
    let cp = cortex_m::Peripherals::take().unwrap();
    let p = pac::Peripherals::take().unwrap();

    // Grab the GPIOB Port and the mode/status leds on pins PB12/13
    let gpiob = p.GPIOB.split();
    let mut mode_led = gpiob.pb12.into_push_pull_output();
    let mut status_led = gpiob.pb13.into_push_pull_output();

    // Constrain clocking registers
    let rcc = p.RCC.constrain();

    // Configure clock and freeze it
    let clocks = rcc.cfgr.sysclk(216.mhz()).freeze();

    // Get delay provider
    let mut delay = Delay::new(cp.SYST, clocks);

    loop {
        // Alternate the mode and status leds
        mode_led.set_high().ok();
        status_led.set_low().ok();
        delay.delay_ms(200_u16);

        mode_led.set_low().ok();
        status_led.set_high().ok();
        delay.delay_ms(200_u16);
    }
}