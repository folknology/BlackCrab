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
    let cp = cortex_m::Peripherals::take().unwrap();
    let p = pac::Peripherals::take().unwrap();

    let gpiob = p.GPIOB.split();
    let mut led = gpiob.pb12.into_push_pull_output();

    // Constrain clocking registers
    let rcc = p.RCC.constrain();

    // Configure clock and freeze it
    let clocks = rcc.cfgr.sysclk(216.mhz()).freeze();

    // Get delay provider
    let mut delay = Delay::new(cp.SYST, clocks);

    loop {
        led.set_high().expect("GPIO can never fail");
        delay.delay_ms(100_u16);

        led.set_low().expect("GPIO can never fail");
        delay.delay_ms(100_u16);
    }
}