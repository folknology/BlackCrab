//#![deny(unsafe_code)]
#![deny(warnings)]
#![no_main]
#![no_std]

extern crate panic_itm;

use cortex_m_rt::entry;
// use stm32f7::stm32f730 as pac;
use stm32f7::stm32f730::{interrupt, EXTI};
use stm32f7xx_hal::{pac, prelude::*};
use stm32f7xx_hal::gpio::{ExtiPin, Edge, Input, Floating};
use cortex_m::interrupt::{Mutex, free};
use core::cell::{Cell, RefCell};
use stm32f7xx_hal::gpio::gpiob::PB7;
use cortex_m::peripheral::NVIC;

// Syhchronisation
static SEMAPHORE: Mutex<Cell<bool>> = Mutex::new(Cell::new(true));
//Mutexed resource
static BUTTON_PIN: Mutex<RefCell<Option<PB7<Input<Floating>>>>> = Mutex::new(RefCell::new(None));

#[entry]
fn main() -> ! {
    // Take control of the peripherals
    //let cp = cortex_m::Peripherals::take().unwrap();
    let p = pac::Peripherals::take().unwrap();

    let mut exti :EXTI  = p.EXTI;
    let mut syscfg = p.SYSCFG;

    // Constrain clocking registers
    let mut rcc = p.RCC;

    // Grab the GPIOB Port and the mode/status leds on pins PB12/13
    let gpiob = p.GPIOB.split();
    let mut mode_led = gpiob.pb12.into_push_pull_output();
    let mut status_led = gpiob.pb13.into_push_pull_output();
    let mut mode_button = gpiob.pb7.into_floating_input();

    mode_button.make_interrupt_source(&mut syscfg, &mut rcc);
    mode_button.trigger_on_edge(&mut exti, Edge::Rising);
    mode_button.enable_interrupt(&mut exti);

    // Configure clock and freeze it
    let _clocks = rcc.constrain().cfgr.sysclk(216.mhz()).freeze();

    // Save info in global
    free(|cs| {
        BUTTON_PIN.borrow(cs).replace(Some(mode_button))
    });

    // Enable interrupt
    unsafe {
        NVIC::unmask(pac::Interrupt::EXTI9_5);
    }
    // Set status green led on
    status_led.set_high().ok();
    // Set mode amber led off
    mode_led.set_high().ok();

    loop {
        free(|cs| {
            if SEMAPHORE.borrow(cs).get() == false {
                if let Ok(true) = mode_led.is_high() {
                    mode_led.set_low().ok();
                } else {
                    mode_led.set_high().ok();
                }

                SEMAPHORE.borrow(cs).set(true);
            }
        })
    }
}

#[interrupt]
fn EXTI9_5() {
    free(|cs| {
        match BUTTON_PIN.borrow(cs).borrow_mut().as_mut() {
            Some(b) => b.clear_interrupt_pending_bit(),
            // Never here
            None => (),
        }
        SEMAPHORE.borrow(cs).set(false);
    })
}
