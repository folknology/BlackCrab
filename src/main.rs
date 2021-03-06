//#![deny(unsafe_code)]
#![feature(maybe_uninit_extra, maybe_uninit_ref)]
#![deny(warnings)]
#![no_main]
#![no_std]

extern crate panic_itm;

use cortex_m_rt::entry;
// use stm32f7::stm32f730 as pac;
use stm32f7::stm32f730::{interrupt, EXTI};
use stm32f7xx_hal::{pac, prelude::*};
use stm32f7xx_hal::otg_fs::{UsbBus, USB};
use stm32f7::stm32f730::Interrupt;
use stm32f7xx_hal::rcc::{HSEClock, HSEClockMode};
use stm32f7xx_hal::gpio::{ExtiPin, Edge, Input, Floating};
use cortex_m::interrupt::{Mutex, free};
use stm32f7xx_hal::gpio::gpiob::PB7;
use cortex_m::peripheral::NVIC;
use usb_device::prelude::*;
use usbd_serial::SerialPort;
use usb_device::class_prelude::UsbBusAllocator;
use core::cell::{Cell, RefCell};
use core::mem::MaybeUninit;

static mut EP_MEMORY: [u32; 1024] = [0; 1024];

static mut USB_BUS: MaybeUninit<UsbBusAllocator<UsbBus<USB>>> = MaybeUninit::uninit();
static mut USB_SERIAL: Option<UsbSerial> = None;

pub struct UsbSerial {
    serial: SerialPort<'static, UsbBus<USB>>,
    device: UsbDevice<'static, UsbBus<USB>>,
}

impl UsbSerial {
    pub fn setup(usb: USB) {
        unsafe { USB_BUS.write(UsbBus::new(usb, &mut EP_MEMORY)) };
        let bus = unsafe { USB_BUS.assume_init_ref() };
        let serial = SerialPort::new(bus);
        let device = UsbDeviceBuilder::new(bus, UsbVidPid(0x16c0, 0x27dd))
            .manufacturer("myStorm")
            .product("IceCore")
            .serial_number("1234")
            .device_class(usbd_serial::USB_CLASS_CDC)
            .max_packet_size_0(64)
            .device_release(0x20)
            .self_powered(true)
            .build();

        free(|_| {
            unsafe { USB_SERIAL = Some(UsbSerial { serial, device}); }
        });

        unsafe {
            NVIC::unmask(Interrupt::OTG_FS);
        }
    }

    pub fn get() -> Option<&'static mut Self> {
        unsafe { USB_SERIAL.as_mut() }
    }
    pub fn poll() {
        if let Some(ref mut s) = Self::get() {
            if s.device.poll(&mut [&mut s.serial]) {
                let mut buf: [u8; 512] = [0u8; 512];

                match s.serial.read(&mut buf) {
                    Ok(count) if count > 0 => {
                        for c in buf[0..count].iter_mut() {
                            if 0x61 <= *c && *c <= 0x7a {
                                *c &= !0x20;
                            }
                        }

                        let mut write_offset = 0;
                        while write_offset < count {
                            match s.serial.write(&buf[write_offset..count]) {
                                Ok(len) if len > 0 => {
                                    write_offset += len;
                                }
                                _ => {}
                            }
                        }
                    }
                    _ => {}
                }
            }
        }
    }
}

// Button Syhchronisation
static BUTTON_SEMAPHORE: Mutex<Cell<bool>> = Mutex::new(Cell::new(true));
//Mutexed button
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
    let clocks = rcc.constrain()
        .cfgr
        .hse(HSEClock::new(25.mhz(), HSEClockMode::Oscillator))
        .use_pll()
        .use_pll48clk()
        .sysclk(216.mhz())
        .freeze();

    let gpioa = p.GPIOA.split();

    let usb = USB::new(
        p.OTG_FS_GLOBAL,
        p.OTG_FS_DEVICE,
        p.OTG_FS_PWRCLK,
        (
            gpioa.pa11.into_alternate_af10(),
            gpioa.pa12.into_alternate_af10(),
        ),
        clocks,
    );

    UsbSerial::setup(usb);

    // Save info in global
    free(|cs| {
        BUTTON_PIN.borrow(cs).replace(Some(mode_button));
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
            if BUTTON_SEMAPHORE.borrow(cs).get() == false {
                if let Ok(true) = mode_led.is_high() {
                    mode_led.set_low().ok();
                } else {
                    mode_led.set_high().ok();
                }

                BUTTON_SEMAPHORE.borrow(cs).set(true);
            }
        });
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
        BUTTON_SEMAPHORE.borrow(cs).set(false);
    })
}

#[interrupt]
fn OTG_FS() {
    free(|_| {
        UsbSerial::poll();
    });
}
