//#![deny(unsafe_code)]
#![feature(maybe_uninit_extra, maybe_uninit_ref)]
#![deny(warnings)]
#![no_main]
#![no_std]

extern crate panic_itm;

// use nb::block;
use cortex_m_rt::entry;
// use stm32f7::stm32f730 as pac;
use stm32f7::stm32f730::{interrupt, EXTI};
use stm32f7xx_hal::{pac, prelude::*};
use stm32f7xx_hal::otg_fs::{UsbBus, USB};
use stm32f7::stm32f730::Interrupt;
// use stm32f7xx_hal::timer::Timer;
use stm32f7xx_hal::rcc::{HSEClock, HSEClockMode};
use stm32f7xx_hal::gpio::{ExtiPin, Edge, Input, Floating, PushPull, Output};
use cortex_m::interrupt::{Mutex, free};
use stm32f7xx_hal::gpio::gpiob::{PB7, PB3, PB4};
use cortex_m::peripheral::NVIC;
use usb_device::prelude::*;
use usbd_serial::SerialPort;
use usb_device::class_prelude::UsbBusAllocator;
use core::cell::{Cell, RefCell};
use core::mem::MaybeUninit;
// use bare_metal::Peripheral;
// use stm32f7xx_hal::pac::Peripherals;
// use bitbang_hal::spi::{MODE_0, SPI};
// use stm32f7xx_hal::spi::{Miso, Mosi, Sck};
use embedded_hal::timer::{CountDown, Periodic};
use embedded_hal::digital::v2::{InputPin, OutputPin};
use stm32f7xx_hal::gpio::gpiod::PD2;
use stm32f7xx_hal::delay::Delay;
use stm32f7xx_hal::rcc::RccExt;
// use embedded_hal::spi::FullDuplex;

trait TimerTrait: CountDown + Periodic {}

static mut EP_MEMORY: [u32; 1024] = [0; 1024];

static mut USB_BUS: MaybeUninit<UsbBusAllocator<UsbBus<USB>>> = MaybeUninit::uninit();
static mut USB_SERIAL: Option<UsbSerial> = None;

pub struct UsbSerial {
    serial: SerialPort<'static, UsbBus<USB>>,
    device: UsbDevice<'static, UsbBus<USB>>,
    sck: PB3<Output<PushPull>>,
    mosi: PB4<Output<PushPull>>,
    ss: PD2<Output<PushPull>>,
    delay: Delay,
    header: bool,
    byte_count:u32,
}

impl UsbSerial {
    pub fn setup(usb: USB, sck: PB3<Output<PushPull>>, mosi: PB4<Output<PushPull>>, ss: PD2<Output<PushPull>>, delay: Delay) {
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
            unsafe { USB_SERIAL = Some(UsbSerial {serial, device, sck, mosi, ss, delay, header: true, byte_count: 0 }); }
        });

        unsafe {
            NVIC::unmask(Interrupt::OTG_FS);
        }
    }

    pub fn get() -> Option<&'static mut Self> {
        unsafe { USB_SERIAL.as_mut() }
    }

    pub fn send(&mut self, byte: u8){
        // self.ss.set_low().ok();
        for bit_offset in 0..8 {
            self.sck.set_low().ok();
            let out_bit = (byte >> (7 - bit_offset)) & 0b1;
            if out_bit == 1 {
                self.mosi.set_high().ok();
            } else {
                self.mosi.set_low().ok();
            }
            self.delay.delay_us(1_u8);
            self.sck.set_high().ok();
            self.delay.delay_us(1_u8);
        }
        self.sck.set_low().ok();
        // self.ss.set_high().ok();
    }

    pub fn poll() {
        if let Some(ref mut s) = Self::get() {
            if s.device.poll(&mut [&mut s.serial]) {
                let mut buf: [u8; 512] = [0u8; 512];

                match s.serial.read(&mut buf) {
                    Ok(count) if count > 0 => {
                        s.byte_count += count as u32;
                        for c in buf[0..count].iter_mut() {
                            if s.header {
                                if *c == 0x7E as u8 {
                                    s.ss.set_low().ok();
                                    s.header = false;
                                    s.send(*c);
                                } else {
                                    continue
                                }
                            } else {
                                s.send(*c);
                            }

                            // if 0x61 <= *c && *c <= 0x7a {
                            //     *c &= !0x20;
                            // }
                        }
                        if s.byte_count >= 135100 as u32 {
                            s.header = true;
                            s.delay.delay_ms(10_u8);
                            for _ in 0..7 {
                                s.send(0x00 as u8);
                            }
                            s.ss.set_high().ok();
                        }

                        // let mut write_offset = 0;
                        // while write_offset < count {
                        //     match s.serial.write(&buf[write_offset..count]) {
                        //         Ok(len) if len > 0 => {
                        //             write_offset += len;
                        //         }
                        //         _ => {}
                        //     }
                        // }
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

// fn configure_mco(rcc: &mut stm32f7::stm32f730::RCC, port: &mut stm32f7::stm32f730::GPIOA) {
//     const MODE_OUTPUT_50MHz: u8 = 0b11;
//     const CNF_AF_OUTPUT_PUSHPULL: u8 = 0b10;
//     // enable port clock
//     rcc.apb2enr.modify(|_r, w| w.iopaen().set_bit());
//     // configure port mode and enable alternate function
//     port.crh.modify(|_r, w| unsafe { port.
//         w
//             .mode8().bits(MODE_OUTPUT_50MHz)
//             .cnf8().bits(CNF_AF_OUTPUT_PUSHPULL)
//     });
//     // enable MCO alternate function (PA8)
//     rcc.cfgr.modify(|_r, w| w.mco1().sysclk());
// }

#[entry]
fn main() -> ! {
    // Take control of the peripherals
    let cp = cortex_m::Peripherals::take().unwrap();
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

    let mut sck = gpiob.pb3.into_push_pull_output();
    // let miso = gpiob.pb4.into_floating_input();
    // TODO I think mosi should actually be the miso PB4 here
    let mut mosi = gpiob.pb4.into_push_pull_output();

    let gpiod = p.GPIOD.split();
    let mut ss = gpiod.pd2.into_push_pull_output();
    let gpioc = p.GPIOC.split();
    let mut hld = gpioc.pc11.into_push_pull_output();
    let mut wp = gpioc.pc12.into_push_pull_output();
    let mut reset = gpioc.pc13.into_push_pull_output();
    let mut _done = gpioc.pc8.into_floating_input();

    let rcc_constrain = rcc.constrain();

    // Configure clock and freeze it
    let clocks = rcc_constrain
        .cfgr
        .hse(HSEClock::new(25.mhz(), HSEClockMode::Oscillator))
        .use_pll()
        .use_pll48clk()
        .sysclk(216.mhz())
        .freeze();

    let mut delay = Delay::new(cp.SYST, clocks);
    // let mut c1_no_pins = tim3(p.TIM3, 9000, 25.mhz(), clocks);
    // let mut ch1 = c1_no_pins.
    //
    // let mut spi = SPI::new(MODE_0, miso, mosi, sck, tmr);

    // prep and reset FPGA, disable Flash chip
    wp.set_low().ok();
    hld.set_low().ok();
    delay.delay_ms(50_u8);
    ss.set_high().ok();
    reset.set_low().ok();
    ss.set_low().ok();
    delay.delay_ms(1_u8);
    reset.set_high().ok();
    delay.delay_ms(2_u8);
    ss.set_high().ok();
    delay.delay_ms(50_u8);

    mosi.set_low().ok();
    sck.set_low().ok();
    ss.set_low().ok();
    for _ in 0..8 {
        delay.delay_us(10_u8);
        sck.set_high().ok();
        delay.delay_us(10_u8);
        sck.set_low().ok();
    }
    ss.set_high().ok();

    // reset.set_low().ok();
    // ss.set_low().ok();
    // delay.delay_ms(50_u8);
    // reset.set_high().ok();
    // delay.delay_ms(50_u8);
    // ss.set_high().ok();
    //delay
    // check done wait
    // delay.delay_ms(50_u8);
    // wp.set_high().ok();
    // hld.set_high().ok();

    // for byte in b"Hello, World!" {
    //     block!(spi.send(*byte)).unwrap();
    // }

    let gpioa = p.GPIOA.split();
    gpioa.pa8.into_alternate_af0();

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

    UsbSerial::setup(usb, sck, mosi, ss, delay);

    // let peripherals = stm32f7::stm32f730::Peripherals::take().unwrap();
    // let rccp = peripherals.RCC;
    // // let mut port_a = peripherals.GPIOA;
    // rccp.cfgr.modify(|_r, w| w.mco1().variant(stm32f7::stm32f730::rcc::cfgr::MCO1_A::HSE));

    // configure_mco(&mut rccp, &mut port_a);

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
