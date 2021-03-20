#![deny(unsafe_code)]
#![deny(warnings)]
#![no_main]
#![no_std]

extern crate panic_itm;

use rtic::app;
use stm32f7::stm32f730::{EXTI};
use stm32f7xx_hal::{pac, prelude::*};
use stm32f7xx_hal::otg_fs::{UsbBus, USB, UsbBusType};
use stm32f7xx_hal::rcc::{HSEClock, HSEClockMode};
use stm32f7xx_hal::gpio::{ExtiPin, Edge, PushPull, Output};
use stm32f7xx_hal::gpio::gpiob::{PB3, PB4};
use stm32f7xx_hal::gpio::gpioc::PC13;
use usb_device::prelude::*;
use usb_device::class_prelude::UsbBusAllocator;
use usbd_serial::SerialPort;
use stm32f7xx_hal::gpio::gpiod::PD2;
use stm32f7xx_hal::delay::Delay;
use stm32f7xx_hal::rcc::RccExt;

pub struct SoftSpi {
    sck: PB3<Output<PushPull>>,
    mosi: PB4<Output<PushPull>>,
    ss: PD2<Output<PushPull>>,
    reset: PC13<Output<PushPull>>,
    delay: Delay,
}

impl SoftSpi  {
    pub fn new(sck: PB3<Output<PushPull>>,
               mosi: PB4<Output<PushPull>>,
               ss: PD2<Output<PushPull>>,
               reset: PC13<Output<PushPull>>,
               delay: Delay) -> SoftSpi {
        SoftSpi {sck, mosi, ss, reset, delay}
    }

    pub fn reset(&mut self) {
        self.ss.set_high().ok();
        self.reset.set_low().ok();
        self.ss.set_low().ok();
        self.delay.delay_ms(1_u8);
        self.reset.set_high().ok();
        self.delay.delay_ms(2_u8);
        self.ss.set_high().ok();
        self.delay.delay_ms(50_u8);

        self.mosi.set_low().ok();
        self.sck.set_low().ok();
        self.ss.set_low().ok();
        for _ in 0..8 {
            self.delay.delay_us(10_u8);
            self.sck.set_high().ok();
            self.delay.delay_us(10_u8);
            self.sck.set_low().ok();
        }
        self.ss.set_high().ok();
    }

    pub fn select(&mut self) {self.ss.set_low().ok();}

    pub fn deselect(&mut self) {self.ss.set_high().ok();}

    fn delay_ms(&mut self, us: u8) {self.delay.delay_ms(us);}

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
}

#[app(device = stm32f7xx_hal::pac, peripherals = true)]
const APP:() = {
    struct Resources {
        serial: SerialPort<'static, UsbBus<USB>>,
        usb_cdc_device: UsbDevice<'static, UsbBus<USB>>,
        spi: SoftSpi,
        header: bool,
        byte_count:u32,
    }


    #[init]
    fn init(cx: init::Context) -> init::LateResources {
        static mut EP_MEMORY: [u32; 1024] = [0; 1024];
        static mut USB_BUS: Option<UsbBusAllocator<UsbBusType>> = None;
        // EP_MEM.write(&mut EP_MEMORY);

        let core: cortex_m::Peripherals = cx.core;
        let device: pac::Peripherals = cx.device;

        let mut exti :EXTI  = device.EXTI;
        let mut syscfg = device.SYSCFG;

        // Constrain clocking registers
        let mut rcc = device.RCC;

        // Grab the GPIOB Port and the mode/status leds on pins PB12/13
        let gpiob = device.GPIOB.split();
        let mut mode_led = gpiob.pb12.into_push_pull_output();
        let mut status_led = gpiob.pb13.into_push_pull_output();
        let mut mode_button = gpiob.pb7.into_floating_input();
    
        mode_button.make_interrupt_source(&mut syscfg, &mut rcc);
        mode_button.trigger_on_edge(&mut exti, Edge::Rising);
        mode_button.enable_interrupt(&mut exti);
    
        let sck = gpiob.pb3.into_push_pull_output();
        // let miso = gpiob.pb4.into_floating_input();
        // TODO I think mosi should actually be the miso PB4 here
        let mosi = gpiob.pb4.into_push_pull_output();
    
        let gpiod = device.GPIOD.split();
        let ss = gpiod.pd2.into_push_pull_output();
        let gpioc = device.GPIOC.split();
        let mut hld = gpioc.pc11.into_push_pull_output();
        let mut wp = gpioc.pc12.into_push_pull_output();
        let reset = gpioc.pc13.into_push_pull_output();
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
    
        let mut delay = Delay::new(core.SYST, clocks);

        // prep and reset FPGA, disable Flash chip
        wp.set_low().ok();
        hld.set_low().ok();
        delay.delay_ms(50_u8);

        // Port for master clock out and usb
        let gpioa = device.GPIOA.split();
        // Master Clock out 1, alternate funtion of PA8
        gpioa.pa8.into_alternate_af0();

        // Usb ports/pins and init
        let usb = USB::new(
            device.OTG_FS_GLOBAL,
            device.OTG_FS_DEVICE,
            device.OTG_FS_PWRCLK,
            (
                gpioa.pa11.into_alternate_af10(),
                gpioa.pa12.into_alternate_af10(),
            ),
            clocks,
        );

        *USB_BUS = Some(UsbBus::new(usb, &mut *EP_MEMORY));

        let serial = SerialPort::new(USB_BUS.as_ref().unwrap());

        let usb_cdc_device = UsbDeviceBuilder::new(USB_BUS.as_ref().unwrap(), UsbVidPid(0x16c0, 0x27dd))
            .manufacturer("myStorm")
            .product("IceCore")
            .serial_number("1234")
            .device_class(usbd_serial::USB_CLASS_CDC)
            .max_packet_size_0(64)
            .device_release(0x20)
            .self_powered(true)
            .build();

        // Set status green led on
        status_led.set_low().ok();
        // Set mode amber led off
        mode_led.set_low().ok();

        let header: bool = true;
        let byte_count:u32 = 0;

        let spi = SoftSpi::new(sck, mosi, ss, reset, delay);

        init::LateResources {
            serial,
            usb_cdc_device,
            spi,
            header,
            byte_count,
        }
    }

    #[idle]
    fn idle(_: idle::Context) -> ! {
        loop {
            cortex_m::asm::nop();
        }
    }

    #[task(binds = OTG_FS, resources = [serial, usb_cdc_device, spi, header, byte_count])]
    fn usb_event(cx: usb_event::Context) {
        let usb_cdc_device: &mut UsbDevice<'static, UsbBus<USB>> = cx.resources.usb_cdc_device;
        let serial: &mut SerialPort<'static, UsbBus<USB>> = cx.resources.serial;
        let spi: &mut SoftSpi = cx.resources.spi;
        let header: &mut bool = cx.resources.header;
        let byte_count: &mut u32  = cx.resources.byte_count;

        if usb_cdc_device.poll(&mut [serial]) {
            let mut buf: [u8; 512] = [0u8; 512];

            match serial.read(&mut buf) {
                Ok(count) if count > 0 => {
                    *byte_count += count as u32;
                    for c in buf[0..count].iter_mut() {
                        if  *header {
                            if *c == 0x7E as u8 {
                                spi.reset();
                                spi.select();
                                *header = false;
                                spi.send(*c);
                            } else {
                                continue
                            }
                        } else {
                            spi.send(*c);
                        }
                    }
                    if *byte_count >= 135100 as u32 {
                        *header = true;
                        *byte_count = 0;
                        spi.delay_ms(10_u8);
                        for _ in 0..7 {
                            spi.send(0x00 as u8);
                        }
                        spi.deselect();
                    }
                }
                _ => {}
            }

        }
    }
};
