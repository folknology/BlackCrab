#![deny(unsafe_code)]
#![deny(warnings)]
#![no_main]
#![no_std]

extern crate panic_itm;
use rtic::{app};
//use stm32f7xx_hal::gpio::gpiob::{PB3, PB4};
//use stm32f7xx_hal::gpio::gpioc::PC13;
use stm32f7xx_hal::gpio::gpiod::{PD10};
use stm32f7xx_hal::delay::Delay;
use stm32f7xx_hal::gpio::{PushPull, Output};
use stm32f7xx_hal::gpio::gpioe::{PE11, PE12, PE13};
use stm32f7xx_hal::prelude::*;

pub struct SoftSpi {
    sck: PE12<Output<PushPull>>,
    mosi: PE13<Output<PushPull>>,
    ss: PE11<Output<PushPull>>,
    reset: PD10<Output<PushPull>>,
    delay: Delay,
}

impl SoftSpi {
    pub fn new(sck: PE12<Output<PushPull>>,
               mosi: PE13<Output<PushPull>>,
               ss: PE11<Output<PushPull>>,
               reset: PD10<Output<PushPull>>,
               delay: Delay) -> SoftSpi {
        SoftSpi { sck, mosi, ss, reset, delay }
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

    pub fn select(&mut self) { self.ss.set_low().ok(); }

    pub fn deselect(&mut self) { self.ss.set_high().ok(); }

    fn delay_ms(&mut self, ms: u8) { self.delay.delay_ms(ms); }

    pub fn send(&mut self, byte: u8) {
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

    fn transfer(&mut self, byte: u8) {
        self.select();
        self.send(byte);
        self.deselect();
    }
}

#[app(device = stm32f7xx_hal::pac, peripherals = true, dispatchers = [LP_TIMER1])]
mod app {
    use crate::SoftSpi;
    use crate::Delay;
    // use crate::{PushPull, Output};
    // use stm32f7::stm32f730::{EXTI};
    use stm32f7xx_hal::{pac, prelude::*};
    use stm32f7xx_hal::otg_fs::{UsbBus, USB, UsbBusType};
    use stm32f7xx_hal::rcc::{HSEClock, HSEClockMode};
    use usb_device::prelude::*;
    use usb_device::class_prelude::UsbBusAllocator;
    use usbd_serial::SerialPort;
    // use stm32f7xx_hal::gpio::{ExtiPin, Edge, Speed};
    use stm32f7xx_hal::gpio::{Speed};
    use stm32f7xx_hal::rcc::RccExt;
    use stm32f7xx_hal::qspi::{Qspi, QspiTransaction, QspiWidth};
    use cortex_m;

    //use cortex_m::asm;
    //use cortex_m_rt::entry;
    //use panic_probe as _;
    //use rtt_target::{rprintln, rtt_init_print};


    #[resources]
    struct MyResources {
        #[task_local]
        serial: SerialPort<'static, UsbBus<USB>>,
        #[task_local]
        usb_cdc_device: UsbDevice<'static, UsbBus<USB>>,
        spi: SoftSpi,
        #[task_local]
        qspi_driver: Qspi,
        header: bool,
        byte_count:u32,
        programmed: bool,
    }


    #[init]
    fn init(cx: init::Context) -> (init::LateResources, init::Monotonics) {
        static mut EP_MEMORY: [u32; 1024] = [0; 1024];
        static mut USB_BUS: Option<UsbBusAllocator<UsbBusType>> = None;
        // EP_MEM.write(&mut EP_MEMORY);

        let core: cortex_m::Peripherals = cx.core;
        let device: pac::Peripherals = cx.device;

        //let mut exti :EXTI  = device.EXTI;
        // let mut syscfg = device.SYSCFG;

        // Constrain clocking registers
        let mut rcc = device.RCC;

        // Grab the GPIOB Port and the mode/status leds on pins PB12/13
        let gpiob = device.GPIOB.split();
        let mut mode_led = gpiob.pb7.into_push_pull_output();
        let mut status_led = gpiob.pb5.into_push_pull_output();
        // let mut mode_button = gpiob.pb4.into_floating_input();
        //
        // mode_button.make_interrupt_source(&mut syscfg, &mut rcc);
        // mode_button.trigger_on_edge(&mut exti, Edge::Rising);
        // mode_button.enable_interrupt(&mut exti);

        let gpioe = device.GPIOE.split();
        let sck = gpioe.pe12.into_push_pull_output();
        // let miso = gpiob.pb4.into_floating_input();
        // TODO I think mosi should actually be the miso PB4 here
        let mosi = gpioe.pe13.into_push_pull_output();
        let ss = gpioe.pe11.into_push_pull_output();
        let mut wp = gpioe.pe10.into_push_pull_output();
        let mut hld = gpioe.pe15.into_push_pull_output();

        let _dd2 = gpioe.pe2.into_alternate_af9()
            .internal_pull_up(true)
            .set_speed(Speed::VeryHigh);

        let _dcs = gpiob.pb6.into_alternate_af10()
            .internal_pull_up(true)
            .set_speed(Speed::VeryHigh);
        let _dsck = gpiob.pb2.into_alternate_af9()
            .internal_pull_up(true)
            .set_speed(Speed::VeryHigh);

        let gpiod = device.GPIOD.split();
        let reset = gpiod.pd10.into_push_pull_output();
        let mut _done = gpiod.pd14.into_floating_input();

        let _dd0 = gpiod.pd11.into_alternate_af9()
            .internal_pull_up(true)
            .set_speed(Speed::VeryHigh);
        let _dd1 = gpiod.pd12.into_alternate_af9()
            .internal_pull_up(true)
            .set_speed(Speed::VeryHigh);
        let _dd3 = gpiod.pd13.into_alternate_af9()
            .internal_pull_up(true)
            .set_speed(Speed::VeryHigh);

        //let gpioc = device.GPIOC.split();

        let qspi_driver = Qspi::new(&mut rcc, device.QUADSPI, 1, 1);

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

        //rtt_init_print!(); // You may prefer to initialize another way
        //rprintln!("Hello, world!");

        // Set status green led on
        // for _i in  1..1000000 {
        //     status_led.set_high().ok();
        //     delay.delay_us(1_u8);
        //     status_led.set_low().ok();
        //     delay.delay_us(1_u8);
        // }
        status_led.set_low().ok();
        // Set mode amber led off
        mode_led.set_low().ok();

        let header: bool = true;
        let byte_count: u32 = 0;
        let programmed: bool = false;

        let spi = SoftSpi::new(sck, mosi, ss, reset, delay);

        // rtic::pend(Interrupt::OTG_FS)

        (init::LateResources {
            serial,
            usb_cdc_device,
            spi,
            qspi_driver,
            header,
            byte_count,
            programmed,
        },
        init::Monotonics())
    }

    #[idle]
    fn idle(_cx: idle::Context) -> ! {
        loop {
            cortex_m::asm::nop();
        }
    }

    #[task(resources = [programmed, spi])]
    fn manage(cx: manage::Context) {
        let spi = cx.resources.spi;
        let programmed = cx.resources.programmed;

        (programmed, spi).lock(|programmed: &mut bool ,spi: &mut SoftSpi| {
            if *programmed {
                for count in 0..16 {
                    spi.delay_ms(100);
                    spi.transfer(count as u8);
                }
            }
        });
    }

    #[task(resources = [programmed, qspi_driver])]
    fn dspi(cx: dspi::Context) {
        let driver = cx.resources.qspi_driver;
        let mut programmed = cx.resources.programmed;

        programmed.lock(|programmed: &mut bool| {
            if *programmed {
                for count in 0..16 {
                    let transaction = QspiTransaction {
                        iwidth: QspiWidth::NONE,
                        awidth: QspiWidth::NONE,
                        dwidth: QspiWidth::QUAD,//DUAL
                        instruction: 0,
                        address: None,
                        dummy: 0,
                        data_len: Some(1),
                    };
                    let mut buf = [count];
                    driver.polling_write(&mut buf, transaction).unwrap();
                }
            }
        });
    }


    #[task(binds = OTG_FS, resources = [serial, usb_cdc_device, spi, header, byte_count, programmed])]
    fn usb_event(cx: usb_event::Context) {
        let usb_cdc_device = cx.resources.usb_cdc_device; //: &mut UsbDevice<'static, UsbBus<USB>>
        let serial = cx.resources.serial;//: &mut SerialPort<'static, UsbBus<USB>>
        let spi = cx.resources.spi; //: &mut SoftSpi
        let header = cx.resources.header;
        let byte_count = cx.resources.byte_count;
        let programmed = cx.resources.programmed;

        if usb_cdc_device.poll(&mut [serial]) {
            let mut buf: [u8; 512] = [0u8; 512];

            match serial.read(&mut buf) {
                Ok(count) if count > 0 => {
                    (header, byte_count, programmed, spi).lock(|header, byte_count, programmed, spi: &mut SoftSpi| {
                        * byte_count += count as u32;
                        for c in buf[0..count].iter_mut() {
                            if *header {
                                if *c == 0x7E as u8 {
                                    *programmed = false;
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
                            *programmed = true;
                            // Maybe add a delay here before sending anything to HDL
                            //manage::spawn().unwrap();
                            //dspi::spawn().unwrap();
                        }
                    });
                }
                _ => {}
            }

        }
    }
}