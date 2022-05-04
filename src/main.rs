// #![deny(unsafe_code)]
#![deny(warnings)]
#![no_main]
#![no_std]

// extern crate panic_itm;
use rtic::{app};
//use stm32f7xx_hal::gpio::gpiob::{PB3, PB4};
//use stm32f7xx_hal::gpio::gpioc::PC13;
//use stm32f7xx_hal::gpio::gpiod::{PD11};
//use stm32f7xx_hal::delay::Delay;
use stm32f7xx_hal::timer::{SysDelay};
use stm32f7xx_hal::gpio::{PushPull, Output, PB6, PB4};
//use stm32f7xx_hal::gpio::gpioe::{PE11, PE12, PE13};
use stm32f7xx_hal::prelude::*;
use stm32f7xx_hal::qspi::{Qspi, QspiTransaction, QspiWidth};
// use stm32f7xx_hal::gpio::{Speed};

pub struct Fpga {
    ss: PB6<Output<PushPull>>,
    reset: PB4<Output<PushPull>>,
    delay: SysDelay,
    bus: Qspi
}

impl Fpga {
    pub fn new(
               ss: PB6<Output<PushPull>>,
               reset: PB4<Output<PushPull>>,
               delay: SysDelay,
               bus : Qspi) -> Fpga {
        Fpga { ss, reset, delay, bus}
    }

    pub fn reset(&mut self) {
        self.ss.set_high();
        self.reset.set_low();
        self.ss.set_low();
        self.delay.delay_ms(1_u8);
        self.reset.set_high();
        self.delay.delay_ms(2_u8);
        self.ss.set_high();
        self.delay.delay_ms(50_u8);

        let transaction = QspiTransaction {
                        iwidth: QspiWidth::NONE,
                        awidth: QspiWidth::NONE,
                        dwidth: QspiWidth::SING,
                        instruction: 0,
                        address: None,
                        dummy: 0,
                        data_len: Some(8),
                    };
        self.bus.write(&[0,0,0,0,0,0,0,0], transaction).unwrap();
    }

    pub fn select(&mut self) { self.ss.set_low(); }

    pub fn deselect(&mut self) { self.ss.set_high(); }

    fn delay_ms(&mut self, ms: u8) { self.delay.delay_ms(ms); }

    pub fn send(&mut self, byte: u8) {
        let transaction = QspiTransaction {
                        iwidth: QspiWidth::NONE,
                        awidth: QspiWidth::NONE,
                        dwidth: QspiWidth::SING,
                        instruction: 0,
                        address: None,
                        dummy: 0,
                        data_len: Some(1),
                    };
        self.bus.write(&[byte], transaction).unwrap();
    }

    fn transfer(&mut self, byte: u8) {
        self.select();
        self.send(byte);
        self.deselect();
    }
}

#[app(device = stm32f7xx_hal::pac, peripherals = true, dispatchers = [LP_TIMER1])]
mod app {
    use crate::Fpga;
    // use crate::{PushPull, Output};
    // use stm32f7::stm32f730::{EXTI};
    use stm32f7xx_hal::{pac, prelude::*};
    use stm32f7xx_hal::otg_fs::{UsbBus, USB, UsbBusType};
    use stm32f7xx_hal::rcc::{HSEClock, HSEClockMode, MCO1, PLL48CLK};
    use usb_device::prelude::*;
    use usb_device::class_prelude::UsbBusAllocator;
    use usbd_serial::SerialPort;
    // use stm32f7xx_hal::gpio::{ExtiPin, Edge, Speed};
    use stm32f7xx_hal::gpio::{Speed};
    use stm32f7xx_hal::qspi::{Qspi};
    use stm32f7xx_hal::rcc::RccExt;
    use cortex_m;

    // use cortex_m::asm;
    // use cortex_m_rt::entry;
    use panic_probe as _;
    use rtt_target::{rprintln, rtt_init_print};

    /* resources shared across RTIC tasks */
    #[shared]
    struct Shared {
        ice: Fpga,
        header: bool,
        byte_count: u32,
        programmed: bool
    }

    /* resources local to specific RTIC tasks */
    #[local]
    struct Local {
        serial: SerialPort<'static, UsbBus<USB>>,
        usb_cdc_device: UsbDevice<'static, UsbBus<USB>>
    }


    #[init(local=[EP_MEMORY:[u32; 1024] = [0; 1024]])]
    fn init(cx: init::Context) -> (Shared, Local, init::Monotonics) {
        // static mut EP_MEMORY: [u32; 1024] = [0; 1024];
        let EP_MEMORY: &'static mut [u32; 1024]  = cx.local.EP_MEMORY;
        static mut USB_BUS: Option<UsbBusAllocator<UsbBusType>> = None;
        // EP_MEM.write(&mut EP_MEMORY);

        let core: cortex_m::Peripherals = cx.core;
        let device: pac::Peripherals = cx.device;

        rtt_init_print!(); // You may prefer to initialize another way
        rprintln!("Hello, over SWO/ProbeRS");

        //let mut exti :EXTI  = device.EXTI;
        // let mut syscfg = device.SYSCFG;

        // Constrain clocking registers
        let mut rcc = device.RCC;
        //let rcc = device.RCC;

        // Grab the GPIOB Port and the mode/status leds on pins PB12/13
        let gpiob = device.GPIOB.split();
        let reset = gpiob.pb4.into_push_pull_output();
        //PB12-15:SS,SCK,SO,SI
        let mut _fss = gpiob.pb12.into_push_pull_output();
        let mut _fck = gpiob.pb13.into_push_pull_output();
        let mut _fso = gpiob.pb14.into_floating_input(); //Miso
        let mut _fsi = gpiob.pb15.into_push_pull_output(); // Mosi

        let _qsck = gpiob.pb2.into_alternate::<9>()
            .internal_pull_up(true)
            .set_speed(Speed::VeryHigh);
        let ss = gpiob.pb6.into_push_pull_output();
            // .internal_pull_up(true)
            // .set_speed(Speed::VeryHigh);

        let gpioe = device.GPIOE.split();
        // let mut mode_button = gpioe.pe13.into_floating_input();
        //
        // mode_button.make_interrupt_source(&mut syscfg, &mut rcc);
        // mode_button.trigger_on_edge(&mut exti, Edge::Rising);
        // mode_button.enable_interrupt(&mut exti);

        let mut mode_led = gpioe.pe5.into_push_pull_output();

        let _q2 = gpioe.pe2.into_alternate::<9>()
            .internal_pull_up(true)
            .set_speed(Speed::VeryHigh);

        let gpiod = device.GPIOD.split();
        //Make int a pullup?
        let mut _int = gpiod.pd3.into_floating_input();

        //let mosi = gpiod.pd11.into_push_pull_output();
        let _q0 = gpiod.pd11.into_alternate::<9>()
            .internal_pull_up(true)
            .set_speed(Speed::VeryHigh);
        let _q1 = gpiod.pd12.into_alternate::<9>()
            .internal_pull_up(true)
            .set_speed(Speed::VeryHigh);
        let _q3 = gpiod.pd13.into_alternate::<9>()
            .internal_pull_up(true)
            .set_speed(Speed::VeryHigh);

        let gpioc = device.GPIOC.split();
        let mut _done = gpioc.pc13.into_floating_input();

        let bus = Qspi::new(&mut rcc, device.QUADSPI, 1, 1);

        let rcc_constrain = rcc.constrain();

        // Configure clock and freeze it
        let clocks = rcc_constrain
            .cfgr
            .hse(HSEClock::new(25_000_000.Hz(), HSEClockMode::Oscillator))
            .use_pll()
            .use_pll48clk(PLL48CLK::Pllq)
            .sysclk(216_000_000.Hz())
            .mco1(MCO1::Hse)
            .freeze();

        //let mut delay = Delay::new(core.SYST, clocks);
        let mut delay = core.SYST.delay(&clocks);

        // prep and reset FPGA
        delay.delay_ms(50_u8);

        // Port for master clock out and usb
        let gpioa = device.GPIOA.split();
        // Master Clock out 1, alternate funtion of PA8
        gpioa.pa8.into_alternate::<0>();

        // Usb ports/pins and init
        let usb = USB::new(
            device.OTG_FS_GLOBAL,
            device.OTG_FS_DEVICE,
            device.OTG_FS_PWRCLK,
            (
                gpioa.pa11.into_alternate::<10>(),
                gpioa.pa12.into_alternate::<10>(),
            ),
            &clocks,
        );

        unsafe {
            // *USB_BUS = Some(UsbBus::new(usb, &mut *EP_MEMORY));
            USB_BUS.replace(UsbBus::new(usb, &mut *EP_MEMORY));
        }

        let serial = SerialPort::new(unsafe{USB_BUS.as_ref().unwrap()});

        let usb_cdc_device = UsbDeviceBuilder::new(unsafe{USB_BUS.as_ref().unwrap()}, UsbVidPid(0x16c0, 0x27dd))
            .manufacturer("myStorm")
            .product("IceCore")
            .serial_number("1234")
            .device_class(usbd_serial::USB_CLASS_CDC)
            .max_packet_size_0(64)
            .device_release(0x20)
            .self_powered(true)
            .build();

        // Set mode amber red led on with power green
        mode_led.set_low();

        //_done.set_low();

        let header: bool = true;
        let byte_count: u32 = 0;
        let programmed: bool = false;

        let ice = Fpga::new(ss, reset, delay, bus);

        rprintln!("Init finishing");

        // rtic::pend(Interrupt::OTG_FS)
        // lastly return the shared and local resources, as per RTIC's spec.
        (
            Shared {
                ice,
                header,
                byte_count,
                programmed,
            },
            Local {
                serial,
                usb_cdc_device,
            },
            init::Monotonics(),
        )
    }

    #[idle]
    fn idle(_cx: idle::Context) -> ! {
        loop {
            cortex_m::asm::nop();
        }
    }

    #[task(shared=[programmed, ice])]
    fn manage(cx: manage::Context) {
        let ice = cx.shared.ice;
        let programmed = cx.shared.programmed;

        (programmed, ice).lock(|programmed: &mut bool, ice: &mut Fpga| {
            if *programmed {
                for count in 0..16 {
                    ice.delay_ms(100);
                    ice.transfer(count as u8);
                }
            }
        });
    }

    // #[task(shared=[programmed], local=[qspi_driver])]
    // fn dspi(cx: dspi::Context) {
    //     let driver = cx.local.qspi_driver;
    //     let mut programmed = cx.shared.programmed;
    //
    //     programmed.lock(|programmed: &mut bool| {
    //         if *programmed {
    //             let count : u8 = 255;
    //             for _count  in 0..count {
    //                 let transaction = QspiTransaction {
    //                     iwidth: QspiWidth::NONE,
    //                     awidth: QspiWidth::NONE,
    //                     dwidth: QspiWidth::QUAD,//DUAL
    //                     instruction: 0,
    //                     address: None,
    //                     dummy: 0,
    //                     data_len: Some(1),
    //                 };
    //                 //rprintln!("count:{}",_count as u8);
    //                 //let mut buf = [_count as u8];
    //                 driver.write(&[_count], transaction).unwrap();
    //                 for _ in 0..100_000 {
    //                     cortex_m::asm::nop();
    //                 }
    //                 //driver.poll_status();
    //             }
    //         }
    //     });
    // }


    #[task(binds = OTG_FS, shared=[ice, header, byte_count, programmed], local=[serial, usb_cdc_device])]
    fn usb_event(cx: usb_event::Context) {
        let usb_cdc_device = cx.local.usb_cdc_device; //: &mut UsbDevice<'static, UsbBus<USB>>
        let serial = cx.local.serial;//: &mut SerialPort<'static, UsbBus<USB>>
        let ice = cx.shared.ice; //: &mut SoftSpi
        let header = cx.shared.header;
        let byte_count = cx.shared.byte_count;
        let programmed = cx.shared.programmed;

        if usb_cdc_device.poll(&mut [serial]) {
            let mut buf: [u8; 512] = [0u8; 512];

            match serial.read(&mut buf) {
                Ok(count) if count > 0 => {
                    (header, byte_count, programmed, ice).lock(|header, byte_count, programmed, ice: &mut Fpga| {
                        * byte_count += count as u32;
                        for c in buf[0..count].iter_mut() {
                            if *header {
                                if *c == 0x7E as u8 {
                                    *programmed = false;
                                    ice.reset();
                                    ice.select();
                                    *header = false;
                                    ice.send(*c);
                                } else {
                                    continue
                                }
                            } else {
                                ice.send(*c);
                            }
                        }
                        if *byte_count >= 135100 as u32 {
                            *header = true;
                            *byte_count = 0;
                            ice.delay_ms(10_u8);
                            for _ in 0..7 {
                                ice.send(0x00 as u8);
                            }
                            ice.deselect();
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