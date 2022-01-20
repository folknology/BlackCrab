// #![deny(unsafe_code)]
#![deny(warnings)]
#![no_main]
#![no_std]

// extern crate panic_itm;
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
        self.ss.set_high();
        self.reset.set_low();
        self.ss.set_low();
        self.delay.delay_ms(1_u8);
        self.reset.set_high();
        self.delay.delay_ms(2_u8);
        self.ss.set_high();
        self.delay.delay_ms(50_u8);

        self.mosi.set_low();
        self.sck.set_low();
        self.ss.set_low();
        for _ in 0..8 {
            self.delay.delay_us(10_u8);
            self.sck.set_high();
            self.delay.delay_us(10_u8);
            self.sck.set_low();
        }
        self.ss.set_high();
    }

    pub fn select(&mut self) { self.ss.set_low(); }

    pub fn deselect(&mut self) { self.ss.set_high(); }

    fn delay_ms(&mut self, ms: u8) { self.delay.delay_ms(ms); }

    pub fn send(&mut self, byte: u8) {
        // self.ss.set_low();
        for bit_offset in 0..8 {
            self.sck.set_low();
            let out_bit = (byte >> (7 - bit_offset)) & 0b1;
            if out_bit == 1 {
                self.mosi.set_high();
            } else {
                self.mosi.set_low();
            }
            self.delay.delay_us(1_u8);
            self.sck.set_high();
            self.delay.delay_us(1_u8);
        }
        self.sck.set_low();
        // self.ss.set_high();
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
    use stm32f7xx_hal::rcc::{HSEClock, HSEClockMode, PLL48CLK};
    use usb_device::prelude::*;
    use usb_device::class_prelude::UsbBusAllocator;
    use usbd_serial::SerialPort;
    // use stm32f7xx_hal::gpio::{ExtiPin, Edge, Speed};
    use stm32f7xx_hal::gpio::{Speed};
    use stm32f7xx_hal::rcc::RccExt;
    use stm32f7xx_hal::qspi::{Qspi, QspiTransaction, QspiWidth};
    use cortex_m;

    // use cortex_m::asm;
    // use cortex_m_rt::entry;
    use panic_probe as _;
    use rtt_target::{rprintln, rtt_init_print};

    /* resources shared across RTIC tasks */
    #[shared]
    struct Shared {
        spi: SoftSpi,
        header: bool,
        byte_count: u32,
        programmed: bool
    }

    /* resources local to specific RTIC tasks */
    #[local]
    struct Local {
        serial: SerialPort<'static, UsbBus<USB>>,
        usb_cdc_device: UsbDevice<'static, UsbBus<USB>>,
        qspi_driver: Qspi,
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

        let _dd2 = gpioe.pe2.into_alternate::<9>()
            .internal_pull_up(true)
            .set_speed(Speed::VeryHigh);

        let _dcs = gpiob.pb6.into_alternate::<10>()
            .internal_pull_up(true)
            .set_speed(Speed::VeryHigh);
        let _dsck = gpiob.pb2.into_alternate::<9>()
            .internal_pull_up(true)
            .set_speed(Speed::VeryHigh);

        let gpiod = device.GPIOD.split();
        let reset = gpiod.pd10.into_push_pull_output();
        let mut _done = gpiod.pd14.into_floating_input();

        let _dd0 = gpiod.pd11.into_alternate::<9>()
            .internal_pull_up(true)
            .set_speed(Speed::VeryHigh);
        let _dd1 = gpiod.pd12.into_alternate::<9>()
            .internal_pull_up(true)
            .set_speed(Speed::VeryHigh);
        let _dd3 = gpiod.pd13.into_alternate::<9>()
            .internal_pull_up(true)
            .set_speed(Speed::VeryHigh);

        //let gpioc = device.GPIOC.split();

        let qspi_driver = Qspi::new(&mut rcc, device.QUADSPI, 1, 1);

        let rcc_constrain = rcc.constrain();

        // Configure clock and freeze it
        let clocks = rcc_constrain
            .cfgr
            .hse(HSEClock::new(25_000_000.Hz(), HSEClockMode::Oscillator))
            .use_pll()
            .use_pll48clk(PLL48CLK::Pllq)
            .sysclk(216_000_000.Hz())
            .freeze();

        let mut delay = Delay::new(core.SYST, clocks);

        // prep and reset FPGA, disable Flash chip
        wp.set_low();
        hld.set_low();
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
            clocks,
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

        // Set status green led on
        // for _i in  1..1000000 {
        //     status_led.set_high();
        //     delay.delay_us(1_u8);
        //     status_led.set_low();
        //     delay.delay_us(1_u8);
        // }
        status_led.set_low();
        // Set mode amber led off
        mode_led.set_low();

        let header: bool = true;
        let byte_count: u32 = 0;
        let programmed: bool = false;

        let spi = SoftSpi::new(sck, mosi, ss, reset, delay);

        // rtic::pend(Interrupt::OTG_FS)
        // lastly return the shared and local resources, as per RTIC's spec.
        (
            Shared {
                spi,
                header,
                byte_count,
                programmed,
            },
            Local {
                serial,
                usb_cdc_device,
                qspi_driver,
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

    #[task(shared=[programmed, spi])]
    fn manage(cx: manage::Context) {
        let spi = cx.shared.spi;
        let programmed = cx.shared.programmed;

        (programmed, spi).lock(|programmed: &mut bool ,spi: &mut SoftSpi| {
            if *programmed {
                for count in 0..16 {
                    spi.delay_ms(100);
                    spi.transfer(count as u8);
                }
            }
        });
    }

    #[task(shared=[programmed], local=[qspi_driver])]
    fn dspi(cx: dspi::Context) {
        let driver = cx.local.qspi_driver;
        let mut programmed = cx.shared.programmed;

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
                    driver.write(&mut buf, transaction).unwrap();
                    //driver.poll_status();
                }
            }
        });
    }


    #[task(binds = OTG_FS, shared=[spi, header, byte_count, programmed], local=[serial, usb_cdc_device])]
    fn usb_event(cx: usb_event::Context) {
        let usb_cdc_device = cx.local.usb_cdc_device; //: &mut UsbDevice<'static, UsbBus<USB>>
        let serial = cx.local.serial;//: &mut SerialPort<'static, UsbBus<USB>>
        let spi = cx.shared.spi; //: &mut SoftSpi
        let header = cx.shared.header;
        let byte_count = cx.shared.byte_count;
        let programmed = cx.shared.programmed;

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