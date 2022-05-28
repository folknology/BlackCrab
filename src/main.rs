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
use stm32f7xx_hal::gpio::{Alternate, Pin,PushPull, Output, PB6, PB4, PB12};
//use stm32f7xx_hal::gpio::gpioe::{PE11, PE12, PE13};
use stm32f7xx_hal::prelude::*;
use stm32f7xx_hal::qspi::{Qspi, QspiTransaction, QspiWidth};
use stm32f7xx_hal::spi::{Spi, Enabled};
use stm32f7xx_hal::pac::SPI2;
// use stm32f7xx_hal::gpio::{Speed};

pub type Hspi = Spi<SPI2, (
    Pin<'B',13,Alternate<5,PushPull>>,
    Pin<'B',14,Alternate<5,PushPull>>,
    Pin<'B',15,Alternate<5,PushPull>>),
    Enabled<u8>>;

pub struct Flash {
    ss: PB12<Output<PushPull>>,
    spi: Hspi,
}
impl Flash {
    pub fn new(ss:PB12<Output<PushPull>>, spi:Hspi) -> Flash { Flash { ss, spi } }

    pub fn id(&mut self) -> u32 {
        let mut idc:[u8;4] = [0x00, 0x00, 0x00, 0x00];
        idc[0] = 0xAB;
        self.ss.set_low();
        self.spi.transfer(&mut idc).unwrap();
        self.ss.set_high();
        idc[0] = 0x9F;
        self.ss.set_low();
        self.spi.transfer(&mut idc).unwrap();
        self.ss.set_high();
        return u32::from(idc[1]) << 16 | u32::from(idc[2]) << 8 | u32::from(idc[3]);
    }

    pub fn transfer(&mut self, data: &mut[u8;4]) {
        self.ss.set_low();
        self.spi.transfer(&mut data.as_mut_slice()).unwrap();
        self.ss.set_high();
    }

    pub fn write(&mut self, data: &mut[u8;4]) {
        // ToDo
        // Erase mem first
        self.ss.set_low();
        self.spi.write(&mut data.as_mut_slice()).unwrap();
        self.ss.set_high();
    }

    pub fn read(&mut self, data: &mut[u8;4]) {
        // ToDo
        self.ss.set_low();
        self.spi.transfer(&mut data.as_mut_slice()).unwrap();
        self.ss.set_high();
    }
}
enum FPGAState {
    Prelude,
    Body,
    Post
}

pub struct Fpga {
    bytes: u32,
    state: FPGAState,
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
        Fpga {
            bytes:0,
            state:FPGAState::Prelude,
            ss,
            reset,
            delay,
            bus
        }
    }

    pub fn prog(&mut self, buf: &mut[u8; 512], count: usize) -> bool {
        for c in buf[0..count].iter_mut() {
            self.bytes += 1;
            match self.state {
                FPGAState::Prelude => {
                    if *c == 0x7E as u8 {
                        self.reset();
                        self.select();
                        self.send(*c);
                        self.state = FPGAState::Body;
                    } else {
                        //rprintln!("Prelude Byte {:02x}", * c);
                        continue
                    }
                }
                FPGAState::Body => {
                    self.send(*c);
                    if self.bytes == 135100 {
                       self.state = FPGAState::Post;
                    }
                }
                FPGAState::Post => {
                    self.send(*c);
                }
            }
        }
        // TODO could end up in FPGAState::Body for 0x7E file < 135100 bytes
        return if let FPGAState::Post = self.state {
            self.delay_ms(10_u8);
            for _ in 0..7 {
                self.send(0x00 as u8);
            }
            self.deselect();
            self.bytes = 0;
            self.state = FPGAState::Prelude;
            true
        } else { false }
    }

    pub fn qbus_write(&mut self, buf: &mut[u8; 512], count: usize) -> bool {
        let mut transactions = (count-7) % 16;
        let mut len: i8 = count as i8;
        let mut index: usize = 5;
        while len > 0 {
            match self.state {
                FPGAState::Prelude => {
                    let comad: u8 = buf[2] & 0b01111111;
                    let address: u32 = u32::from(buf[3]) << 8 |
                        u32::from(buf[4]);
                    self.bytes = u32::from(buf[5]) << 24 |
                        u32::from(buf[6]) << 16 |
                        u32::from(buf[7]) << 8 |
                        u32::from(buf[8]);
                    let oldindex = index;
                    index += 16;
                    self.bytes -= self.qbus_command(comad, address, &mut buf[oldindex..index], 16 as usize);
                    len -= 16;
                    transactions -= 1;
                    self.state = FPGAState::Body;
                }
                FPGAState::Body => {
                    for _t in 0..transactions {
                        self.bytes -= self.qbus_data(0 as u8, &mut buf[index..], len as usize);
                    }
                }
                FPGAState::Post => { // is this needed?
                    self.deselect();
                    self.state = FPGAState::Prelude;
                }
            }
        }
        if self.bytes == 0 {
            self.deselect();
            self.state = FPGAState::Prelude;
            true
        } else {false}
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

    pub fn qbus1_send(&mut self, address: u32, byte: u8) {
        let transaction = QspiTransaction {
            iwidth: QspiWidth::SING,
            awidth: QspiWidth::SING,
            dwidth: QspiWidth::SING,//DUAL
            instruction: 0,
            address: Some(address),
            dummy: 0,
            data_len: Some(1),
        };
        self.ss.set_low();
        self.bus.write(&[byte], transaction).unwrap();
        self.ss.set_high();
    }

    pub fn qbus_reg(&mut self, command: u8, address: u32, buf: &mut[u8; 1]) {
        //let mut nibbles= if let (command & 0x80) == 0 {0} else {2}
        let read_nibbles = if command & 0x80 == 0 {0} else {2};
        let transaction = QspiTransaction {
            iwidth: QspiWidth::QUAD,
            awidth: QspiWidth::QUAD,
            dwidth: QspiWidth::QUAD,
            instruction: command,
            address: Some(address),
            dummy: read_nibbles,
            data_len: Some(1),
        };
        self.ss.set_low();
        self.bus.write(buf, transaction).unwrap();
        self.ss.set_high();
    }

    pub fn qbus_address(&mut self, address: u32, byte: u8) {
        let transaction = QspiTransaction {
            iwidth: QspiWidth::QUAD,
            awidth: QspiWidth::QUAD,
            dwidth: QspiWidth::QUAD,
            instruction: 0,
            address: Some(address),
            dummy: 0,
            data_len: Some(1),
        };
        self.bus.write(&[byte], transaction).unwrap();
    }

    pub fn qbus_send(&mut self, buf: &mut[u8; 16], len: usize) {
        let transaction = QspiTransaction {
            iwidth: QspiWidth::NONE,
            awidth: QspiWidth::NONE,
            dwidth: QspiWidth::QUAD,
            instruction: 0,
            address: None,
            dummy: 0,
            data_len: Some(len),
        };
        self.bus.write(buf, transaction).unwrap();
    }

    pub fn qbus_command(&mut self, command: u8, address:u32, buf: &mut[u8], len:usize) -> u32 {
        //const MAX_REG: u32 = 0x0000_FFFF;
        let read_nibbles = if command & 0x80 == 0 {0} else {2*len};
        let transaction = QspiTransaction {
            iwidth: QspiWidth::QUAD,
            awidth: QspiWidth::QUAD,
            dwidth: QspiWidth::QUAD,
            instruction: command,
            address: Some(address),
            dummy: read_nibbles as u8,
            data_len: Some(len),
        };
        //rprintln!("count:{}",_count as u8);
        //let mut buf = [_count as u8];
        self.ss.set_low();
        self.bus.write(buf, transaction).unwrap();
        len as u32
        // self.ss.set_high();
    }

    pub fn qbus_data(&mut self, command:u8, buf: &mut[u8], len: usize) -> u32 {
        let read_nibbles = if command & 0x80 == 0 {0} else {2*len};
        let transaction = QspiTransaction {
            iwidth: QspiWidth::NONE,
            awidth: QspiWidth::NONE,
            dwidth: QspiWidth::QUAD,//DUAL
            instruction: 0,
            address: None,
            dummy: read_nibbles as u8,
            data_len: Some(len),
        };
        //rprintln!("count:{}",_count as u8);
        //let mut buf = [_count as u8];
        self.bus.write(buf, transaction).unwrap();
        len as u32
    }

    pub fn bus_send(&mut self, address:u32, buf: &mut[u8; 16], len: usize) {
        const MAX_REG: u32 = 0x0000_FFFF;
        let transaction = QspiTransaction {
            iwidth: QspiWidth::NONE,
            awidth: QspiWidth::QUAD,
            dwidth: QspiWidth::QUAD,//DUAL
            instruction: 0,
            address: Some(address as u32 & MAX_REG),
            dummy: 0,
            data_len: Some(len),
        };
        //rprintln!("count:{}",_count as u8);
        //let mut buf = [_count as u8];
        self.ss.set_low();
        self.bus.write(buf, transaction).unwrap();
        self.ss.set_high();
    }

    pub fn val_send(&mut self, byte: u8) {
        let transaction = QspiTransaction {
            iwidth: QspiWidth::NONE,
            awidth: QspiWidth::NONE,
            dwidth: QspiWidth::QUAD,//DUAL
            instruction: 0,
            address: None,
            dummy: 0,
            data_len: Some(1),
        };
        self.ss.set_low();
        self.bus.write(&[byte], transaction).unwrap();
        self.ss.set_high();
    }

    pub fn reg_send(&mut self, reg: u8, byte: u8) {
        const MAX_REG: u32 = 0x0000_00FF;
        let transaction = QspiTransaction {
            iwidth: QspiWidth::NONE,
            awidth: QspiWidth::QUAD,
            dwidth: QspiWidth::QUAD,//DUAL
            instruction: 0,
            address: Some(reg as u32 & MAX_REG),
            dummy: 0,
            data_len: Some(1),
        };
        self.ss.set_low();
        self.bus.write(&[byte], transaction).unwrap();
        self.ss.set_high();
    }

    pub fn test_qspi(&mut self) {
        let count : u8 = 255;
        for _count  in 0..count {
            let transaction = QspiTransaction {
                iwidth: QspiWidth::NONE,
                awidth: QspiWidth::NONE,
                dwidth: QspiWidth::QUAD,//DUAL
                instruction: 0,
                address: None,
                dummy: 0,
                data_len: Some(1),
            };
            //rprintln!("count:{}",_count as u8);
            //let mut buf = [_count as u8];
            self.ss.set_low();
            self.bus.write(&[_count], transaction).unwrap();
            self.ss.set_high();
            for _ in 0..100_000 {
                cortex_m::asm::nop();
            }
        }
    }

    // fn transfer(&mut self, byte: u8) {
    //     self.select();
    //     self.send(byte);
    //     self.deselect();
    // }


}

#[app(device = stm32f7xx_hal::pac, peripherals = true, dispatchers = [LP_TIMER1])]
mod app {
    use crate::Fpga;
    use crate::Flash;
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
    use embedded_hal::spi::{Mode, Phase, Polarity};

    // use cortex_m::asm;
    // use cortex_m_rt::entry;
    use panic_probe as _;
    use rtt_target::{rprintln, rtt_init_print};
    use stm32f7xx_hal::spi::Spi;

    /* resources shared across RTIC tasks */
    #[shared]
    struct Shared {
        programmed: bool
    }

    /* resources local to specific RTIC tasks */
    #[local]
    struct Local {
        command: u8,
        ice: Fpga,
        flash: Flash,
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
        let fss = gpiob.pb12.into_push_pull_output();
        let fck = gpiob.pb13.into_alternate::<5>();
        let fso = gpiob.pb14.into_alternate::<5>(); //Miso
        let fsi = gpiob.pb15.into_alternate::<5>(); // Mosi

        let _qsck = gpiob.pb2.into_alternate::<9>()
            .internal_pull_up(true)
            .set_speed(Speed::VeryHigh);
        let mut ss = gpiob.pb6.into_push_pull_output();
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

        let bus = Qspi::new(&mut rcc, device.QUADSPI, 1, 4);

        let mut rcc_constrain = rcc.constrain();

        // Configure clock and freeze it
        let clocks = rcc_constrain
            .cfgr
            .hse(HSEClock::new(25_000_000.Hz(), HSEClockMode::Oscillator))
            .use_pll()
            .use_pll48clk(PLL48CLK::Pllq)
            .sysclk(216_000_000.Hz())
            .mco1(MCO1::Hse)
            .freeze();

        let spi = Spi::new(device.SPI2,(fck, fso, fsi))
            .enable(
                Mode{polarity: Polarity::IdleLow, phase: Phase::CaptureOnFirstTransition},
                1_000_000.Hz(),
                &clocks,
                &mut rcc_constrain.apb1
            );
        let mut flash = Flash{ ss: fss, spi };
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

        // Set mode to green red mode led off on with power green on
        mode_led.set_high();

        // Set fpga select high, deselected
        ss.set_high();

        //_done.set_low();

        let programmed: bool = false;
        let command = 0x00;

        let ice = Fpga::new(ss, reset, delay, bus);
        rprintln!("Flash Id {:08x}", flash.id());
        rprintln!("Init finishing");

        // rtic::pend(Interrupt::OTG_FS)
        // lastly return the shared and local resources, as per RTIC's spec.
        (
            Shared {
                programmed,
            },
            Local {
                command,
                ice,
                flash,
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

    // #[task(shared=[programmed, ice])]
    // fn manage(cx: manage::Context) {
    //     let ice = cx.shared.ice;
    //     let programmed = cx.shared.programmed;
    //
    //     (programmed, ice).lock(|programmed: &mut bool, ice: &mut Fpga| {
    //         if *programmed {
    //             for count in 0..16 {
    //                 ice.delay_ms(100);
    //                 ice.transfer(count as u8);
    //             }
    //         }
    //     });
    // }


    #[task(binds = OTG_FS, shared=[programmed], local=[command, ice, flash, serial, usb_cdc_device])]
    fn usb_event(cx: usb_event::Context) {
        let usb_cdc_device = cx.local.usb_cdc_device; //: &mut UsbDevice<'static, UsbBus<USB>>
        let serial = cx.local.serial;//: &mut SerialPort<'sStatic, UsbBus<USB>>
        let ice = cx.local.ice; //: &mut FPGA
        let command = cx.local.command; // u8
        let mut programmed = cx.shared.programmed; //bool

        if usb_cdc_device.poll(&mut [serial]) {
            let mut buf: [u8; 512] = [0u8; 512];

            match serial.read(&mut buf) {
                Ok(count) if count > 0 => {
                    if *command == 0x00 {
                        *command = buf[0];
                    }
                    match *command {
                        0xFF => {
                            (programmed).lock(|programmed | {
                                *programmed = ice.prog(&mut buf, count);
                                if *programmed {
                                    *command = 0x00;
                                        rprintln!("Programed ILB");
                                    // ice.delay_ms(100_u8);
                                    // ice.test_qspi();
                                }
                            });
                        }
                        0x01 => {
                            rprintln!("val:{}",buf[1]);
                            ice.val_send(buf[1]);
                            *command = 0x00;
                        }
                        0x02 => {
                            rprintln!("reg,val:{},{}",buf[1],buf[2]);
                            ice.reg_send(buf[1], buf[2]);
                            *command = 0x00;
                        }
                        0x03 => { //Write QSPI bit should 0
                            if ice.qbus_write(&mut buf, count) {
                                *command = 0x00;
                                rprintln!("Transferred data to ILB");
                            }
                        }
                        0x04 => { //Read QSPI bit should 1
                            // let comad : u8 = 0b10000000 | (buf[2] & 0b01111111);
                            // let address:u32 = u32::from(buf[3]) << 8 |
                            //                     u32::from(buf[4]);
                            *command = 0x00;
                        }
                        _ => {rprintln!("count:{} bytes",count as u8);}
                    }

                }
                _ => {}
            }

        }
    }
}