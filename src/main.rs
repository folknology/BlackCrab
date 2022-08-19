// #![deny(unsafe_code)]
#![deny(warnings)]
#![no_main]
#![no_std]

pub mod control;
pub mod flash;
pub mod fpga;

// extern crate panic_itm;
use rtic::{app};
//use stm32f7xx_hal::gpio::gpiob::{PB3, PB4};
//use stm32f7xx_hal::gpio::gpioc::PC13;
//use stm32f7xx_hal::gpio::gpiod::{PD11};
//use stm32f7xx_hal::delay::Delay;
//use stm32f7xx_hal::gpio::gpioe::{PE11, PE12, PE13};
//use stm32f7xx_hal::prelude::*;

// use stm32f7xx_hal::gpio::{Speed};
//use rtt_target::{rprintln};
use control::{Action, Command};
use fpga::Fpga;
use flash::Flash;

#[app(device = stm32f7xx_hal::pac, peripherals = true, dispatchers = [LP_TIMER1])]
mod app {
    use crate::{Command, Action, Fpga, Flash};
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

        let bus = Qspi::new(&mut rcc, device.QUADSPI, 1, 1);

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
                        Command::STM_FLASH_WRITE => { //TODO
                            rprintln!("STM Flash write not implemented");
                            *command = 0x00;
                        }
                        Command::STM_FLASH_READ => { //TODO
                            rprintln!("STM Flash read not implemented");
                            *command = 0x00;
                        }
                        Command::QSPI_BUS_WRITE => { //Write QSPI bit should 0
                            rprintln!("Transferring data {} bytes", count);
                            if ice.run(*command, &mut buf, count) {
                                *command = 0x00;
                                rprintln!("Transferred data to ILB");

                            }
                        }
                        Command::QSPI_BUS_READ => { //Read QSPI bit should 1
                            rprintln!("ILB qbus Read not implemented");
                            *command = 0x00;
                            let bytes = buf[8] as usize;
                            for b in 0..bytes {
                                buf[b] = b as u8;
                            }
                            let mut write_offset = 0;
                            while write_offset < bytes {
                                match serial.write(&buf[write_offset..bytes]) {
                                    Ok(len) if len > 0 => {
                                        write_offset += len;
                                    }
                                    _ => {}
                                }
                            }
                        }
                        Command::SPI_FLASH_WRITE => {//TODO
                            rprintln!("SPI Flash write not implemented");
                            *command = 0x00;
                        }
                        Command::SPI_FLASH_READ => {//TODO
                            rprintln!("SPI Flash read not implemented");
                            *command = 0x00;
                        }

                        Command::SPI_FPGA_PROGRAM => { //program FPGA over qspi in single mode
                            (programmed).lock(|programmed | {
                                *programmed = ice.run(*command, &mut buf, count);
                                if *programmed {
                                    *command = 0x00;
                                    rprintln!("Programed ILB");

                                    // ice.delay_ms(100_u8);
                                    // ice.test_qspi();
                                }
                            });
                        }
                        _ => {rprintln!("No command, count:{} bytes",count as u8);}
                    }

                }
                _ => {}
            }

        }
    }
}