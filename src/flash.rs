use cortex_m::prelude::{_embedded_hal_blocking_spi_Transfer, _embedded_hal_blocking_spi_Write};
use stm32f7xx_hal::gpio::{Alternate, Pin,PushPull, Output, PB12};
use stm32f7xx_hal::spi::{Spi, Enabled};
use stm32f7xx_hal::pac::SPI2;
use stm32f7xx_hal::qspi::{QspiWidth};

use crate::control::{Action, Command, ActionVar};

pub type Hspi = Spi<SPI2, (
    Pin<'B',13,Alternate<5,PushPull>>,
    Pin<'B',14,Alternate<5,PushPull>>,
    Pin<'B',15,Alternate<5,PushPull>>),
    Enabled<u8>>;

pub struct FlashDevice;

#[allow(dead_code)]
impl FlashDevice {
    pub const RESUME: u8 = 0xAB; //Wake up
    pub const READID: u8 = 0x9F; //Read Id
    pub const READ: u8 = 0x03;
    pub const WRITEENABLE: u8 = 0x06;//Enable writing to flash
    pub const ERASE: u8 =  0x60;//Chip erase  
    pub const ERASE4K: u8 = 0x20;//4k block erase
    pub const ERASE32K: u8 = 0x52;//32k block erase
    pub const ERASE64K: u8 = 0xD8;//64k block erase
    pub const PAGEWRITE: u8 = 0x02;//Page/Byte Program (1..256)
    pub const WRITESR: u8 = 0x01;//Write status register 1/2 bytes
    pub const READSRLSB: u8 = 0x05;//Read status register byte 1
    pub const READSRMSB: u8 = 0x35;//Read status register byte 2
    // Sizes
    pub const PAGE: u16 = 256;//256 Page size, bytes
    pub const SECTOR: u16 = 4096;//4k Sector size, bytes
}

pub struct Flash {
    var: ActionVar,
    pub ss: PB12<Output<PushPull>>,
    pub spi: Hspi,
}

impl Action for Flash {
    fn run(&mut self, command: u8, buf: &mut[u8; 512], count: usize) -> bool {
        self.var.command = command;
        self.var.len = count;
        self.var.index = 0;
        // if self.var.target == 0 {
        //     if !self.prep(buf) { return false };
        // }
        // while self.var.len > 0 {
        //     self.act(buf) ;
        // }

        self.ss.set_low();
        self.spi.write(&mut [FlashDevice::WRITEENABLE]).unwrap();
        self.ss.set_high();

        let mut idc:[u8;5] = [0x00, 0x00, 0x00, 0x00, 0x07];
        idc[0] = FlashDevice::PAGEWRITE;
        self.ss.set_low();
        self.spi.write(&mut idc).unwrap();
        self.ss.set_high();

        idc[4] = 0x00;
        idc[0] = FlashDevice::READ;
        self.ss.set_low();
        self.spi.transfer(&mut idc).unwrap();
        self.ss.set_high();
        buf[0] = idc[4];

        if self.complete() {
            // This can probably be handled by the calling task, which then calls Command::SPI_FLASH_PROGRAM
            // if self.var.command == Command::SPI_FLASH_PROGRAM {
            //     if ice.run(Command::SPI_FPGA_PROGRAM, buf, count) {return true}                    
            //     else {false}
            // }
            true
        } else {false}
    }

    fn prep(&mut self, _buf: &mut[u8; 512]) -> bool {
        //TODO:
        false
    }

    fn act(&mut self, buf: &mut[u8; 512]) {
        let mut head: usize = 9;
        self.var.width = QspiWidth::QUAD;
        self.var.target = usize::from(buf[5]) << 24 |
            usize::from(buf[6]) << 16 |
            usize::from(buf[7]) << 8 |
            usize::from(buf[8]);

        match self.var.command {
            Command::SPI_FLASH_PROGRAM=> {
                //TODO:
            }
            Command::SPI_FLASH_WRITE => {
                //TODO:
            }
            Command::SPI_FLASH_READ => {
                //TODO:
            }
            _ => ()
        }
        head -= 1;
        self.var.index = head;
        self.var.len -= head;
    }

    fn complete(&mut self) -> bool {
        //TODO:
        false
    }
}

impl Flash {
    //TODO convert this to support Device + Action traits
    pub fn new(
        ss:PB12<Output<PushPull>>, 
        spi:Hspi
    ) -> Flash { 
        Flash {
            var:ActionVar::new(Command::NONE, QspiWidth::SING),
            ss,
            spi
        } 
    }

    pub fn id(&mut self) -> u32 {
        let mut idc:[u8;4] = [0x00, 0x00, 0x00, 0x00];
        idc[0] = FlashDevice::RESUME;
        self.ss.set_low();
        self.spi.transfer(&mut idc).unwrap();
        self.ss.set_high();
        idc[0] = FlashDevice::READID;
        self.ss.set_low();
        self.spi.transfer(&mut idc).unwrap();
        self.ss.set_high();
        return u32::from(idc[1]) << 16 | u32::from(idc[2]) << 8 | u32::from(idc[3]);
    }

    pub fn transfer(&mut self, data: &mut[u8;4]) {
        //rprintln!("SPI Flash transfer");
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
