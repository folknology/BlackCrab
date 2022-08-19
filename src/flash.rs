use cortex_m::prelude::{_embedded_hal_blocking_spi_Transfer, _embedded_hal_blocking_spi_Write};
use stm32f7xx_hal::gpio::{Alternate, Pin,PushPull, Output, PB12};
use stm32f7xx_hal::spi::{Spi, Enabled};
use stm32f7xx_hal::pac::SPI2;
//use crate::control::{ActionVar, Action, Command};

pub type Hspi = Spi<SPI2, (
    Pin<'B',13,Alternate<5,PushPull>>,
    Pin<'B',14,Alternate<5,PushPull>>,
    Pin<'B',15,Alternate<5,PushPull>>),
    Enabled<u8>>;

pub struct Flash {
    pub ss: PB12<Output<PushPull>>,
    pub spi: Hspi,
}
impl Flash {
    //TODO convert this to support Device + Action traits
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
