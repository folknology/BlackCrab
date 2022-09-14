
use stm32f7xx_hal::timer::{SysDelay};
use stm32f7xx_hal::gpio::{PushPull, Output, PB6, PB4};
use stm32f7xx_hal::qspi::{Qspi, QspiTransaction, QspiWidth};
use stm32f7xx_hal::prelude::_embedded_hal_blocking_delay_DelayMs;

use crate::control::{ActionVar, Action, Command, Device};


pub struct Fpga {
    var: ActionVar,
    ss: PB6<Output<PushPull>>,
    reset: PB4<Output<PushPull>>,
    delay: SysDelay,
    bus: Qspi
}

impl Action for Fpga {
    fn run(&mut self, command: u8, buf: &mut[u8; 512], count: usize ) -> bool {
        self.var.command = command;
        self.var.len = count;
        self.var.index = 0;
        if self.var.target == 0 {
            if !self.prep(buf) { return false };
        }
        while self.var.len > 0 {
            self.act(buf) ;
        }
        if self.complete() {
            if self.var.command == Command::SPI_FPGA_PROGRAM {
                for _ in 0..5 { // Flush FPGA serial buffer and delay
                    self.write(&mut[0x00, 0x00], 2);
                }
                self.delay_ms(10_u8);
            }
            self.deselect();
            true
        } else {false}
    }

    fn prep(&mut self, buf: &mut[u8; 512]) -> bool {
        let mut head: usize = 9;
        self.var.width = QspiWidth::QUAD;
        self.bus.prescale(7);
        self.var.target = usize::from(buf[5]) << 24 |
            usize::from(buf[6]) << 16 |
            usize::from(buf[7]) << 8 |
            usize::from(buf[8]);

        match self.var.command {
            Command::SPI_FPGA_PROGRAM => {
                self.var.width = QspiWidth::SING;
                self.bus.prescale(15);
                self.var.target = crate::IMGSIZE;//135100 as usize;
                head = 0;
                while buf[head] != 0x7E as u8  {
                    if head < self.var.len { head += 1 } else { return false }
                }
                self.select();
                self.var.target-= head - 1;
                self.var.target -= self.write(&mut buf[head-1..head], 1);
                self.reset();
            }
            Command::QSPI_BUS_WRITE => {
                buf[2] &= 0b01111111;
                self.select();
                self.command(buf[2],&mut buf[3..5], 2);
            }
            Command::QSPI_BUS_READ => {
                buf[2] |= 0b10000000;
                self.select();
                self.command(buf[2],&mut buf[3..5], 2);
            }
            _ => ()
        }
        self.var.index = head;
        self.var.len -= head;
        true
    }

    fn act(&mut self, buf: &mut[u8; 512]) {
        let bytes = if self.var.len >= 2 { 2 } else { self.var.len };
        for _t in 0..self.var.len/bytes {
            self.var.last_index =  self.var.index;
            self.var.index += bytes;
            self.var.target -= self.write(&mut buf[self.var.last_index..self.var.index], bytes);
            self.var.len -= bytes;
        }
    }

    fn complete(&mut self) -> bool {
        self.var.target == 0
    }
}

impl Device for Fpga {
    fn command(&mut self, command: u8, buf: &mut [u8], len: usize) -> usize {
        let transaction = QspiTransaction {
            iwidth: self.var.width,
            awidth: QspiWidth::NONE,
            dwidth: self.var.width,
            instruction: command,
            address: None,
            dummy: 0,
            data_len: Some(len),
        };
        self.bus.write(buf, transaction).unwrap();
        len
    }
    fn write(&mut self, buf: &mut [u8], len: usize) -> usize {
        let transaction = QspiTransaction {
            iwidth: QspiWidth::NONE,
            awidth: QspiWidth::NONE,
            dwidth: self.var.width,
            instruction: 0,
            address: None,
            dummy: 0,
            data_len: Some(len),
        };
        self.bus.write(buf, transaction).unwrap();
        len
    }
    fn read(&mut self, buf: &mut [u8], len: usize) -> usize {
        let transaction = QspiTransaction {
            iwidth: QspiWidth::NONE,
            awidth: QspiWidth::NONE,
            dwidth: self.var.width,
            instruction: 0,
            address: None,
            dummy: 0,
            data_len: Some(len),
        };
        self.bus.read(buf, transaction).unwrap();
        len
    }
}

impl Fpga {
    pub fn new(
               ss: PB6<Output<PushPull>>,
               reset: PB4<Output<PushPull>>,
               delay: SysDelay,
               bus : Qspi) -> Fpga {
        Fpga {
            var: ActionVar::new(Command::NONE, QspiWidth::SING),
            ss,
            reset,
            delay,
            bus
        }
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
