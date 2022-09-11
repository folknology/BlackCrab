//Supported commands
pub struct Command;

#[allow(dead_code)]
impl Command {
    pub const NONE: u8 = 0x00;
    pub const QSPI_BUS_WRITE: u8 = 0x03;
    pub const QSPI_BUS_READ: u8 = 0x04;
    pub const SPI_FLASH_READ: u8 = 0x05;
    pub const SPI_FLASH_WRITE: u8 = 0x06;
    pub const SPI_FLASH_ERASE: u8 = 0x07;
    pub const SPI_FLASH_PROGRAM: u8 = 0x08;
    pub const SPI_FPGA_PROGRAM: u8 = 0xFF;
}

pub struct ActionVar {
    pub command: u8,
    pub width: u8,
    pub len: usize,
    pub index: usize,
    pub last_index: usize,
    pub target: usize,
}

impl ActionVar {
    pub fn new(command: u8, width :u8) -> Self {
        Self { command, width, len: 0, index: 0, last_index: 0, target: 0 }
    }
}

pub trait Device {
    fn command(&mut self, command: u8, buf: &mut[u8], len: usize) -> usize;
    fn write(&mut self, buf: &mut[u8], len: usize) -> usize;
    fn read(&mut self, buf: &mut[u8], len: usize) -> usize;
}

pub trait Action {
    fn run(&mut self, command: u8, buf: &mut[u8; 512], count: usize) -> bool;
    fn prep(&mut self, buf: &mut[u8; 512]) -> bool;
    fn act(&mut self, buf: &mut[u8; 512]);
    fn complete(&mut self) -> bool;
}
