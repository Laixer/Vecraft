use stm32h7xx_hal::prelude::*;
use stm32h7xx_hal::{device::I2C1, i2c::I2c};

const EEPROM_I2C_ADDRESS: u8 = 0x50; // 0b0101_0000
const _EEPROM_SIZE: usize = 64_000;
const EEPROM_PAGE_SIZE: usize = 128;
const _EEPROM_PAGE_COUNT: usize = _EEPROM_SIZE / EEPROM_PAGE_SIZE;

/// EEPROM driver
///
/// This driver is used to read and write data to the 24LC512 EEPROM.
pub struct Eeprom {
    i2c: I2c<I2C1>,
}

impl Eeprom {
    pub fn new(i2c: I2c<I2C1>) -> Self {
        Self { i2c }
    }

    pub fn write_page(&mut self, page: usize, buffer: &[u8]) {
        let offset = (page * EEPROM_PAGE_SIZE) as u16;
        self.write_wait(offset, buffer);
    }

    pub fn read_page(&mut self, page: usize, buffer: &mut [u8]) {
        let offset = (page * EEPROM_PAGE_SIZE) as u16;
        self.read_wait(offset, buffer);
    }

    pub fn write(&mut self, address: u16, buffer: &[u8]) -> Result<(), stm32h7xx_hal::i2c::Error> {
        let mut i2c_buffer = [0; EEPROM_PAGE_SIZE + 2];

        let page_boundary = EEPROM_PAGE_SIZE - (address as usize % EEPROM_PAGE_SIZE);
        let buffer_len = buffer.len().min(page_boundary);

        i2c_buffer[0..2].copy_from_slice(&address.to_be_bytes());
        i2c_buffer[2..buffer_len + 2].copy_from_slice(buffer);

        self.i2c
            .write(EEPROM_I2C_ADDRESS, &i2c_buffer[..buffer.len() + 2])
    }

    pub fn write_wait(&mut self, address: u16, buffer: &[u8]) {
        for _ in 0..100 {
            if self.write(address, buffer).is_ok() {
                break;
            }
        }
    }

    pub fn read(
        &mut self,
        address: u16,
        buffer: &mut [u8],
    ) -> Result<(), stm32h7xx_hal::i2c::Error> {
        let buffer_len = buffer.len().min(EEPROM_PAGE_SIZE);

        self.i2c.write_read(
            EEPROM_I2C_ADDRESS,
            &address.to_be_bytes(),
            &mut buffer[..buffer_len],
        )
    }

    pub fn read_wait(&mut self, address: u16, buffer: &mut [u8]) {
        for _ in 0..100 {
            if self.read(address, buffer).is_ok() {
                break;
            }
        }
    }
}
