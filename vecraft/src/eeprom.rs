use stm32h7xx_hal::prelude::*;
use stm32h7xx_hal::{device::I2C1, i2c::I2c};

const EEPROM_I2C_ADDRESS: u8 = 0x50;
pub const EEPROM_SIZE: u16 = 64_000;

pub struct Eeprom {
    i2c: I2c<I2C1>,
}

impl Eeprom {
    pub fn new(i2c: I2c<I2C1>) -> Self {
        Self { i2c }
    }

    pub fn write(&mut self, address: u16, buffer: &[u8]) -> Result<(), stm32h7xx_hal::i2c::Error> {
        // let mut i2c_buffer = [0; 32];
        // i2c_buffer[..2].copy_from_slice(&address.to_le_bytes());
        // i2c_buffer[2..buffer.len() + 2].copy_from_slice(buffer);

        let mut i2c_buffer = [0; 34];
        i2c_buffer[0] = (address & 0xFF) as u8;
        i2c_buffer[1] = (address >> 8) as u8;
        i2c_buffer[2..buffer.len() + 2].copy_from_slice(buffer);

        self.i2c
            .write(EEPROM_I2C_ADDRESS, &i2c_buffer[..buffer.len() + 2])
    }

    pub fn read(
        &mut self,
        address: u16,
        buffer: &mut [u8],
    ) -> Result<(), stm32h7xx_hal::i2c::Error> {
        self.i2c
            .write_read(EEPROM_I2C_ADDRESS, &address.to_le_bytes(), buffer)
    }
}
