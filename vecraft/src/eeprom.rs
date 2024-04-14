use stm32h7xx_hal::prelude::*;
use stm32h7xx_hal::{device::I2C1, i2c::I2c};

const EEPROM_I2C_ADDRESS: u8 = 0x50; // 0b0101_0000
const EEPROM_SIZE: usize = 64_000;
const EEPROM_PAGE_SIZE: usize = 128;
const EEPROM_PAGE_COUNT: usize = EEPROM_SIZE / EEPROM_PAGE_SIZE;

// FUTURE: Replace I2C1 with a generic I2C peripheral.
/// EEPROM driver
///
/// This driver is used to read and write data to the 24LC512 EEPROM.
pub struct Eeprom {
    i2c: I2c<I2C1>,
}

/// Represents an EEPROM device.
impl Eeprom {
    /// Creates a new instance of `Eeprom` with the specified I2C peripheral.
    ///
    /// # Arguments
    ///
    /// * `i2c` - The I2C peripheral to be used for communication with the EEPROM.
    ///
    /// # Returns
    ///
    /// A new instance of `Eeprom`.
    pub fn new(i2c: I2c<I2C1>) -> Self {
        Self { i2c }
    }

    /// Writes a page of data to the EEPROM.
    ///
    /// # Arguments
    ///
    /// * `page` - The page number to write to.
    /// * `buffer` - The data to be written.
    pub fn write_page(&mut self, page: usize, buffer: &[u8]) {
        let offset = (page.min(EEPROM_PAGE_COUNT) * EEPROM_PAGE_SIZE) as u16;
        self.write_wait(offset, buffer);
    }

    /// Reads a page of data from the EEPROM.
    ///
    /// # Arguments
    ///
    /// * `page` - The page number to read from.
    /// * `buffer` - The buffer to store the read data.
    pub fn read_page(&mut self, page: usize, buffer: &mut [u8]) {
        let offset = (page.min(EEPROM_PAGE_COUNT) * EEPROM_PAGE_SIZE) as u16;
        self.read_wait(offset, buffer);
    }

    /// Writes data to the EEPROM at the specified address.
    ///
    /// # Arguments
    ///
    /// * `address` - The address to write to.
    /// * `buffer` - The data to be written.
    ///
    /// # Returns
    ///
    /// A `Result` indicating whether the write operation was successful or not.
    pub fn write(&mut self, address: u16, buffer: &[u8]) -> Result<(), stm32h7xx_hal::i2c::Error> {
        let mut i2c_buffer = [0; EEPROM_PAGE_SIZE + 2];

        let page_boundary = EEPROM_PAGE_SIZE - (address as usize % EEPROM_PAGE_SIZE);
        let buffer_len = buffer.len().min(page_boundary);

        i2c_buffer[0..2].copy_from_slice(&address.to_be_bytes());
        i2c_buffer[2..buffer_len + 2].copy_from_slice(buffer);

        self.i2c
            .write(EEPROM_I2C_ADDRESS, &i2c_buffer[..buffer.len() + 2])
    }

    /// Writes data to the EEPROM at the specified address and waits for the write operation to complete.
    ///
    /// # Arguments
    ///
    /// * `address` - The address to write to.
    /// * `buffer` - The data to be written.
    pub fn write_wait(&mut self, address: u16, buffer: &[u8]) {
        for _ in 0..1_000 {
            if self.write(address, buffer).is_ok() {
                return;
            }
        }
        panic!("Failed to write to EEPROM");
    }

    /// Reads data from the EEPROM at the specified address.
    ///
    /// # Arguments
    ///
    /// * `address` - The address to read from.
    /// * `buffer` - The buffer to store the read data.
    ///
    /// # Returns
    ///
    /// A `Result` indicating whether the read operation was successful or not.
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

    /// Reads data from the EEPROM at the specified address and waits for the read operation to complete.
    ///
    /// # Arguments
    ///
    /// * `address` - The address to read from.
    /// * `buffer` - The buffer to store the read data.
    pub fn read_wait(&mut self, address: u16, buffer: &mut [u8]) {
        loop {
            if self.read(address, buffer).is_ok() {
                return;
            }
        }
    }
}
