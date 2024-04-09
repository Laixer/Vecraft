// use core::cell::RefCell;

// use cortex_m::{interrupt::Mutex, prelude::*};
use stm32h7xx_hal::prelude::*;
use stm32h7xx_hal::{device::I2C1, i2c::I2c};

const EEPROM_I2C_ADDRESS: u8 = 0x50;
pub const EEPROM_SIZE: u16 = 64_000;

// static EEPROM: Mutex<RefCell<Option<Eeprom>>> = Mutex::new(RefCell::new(None));

// pub fn eeprom_command<F, R>(f: F) -> R
// where
//     F: FnOnce(&mut Eeprom) -> R,
// {
//     cortex_m::interrupt::free(|cs| {
//         let mut g_ref = EEPROM.borrow(cs).borrow_mut();
//         let l_self = g_ref.as_mut().unwrap();
//         f(l_self)
//     })
// }

// pub fn eeprom_init(eeprom: Eeprom) {
//     cortex_m::interrupt::free(|cs| {
//         EEPROM.borrow(cs).replace(Some(eeprom));
//     });
// }

pub struct Eeprom {
    i2c: I2c<I2C1>,
}

impl Eeprom {
    pub fn new(i2c: I2c<I2C1>) -> Self {
        Self { i2c }
    }

    pub fn write(&mut self, address: u16, buffer: &[u8]) {
        let mut i2c_buffer = [0; 32];
        i2c_buffer[..2].copy_from_slice(&address.to_le_bytes());
        i2c_buffer[2..buffer.len() + 2].copy_from_slice(buffer);

        loop {
            if self
                .i2c
                .write(EEPROM_I2C_ADDRESS, &i2c_buffer[..buffer.len() + 2])
                .is_ok()
            {
                break;
            }
        }
    }

    pub fn read(&mut self, address: u16, buffer: &mut [u8]) {
        loop {
            if self
                .i2c
                .write_read(EEPROM_I2C_ADDRESS, &address.to_le_bytes(), buffer)
                .is_ok()
            {
                break;
            }
        }
    }
}
