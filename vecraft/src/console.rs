use core::fmt;

use stm32h7xx_hal::hal::serial::Write;
use stm32h7xx_hal::serial::Serial;

pub struct Console(Serial<stm32h7xx_hal::device::USART2>);

impl Console {
    pub fn new(serial: Serial<stm32h7xx_hal::device::USART2>) -> Self {
        Self(serial)
    }
}

impl fmt::Write for Console {
    fn write_str(&mut self, s: &str) -> fmt::Result {
        s.as_bytes()
            .iter()
            .map(|c| {
                if c == &b'\n' {
                    stm32h7xx_hal::block!(self.0.write(b'\r'))?;
                }

                stm32h7xx_hal::block!(self.0.write(*c))
            })
            .last();
        Ok(())
    }
}
