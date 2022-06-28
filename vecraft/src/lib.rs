#![deny(unsafe_code)]
#![deny(warnings)]
#![no_main]
#![no_std]

use stm32h7xx_hal::gpio::gpiob::{PB12, PB13, PB14};
use stm32h7xx_hal::gpio::{Output, PushPull};

pub const PKG_NAME: &str = env!("CARGO_PKG_NAME");
pub const PKG_VERSION: &str = env!("CARGO_PKG_VERSION");

pub type PinLedGreen = PB13<Output<PushPull>>;
pub type PinLedBlue = PB14<Output<PushPull>>;
pub type PinLedRed = PB12<Output<PushPull>>;

pub enum LedState {
    On,
    Off,
    Toggle,
}

pub struct Led {
    green: PinLedGreen,
    blue: PinLedBlue,
    red: PinLedRed,
}

impl Led {
    pub fn new(green: PinLedGreen, blue: PinLedBlue, red: PinLedRed) -> Self {
        let mut led_self = Self { green, blue, red };
        led_self.set_green(LedState::Off);
        led_self.set_blue(LedState::Off);
        led_self.set_red(LedState::Off);
        led_self
    }

    pub fn set_green(&mut self, state: LedState) {
        match state {
            LedState::On => self.green.set_low(),
            LedState::Off => self.green.set_high(),
            LedState::Toggle => self.green.toggle(),
        }
    }

    pub fn set_blue(&mut self, state: LedState) {
        match state {
            LedState::On => self.blue.set_low(),
            LedState::Off => self.blue.set_high(),
            LedState::Toggle => self.blue.toggle(),
        }
    }

    pub fn set_red(&mut self, state: LedState) {
        match state {
            LedState::On => self.red.set_low(),
            LedState::Off => self.red.set_high(),
            LedState::Toggle => self.red.toggle(),
        }
    }
}
