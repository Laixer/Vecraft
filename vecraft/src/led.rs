
use stm32h7xx_hal::gpio::gpiob::{PB12, PB13, PB14};
use stm32h7xx_hal::gpio::{Output, PushPull};

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

pub struct Color {
    red: bool,
    green: bool,
    blue: bool,
}

impl Color {
    pub const fn new(red: bool, green: bool, blue: bool) -> Self {
        Self { red, green, blue }
    }
}

/// Predefined color orange.
pub const ORANGE: Color = Color::new(true, true, false);
/// Predefined color red.
pub const RED: Color = Color::new(true, false, false);
/// Predefined color green.
pub const GREEN: Color = Color::new(false, true, false);
/// Predefined color blue.
pub const BLUE: Color = Color::new(false, false, true);

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
