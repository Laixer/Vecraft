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

pub struct RGBLed {
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

/// Predefined color red.
pub const RED: Color = Color::new(true, false, false);
/// Predefined color green.
pub const GREEN: Color = Color::new(false, true, false);
/// Predefined color blue.
pub const BLUE: Color = Color::new(false, false, true);
/// Predefined color yellow.
pub const YELLOW: Color = Color::new(true, true, false);
/// Predefined color magenta.
pub const MAGENTA: Color = Color::new(true, false, true);
/// Predefined color cyan.
pub const CYAN: Color = Color::new(false, true, true);
/// Predefined color white.
pub const _WHITE: Color = Color::new(true, true, true);

impl RGBLed {
    pub fn new(green: PinLedGreen, blue: PinLedBlue, red: PinLedRed) -> Self {
        let mut led_self = Self { green, blue, red };
        led_self.set_green(&LedState::Off);
        led_self.set_blue(&LedState::Off);
        led_self.set_red(&LedState::Off);
        led_self
    }

    pub fn new_with_color(
        green: PinLedGreen,
        blue: PinLedBlue,
        red: PinLedRed,
        color: &Color,
    ) -> Self {
        let mut led_self = Self { green, blue, red };
        led_self.set_color(color, &LedState::On);
        led_self
    }

    pub fn set_color(&mut self, color: &Color, state: &LedState) {
        if color.red {
            self.set_red(state)
        } else {
            self.set_red(&LedState::Off)
        }
        if color.green {
            self.set_green(state)
        } else {
            self.set_green(&LedState::Off)
        }
        if color.blue {
            self.set_blue(state)
        } else {
            self.set_blue(&LedState::Off)
        }
    }

    pub fn set_green(&mut self, state: &LedState) {
        match state {
            LedState::On => self.green.set_low(),
            LedState::Off => self.green.set_high(),
            LedState::Toggle => self.green.toggle(),
        }
    }

    pub fn set_blue(&mut self, state: &LedState) {
        match state {
            LedState::On => self.blue.set_low(),
            LedState::Off => self.blue.set_high(),
            LedState::Toggle => self.blue.toggle(),
        }
    }

    pub fn set_red(&mut self, state: &LedState) {
        match state {
            LedState::On => self.red.set_low(),
            LedState::Off => self.red.set_high(),
            LedState::Toggle => self.red.toggle(),
        }
    }
}
