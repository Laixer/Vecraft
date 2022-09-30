use core::fmt::Display;

#[derive(PartialEq, Eq, Clone, Copy, Debug)]
pub enum State {
    Boot,
    Nominal,
    Ident,
    Faulty,
}

impl State {
    pub fn as_byte(&self) -> u8 {
        match self {
            State::Boot => 0x12,
            State::Nominal => 0x14,
            State::Ident => 0x16,
            State::Faulty => 0xfa,
        }
    }

    pub fn as_led(&self) -> crate::led::Color {
        match self {
            State::Boot => crate::led::ORANGE,
            State::Nominal => crate::led::GREEN,
            State::Ident => crate::led::BLUE,
            State::Faulty => crate::led::RED,
        }
    }

    #[inline]
    pub fn is_running(&self) -> bool {
        self != &State::Boot
    }
}

impl Display for State {
    fn fmt(&self, f: &mut core::fmt::Formatter<'_>) -> core::fmt::Result {
        core::fmt::Debug::fmt(&self, f)
    }
}

pub struct System {
    ident: bool,
    boot: bool,
    bus_error: bool,
}

impl System {
    pub fn with_boot() -> Self {
        Self {
            ident: false,
            boot: true,
            bus_error: false,
        }
    }

    #[inline]
    pub fn boot_complete(&mut self) {
        self.boot = false;
    }

    #[inline]
    pub fn set_ident(&mut self, on: bool) {
        self.ident = on
    }

    #[inline]
    pub fn set_bus_error(&mut self, on: bool) {
        self.bus_error = on
    }

    pub fn state(&self) -> State {
        if self.boot {
            State::Boot
        } else if self.bus_error {
            State::Faulty
        } else if self.ident {
            State::Ident
        } else {
            State::Nominal
        }
    }
}
