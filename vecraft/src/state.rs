use core::fmt::{Debug, Display, Formatter};

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum State {
    Nominal,
    Ident,
    ApplicationSpecific,
    FaultyGenericError,
    FaultyBusError,
    ConfigurationError,
}

impl State {
    pub fn as_byte(&self) -> u8 {
        match self {
            State::Nominal => 0x14,
            State::Ident => 0x16,
            State::ApplicationSpecific => 0x17,
            State::FaultyGenericError => 0xfa,
            State::FaultyBusError => 0xfb,
            State::ConfigurationError => 0xfc,
        }
    }

    pub fn as_led(&self) -> crate::led::Color {
        match self {
            State::Nominal => crate::led::GREEN,
            State::Ident => crate::led::BLUE,
            State::ApplicationSpecific => crate::led::CYAN,
            State::FaultyGenericError => crate::led::YELLOW,
            State::FaultyBusError => crate::led::RED,
            State::ConfigurationError => crate::led::MAGENTA,
        }
    }
}

impl Display for State {
    fn fmt(&self, f: &mut Formatter<'_>) -> core::fmt::Result {
        Debug::fmt(&self, f)
    }
}

/// Keeps track of the system state.
///
/// The system can hold any of the following states at the same time:
/// - Nominal
/// - Identification mode
/// - Application-specific state
/// - Faulty generic error
/// - Faulty bus error
/// - Configuration error
///
/// However, a single state is returned depending on the current state of the system
/// and the priority of the states.
pub struct System {
    /// Enable or disable the identification mode.
    ident: bool,
    /// Enable or disable the application-specific state.
    application_specific: bool,
    /// Indicate a bus error.
    bus_error: bool,
    /// Indicate configuration error.
    configuration_error: bool,
}

impl System {
    /// Prepare the system for boot.
    pub fn boot() -> Self {
        Self {
            ident: false,
            application_specific: false,
            bus_error: false,
            configuration_error: false,
        }
    }

    /// Enable or disable the identification mode.
    #[inline]
    pub fn set_ident(&mut self, on: bool) {
        self.ident = on
    }

    /// Enable or disable the application-specific state.
    #[inline]
    pub fn set_application_specific(&mut self, on: bool) {
        self.application_specific = on
    }

    /// Enable or disable the bus error.
    #[inline]
    pub fn set_bus_error(&mut self, on: bool) {
        self.bus_error = on
    }

    /// Enable or disable the configuration error.
    #[inline]
    pub fn set_configuration_error(&mut self, on: bool) {
        self.configuration_error = on
    }

    /// Get the current state of the system.
    pub fn state(&self) -> State {
        if self.ident {
            State::Ident
        } else if self.bus_error {
            State::FaultyBusError
        } else if self.configuration_error {
            State::ConfigurationError
        } else if self.application_specific {
            State::ApplicationSpecific
        } else {
            State::Nominal
        }
    }
}
