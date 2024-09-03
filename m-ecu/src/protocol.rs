use vecraft::j1939::{Frame, FrameBuilder, IdBuilder, PGN};

/// Engine mode.
#[derive(PartialEq, Eq, Copy, Clone, Debug)]
pub enum VolvoEngineState {
    /// Engine shutdown.
    Shutdown = 0b0000_0111,
    /// Engine starter locked.
    Locked = 0b0100_0111,
    /// Engine running at requested speed.
    Nominal = 0b0100_0011,
    /// Engine starter engaged.
    Starting = 0b1100_0011,
}

pub struct VolvoSpeedRequestMessage {
    /// Engine Mode.
    pub engine_mode: Option<VolvoEngineState>,
    /// Engine Speed.
    pub rpm: Option<u16>,
}

impl VolvoSpeedRequestMessage {
    pub fn from_pdu(pdu: &[u8]) -> Self {
        Self {
            engine_mode: match pdu[1] {
                0b0000_0111 => Some(VolvoEngineState::Shutdown),
                0b0100_0111 => Some(VolvoEngineState::Locked),
                0b0100_0011 => Some(VolvoEngineState::Nominal),
                0b1100_0011 => Some(VolvoEngineState::Starting),
                _ => unreachable!("Invalid engine mode"),
            },
            rpm: Some(pdu[7] as u16 * 10),
        }
    }

    pub fn to_pdu(&self) -> [u8; 8] {
        [
            0x00,
            self.engine_mode.map_or(0, |mode| mode as u8),
            0x1f,
            0x00,
            0x00,
            0x00,
            0x20,
            self.rpm.map_or(0xFF, |rpm| (rpm / 10) as u8),
        ]
    }
}

///////////////////////////////

pub struct VolvoD7E {
    /// Destination address.
    destination_address: u8,
    /// Source address.
    source_address: u8,
    // ems: EngineManagementSystem,
}

impl VolvoD7E {
    /// Construct a new engine management system.
    pub fn new(da: u8, sa: u8) -> Self {
        Self {
            destination_address: da,
            source_address: sa,
            // ems: EngineManagementSystem::new(da, sa),
        }
    }

    /// Request speed control
    pub fn speed_control(&self, state: VolvoEngineState, rpm: u16) -> Frame {
        let message = crate::protocol::VolvoSpeedRequestMessage {
            engine_mode: Some(state),
            rpm: Some(rpm),
        };

        FrameBuilder::new(
            IdBuilder::from_pgn(PGN::ProprietaryB(65_282))
                .priority(3)
                .sa(self.source_address)
                .build(),
        )
        .copy_from_slice(&message.to_pdu())
        .build()
    }
}
