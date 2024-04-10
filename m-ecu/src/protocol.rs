/// Engine mode.
#[derive(PartialEq, Eq, Copy, Clone, Debug)]
pub enum EngineMode {
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
    pub engine_mode: Option<EngineMode>,
    /// Engine Speed.
    pub rpm: Option<u16>,
}

impl VolvoSpeedRequestMessage {
    pub fn from_pdu(pdu: &[u8]) -> Self {
        Self {
            engine_mode: match pdu[1] {
                0b0000_0111 => Some(EngineMode::Shutdown),
                0b0100_0111 => Some(EngineMode::Locked),
                0b0100_0011 => Some(EngineMode::Nominal),
                0b1100_0011 => Some(EngineMode::Starting),
                _ => None,
            },
            rpm: Some(u16::from_be_bytes([pdu[6], pdu[7]]) * 10),
        }
    }

    #[allow(dead_code)]
    pub fn to_pdu(&self) -> [u8; 8] {
        [
            0x00,
            self.engine_mode.map_or(0, |mode| mode as u8),
            0x1f,
            0x00,
            0x00,
            0x00,
            0x20,
            (self.rpm.unwrap_or(0) / 10) as u8,
        ]
    }
}

// /// Engine mode.
// #[derive(PartialEq, Eq, Clone, Debug)]
// pub enum EngineMode {
//     /// Engine shutdown.
//     Shutdown = 0b0000_0111,
//     /// Engine starter locked.
//     _Locked = 0b0100_0111,
//     /// Engine running at requested speed.
//     _Nominal = 0b0100_0011,
//     /// Engine starter engaged.
//     _Starting = 0b1100_0011,
// }

// pub fn volvo_speed_request(engine_mode: EngineMode, rpm: u16) -> Frame {
//     FrameBuilder::new(
//         IdBuilder::from_pgn(PGN::ProprietaryB(65_282))
//             .priority(3)
//             .sa(0x11)
//             .build(),
//     )
//     .copy_from_slice(&[
//         0x00,
//         engine_mode as u8,
//         0x1f,
//         0x00,
//         0x00,
//         0x00,
//         0x20,
//         (rpm as f32 / 10.0) as u8,
//     ])
//     .build()
// }
