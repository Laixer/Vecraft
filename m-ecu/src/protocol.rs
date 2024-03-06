use vecraft::j1939::{Frame, FrameBuilder, IdBuilder, PGN};

/// Engine mode.
#[derive(PartialEq, Eq, Clone, Debug)]
pub enum EngineMode {
    /// Engine shutdown.
    Shutdown = 0b0000_0111,
    /// Engine starter locked.
    _Locked = 0b0100_0111,
    /// Engine running at requested speed.
    _Nominal = 0b0100_0011,
    /// Engine starter engaged.
    _Starting = 0b1100_0011,
}

pub fn volvo_speed_request(engine_mode: EngineMode, rpm: u16) -> Frame {
    FrameBuilder::new(
        IdBuilder::from_pgn(PGN::ProprietaryB(65_282))
            .priority(3)
            .sa(0x11)
            .build(),
    )
    .copy_from_slice(&[
        0x00,
        engine_mode as u8,
        0x1f,
        0x00,
        0x00,
        0x00,
        0x20,
        (rpm as f32 / 10.0) as u8,
    ])
    .build()
}
