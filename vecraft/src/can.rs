pub type FdCan1 =
    fdcan::FdCan<stm32h7xx_hal::can::Can<stm32h7xx_hal::stm32::FDCAN1>, fdcan::NormalOperationMode>;

pub struct J1939Message {
    pub pgn: u16,
    pub data: [u8; 8],
}

impl Default for J1939Message {
    fn default() -> Self {
        Self {
            pgn: Default::default(),
            data: [0xff; 8],
        }
    }
}

pub struct J1939Session {
    bus: Bus,
    address: u8,
}

impl J1939Session {
    pub fn new(address: u8, bus: Bus) -> Self {
        Self { address, bus }
    }

    pub fn send(&mut self, message: J1939Message) {
        let id = j1939::IdBuilder::from_pgn(message.pgn)
            .sa(self.address)
            .build();

        self.bus.write(id.as_raw(), &message.data)
    }

    pub fn recv(&mut self, message: &mut J1939Message) {
        if let Some(id) = self.bus.read(&mut message.data) {
            let id = j1939::Id::new(id);
            message.pgn = id.pgn()
        }
    }
}

pub struct Bus(FdCan1);

impl Bus {
    pub fn new(interface: FdCan1) -> Self {
        Self(interface)
    }

    pub fn write(&mut self, address: u32, buffer: &[u8]) {
        let header = fdcan::frame::TxFrameHeader {
            len: buffer.len() as u8,
            id: fdcan::id::ExtendedId::new(address).unwrap().into(),
            frame_format: fdcan::frame::FrameFormat::Standard,
            bit_rate_switching: false,
            marker: None,
        };

        self.0.transmit(header, &buffer).ok();
    }

    pub fn read(&mut self, buffer: &mut [u8]) -> Option<u32> {
        let id = if let Ok(header) = self.0.receive0(buffer) {
            let header = header.unwrap();
            if let fdcan::id::Id::Extended(id) = header.id {
                Some(id.as_raw())
            } else {
                None
            }
        } else {
            None
        };

        self.0
            .clear_interrupt(fdcan::interrupt::Interrupt::RxFifo0NewMsg);
        self.0
            .clear_interrupt(fdcan::interrupt::Interrupt::RxFifo0Full);

        id
    }
}
