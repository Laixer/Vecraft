pub type FdCan1 =
    fdcan::FdCan<stm32h7xx_hal::can::Can<stm32h7xx_hal::stm32::FDCAN1>, fdcan::NormalOperationMode>;
pub type FdCan2 =
    fdcan::FdCan<stm32h7xx_hal::can::Can<stm32h7xx_hal::stm32::FDCAN2>, fdcan::NormalOperationMode>;

pub struct Bus<I: fdcan::Instance, M>(fdcan::FdCan<I, M>);

impl<I, M> Bus<I, M>
where
    I: fdcan::Instance,
    M: fdcan::Transmit + fdcan::Receive,
{
    pub fn new(interface: fdcan::FdCan<I, M>) -> Self {
        Self(interface)
    }

    pub fn error(&self) -> fdcan::ErrorCounters {
        self.0.error_counters()
    }

    pub fn get_protocol_status(&self) -> fdcan::ProtocolStatus {
        self.0.get_protocol_status()
    }

    pub fn is_bus_error(&mut self) -> bool {
        let bus_error = self
            .0
            .has_interrupt(fdcan::interrupt::Interrupt::ErrPassive)
            || self.0.has_interrupt(fdcan::interrupt::Interrupt::BusOff);

        self.0
            .clear_interrupt(fdcan::interrupt::Interrupt::ErrPassive);
        self.0.clear_interrupt(fdcan::interrupt::Interrupt::BusOff);

        bus_error
    }

    pub fn is_bus_ok(&mut self) -> bool {
        let protocol_status = self.0.get_protocol_status();

        !protocol_status.bus_off_status
            && !protocol_status.error_passive_state
            && !protocol_status.error_warning
    }

    pub fn send(&mut self, frame: j1939::Frame) {
        self.write(frame.id().as_raw(), frame.pdu())
    }

    pub fn recv(&mut self) -> Option<j1939::Frame> {
        if self
            .0
            .has_interrupt(fdcan::interrupt::Interrupt::RxFifo0NewMsg)
        {
            let mut builder = j1939::FrameBuilder::default();

            if let Some(id) = self.read(builder.as_mut()) {
                builder = builder.id(j1939::Id::new(id));
            }

            Some(builder.build())
        } else {
            None
        }
    }

    pub fn write(&mut self, address: u32, buffer: &[u8]) {
        let header = fdcan::frame::TxFrameHeader {
            len: buffer.len() as u8,
            id: fdcan::id::ExtendedId::new(address).unwrap().into(),
            frame_format: fdcan::frame::FrameFormat::Standard,
            bit_rate_switching: false,
            marker: None,
        };

        self.0.transmit(header, buffer).ok();
    }

    pub fn inner(&mut self) -> &mut fdcan::FdCan<I, M> {
        &mut self.0
    }

    pub fn has_new_message(&mut self) -> bool {
        self.0
            .has_interrupt(fdcan::interrupt::Interrupt::RxFifo0NewMsg)
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

        id
    }
}
