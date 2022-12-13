use core::num::{NonZeroU16, NonZeroU8};

use fdcan::{
    config::{GlobalFilter, NominalBitTiming, NonMatchingFilter},
    filter::{Action, ExtendedFilter, ExtendedFilterSlot, FilterType},
    interrupt::{Interrupt, InterruptLine},
    NormalOperationMode,
};
use stm32h7xx_hal::device::{FDCAN1, FDCAN2};

pub type FdCan1 = fdcan::FdCan<stm32h7xx_hal::can::Can<FDCAN1>, fdcan::NormalOperationMode>;
pub type FdCan2 = fdcan::FdCan<stm32h7xx_hal::can::Can<FDCAN2>, fdcan::NormalOperationMode>;

pub const BITRATE_1M: NominalBitTiming = NominalBitTiming {
    prescaler: unsafe { NonZeroU16::new_unchecked(2) },
    seg1: unsafe { NonZeroU8::new_unchecked(13) },
    seg2: unsafe { NonZeroU8::new_unchecked(2) },
    sync_jump_width: unsafe { NonZeroU8::new_unchecked(1) },
};
pub const BITRATE_500K: NominalBitTiming = NominalBitTiming {
    prescaler: unsafe { NonZeroU16::new_unchecked(4) },
    seg1: unsafe { NonZeroU8::new_unchecked(13) },
    seg2: unsafe { NonZeroU8::new_unchecked(2) },
    sync_jump_width: unsafe { NonZeroU8::new_unchecked(1) },
};
pub const BITRATE_250K: NominalBitTiming = NominalBitTiming {
    prescaler: unsafe { NonZeroU16::new_unchecked(8) },
    seg1: unsafe { NonZeroU8::new_unchecked(13) },
    seg2: unsafe { NonZeroU8::new_unchecked(2) },
    sync_jump_width: unsafe { NonZeroU8::new_unchecked(1) },
};
pub const BITRATE_125K: NominalBitTiming = NominalBitTiming {
    prescaler: unsafe { NonZeroU16::new_unchecked(16) },
    seg1: unsafe { NonZeroU8::new_unchecked(13) },
    seg2: unsafe { NonZeroU8::new_unchecked(2) },
    sync_jump_width: unsafe { NonZeroU8::new_unchecked(1) },
};
pub const BITRATE_100K: NominalBitTiming = NominalBitTiming {
    prescaler: unsafe { NonZeroU16::new_unchecked(20) },
    seg1: unsafe { NonZeroU8::new_unchecked(13) },
    seg2: unsafe { NonZeroU8::new_unchecked(2) },
    sync_jump_width: unsafe { NonZeroU8::new_unchecked(1) },
};
pub const BITRATE_50K: NominalBitTiming = NominalBitTiming {
    prescaler: unsafe { NonZeroU16::new_unchecked(40) },
    seg1: unsafe { NonZeroU8::new_unchecked(13) },
    seg2: unsafe { NonZeroU8::new_unchecked(2) },
    sync_jump_width: unsafe { NonZeroU8::new_unchecked(1) },
};

pub struct CanBuilder<I: fdcan::Instance, P: stm32h7xx_hal::hal::digital::v2::OutputPin> {
    fdcan: fdcan::FdCan<I, fdcan::ConfigMode>,
    term: P,
}

impl<I: fdcan::Instance, P: stm32h7xx_hal::hal::digital::v2::OutputPin> CanBuilder<I, P> {
    pub fn new(mut fdcan: fdcan::FdCan<I, fdcan::ConfigMode>, mut term: P) -> Self {
        term.set_low().ok();

        fdcan.set_protocol_exception_handling(false);

        fdcan.set_global_filter(
            GlobalFilter::default()
                .set_handle_standard_frames(NonMatchingFilter::Reject)
                .set_reject_remote_standard_frames(true)
                .set_reject_remote_extended_frames(true),
        );

        fdcan.enable_interrupt_line(InterruptLine::_0, true);
        fdcan.enable_interrupt(Interrupt::RxFifo0NewMsg);
        fdcan.enable_interrupt(Interrupt::ErrPassive);
        fdcan.enable_interrupt(Interrupt::BusOff);

        Self { fdcan, term }
    }

    pub fn set_termination(mut self, on: bool) -> Self {
        if on {
            self.term.set_low().ok();
        } else {
            self.term.set_high().ok();
        }
        self
    }

    pub fn set_bit_timing(mut self, btr: NominalBitTiming) -> Self {
        self.fdcan.set_nominal_bit_timing(btr);
        self
    }

    pub fn set_net_address_filter(mut self, address: u8) -> Self {
        let filter = j1939::can::destination_address_filter(address);

        self.fdcan.set_extended_filter(
            ExtendedFilterSlot::_0,
            ExtendedFilter {
                filter: FilterType::BitMask {
                    filter: filter.filter,
                    mask: filter.mask,
                },
                action: Action::StoreInFifo0,
            },
        );
        self.fdcan
            .set_extended_filter(ExtendedFilterSlot::_1, ExtendedFilter::reject_all());
        self
    }

    pub fn build(self) -> Can<I, NormalOperationMode> {
        Can::new(self.fdcan.into_normal())
    }
}

pub struct Can<I: fdcan::Instance, M>(fdcan::FdCan<I, M>);

impl<I, M> Can<I, M>
where
    I: fdcan::Instance,
    M: fdcan::Transmit + fdcan::Receive,
{
    pub fn new(interface: fdcan::FdCan<I, M>) -> Self {
        Self(interface)
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
        let mut builder = j1939::FrameBuilder::default();

        if let Some((id, len)) = self.read(builder.as_mut()) {
            builder = builder.id(j1939::Id::new(id)).set_len(len);
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

    pub fn read(&mut self, buffer: &mut [u8]) -> Option<(u32, usize)> {
        let id = if let Ok(header) = self.0.receive0(buffer) {
            let header = header.unwrap();
            if let fdcan::id::Id::Extended(id) = header.id {
                Some((id.as_raw(), header.len as usize))
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
