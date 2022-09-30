use usb_device::class_prelude::*;

const AVIC_BRIDGE_CAN_IFACE_CLASS: u8 = 0xfe;
const AVIC_BRIDGE_CAN_IFACE_SUBCLASS_CONTROL: u8 = 0x4;
const AVIC_BRIDGE_CAN_IFACE_SUBCLASS_DATA: u8 = 0xc;
const AVIC_BRIDGE_CAN_IFACE_PROTO: u8 = 0;

const USB2_MAX_BULK_SIZE: u16 = 64;

pub enum ClassInterface {
    Interface0,
    Interface1,
}

pub struct AvicClass<'a, B: UsbBus> {
    data_if: InterfaceNumber,
    data_out_ep: EndpointOut<'a, B>,
    data_in_ep: EndpointIn<'a, B>,
    data2_if: InterfaceNumber,
    data2_out_ep: EndpointOut<'a, B>,
    data2_in_ep: EndpointIn<'a, B>,
    on_config_bittiming: Option<fn()>,
    on_config_termination: Option<fn()>,
}

impl<'a, B: UsbBus> AvicClass<'a, B> {
    pub fn new(allocator: &'a UsbBusAllocator<B>) -> Self {
        Self {
            data_if: allocator.interface(),
            data_out_ep: allocator.bulk(USB2_MAX_BULK_SIZE),
            data_in_ep: allocator.bulk(USB2_MAX_BULK_SIZE),
            data2_if: allocator.interface(),
            data2_out_ep: allocator.bulk(USB2_MAX_BULK_SIZE),
            data2_in_ep: allocator.bulk(USB2_MAX_BULK_SIZE),
            on_config_bittiming: None,
            on_config_termination: None,
        }
    }

    pub fn read_frame(&self) -> Result<crate::usb_frame::Frame, UsbError> {
        let mut buffer = [0; 64];

        match self.data_out_ep.read(&mut buffer) {
            Ok(_) => Ok(crate::usb_frame::Frame::parse(&buffer)),
            Err(e) => Err(e),
        }
    }

    pub fn write_frame(&self, interface: ClassInterface, data: &[u8]) -> Result<usize, UsbError> {
        match interface {
            ClassInterface::Interface0 => self.data_in_ep.write(data),
            ClassInterface::Interface1 => self.data2_in_ep.write(data),
        }
    }

    pub fn configure_bittiming(&mut self, f: fn()) {
        self.on_config_bittiming = Some(f);
    }

    pub fn configure_termination(&mut self, f: fn()) {
        self.on_config_termination = Some(f);
    }
}

impl<'a, B: UsbBus> UsbClass<B> for AvicClass<'a, B> {
    fn get_configuration_descriptors(
        &self,
        writer: &mut DescriptorWriter,
    ) -> usb_device::Result<()> {
        writer.iad(
            self.data_if,
            2,
            AVIC_BRIDGE_CAN_IFACE_CLASS,
            AVIC_BRIDGE_CAN_IFACE_SUBCLASS_CONTROL,
            AVIC_BRIDGE_CAN_IFACE_PROTO,
        )?;

        writer.interface(
            self.data_if,
            AVIC_BRIDGE_CAN_IFACE_CLASS,
            AVIC_BRIDGE_CAN_IFACE_SUBCLASS_DATA,
            AVIC_BRIDGE_CAN_IFACE_PROTO,
        )?;

        writer.endpoint(&self.data_in_ep)?;
        writer.endpoint(&self.data_out_ep)?;

        writer.interface(
            self.data2_if,
            AVIC_BRIDGE_CAN_IFACE_CLASS,
            AVIC_BRIDGE_CAN_IFACE_SUBCLASS_DATA,
            AVIC_BRIDGE_CAN_IFACE_PROTO,
        )?;

        writer.endpoint(&self.data2_in_ep)?;
        writer.endpoint(&self.data2_out_ep)?;

        Ok(())
    }

    /// Accept control packets from the host.
    fn control_out(&mut self, xfer: ControlOut<B>) {
        let req = xfer.request();

        if !(req.request_type == control::RequestType::Class
            && req.recipient == control::Recipient::Interface
            && (req.index == u8::from(self.data_if) as u16
                || req.index == u8::from(self.data2_if) as u16))
        {
            return;
        }

        /// Set CAN bit timing.
        pub const USB_CAN_REQ_SET_BITTIMING: u8 = 5;
        /// Set CAN line termination resistance.
        pub const USB_CAN_REQ_SET_TERMINATION: u8 = 6;

        match req.request {
            USB_CAN_REQ_SET_BITTIMING => {
                if xfer.data().len() != 4 {
                    xfer.reject().unwrap();
                } else {
                    let _bit_timing = u32::from_le_bytes(xfer.data().try_into().unwrap());
                    if let Some(on_config_bittiming) = self.on_config_bittiming {
                        on_config_bittiming()
                    }

                    xfer.accept().unwrap();
                }
            }
            USB_CAN_REQ_SET_TERMINATION => {
                if xfer.data().len() != 1 {
                    xfer.reject().unwrap();
                } else {
                    let _term_resistance = u8::from_le_bytes(xfer.data().try_into().unwrap());
                    if let Some(on_config_termination) = self.on_config_termination {
                        on_config_termination()
                    }
                    xfer.accept().unwrap();
                }
            }
            _ => xfer.reject().unwrap(),
        }
    }
}
