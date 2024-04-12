// EEPROM
// Header + version + firmware mode: [0x4C, 0x58, 0x52, 0x1, 0x16, 0xFF, 0xFF, 0xF]
// Serial number: [0x1, 0x2, 0x3, 0x4, 0x5, 0x6, 0x7, 0x8]

// Uart
// Selection : [2]
// Baudrate  : [0, 194, 1, 0]

// Canbus 1
// bit timing  : [144, 208, 3, 0]
// Termination : [0]

// J1939
// Address : [0x4A]
// Name    : [0x1, 0x2, 0x3, 0x4, 0x5, 0x6, 0x7, 0x8]

pub const VECRAFT_CONFIG_PAGE: usize = 1;

#[derive(Copy, Clone, Debug)]
pub struct VecraftConfig {
    ecu_mode: u8,
    serial_number: [u8; 8],
    pub uart_selected: u8,
    pub uart_baudrate: u32,
    pub canbus1_bitrate: u32,
    pub canbus1_termination: bool,
    pub j1939_address: u8,
    j1939_name: [u8; 8],
}

impl VecraftConfig {
    pub fn ecu_mode(&self) -> u8 {
        self.ecu_mode
    }

    pub fn serial_number(&self) -> (u32, u32) {
        (
            u32::from_le_bytes(self.serial_number[0..4].try_into().unwrap()),
            u32::from_le_bytes(self.serial_number[4..8].try_into().unwrap()),
        )
    }

    pub fn j1939_address(&self) -> u8 {
        self.j1939_address
    }

    pub fn j1939_name(&self) -> j1939::Name {
        j1939::Name::from_bytes(self.j1939_name)
    }
}

// TODO: Replace with an actual error type
impl TryFrom<&[u8]> for VecraftConfig {
    type Error = u8;

    fn try_from(value: &[u8]) -> Result<Self, Self::Error> {
        if &value[0..3] != b"LXR" {
            return Err(1);
        }
        if value[3] != 0x1 {
            return Err(2);
        }

        Ok(Self {
            ecu_mode: value[4],
            serial_number: value[8..16].try_into().unwrap(),
            uart_selected: value[32],
            uart_baudrate: u32::from_le_bytes(value[33..37].try_into().unwrap()),
            canbus1_bitrate: u32::from_le_bytes(value[40..44].try_into().unwrap()),
            canbus1_termination: value[44] != 0,
            j1939_address: value[48],
            j1939_name: value[49..57].try_into().unwrap(),
        })
    }
}
