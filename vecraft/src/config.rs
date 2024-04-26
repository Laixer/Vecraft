// EEPROM
// Header + version + firmware mode: [0x4C, 0x58, 0x52, 0x1, 0x16, 0xFF, 0xFF, 0xF]
// Serial number: [0x1, 0x2, 0x3, 0x4, 0x5, 0x6, 0x7, 0x8]
//
// Uart
// Selection : [2]
// Baudrate  : [0, 194, 1, 0]
//
// Canbus 1
// bit timing  : [144, 208, 3, 0]
// Termination : [0]
//
// J1939
// Address        : [0x4A]
// Name           : [0x1, 0x2, 0x3, 0x4, 0x5, 0x6, 0x7, 0x8]
// Source address : [0xFF]

pub const VECRAFT_CONFIG_PAGE: usize = 1;
pub const VECRAFT_CONFIG_SIZE: usize = 64;

const VECRAFT_CONFIG_HEADER: [u8; 3] = *b"LXR";
const VECRAFT_CONFIG_VERSION: u8 = 0x1;

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
    pub j1939_source_address: Option<u8>,
    pub is_dirty: bool,
    pub is_factory_reset: bool,
}

impl VecraftConfig {
    pub fn new(ecu_mode: u8, serial_number: [u8; 8], j1939_name: [u8; 8]) -> Self {
        Self {
            is_dirty: false,
            is_factory_reset: false,
            ecu_mode,
            serial_number,
            uart_selected: 0,
            uart_baudrate: 0,
            canbus1_bitrate: 0,
            canbus1_termination: false,
            j1939_address: 0,
            j1939_name,
            j1939_source_address: None,
        }
    }

    #[inline]
    pub fn ecu_mode(&self) -> crate::EcuApplication {
        self.ecu_mode
            .try_into()
            .unwrap_or(crate::EcuApplication::Unknown)
    }

    pub fn serial_number(&self) -> (u32, u32) {
        (
            u32::from_le_bytes(self.serial_number[0..4].try_into().unwrap_or([0; 4])),
            u32::from_le_bytes(self.serial_number[4..8].try_into().unwrap_or([0; 4])),
        )
    }

    pub fn j1939_name(&self) -> j1939::Name {
        j1939::Name::from_bytes(self.j1939_name)
    }

    pub fn to_bytes(&self) -> [u8; VECRAFT_CONFIG_SIZE] {
        let mut bytes = [0xFF; VECRAFT_CONFIG_SIZE];

        bytes[0..3].copy_from_slice(&VECRAFT_CONFIG_HEADER);
        bytes[3] = VECRAFT_CONFIG_VERSION;
        bytes[4] = self.ecu_mode;
        bytes[8..16].copy_from_slice(&self.serial_number);
        bytes[32] = self.uart_selected;
        bytes[33..37].copy_from_slice(&self.uart_baudrate.to_le_bytes());
        bytes[40..44].copy_from_slice(&self.canbus1_bitrate.to_le_bytes());
        bytes[44] = self.canbus1_termination as u8;
        bytes[48] = self.j1939_address;
        bytes[49..57].copy_from_slice(&self.j1939_name);
        bytes[57] = self.j1939_source_address.unwrap_or(0xFF);

        bytes
    }
}

impl Default for VecraftConfig {
    fn default() -> Self {
        Self {
            is_dirty: false,
            is_factory_reset: false,
            ecu_mode: 0x0,
            serial_number: [0; 8],
            uart_selected: 0x2,
            uart_baudrate: 115_200,
            canbus1_bitrate: 250_000,
            canbus1_termination: false,
            j1939_address: 0x25,
            j1939_name: [0; 8],
            j1939_source_address: None,
        }
    }
}

#[derive(Copy, Clone, Debug, PartialEq, Eq)]
pub enum ConfigError {
    InvalidHeader,
    InvalidVersion,
}

// TODO: Replace with an actual error type
impl TryFrom<&[u8]> for VecraftConfig {
    type Error = ConfigError;

    fn try_from(value: &[u8]) -> Result<Self, Self::Error> {
        if value[0..3] != VECRAFT_CONFIG_HEADER {
            return Err(ConfigError::InvalidHeader);
        }
        if value[3] != VECRAFT_CONFIG_VERSION {
            return Err(ConfigError::InvalidVersion);
        }

        Ok(Self {
            is_dirty: false,
            is_factory_reset: false,
            ecu_mode: value[4],
            serial_number: value[8..16].try_into().unwrap_or([0; 8]),
            uart_selected: value[32],
            uart_baudrate: u32::from_le_bytes(value[33..37].try_into().unwrap_or([0; 4])),
            canbus1_bitrate: u32::from_le_bytes(value[40..44].try_into().unwrap_or([0; 4])),
            canbus1_termination: value[44] != 0,
            j1939_address: value[48],
            j1939_name: value[49..57].try_into().unwrap_or([0; 8]),
            j1939_source_address: match value[57] {
                0xFF => None,
                _ => Some(value[57]),
            },
        })
    }
}

// #[cfg(test)]
// mod tests {
//     use super::*;

//     #[test]
//     fn test_config_roundtrip() {
//         let config = VecraftConfig {
//             ecu_mode: 0x1,
//             serial_number: [0x1, 0x2, 0x3, 0x4, 0x5, 0x6, 0x7, 0x8],
//             uart_selected: 2,
//             uart_baudrate: 115200,
//             canbus1_bitrate: 500_000,
//             canbus1_termination: false,
//             j1939_address: 0x4A,
//             j1939_name: [0x1, 0x2, 0x3, 0x4, 0x5, 0x6, 0x7, 0x8],
//         };

//         let bytes = config.to_bytes();
//         let roundtrip = VecraftConfig::try_from(&bytes[..]).unwrap();

//         assert_eq!(config.ecu_mode, roundtrip.ecu_mode);
//         assert_eq!(config.serial_number, roundtrip.serial_number);
//         assert_eq!(config.uart_selected, roundtrip.uart_selected);
//         assert_eq!(config.uart_baudrate, roundtrip.uart_baudrate);
//         assert_eq!(config.canbus1_bitrate, roundtrip.canbus1_bitrate);
//         assert_eq!(config.canbus1_termination, roundtrip.canbus1_termination);
//         assert_eq!(config.j1939_address, roundtrip.j1939_address);
//         assert_eq!(config.j1939_name, roundtrip.j1939_name);
//     }
// }
