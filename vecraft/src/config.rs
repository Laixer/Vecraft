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

// let vecraft_config_init = [
//     // Header + version + firmware mode
//     b'L', b'X', b'R', 0x1, 0x17, 0xFF, 0xFF, 0xF,
//     // Serial number
//     0x1, 0x2, 0x3, 0x4, 0x5, 0x6, 0x7, 0x8,
//     // Reserved
//     0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF,
//     0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF,
//     // UART
//     0x2, 0x0, 0xC2, 0x1, 0x0, 0xFF, 0xFF, 0xFF,
//     // Canbus 1
//     0x90, 0xD0, 0x3, 0x0, 0x0, 0xFF, 0xFF, 0xFF,
//     // J1939
//     0x15, 0x1, 0x2, 0x3, 0x4, 0x5, 0x6, 0x7,
//     0x08, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF,
// ];

// const VECRAFT_CONFIG_PAGE: u16 = 0;

// // eeprom.write(VECRAFT_CONFIG_PAGE, &vecraft_config_init).unwrap();

// let vecraft_config = loop {
//     let mut vecraft_config = [0; 64];
//     let rs = eeprom.read(VECRAFT_CONFIG_PAGE, &mut vecraft_config);
//     // if let Err(e) = rs {
//     //     panic!("EEPROM read error: {:?}", e);
//     // }
//     if rs.is_ok() {
//         break vecraft_config;
//     }
// };

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

    pub fn serial_number(&self) -> [u8; 8] {
        self.serial_number
    }

    pub fn j1939_address(&self) -> u8 {
        self.j1939_address
    }

    pub fn j1939_name(&self) -> [u8; 8] {
        self.j1939_name
    }
}

impl TryFrom<&[u8]> for VecraftConfig {
    type Error = ();

    fn try_from(value: &[u8]) -> Result<Self, Self::Error> {
        if &value[0..3] != b"LXR" {
            return Err(());
        }
        if value[3] != 0x1 {
            return Err(());
        }

        Ok(Self {
            ecu_mode: value[4],
            serial_number: value[8..16].try_into().unwrap(),
            uart_selected: value[32],
            uart_baudrate: u32::from_le_bytes(value[33..37].try_into().unwrap()),
            canbus1_bitrate: u32::from_le_bytes(value[40..44].try_into().unwrap()),
            canbus1_termination: value[44] != 0,
            j1939_address: value[48],
            j1939_name: value[49..58].try_into().unwrap(),
        })
    }
}
