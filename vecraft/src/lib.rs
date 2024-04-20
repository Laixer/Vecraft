#![deny(warnings)]
#![no_main]
#![no_std]

pub use fdcan;
pub use j1939;
pub use panic_halt;
pub use systick_monotonic::Systick;

pub mod can;
mod config;
pub mod console;
pub mod eeprom;
mod led;
pub mod lsgc;
pub mod state;
pub mod usb_avic;
pub mod usb_frame;

pub use config::*;
pub use led::{LedState, RGBLed};

/// Represents the different applications of an ECU.
///
/// The ECU application is used to determine firmware behavior and message handling. A single ECU
/// can have multiple applications, but only one application can be active at a time.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum EcuApplication {
    PumpControl,
    StarterControl,
    HydraulicControl,
}

/// Converts a u8 value into an `EcuApplication` enum variant.
impl TryFrom<u8> for EcuApplication {
    type Error = ();

    /// Converts the given `u8` value into its corresponding `EcuApplication` variant.
    ///
    /// # Arguments
    ///
    /// * `value` - The `u8` value to convert.
    ///
    /// # Returns
    ///
    /// The `EcuApplication` variant corresponding to the given `u8` value.
    fn try_from(value: u8) -> Result<Self, Self::Error> {
        match value {
            0x10 => Ok(Self::PumpControl),
            0x11 => Ok(Self::StarterControl),
            0x15 => Ok(Self::HydraulicControl),
            _ => Err(()),
        }
    }
}

/// Converts an `EcuApplication` enum variant into a `u8` value.
impl From<EcuApplication> for u8 {
    /// Converts the given `EcuApplication` variant into its corresponding `u8` value.
    ///
    /// # Arguments
    ///
    /// * `value` - The `EcuApplication` variant to convert.
    ///
    /// # Returns
    ///
    /// The `u8` value corresponding to the given `EcuApplication` variant.
    fn from(value: EcuApplication) -> Self {
        match value {
            EcuApplication::PumpControl => 0x10,
            EcuApplication::StarterControl => 0x11,
            EcuApplication::HydraulicControl => 0x15,
        }
    }
}

use stm32h7xx_hal::hal::blocking::i2c::{Read, Write, WriteRead};
use stm32h7xx_hal::i2c::Error as I2cError;

pub trait I2CReadWriteTrait:
    WriteRead<Error = I2cError> + Read<Error = I2cError> + Write<Error = I2cError>
{
}

pub fn get_config<
    T: WriteRead<Error = I2cError> + Read<Error = I2cError> + Write<Error = I2cError>,
>(
    eeprom: &mut eeprom::Eeprom<T>,
) -> VecraftConfig {
    let mut vecraft_config = [0; VECRAFT_CONFIG_SIZE];
    eeprom.read_page(VECRAFT_CONFIG_PAGE, &mut vecraft_config);

    match VecraftConfig::try_from(&vecraft_config[..]) {
        Err(ConfigError::InvalidHeader) => {
            let default_config = VecraftConfig::default();
            eeprom.write_page(VECRAFT_CONFIG_PAGE, &default_config.to_bytes());

            default_config
        }
        Err(ConfigError::InvalidVersion) => panic!("Invalid config"),
        Ok(config) => config,
    }
}

pub fn put_config<
    T: WriteRead<Error = I2cError> + Read<Error = I2cError> + Write<Error = I2cError>,
>(
    eeprom: &mut eeprom::Eeprom<T>,
    config: &VecraftConfig,
) {
    eeprom.write_page(VECRAFT_CONFIG_PAGE, &config.to_bytes());
}

pub fn reset_config<
    T: WriteRead<Error = I2cError> + Read<Error = I2cError> + Write<Error = I2cError>,
>(
    eeprom: &mut eeprom::Eeprom<T>,
) -> VecraftConfig {
    let default_config = VecraftConfig::default();
    eeprom.write_page(VECRAFT_CONFIG_PAGE, &default_config.to_bytes());

    default_config
}

/// Trigger a full system reset.
#[inline]
pub fn sys_reboot() {
    rtic::export::SCB::sys_reset();
}
