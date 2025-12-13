#![allow(dead_code)]
pub mod fusion;
pub mod gps;
pub mod imu;
pub mod pitot;
use embassy_stm32::i2c::Error as I2cError;

#[derive(Debug, defmt::Format)]
pub enum SensorError {
    I2cError(I2cError),
    IdentityMismatch,
    NoCalibrationData,
}

impl From<I2cError> for SensorError {
    fn from(e: I2cError) -> Self {
        SensorError::I2cError(e)
    }
}
