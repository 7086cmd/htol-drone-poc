#![allow(dead_code)]
use ahrs::{Ahrs, AhrsError, Madgwick};
use nalgebra::Vector3;

pub enum FusionError {
    ImuError,
    GpsError,
    BarometerError,
    AhrsError(AhrsError),
}

pub struct SensorFusion {
    pub imu: Madgwick<f32>,

    // Position state (NED frame)
    pub position: Vector3<f32>,
    pub velocity: Vector3<f32>,

    // Altitude
    pub altitude_agl: f32, // Above Ground Level
    pub altitude_msl: f32, // Mean Sea Level
    pub vertical_speed: f32,
}

impl SensorFusion {
    pub fn new(sample_freq_hz: f32, beta: f32) -> Self {
        Self {
            imu: Madgwick::new(sample_freq_hz, beta),
            position: Vector3::zeros(),
            velocity: Vector3::zeros(),
            altitude_agl: 0.0,
            altitude_msl: 0.0,
            vertical_speed: 0.0,
        }
    }

    pub fn update_imu(
        &mut self,
        gyro: Vector3<f32>,
        accel: Vector3<f32>,
    ) -> Result<&nalgebra::UnitQuaternion<f32>, FusionError> {
        self.imu
            .update_imu(&gyro, &accel)
            .map_err(FusionError::AhrsError)
    }

    pub fn update_mag(
        &mut self,
        gyro: Vector3<f32>,
        accel: Vector3<f32>,
        mag: Vector3<f32>,
    ) -> Result<&nalgebra::UnitQuaternion<f32>, FusionError> {
        self.imu
            .update(&gyro, &accel, &mag)
            .map_err(FusionError::AhrsError)
    }

    pub fn get_euler(&self) -> (f32, f32, f32) {
        self.imu.quat.euler_angles()
    }
}
