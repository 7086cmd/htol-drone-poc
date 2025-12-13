#![allow(dead_code)]
use embassy_stm32::{
    i2c::{I2c, Master},
    mode::Async,
};
use num_traits::float::Float;

pub struct Pitot<'a> {
    i2c: I2c<'a, Async, Master>,
}

impl<'a> Pitot<'a> {
    pub const MS4525DO_ADDR: u8 = 0x28;

    pub fn new(i2c: I2c<'a, Async, Master>) -> Self {
        Self { i2c }
    }

    async fn read_raw(&mut self) -> Result<(u16, u16, u16), super::SensorError> {
        let mut buf = [0u8; 4];
        self.i2c.read(Self::MS4525DO_ADDR, &mut buf).await?;

        // Extract status (bits 7-6 of byte 0)
        let status = (buf[0] >> 6) & 0x03;

        // Extract pressure (14 bits)
        let pressure_raw = (((buf[0] & 0x3F) as u16) << 8) | (buf[1] as u16);

        // Extract temperature (11 bits)
        let temperature_raw = (((buf[2] as u16) << 3) | ((buf[3] >> 5) as u16)) & 0x07FF;

        Ok((status as u16, pressure_raw, temperature_raw))
    }

    #[inline]
    fn convert_pressure(raw: u16) -> f32 {
        // MS4525DO: 1 PSI differential pressure sensor
        // 1 PSI = 6894.76 Pa
        const P_MIN_PSI: f32 = -1.0;
        const P_MAX_PSI: f32 = 1.0;
        const PSI_TO_PA: f32 = 6894.76;

        const P_MIN_PA: f32 = P_MIN_PSI * PSI_TO_PA; // -6894.76 Pa
        const P_MAX_PA: f32 = P_MAX_PSI * PSI_TO_PA; // 6894.76 Pa

        const OUTPUT_MIN: f32 = 1638.0; // 10% of 2^14
        const OUTPUT_MAX: f32 = 14745.0; // 90% of 2^14

        let raw_f = raw as f32;
        (raw_f - OUTPUT_MIN) * (P_MAX_PA - P_MIN_PA) / (OUTPUT_MAX - OUTPUT_MIN) + P_MIN_PA
    }

    #[inline]
    fn convert_temperature(raw: u16) -> f32 {
        // Temperature formula from datasheet
        // Temp (°C) = (raw * 200 / 2047) - 50
        (raw as f32 * 200.0 / 2047.0) - 50.0
    }

    #[inline]
    fn calculate_air_density(temp_celsius: f32) -> f32 {
        // Simplified air density calculation at sea level
        // ρ = P / (R × T)
        // Where:
        //   P = atmospheric pressure (Pa) = 101325 Pa at sea level
        //   R = specific gas constant for dry air = 287.05 J/(kg·K)
        //   T = absolute temperature (K)

        const P_ATM: f32 = 101325.0; // Pa
        const R_AIR: f32 = 287.05; // J/(kg·K)

        let temp_kelvin = temp_celsius + 273.15;
        P_ATM / (R_AIR * temp_kelvin)
    }

    #[inline]
    fn calculate_airspeed(pressure_pa: f32, air_density: f32) -> f32 {
        // Bernoulli's equation: v = sqrt(2 * ΔP / ρ)
        if pressure_pa.abs() < f32::EPSILON {
            0.0
        } else {
            let velocity = (2.0 * pressure_pa / air_density).sqrt();
            if pressure_pa < 0.0 { -velocity } else { velocity }
        }
    }
}
