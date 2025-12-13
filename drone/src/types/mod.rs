#![allow(dead_code)]
#[derive(Clone, Copy, Debug, defmt::Format)]
pub struct Telemetry {
    pub roll: f32,
    pub pitch: f32,
    pub yaw: f32,
    pub alt: f32,
    pub speed: f32,
    pub speed_z: f32,
    pub timestamp_us: u64,
}

#[derive(Clone, Copy, Debug, defmt::Format)]
pub struct Command {
    pub yaw: f32,
    pub roll: f32,
    pub alt: f32,
    pub speed: f32,
    pub steer: bool,
}