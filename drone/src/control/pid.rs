#![allow(dead_code)]
use pid::Pid;

use crate::types::{Command, Telemetry};

pub struct PidController {
    pub speed: Pid<f32>,
    pub altitude: Pid<f32>,

    pub roll: Pid<f32>,
    pub pitch: Pid<f32>,
    pub yaw: Pid<f32>,

    pub update_counter: u32,
}

impl PidController {
    pub fn new() -> Self {
        Self {
            // Altitude: slow outer loop
            altitude: *Pid::new(0.0, 15.0) // setpoint=0, output_limit=±15°
                .p(0.3, 10.0) // Kp=0.3, p_limit=10°
                .i(0.01, 5.0) // Ki=0.01, i_limit=5°
                .d(0.5, 5.0), // Kd=0.5, d_limit=5°
            // Speed: slow outer loop
            speed: *Pid::new(0.0, 1.0).p(0.1, 0.5).i(0.02, 0.3).d(0.05, 0.2),
            // Pitch: fast inner loop
            pitch: *Pid::new(0.0, 1.0).p(0.7, 0.8).i(0.04, 0.2).d(0.18, 0.3),
            // Roll: fast inner loop
            roll: *Pid::new(0.0, 1.0).p(0.8, 0.9).i(0.05, 0.2).d(0.15, 0.3),
            // Yaw: fast inner loop
            yaw: *Pid::new(0.0, 1.0).p(0.5, 0.7).i(0.02, 0.2).d(0.1, 0.2),
            update_counter: 0,
        }
    }

    pub fn update(&mut self, telem: &Telemetry) -> Command {
        self.update_counter = self.update_counter.wrapping_add(1);

        // Outer loops (altitude and speed) @ 50 Hz
        let (pitch_setpoint, throttle) = if self.update_counter % 5 == 0 {
            let pitch_sp = self.altitude.next_control_output(telem.alt).output;
            let throttle = self.speed.next_control_output(telem.speed).output;
            (pitch_sp, throttle)
        } else {
            (self.altitude.setpoint, self.speed.setpoint)
        };

        self.speed.setpoint = throttle;
        self.altitude.setpoint = pitch_setpoint;

        // Inner loops (pitch, roll, yaw) @ 250 Hz
        let pitch_output = self
            .pitch
            .next_control_output(telem.pitch)
            .output;
        let roll_output = self.roll.next_control_output(telem.roll).output;
        let yaw_output = self.yaw.next_control_output(telem.yaw).output;

        Command {
            yaw: yaw_output,
            roll: roll_output,
            alt: pitch_output,
            speed: throttle,
            steer: false,
        }
    }

    /// Reset all integrators (e.g., on mode change)
    pub fn reset(&mut self) {
        // pid crate doesn't expose direct reset, so recreate
        *self = Self::new();
    }
}
