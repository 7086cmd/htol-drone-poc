#![allow(dead_code)]
pub mod pid;

use dshot_frame::{Frame, NormalDshot};
use embassy_stm32::peripherals::{TIM1, TIM2, TIM5};
use embassy_stm32::timer::simple_pwm::SimplePwm;
use embassy_stm32::timer::Channel::{Ch1, Ch2};
use embassy_stm32::{Peri, peripherals};

pub struct MotionController<'a> {
    /// Ch1 - Left EDF
    /// Ch2 - Right EDF
    pub edf: SimplePwm<'a, TIM1>,

    /// Ch1 - Main steering servo
    /// Ch2 - Nose steering servo
    pub steer: SimplePwm<'a, TIM2>,

    /// Ch1 - Left aileron servo
    /// Ch2 - Right aileron servo
    /// Ch3 - Elevator servo
    /// Ch4 - Rudder servo
    pub rudder: SimplePwm<'a, TIM5>,

    /// DMA peripheral for DShot transmission
    pub dma: Peri<'a, peripherals::DMA1_CH0>,
}

pub enum ControlError {
    PwmError,
    DshotError,
    ExceedsMaxAngle,
}

impl<'a> MotionController<'a> {
    pub fn new(
        edf: SimplePwm<'a, TIM1>,
        steer: SimplePwm<'a, TIM2>,
        rudder: SimplePwm<'a, TIM5>,
        dma: Peri<'a, peripherals::DMA1_CH0>,
    ) -> Self {
        Self {
            edf,
            steer,
            rudder,
            dma,
        }
    }

    pub async fn set_edf_speeds(
        &mut self,
        ratio: f32,
        request_telemetry: bool,
    ) -> Result<(), ControlError> {
        let duty_cycle = Self::get_edf_duty_cycle(ratio);
        let max_duty_cycles = self.edf.max_duty_cycle();
        let frame = Frame::<NormalDshot>::new(duty_cycle, request_telemetry)
            .ok_or_else(|| ControlError::DshotError)?;

        self.edf
            .waveform_up_multi_channel(self.dma.reborrow(), Ch1, Ch2, &frame.duty_cycles(max_duty_cycles))
            .await;

        // After transmission, set duty cycle to 0 to stop corrupted signals.

        self.edf.ch1().set_duty_cycle(0);
        self.edf.ch2().set_duty_cycle(0);

        self.edf.ch1().enable();
        self.edf.ch2().enable();

        Ok(())
    }

    pub async fn set_steering(&mut self, open: bool) -> Result<(), ControlError> {
        // For servos, the 1.5ms pulse in 50 Hz is the neutral position, and we can vary from 1ms (0°) to 2ms (180°).
        // For steering, we only need to open or close the steering mechanism, so we can define two positions: 90° (closed) and 0° (open).
        let duty_cycle = if open {
            // Open steering (0°)
            ((1.0 / 20.0) * (self.steer.max_duty_cycle() as f32)) as u16 // 1ms pulse
        } else {
            // Closed steering (90°)
            ((1.5 / 20.0) * (self.steer.max_duty_cycle() as f32)) as u16 // 1.5ms pulse
        };

        self.steer.ch1().set_duty_cycle(duty_cycle);
        self.steer.ch2().set_duty_cycle(duty_cycle);
        self.steer.ch1().enable();
        self.steer.ch2().enable();

        Ok(())
    }

    pub async fn set_aileron(
        &mut self,
        left_angle_deg: f32,
        right_angle_deg: f32,
    ) -> Result<(), ControlError> {
        // Assuming servo range is -90° to +90°. Convert it to 1ms to 2ms pulse width.
        let left_percent = (left_angle_deg + 90.0) / 180.0; // Normalize to 0.0 - 1.0
        let right_percent = (right_angle_deg + 90.0) / 180.0; // Normalize to 0.0 - 1.0

        if left_percent < 0.0 || left_percent > 1.0 || right_percent < 0.0 || right_percent > 1.0 {
            return Err(ControlError::ExceedsMaxAngle);
        }

        let max_duty_cycles = self.rudder.max_duty_cycle();
        let left_duty_cycle = ((1.0 + left_percent) / 20.0 * (max_duty_cycles as f32)) as u16; // 1ms to 2ms
        let right_duty_cycle = ((1.0 + right_percent) / 20.0 * (max_duty_cycles as f32)) as u16; // 1ms to 2ms

        self.rudder.ch1().set_duty_cycle(left_duty_cycle);
        self.rudder.ch2().set_duty_cycle(right_duty_cycle);
        self.rudder.ch1().enable();
        self.rudder.ch2().enable();

        Ok(())
    }

    pub async fn set_elevator(&mut self, angle_deg: f32) -> Result<(), ControlError> {
        // Assuming elevator can only operate in a limited range, e.g., -30° to +30°.
        if angle_deg < -30.0 || angle_deg > 30.0 {
            return Err(ControlError::ExceedsMaxAngle);
        }
        let percent = (((angle_deg + 90.0) / 180.0) * (self.rudder.max_duty_cycle() as f32)) as u16; // Map -30° to +30° to 0 to max duty cycle
        let duty_cycle = ((1.0 + percent as f32 / (self.rudder.max_duty_cycle() as f32)) / 20.0
            * (self.rudder.max_duty_cycle() as f32)) as u16;

        self.rudder.ch3().set_duty_cycle(duty_cycle);
        self.rudder.ch3().enable();

        Ok(())
    }

    pub async fn set_rudder(&mut self, angle_deg: f32) -> Result<(), ControlError> {
        // Assuming rudder can only operate in a limited range, e.g., -30° to +30°.
        if angle_deg < -30.0 || angle_deg > 30.0 {
            return Err(ControlError::ExceedsMaxAngle);
        }
        let percent = (((angle_deg + 90.0) / 180.0) * (self.rudder.max_duty_cycle() as f32)) as u16; // Map -30° to +30° to 0 to max duty cycle
        let duty_cycle = ((1.0 + percent as f32 / (self.rudder.max_duty_cycle() as f32)) / 20.0
            * (self.rudder.max_duty_cycle() as f32)) as u16;

        self.rudder.ch4().set_duty_cycle(duty_cycle);
        self.rudder.ch4().enable();

        Ok(())
    }

    #[inline]
    fn get_edf_duty_cycle(ratio: f32) -> u16 {
        // In the `dshot-frame` crate, it already defines the DSHOT range as 48-2047 for normal commands, so we map 0.0-1.0 to that range.
        const DSHOT_MAX: u16 = 2000;
        const DSHOT_MIN: u16 = 0; // minimum throttle

        let clamped_ratio = if ratio < 0.0 {
            0.0
        } else if ratio > 1.0 {
            1.0
        } else {
            ratio
        };

        (clamped_ratio * (DSHOT_MAX - DSHOT_MIN) as f32) as u16 + DSHOT_MIN
    }
}
