use rppal::pwm::{Channel, Polarity, Pwm};
use std::{error::Error, f32::consts::PI};

/// Represents a servo connected to one of the Pi's PWM channels.
pub struct Servo {
    pwm: Pwm,
    trim: f32,      // Trim offset for servo (in degrees)
    min_angle: f32, // Minimum angle (in degrees)
    max_angle: f32, // Maximum angle (in degrees)
}

impl Servo {
    pub fn new(
        channel: Channel,
        trim: f32,
        min_angle: f32,
        max_angle: f32,
    ) -> Result<Self, Box<dyn Error>> {
        let pwm = Pwm::with_frequency(channel, 50.0, 0.0, Polarity::Normal, true)?;
        Ok(Self {
            pwm,
            trim,
            min_angle,
            max_angle,
        })
    }

    /// Sets the servo angle in degrees (0–180)
    pub fn set_angle(&mut self, angle_rad: f32) -> Result<(), Box<dyn Error>> {
        // Apply trim offset
        let adjusted_angle = angle_rad / PI * 180.0 + self.trim;

        // Clamp the angle within the limits
        let clamped_angle = adjusted_angle.clamp(self.min_angle, self.max_angle);

        // Calculate pulse width (1ms–2ms for 0–180°)
        let pulse_ms = 1.0 + (clamped_angle / 180.0) * 1.0;
        let duty_cycle = pulse_ms / 20.0; // 20ms period for 50Hz

        // Set the duty cycle to control the servo
        self.pwm.set_duty_cycle(duty_cycle as f64)?;
        Ok(())
    }
}
