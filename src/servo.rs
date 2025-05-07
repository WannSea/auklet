use rppal::pwm::{Channel, Pwm};
use std::{
    f32::{consts::PI, INFINITY},
    thread::sleep,
    time::Duration,
};

/// Represents a servo connected to one of the Pi's PWM channels.
pub struct Servo {
    pwm: Pwm,
    trim: f32,      // Trim offset for servo (in degrees)
    min_angle: f32, // Minimum angle (in degrees)
    max_angle: f32, // Maximum angle (in degrees)
}

impl Servo {
    /// Creates a new Servo on a rppal PWM Channel.
    /// trim and the angle limits are in degress
    /// the limits are applied before trim
    pub fn new(channel: Channel, trim: f32, min_angle: f32, max_angle: f32) -> Self {
        let pwm = Pwm::with_pwmchip(0, channel as u8).unwrap();
        pwm.set_pulse_width(Duration::from_micros(0)).unwrap();
        pwm.set_period(Duration::from_micros(2500)).unwrap();
        pwm.set_pulse_width(Duration::from_micros(1500)).unwrap();
        pwm.set_polarity(rppal::pwm::Polarity::Normal).unwrap();
        pwm.enable().unwrap();
        // return;

        //let pwm = Pwm::with_frequency(channel, 50.0, 0.5, Polarity::Normal, true).unwrap();
        let mut s = Self {
            pwm,
            trim,
            min_angle,
            max_angle,
        };

        //      let sleep_dur = Duration::from_millis(200);
        //      sleep(sleep_dur);
        //      s.set_angle(-INFINITY);
        //      sleep(sleep_dur);
        //      s.set_angle(INFINITY);
        //      sleep(sleep_dur);
        s.set_angle(0.0);
        //      sleep(sleep_dur);

        return s;
    }

    /// Sets the servo angle in grad
    pub fn set_angle(&mut self, angle_grad: f32) {
        // apply limits and trim
        let angle = angle_grad.clamp(self.min_angle, self.max_angle) + self.trim;

        let servo_angle = 135.0 + angle;
        let pulse_width = 500.0 + ((servo_angle / 270.0) * 2000.0);

        // Calculate pulse width (1ms–2ms for 0–180°)
        self.pwm
            .set_pulse_width(Duration::from_micros(pulse_width as u64))
            .unwrap();
    }
}
