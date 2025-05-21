use std::time::Duration;

use crate::influx::{Log, Measurement};

pub struct RateRingBuffer {
    buffer: [Duration; 100],
    index: usize,
}

impl RateRingBuffer {
    pub fn new() -> Self {
        Self {
            buffer: [Duration::ZERO; 100],
            index: 0,
        }
    }

    pub fn push(&mut self, duration: Duration) {
        self.buffer[self.index] = duration;
        self.index = self.index % self.buffer.len();
    }

    fn get_max_hz(&self) -> f64 {
        1.0 / self.buffer.into_iter().max().unwrap().as_secs_f64()
    }
    fn get_average_hz(&self) -> f64 {
        let sum: f64 = self.buffer.into_iter().map(|d| d.as_secs_f64()).sum();
        self.buffer.len() as f64 / sum
    }
}
impl Log for RateRingBuffer {
    fn measurements(&self) -> Vec<Measurement> {
        vec![
            Measurement {
                name: "average",
                value: self.get_average_hz() as f32,
            },
            Measurement {
                name: "max",
                value: self.get_max_hz() as f32,
            },
        ]
    }
}
