use serialport;
use std::sync::{Arc, Mutex};
use std::thread;
use std::time::Duration;

use crate::influx::{influx_log, Log, Measurement};

const START: u8 = 0xFF;
const SONAR_TO_SONAR_DISTANCE: f32 = 0.515;

#[derive(Debug, Clone, Copy)]
pub struct SonarData {
    pub port: f32,      // distance in meter
    pub starboard: f32, // distance in meter
    pub roll: f32,      // roll angle from trigonometry in rad
}

impl Default for SonarData {
    fn default() -> Self {
        Self {
            port: 0.0,
            starboard: 0.0,
            roll: 0.0,
        }
    }
}

impl Log for SonarData {
    fn measurements(&self) -> Vec<crate::influx::Measurement> {
        vec![
            Measurement {
                name: "Port",
                value: self.port,
            },
            Measurement {
                name: "Starboard",
                value: self.starboard,
            },
            Measurement {
                name: "Roll",
                value: self.roll,
            },
        ]
    }
}

enum Side {
    PORT,
    STARBOARD,
}

pub struct Sonar {
    port_serial_port: String,
    starboard_serial_port: String,
    measurement: Arc<Mutex<SonarData>>,
}

impl Sonar {
    pub fn new() -> Self {
        Sonar {
            port_serial_port: String::from("/dev/ttyAMA2"),
            starboard_serial_port: String::from("/dev/ttyAMA3"),
            measurement: Arc::new(Mutex::new(SonarData::default())),
        }
    }

    pub fn run(&self) {
        Self::read(
            self.port_serial_port.clone(),
            Side::PORT,
            self.measurement.clone(),
        );
        Self::read(
            self.starboard_serial_port.clone(),
            Side::STARBOARD,
            self.measurement.clone(),
        );

        influx_log(
            self.measurement.clone(),
            "Sonar".to_string(),
            Duration::from_millis(500),
        );
    }

    fn read(port: String, side: Side, data: Arc<Mutex<SonarData>>) {
        thread::spawn(move || {
            let mut port = serialport::new(port, 9600)
                .timeout(Duration::from_millis(30))
                .open()
                .expect("Failed to open port");
            let mut buffer = [0u8; 4];
            loop {
                let _ = port.read_exact(&mut buffer);
                if buffer[0] == START {
                    let distance_mm: u16 = u16::from_be_bytes([buffer[1], buffer[2]]);
                    let distance_m = distance_mm as f32 / 1000.0;
                    // TODO add outlier rejection based on delta
                    {
                        let mut unlocked = data.lock().unwrap();
                        match side {
                            Side::PORT => unlocked.port = distance_m,
                            Side::STARBOARD => unlocked.starboard = distance_m,
                        }
                        let delta = unlocked.port - unlocked.starboard;
                        unlocked.roll = f32::atan2(delta, SONAR_TO_SONAR_DISTANCE);
                    }
                } else {
                    let mut null = [0u8; 0];
                    let _ = port.read_exact(&mut null);
                }
            }
        });
    }

    pub fn get_data(&self) -> SonarData {
        *self.measurement.lock().unwrap()
    }
}
