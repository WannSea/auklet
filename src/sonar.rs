use serde::Deserialize;
use serialport;
use std::sync::mpsc::{channel, Sender, SyncSender};
use std::thread;
use std::time::Duration;

use crate::estimator::Measurement;
use crate::influx::{DataPoint, Log};

const START: u8 = 0xFF;
const SENSOR_SPACING: f32 = 0.515;

#[derive(Debug, Clone, Copy)]
pub struct SonarMeasurement {
    pub altitude: f32, // distance in meter
    pub roll: f32,     // roll angle from trigonometry in rad
}

impl Default for SonarMeasurement {
    fn default() -> Self {
        Self {
            altitude: 0.0,
            roll: 0.0,
        }
    }
}

impl Log for SonarMeasurement {
    fn measurements(&self) -> Vec<(String, f32)> {
        vec![
            ("Port".to_owned(), self.altitude),
            ("Roll".to_owned(), self.roll),
        ]
    }
}

#[derive(Clone, Copy)]
enum Side {
    PORT,
    STARBOARD,
}

#[derive(Deserialize)]
pub struct Sonar {
    port_serial_port: String,
    starboard_serial_port: String,
}

#[derive(Clone, Copy)]
struct SingleSonarMeasurement {
    side: Side,
    height: f32,
}

impl Sonar {
    pub fn new() -> Self {
        Sonar {
            port_serial_port: String::from("/dev/ttyAMA2"),
            starboard_serial_port: String::from("/dev/ttyAMA3"),
        }
    }

    pub fn run(&self, measure_tx: Sender<Measurement>, influx_tx: SyncSender<DataPoint>) {
        // create channels for sensors
        let (tx, rx) = channel::<SingleSonarMeasurement>();

        Self::read(self.port_serial_port.clone(), Side::PORT, tx.clone());
        Self::read(
            self.starboard_serial_port.clone(),
            Side::STARBOARD,
            tx.clone(),
        );

        // second thread that processes data, averages two sonars and calculates angle if the
        // measuremant are both accurate enough

        thread::spawn(move || {
            let mut recent_port: Option<SingleSonarMeasurement> = None;
            let mut recent_starboard: Option<SingleSonarMeasurement> = None;

            loop {
                let new_measurement = rx.recv().unwrap();
                // check if the vaule is plausible
                if new_measurement.height < 3.0 {
                    match new_measurement.side {
                        Side::PORT => recent_port = Some(new_measurement),
                        Side::STARBOARD => recent_starboard = Some(new_measurement),
                    }

                    match (recent_port, recent_starboard) {
                        (Some(port), Some(starboard)) => {
                            let height = 0.5 * (port.height + starboard.height);
                            let roll = f32::atan2(port.height - starboard.height, SENSOR_SPACING);
                            let measurement = SonarMeasurement {
                                altitude: height,
                                roll: roll,
                            };
                            measure_tx.send(Measurement::SonarMeasurement(measurement)).unwrap();
                            influx_tx.send(DataPoint::new("sonar".to_owned(), measurement)).unwrap();
                        }
                        _ => {}
                    }
                }
            }
        });
    }

    fn read(port: String, side: Side, rx: Sender<SingleSonarMeasurement>) {
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
                    rx.send(SingleSonarMeasurement {
                        side: side,
                        height: distance_m,
                    })
                    .expect("rx send failed");
                } else {
                    let mut null = [0u8; 0];
                    let _ = port.read_exact(&mut null);
                }
            }
        });
    }
}
