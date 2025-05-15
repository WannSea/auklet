use bno085::{
    bno_constants::{SENSOR_REPORTID_GYRO_CALIBRATED, SENSOR_REPORTID_ROTATION_VECTOR},
    bno_driver::BnoDriver,
    bno_packet::{BnoPacket, ChannelExecutableData, SensorReportData},
    interface::i2c::I2CInterface,
};
use std::{
    f32::consts::PI,
    sync::{Arc, Mutex},
    time::Duration,
};

use nalgebra::geometry::{Quaternion, UnitQuaternion};

use crate::{
    control::State,
    influx::{Log, Measurement},
};

struct RateRingBuffer {
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
        self.index = self.index + 1 % self.buffer.len();
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

#[derive(Clone, Copy)]
struct Attitude {
    roll: f32,
    pitch: f32,
}

pub fn handle_imu(measurement: Arc<Mutex<State>>) {
    let rpi_interface = rppal::i2c::I2c::new().unwrap();
    let interface = I2CInterface::new(rpi_interface);

    let interval = 16;

    let mut driver = BnoDriver::new(interface);
    driver.setup();
    driver.soft_reset().unwrap();

    let mut offset: Option<Attitude> = None;

    loop {
        match driver.receive_packet() {
            Ok(packet) => match packet {
                BnoPacket::ChannelExec(ce) => match ce {
                    ChannelExecutableData::ResetComplete => {
                        print!("Reset Complete, enabling Reports!");
                        // Enable reports after reset
                        driver
                            .enable_report(SENSOR_REPORTID_ROTATION_VECTOR, interval, interval - 1)
                            .unwrap();
                        driver
                            .enable_report(SENSOR_REPORTID_GYRO_CALIBRATED, interval, interval - 1)
                            .unwrap();
                    }
                    ChannelExecutableData::Unknown(_ced) => {
                        //println!("CED {:?}", ced);
                    }
                },
                BnoPacket::SensorReports(reports) => {
                    for report in reports {
                        match report {
                            SensorReportData::Rotation(d) => {
                                let euler_angles_rad =
                                    UnitQuaternion::from_quaternion(Quaternion::new(
                                        d.values[3],
                                        d.values[0],
                                        d.values[1],
                                        d.values[2],
                                    ))
                                    .euler_angles();
                                let euler_angles = (
                                    euler_angles_rad.0 / PI * 180.0,
                                    euler_angles_rad.1 / PI * 180.0,
                                    euler_angles_rad.2 / PI * 180.0,
                                );
                                match offset {
                                    None => {
                                        offset = Some(Attitude {
                                            roll: euler_angles.0,
                                            pitch: euler_angles.1,
                                        })
                                    }
                                    Some(offset) => {
                                        let mut unlocked = measurement.lock().unwrap();
                                        unlocked.roll = euler_angles.0 - offset.roll;
                                        unlocked.pitch = euler_angles.1 - offset.pitch;
                                    }
                                };
                                {}
                            }
                            SensorReportData::GyroCalibrated(d) => {
                                measurement.lock().unwrap().yaw_rate = d.values[2] / PI * 180.0;
                            }
                            d => {
                                print!("Unknown Sensor Data {:?}", d);
                            }
                        };
                    }
                }
                d => {
                    println!("CED: {:?}", d);
                }
            },
            Err(err) => {
                match err {
                    bno085::bno_driver::DriverError::NoDataAvailable => { /* Nothing to do, can happen due to sleep/clock drift */
                    }
                    _e => {
                        //print!("BNO Driver Error {:?}", e);
                    }
                }
            }
        }
    }
}
