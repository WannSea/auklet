use bno085::{
    bno_constants::{SENSOR_REPORTID_GAME_ROTATION_VECTOR, SENSOR_REPORTID_GYRO_CALIBRATED},
    bno_driver::BnoDriver,
    bno_packet::{BnoPacket, ChannelExecutableData, SensorReportData},
    interface::i2c::I2CInterface,
};
use std::{
    f32::consts::PI,
    sync::{Arc, Mutex},
};

use nalgebra::geometry::{Quaternion, UnitQuaternion};

use crate::control::State;

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
                            .enable_report(
                                SENSOR_REPORTID_GAME_ROTATION_VECTOR,
                                interval,
                                interval - 1,
                            )
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
