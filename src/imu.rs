use bno085::{
    bno_constants::{
        SENSOR_REPORTID_ACCEL, SENSOR_REPORTID_GYRO_CALIBRATED, SENSOR_REPORTID_MAG_CALIBRATED,
        SENSOR_REPORTID_ROTATION_VECTOR,
    },
    bno_driver::BnoDriver,
    bno_packet::{BnoPacket, ChannelExecutableData, SensorReportData},
    interface::i2c::I2CInterface,
};
use rppal::i2c::I2c;
use serde::Deserialize;
use std::{
    f32::consts::PI,
    sync::mpsc::{Sender, SyncSender},
    thread,
};

use nalgebra::{
    geometry::{Quaternion, UnitQuaternion},
    Vector3,
};

use crate::{
    estimator::Measurement,
    influx::{DataPoint, Log},
};

#[derive(Clone, Copy)]
pub struct IMUMeasurement {
    pub acceleration: Vector3<f32>,
    pub gyroscope: Vector3<f32>,
    pub compass: Vector3<f32>,
    pub orientation: UnitQuaternion<f32>,
}

impl Log for IMUMeasurement {
    fn measurements(&self) -> Vec<(String, f32)> {
        let (roll, pitch, yaw) = self.orientation.euler_angles();
        vec![
            ("acc_x".to_owned(), self.acceleration[0]),
            ("acc_y".to_owned(), self.acceleration[1]),
            ("acc_z".to_owned(), self.acceleration[2]),
            ("gyro_x".to_owned(), self.gyroscope[0]),
            ("gyro_y".to_owned(), self.gyroscope[1]),
            ("gyro_z".to_owned(), self.gyroscope[2]),
            ("mag_x".to_owned(), self.compass[0]),
            ("mag_y".to_owned(), self.compass[1]),
            ("mag_z".to_owned(), self.compass[2]),
            ("roll".to_owned(), roll / PI * 180.0),
            ("pitch".to_owned(), pitch / PI * 180.0),
            ("yaw".to_owned(), yaw / PI * 180.0),
        ]
    }
}

pub struct IMUMeasurementMaybe {
    acceleration: Option<Vector3<f32>>,
    gyroscope: Option<Vector3<f32>>,
    compass: Option<Vector3<f32>>,
    orientation: Option<UnitQuaternion<f32>>,
}

impl IMUMeasurementMaybe {
    fn unpack(&self) -> Option<IMUMeasurement> {
        match (
            self.acceleration,
            self.gyroscope,
            self.compass,
            self.orientation,
        ) {
            (Some(acceleration), Some(gyroscope), Some(compass), Some(orientation)) => {
                Some(IMUMeasurement {
                    acceleration,
                    gyroscope,
                    compass,
                    orientation,
                })
            }
            _ => None,
        }
    }
}

#[derive(Deserialize)]
pub struct IMUReader {
    interval_ms: u16,
}

impl IMUReader {
    pub fn run(self, measurements_tx: Sender<Measurement>, influx_tx: SyncSender<DataPoint>) {
        let rpi_interface = rppal::i2c::I2c::new().unwrap();
        let interface = I2CInterface::new(rpi_interface);

        let mut driver = BnoDriver::new(interface);
        driver.setup();
        driver.soft_reset().unwrap();

        thread::spawn(move || loop {
            // initialize uncomplete measurement
            let mut current_measurement = IMUMeasurementMaybe {
                acceleration: None,
                gyroscope: None,
                compass: None,
                orientation: None,
            };

            loop {
                // fill current measurements with new measurements
                self.read(&mut driver, &mut current_measurement);

                // if the measurement is complet, send it and reset
                let maybe_complete = current_measurement.unpack();
                if let Some(complete) = maybe_complete {
                    measurements_tx
                        .send(Measurement::IMUMeasurement(complete))
                        .unwrap();
                    influx_tx
                        .send(DataPoint::new("imu".to_owned(), complete))
                        .unwrap();
                    break;
                }
            }
        });
    }

    fn read(
        &self,
        driver: &mut BnoDriver<I2CInterface<I2c>>,
        uncomplete_measurement: &mut IMUMeasurementMaybe,
    ) {
        let interval = self.interval_ms;
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
                        driver
                            .enable_report(SENSOR_REPORTID_ACCEL, interval, interval - 1)
                            .unwrap();
                        driver
                            .enable_report(SENSOR_REPORTID_MAG_CALIBRATED, interval, interval - 1)
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
                                uncomplete_measurement.orientation =
                                    Some(UnitQuaternion::from_quaternion(Quaternion::new(
                                        d.values[3],
                                        d.values[0],
                                        d.values[1],
                                        d.values[2],
                                    )));
                            }
                            SensorReportData::GyroCalibrated(d) => {
                                uncomplete_measurement.gyroscope =
                                    Some(Vector3::new(d.values[0], d.values[1], d.values[2]))
                            }
                            SensorReportData::Acceleration(d) => {
                                uncomplete_measurement.acceleration =
                                    Some(Vector3::new(d.values[0], d.values[1], d.values[2]))
                            }
                            SensorReportData::MagFieldCalibrated(d) => {
                                uncomplete_measurement.compass =
                                    Some(Vector3::new(d.values[0], d.values[1], d.values[2]))
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
