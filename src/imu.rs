use bno085::{
    bno_constants::{SENSOR_REPORTID_GYRO_CALIBRATED, SENSOR_REPORTID_ROTATION_VECTOR},
    bno_driver::BnoDriver,
    bno_packet::{BnoPacket, ChannelExecutableData, SensorReportData},
    interface::i2c::I2CInterface,
};
use std::sync::{Arc, Mutex};

pub fn handle_imu(rotation: Arc<Mutex<Vec<f32>>>, gyro: Arc<Mutex<Vec<f32>>>) {
    let rpi_interface = rppal::i2c::I2c::new().unwrap();
    let interface = I2CInterface::new(rpi_interface);

    let interval = 10;

    let mut driver = BnoDriver::new(interface);
    driver.setup();
    driver.soft_reset().unwrap();

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
                    ChannelExecutableData::Unknown(ced) => {
                        println!("CED {:?}", ced);
                    }
                },
                BnoPacket::SensorReports(reports) => {
                    for report in reports {
                        match report {
                            SensorReportData::Rotation(d) => {
                                *rotation.lock().unwrap() = d.get_vec()
                            }
                            SensorReportData::GyroCalibrated(d) => {
                                *gyro.lock().unwrap() = d.get_vec()
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
                    e => {
                        print!("BNO Driver Error {:?}", e);
                    }
                }
            }
        }
    }
}
