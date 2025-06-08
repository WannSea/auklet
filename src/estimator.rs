use std::{
    f32::consts::PI,
    sync::mpsc::{Receiver, Sender, SyncSender},
    thread,
};

use nalgebra::{UnitQuaternion, Vector3};
use serde::Deserialize;

use crate::{
    imu::IMUMeasurement,
    influx::{DataPoint, Log},
    sonar::SonarMeasurement,
};

// pub struct GPSMeasurement {
//     position: Vector2<f32>,
//     velocity: Vector2<f32>,
// }

pub enum Measurement {
    SonarMeasurement(SonarMeasurement),
    IMUMeasurement(IMUMeasurement),
    // GPSMeasurement(GPSMeasurement),
}

#[derive(Clone, Copy)]
pub struct State {
    pub position: Vector3<f32>,
    pub orientation: UnitQuaternion<f32>,
    pub velocity: Vector3<f32>,
    pub angular_vel: Vector3<f32>,
}

impl Log for State {
    fn measurements(&self) -> Vec<(String, f32)> {
        let (roll, pitch, yaw) = self.orientation.euler_angles();
        vec![
            ("x".to_owned(), self.position[0]),
            ("y".to_owned(), self.position[1]),
            ("z".to_owned(), self.position[2]),
            ("roll".to_owned(), roll / PI * 180.0),
            ("pitch".to_owned(), pitch / PI * 180.0),
            ("yaw".to_owned(), yaw / PI * 180.0),
            ("vx".to_owned(), self.velocity[0]),
            ("vy".to_owned(), self.velocity[1]),
            ("vz".to_owned(), self.velocity[2]),
            ("roll_rate".to_owned(), self.angular_vel[0] / PI * 180.0),
            ("pitch_rate".to_owned(), self.angular_vel[1] / PI * 180.0),
            ("yaw_rate".to_owned(), self.angular_vel[2] / PI * 180.0),
        ]
    }
}

#[derive(Deserialize)]
pub struct Filter {}

impl Filter {
    pub fn run(
        &self,
        measurements_rx: Receiver<Measurement>,
        state_estimation_tx: Sender<State>,
        influx_tx: SyncSender<DataPoint>,
    ) {
        thread::spawn(move || -> ! {
            let mut current_state_estimation = State {
                position: Vector3::default(),
                orientation: UnitQuaternion::default(),
                velocity: Vector3::default(),
                angular_vel: Vector3::default(),
            };

            loop {
                // place holder state estimation
                let new_measurement = measurements_rx.recv().unwrap();

                match new_measurement {
                    Measurement::IMUMeasurement(imu) => {
                        current_state_estimation.orientation = imu.orientation;
                    }
                    Measurement::SonarMeasurement(sonar) => {
                        current_state_estimation.position[2] = sonar.altitude;
                    }
                }

                state_estimation_tx.send(current_state_estimation).unwrap();
                influx_tx
                    .send(DataPoint::new(
                        "state_estimation".to_owned(),
                        current_state_estimation,
                    ))
                    .unwrap();
            }
        });
    }
}
