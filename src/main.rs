mod control;
mod estimator;
mod imu;
mod influx;
mod receiver;
mod servo;
mod sonar;

use control::{ControlAction, FlightController};
use estimator::Filter;
use imu::IMUReader;
use influx::InfluxHandler;
use receiver::Receiver;
use serde::Deserialize;
use servo::Servo;
use sonar::Sonar;

use std::env;
use std::sync::mpsc::{channel, sync_channel};

#[derive(Deserialize)]
struct Configuration {
    controller: FlightController,
    receiver: Receiver,
    trim: ControlAction,
    imu: IMUReader,
}

fn main() -> () {
    let yaml_path = match env::var("CONFIG_PATH") {
        Ok(path) => path,
        Err(_) => String::from("config.yaml"),
    };
    println!("reading config: {}", yaml_path);
    let yaml_str = std::fs::read_to_string(yaml_path).unwrap();

    let config: Configuration = serde_yaml::from_str(&yaml_str).unwrap();

    let (influx_tx, influx_rx) = sync_channel(2000);
    let (measurements_tx, measurements_rx) = channel();
    let (state_estimation_tx, state_estimation_rx) = channel();
    let (controls_tx, controls_rx) = channel();

    let influx = InfluxHandler {};
    influx.run(influx_rx);

    let receiver: Receiver = config.receiver;
    receiver.run();

    let sonar: Sonar = Sonar::new();
    sonar.run(measurements_tx.clone(), influx_tx.clone());

    let imu = config.imu;
    imu.run(measurements_tx.clone(), influx_tx.clone());

    let filter: Filter = Filter {};
    filter.run(measurements_rx, state_estimation_tx, influx_tx.clone());

    let controller: FlightController = config.controller;
    controller.run(
        state_estimation_rx,
        controls_tx,
        influx_tx.clone(),
        receiver.inputs,
    );

    let mut port_servo = Servo::new(rppal::pwm::Channel::Pwm2, config.trim.port, -13.0, 13.0);
    let mut starboard_servo = Servo::new(
        rppal::pwm::Channel::Pwm0,
        config.trim.starboard,
        -13.0,
        13.0,
    );
    let mut aft_servo = Servo::new(rppal::pwm::Channel::Pwm1, config.trim.aft, -13.0, 13.0);
    let mut rudder_servo = Servo::new(rppal::pwm::Channel::Pwm3, config.trim.rudder, -135.0, 135.0);

    // write actions
    loop {
        let control_action = controls_rx.recv().unwrap();
        port_servo.set_angle(control_action.port);
        starboard_servo.set_angle(control_action.starboard);
        aft_servo.set_angle(control_action.aft);
        rudder_servo.set_angle(control_action.rudder * 3.0); // the servo has a gear ratio of 3
    }
}
