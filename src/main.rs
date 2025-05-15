mod control;
mod imu;
mod influx;
mod receiver;
mod servo;
mod sonar;

use control::{ControlAction, FlightController, State};
use imu::handle_imu;
use influx::influx_log;
use receiver::Receiver;
use serde::Deserialize;
use servo::Servo;
use sonar::handle_sonar;

use std::env;
use std::sync::{Arc, Mutex};
use std::thread::{self, sleep};
use std::time::{Duration, SystemTime};

#[derive(Deserialize)]
struct Configuration {
    controller: FlightController,
    receiver: Receiver,
    trim: ControlAction,
    logging_interval_ms: u64,
}

fn main() -> () {
    println!("Version 0.1");
    let yaml_path = match env::var("CONFIG_PATH") {
        Ok(path) => path,
        Err(_) => String::from("config.yaml"),
    };
    println!("reading config: {}", yaml_path);
    let yaml_str = std::fs::read_to_string(yaml_path).unwrap();

    let config: Configuration = serde_yaml::from_str(&yaml_str).unwrap();

    let mut controller: FlightController = config.controller;

    let receiver: Receiver = config.receiver;
    receiver.run();

    let measurement: Arc<Mutex<State>> = Arc::new(Mutex::new(State::default()));

    let action: Arc<Mutex<ControlAction>> = Arc::new(Mutex::new(ControlAction {
        port: 0.0,
        starboard: 0.0,
        aft: 0.0,
        rudder: 0.0,
    }));

    let measurement_clone = measurement.clone();
    thread::spawn(move || {
        handle_imu(measurement_clone);
    });

    let measurement_clone2 = measurement.clone();
    thread::spawn(move || {
        handle_sonar(measurement_clone2);
    });

    influx_log(
        receiver.inputs.clone(),
        "setpoint".to_string(),
        Duration::from_millis(config.logging_interval_ms),
    );
    influx_log(
        measurement.clone(),
        "measurement".to_string(),
        Duration::from_millis(config.logging_interval_ms),
    );
    influx_log(
        action.clone(),
        "action".to_string(),
        Duration::from_millis(config.logging_interval_ms),
    );
    influx_log(
        controller.current_pid.clone(),
        "pid".to_string(),
        Duration::from_millis(config.logging_interval_ms),
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

    let control_rate = Duration::from_millis(10);
    loop {
        let start = SystemTime::now();
        {
            let inputs = receiver.get_inputs();
            if inputs.controller_enable {
                *action.lock().unwrap() = controller.update_controller(
                    inputs.setpoint,
                    *measurement.lock().unwrap(),
                    control_rate.as_secs_f32(),
                );
            } else {
                *action.lock().unwrap() = ControlAction::default();
                controller.reset();
            }
        }
        {
            let unlocked_action = action.lock().unwrap();
            port_servo.set_angle(unlocked_action.port);
            starboard_servo.set_angle(unlocked_action.starboard);
            aft_servo.set_angle(unlocked_action.aft);
            rudder_servo.set_angle(unlocked_action.rudder * 3.0); // the servo has a gear ratio of 3
        }
        match control_rate.checked_sub(SystemTime::now().duration_since(start).unwrap()) {
            Some(sleep_time) => sleep(sleep_time),
            None => println!("Wir sind am Arsch!"),
        }
    }
}
