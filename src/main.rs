mod control;
mod imu;
mod influx;
mod receiver;
mod servo;
mod sonar;

use control::{ControlAction, FlightController, State};
use imu::handle_imu;
use influx::influx_log;
use receiver::handle_receiver;
use servo::Servo;
use sonar::handle_sonar;

use std::sync::{Arc, Mutex};
use std::thread::{self, sleep};
use std::time::{Duration, SystemTime};

const LOG_RATE: Duration = Duration::from_millis(500); // delay between logs

fn main() -> () {
    println!("HELLO");

    let yaml_str = std::fs::read_to_string("config.yaml").unwrap();
    let mut controller: FlightController = serde_yaml::from_str(&yaml_str).unwrap();

    let setpoint: Arc<Mutex<State>> = Arc::new(Mutex::new(State {
        roll: 0.0,
        pitch: 0.0,
        yaw_rate: 0.0,
        altitude: 0.4,
    }));
    let measurement: Arc<Mutex<State>> = Arc::new(Mutex::new(State {
        roll: 0.0,
        pitch: 0.0,
        yaw_rate: 0.0,
        altitude: 0.0,
    }));

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

    let setpoint_clone = setpoint.clone();
    thread::spawn(move || {
        handle_receiver(setpoint_clone);
    });

    influx_log(
        setpoint.clone(),
        "setpoint".to_string(),
        Duration::from_millis(500),
    );
    influx_log(
        measurement.clone(),
        "measurement".to_string(),
        Duration::from_millis(500),
    );

    let mut port_servo = Servo::new(rppal::pwm::Channel::Pwm2, 0.0, -10.0, 10.0);
    let mut starboard_servo = Servo::new(rppal::pwm::Channel::Pwm0, 0.0, -10.0, 10.0);
    let mut aft_servo = Servo::new(rppal::pwm::Channel::Pwm1, 0.0, -10.0, 10.0);
    let mut rudder_servo = Servo::new(rppal::pwm::Channel::Pwm3, 0.0, -135.0, 135.0);

    let control_rate = Duration::from_millis(10);
    let mut last_log = SystemTime::now();
    loop {
        let start = SystemTime::now();
        {
            *action.lock().unwrap() = controller.update_controller(
                *setpoint.lock().unwrap(),
                *measurement.lock().unwrap(),
                control_rate.as_secs_f32(),
            );
        }
        {
            let unlocked_action = action.lock().unwrap();
            port_servo.set_angle(unlocked_action.port);
            starboard_servo.set_angle(unlocked_action.starboard);
            aft_servo.set_angle(unlocked_action.aft);
            rudder_servo.set_angle(unlocked_action.rudder * 3.0); // the servo has a gear ratio of 3
        }

        if SystemTime::now().duration_since(last_log).unwrap() > LOG_RATE {
            dbg!(&action);
            dbg!(&measurement);
            dbg!(&setpoint.lock().unwrap());
            last_log = SystemTime::now();
        }
        match control_rate.checked_sub(SystemTime::now().duration_since(start).unwrap()) {
            Some(sleep_time) => sleep(sleep_time),
            None => println!("Wir sind am Arsch!"),
        }
    }
}
