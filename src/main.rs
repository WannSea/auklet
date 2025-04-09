mod control;
mod imu;
mod receiver;
mod servo;
mod sonar;

use control::{FlightController, State};
use imu::handle_imu;
use receiver::handle_receiver;
use rppal::pwm::Pwm;
use servo::Servo;
use sonar::handle_sonar;

use std::sync::{Arc, Mutex};
use std::thread::{self, sleep};
use std::time::{Duration, SystemTime};

fn main() -> () {
    println!("HELLO");

    let mut controller = FlightController::new();

    let rotation: Arc<Mutex<[f32; 3]>> = Arc::new(Mutex::new([0.0; 3]));
    let gyro: Arc<Mutex<[f32; 3]>> = Arc::new(Mutex::new([0.0; 3]));
    let altitude: Arc<Mutex<f32>> = Arc::new(Mutex::new(0.4));
    let setpoint: Arc<Mutex<State>> = Arc::new(Mutex::new(State {
        roll: 0.0,
        pitch: 0.0,
        yaw_rate: 0.0,
        altitude: 0.4,
    }));

    let rotation_clone = rotation.clone();
    let gyro_clone = gyro.clone();
    thread::spawn(move || {
        handle_imu(rotation_clone, gyro_clone);
    });

    let altitude_clone = altitude.clone();
    thread::spawn(move || {
        handle_sonar(altitude_clone);
    });

    let setpoint_clone = setpoint.clone();
    thread::spawn(move || {
        handle_receiver(setpoint_clone);
    });

    let mut port_servo = Servo::new(rppal::pwm::Channel::Pwm2, 0.0, -10.0, 10.0);
    let mut starboard_servo = Servo::new(rppal::pwm::Channel::Pwm0, 0.0, -10.0, 10.0);
    let mut aft_servo = Servo::new(rppal::pwm::Channel::Pwm1, 0.0, -10.0, 10.0);
    let mut rudder_servo = Servo::new(rppal::pwm::Channel::Pwm3, 0.0, -135.0, 135.0);

    let control_rate = Duration::from_millis(10);
    loop {
        let start = SystemTime::now();
        {
            let rot = rotation.lock().unwrap();
            let measurement = State {
                roll: rot[0],
                pitch: rot[1],
                yaw_rate: gyro.lock().unwrap().clone()[2],
                altitude: altitude.lock().unwrap().clone(),
            };
            let action = controller.update_controller(
                *setpoint.lock().unwrap(),
                measurement,
                control_rate.as_secs_f32(),
            );

            dbg!(&action);
            dbg!(measurement);
            dbg!(&setpoint);

            port_servo.set_angle(action.port);
            starboard_servo.set_angle(action.starboard);
            aft_servo.set_angle(action.aft);
            rudder_servo.set_angle(action.rudder * 3.0);
        }

        match control_rate.checked_sub(SystemTime::now().duration_since(start).unwrap()) {
            Some(sleep_time) => sleep(sleep_time),
            None => println!("Wir sind am Arsch!"),
        }
    }
}
