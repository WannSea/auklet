mod control;
mod imu;
mod receiver;
mod servo;
mod sonar;

use control::{FlightController, State};
use imu::handle_imu;
use receiver::handle_receiver;
use servo::Servo;
use sonar::handle_sonar;

use std::sync::{Arc, Mutex};
use std::thread::{self, sleep};
use std::time::{Duration, SystemTime};

fn main() -> ! {
    let mut controller = FlightController::new();

    let rotation: Arc<Mutex<Vec<f32>>> = Arc::new(Mutex::new(vec![0.0, 0.0, 0.0]));
    let gyro: Arc<Mutex<Vec<f32>>> = Arc::new(Mutex::new(vec![0.0, 0.0, 0.0]));
    let altitude: Arc<Mutex<f32>> = Arc::new(Mutex::new(0.0));
    let setpoint: Arc<Mutex<State>> = Arc::new(Mutex::new(State {
        roll: 0.0,
        pitch: 0.0,
        yaw_rate: 0.0,
        altitude: 30.0,
    }));

    let rotation_clone = rotation.clone();
    let gyro_clone = rotation.clone();
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

    let mut port_servo = Servo::new(rppal::pwm::Channel::Pwm0, 0.0, -15.0, 15.0);
    let mut starboard_servo = Servo::new(rppal::pwm::Channel::Pwm1, 0.0, -15.0, 15.0);
    let mut aft_servo = Servo::new(rppal::pwm::Channel::Pwm2, 0.0, -15.0, 15.0);
    let mut rudder_servo = Servo::new(rppal::pwm::Channel::Pwm3, 0.0, -30.0, 30.0);

    let control_rate = Duration::from_millis(20);
    loop {
        let start = SystemTime::now();
        {
            let measurement = State {
                roll: rotation.lock().unwrap().clone()[0],
                pitch: rotation.lock().unwrap().clone()[1],
                yaw_rate: gyro.lock().unwrap().clone()[2],
                altitude: altitude.lock().unwrap().clone(),
            };
            let action = controller.update_controller(
                *setpoint.lock().unwrap(),
                measurement,
                control_rate.as_secs_f32(),
            );

            port_servo.set_angle(action.port);
            starboard_servo.set_angle(action.starboard);
            aft_servo.set_angle(action.aft);
            rudder_servo.set_angle(action.rudder);
        }
        sleep(control_rate - SystemTime::now().duration_since(start).unwrap());
    }
}
