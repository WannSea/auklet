use std::sync::{Arc, Mutex};
use std::thread;
use std::time::Duration;

use crate::control::State;
use crate::influx::{Log, Measurement};
use parse_rc_ibus::{IbusPacket, ParsingError};
use serde::Deserialize;
use serialport;

// const IBUS_HEADER: [u8; 2] = [0x20, 0x40];

#[derive(Clone, Copy)]
pub struct Inputs {
    pub setpoint: State,
    pub controller_enable: bool,
}

impl Default for Inputs {
    fn default() -> Self {
        Self {
            setpoint: State::default(),
            controller_enable: false,
        }
    }
}

impl Log for Inputs {
    fn measurements(&self) -> Vec<Measurement> {
        self.setpoint.measurements()
    }
}

#[derive(Deserialize)]
pub struct Receiver {
    sensitivity: State,

    #[serde(skip_deserializing)]
    pub inputs: Arc<Mutex<Inputs>>,
}

impl Receiver {
    pub fn run(&self) {
        let mut port = serialport::new("/dev/ttyAMA1", 115_200)
            .timeout(Duration::from_millis(14))
            .open()
            .expect("Failed to open port");

        let mut buffer = [0u8; 32];
        let mut header_buffer = [0u8; 1];

        let inputs = Arc::clone(&self.inputs);
        let sensitivity = self.sensitivity;

        thread::spawn(move || {
            loop {
                match port.read_exact(&mut buffer) {
                    Ok(()) => {
                        match IbusPacket::try_from_bytes(&buffer) {
                            Ok(packet) => {
                                // get channels and map from -1 to 1
                                let channels: [f32; 14] = packet
                                    .get_all_channels()
                                    .map(|c| (c as f32 - 1500.0) / 500.0);

                                let mut unlocked = inputs.lock().unwrap();
                                unlocked.setpoint.roll = channels[0] * sensitivity.roll;
                                unlocked.setpoint.pitch = channels[1] * sensitivity.pitch;
                                unlocked.setpoint.yaw_rate = channels[3] * sensitivity.yaw_rate;
                                unlocked.setpoint.altitude = channels[6] * sensitivity.altitude;
                                unlocked.controller_enable = channels[5] > 0.6;
                            }
                            Err(e) => match e {
                                ParsingError::FailsChecksum => println!("invalid package"),
                                ParsingError::InvalidPacket => {
                                    println!("ibus desync!");
                                    let _ = port.read(&mut header_buffer);
                                }
                            },
                        }
                    }
                    Err(_) => {} //println!("Reveiver Error {:?}", e),
                }
            }
        });
    }

    pub fn get_inputs(&self) -> Inputs {
        *self.inputs.lock().unwrap()
    }
}
