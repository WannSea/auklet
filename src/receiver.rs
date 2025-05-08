use std::f32::consts::PI;
use std::sync::{Arc, Mutex};
use std::time::Duration;

use crate::control::State;
use parse_rc_ibus::{IbusPacket, ParsingError};
use serialport;

const MAX_ROLL: f32 = 10.0;
const MAX_PITCH: f32 = 10.0;
const MAX_YAW_RATE: f32 = 180.0;

// const IBUS_HEADER: [u8; 2] = [0x20, 0x40];

pub fn handle_receiver(setpoint: Arc<Mutex<State>>) {
    let mut port = serialport::new("/dev/ttyAMA1", 115_200)
        .timeout(Duration::from_millis(14))
        .open()
        .expect("Failed to open port");

    let mut buffer = [0u8; 32];
    let mut header_buffer = [0u8; 1];

    loop {
        match port.read_exact(&mut buffer) {
            Ok(()) => {
                match IbusPacket::try_from_bytes(&buffer) {
                    Ok(packet) => {
                        // get channels and map from -1 to 1
                        let channels: [f32; 14] = packet
                            .get_all_channels()
                            .map(|c| (c as f32 - 1500.0) / 500.0);

                        let mut unlocked = setpoint.lock().unwrap();
                        unlocked.roll = channels[0] * MAX_ROLL;
                        unlocked.pitch = channels[1] * MAX_PITCH + 5.0;
                        unlocked.yaw_rate = channels[3] * MAX_YAW_RATE;
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
}
