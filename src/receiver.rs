use std::f32::consts::PI;
use std::sync::{Arc, Mutex};
use std::time::Duration;

use crate::control::State;
use parse_rc_ibus::{IbusPacket, ParsingError};
use serialport;

const MAX_ROLL: f32 = 30.0 / 180.0 * PI;
const MAX_PITCH: f32 = 20.0 / 180.0;
const MAX_YAW_RATE: f32 = 90.0 / 180.0 * PI;

const IBUS_HEADER: [u8; 2] = [0x20, 0x10];

pub fn handle_receiver(setpoint: Arc<Mutex<State>>) {
    let mut port = serialport::new("/dev/ttyAMA2", 115_200)
        .timeout(Duration::from_millis(14))
        .open()
        .expect("Failed to open port");

    let mut buffer = [0u8; 32];
    let mut header_buffer = [0u8; 2];

    loop {
        match port.read_exact(&mut buffer) {
            Ok(()) => {
                if buffer[0..2] == IBUS_HEADER {
                    match IbusPacket::try_from_bytes(&buffer) {
                        Ok(packet) => {
                            // get channels and map from -1 to 1
                            let channels: [f32; 14] =
                                packet.get_all_channels().map(|c| (c - 1500) as f32 / 500.0);
                            *setpoint.lock().unwrap() = State {
                                roll: channels[0] * MAX_ROLL,
                                pitch: channels[1] * MAX_PITCH,
                                yaw_rate: channels[4] * MAX_YAW_RATE,
                                altitude: 30.0,
                            }
                        }
                        Err(e) => match e {
                            ParsingError::FailsChecksum => println!("invalid package"),
                            ParsingError::InvalidPacket => {
                                println!("desynced");
                                loop {
                                    // searching for header
                                    let _ = port.read_exact(&mut header_buffer);
                                    if header_buffer == IBUS_HEADER {
                                        // drop the last 32 bytes to get back in sync
                                        let _ = port.read_exact(&mut buffer[2..]);
                                        break;
                                    }
                                }
                            }
                        },
                    }
                }
            }
            Err(e) => println!("{:?}",e),
        }
    }
}
