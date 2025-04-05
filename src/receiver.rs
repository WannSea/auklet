use std::f32::consts::PI;
use std::sync::{Arc, Mutex};
use std::time::Duration;

use crate::control::State;
use parse_rc_ibus::IbusPacket;
use serialport;

const MAX_ROLL: f32 = 30.0 / 180.0 * PI;
const MAX_PITCH: f32 = 20.0 / 180.0;
const MAX_YAW_RATE: f32 = 90.0 / 180.0 * PI;

pub fn handle_receiver(setpoint: Arc<Mutex<State>>) {
    let mut port = serialport::new("/dev/serial0", 115_200)
        .timeout(Duration::from_millis(14))
        .open()
        .expect("Failed to open port");

    let mut raw = [0u8; 32];

    loop {
        for i in 0..32 {
            let mut byte = [0u8; 1];
            if port.read(&mut byte).is_ok() {
                raw[i] = byte[0]
            }
        }

        if let Ok(packet) = IbusPacket::try_from_bytes(&raw) {
            // get channels and map from -1 to 1
            let channels: [f32; 14] = packet.get_all_channels().map(|c| (c - 1500) as f32 / 500.0);
            *setpoint.lock().unwrap() = State {
                roll: channels[0] * MAX_ROLL,
                pitch: channels[1] * MAX_PITCH,
                yaw_rate: channels[4] * MAX_YAW_RATE,
                altitude: 30.0,
            }
        }
    }
}
