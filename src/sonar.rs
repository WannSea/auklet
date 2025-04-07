use serialport;
use std::sync::{Arc, Mutex};
use std::time::Duration;

const START: u8 = 0xFF;

pub fn handle_sonar(distance: Arc<Mutex<f32>>) {
    let mut port = serialport::new("/dev/serial1", 115_200)
        .timeout(Duration::from_millis(14))
        .open()
        .expect("Failed to open port");

    let mut buffer = [0u8; 3];

    loop {
        let mut start = [0u8];
        let _ = port.read_exact(&mut start);

        if start[0] == START {
            let _ = port.read(&mut buffer);
            let distance_mm: u16 = u16::from_be_bytes([buffer[0], buffer[1]]);

            // checksum
            if buffer.iter().sum::<u8>() & 0xFF != buffer[2] {
                println!("sonar, checksum error");
            } else {
                *distance.lock().unwrap() = distance_mm as f32 / 1000.0;
            }
        }
    }
}
