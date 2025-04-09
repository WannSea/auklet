use serialport;
use std::sync::{Arc, Mutex};
use std::time::Duration;

const START: u8 = 0xFF;

pub fn handle_sonar(distance: Arc<Mutex<f32>>) {
    let mut port = serialport::new("/dev/ttyAMA2", 9600)
        .timeout(Duration::from_millis(30))
        .open()
        .expect("Failed to open port");

    let mut buffer = [0u8; 4];

    loop {
        let _ = port.read_exact(&mut buffer);

        if buffer[0] == START {
            let distance_mm: u16 = u16::from_be_bytes([buffer[1], buffer[2]]);

            // dbg!(buffer);
            //dbg!(distance_mm);
            *distance.lock().unwrap() = distance_mm as f32 / 1000.0;
            // checksum
            // ToDo: fix
            // if buffer[1] + buffer[2] != buffer[3] {
            //     println!("sonar, checksum error");
            // } else {
            //     *distance.lock().unwrap() = distance_mm as f32 / 1000.0;
            // }
        } else {
            let mut null = [0u8; 0];
            let _ = port.read_exact(&mut null);
        }
    }
}
