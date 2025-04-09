use chrono::Utc;
use std::sync::{Arc, Mutex};
use std::thread::{self, sleep};
use std::time::Duration;

/// Represents a single measurement for InfluxDB (name + value)
pub struct Measurement {
    pub name: &'static str,
    pub value: f32,
}

pub trait Log: Send + Sync + 'static {
    /// Returns all measurements this struct represents
    fn measurements(&self) -> Vec<Measurement>;

    /// Generates InfluxDB line protocol strings
    fn to_line_protocol(&self) -> Vec<String> {
        let timestamp = Utc::now().timestamp_nanos();
        self.measurements()
            .into_iter()
            .map(|m| format!("{} value={} {}", m.name, m.value, timestamp))
            .collect()
    }
}

pub fn influx_log<T: Log>(shared: Arc<Mutex<T>>, interval: Duration) {
    thread::spawn(move || {
        let influx_url = "http://localhost:8086/write?db=example_db";

        loop {
            let lines = {
                let data = shared.lock().unwrap();
                data.to_line_protocol()
            };

            for line in lines {
                let response = ureq::post(influx_url)
                    .header("Content-Type", "text/plain")
                    .send(&line);

                match response {
                    Ok(resp) if resp.status() == 204 => {
                        println!("[Influx] Logged: {}", line);
                    }
                    Ok(resp) => {
                        eprintln!("[Influx] Error {}: {}", resp.status(), line);
                    }
                    Err(e) => {
                        eprintln!("[Influx] Network error: {:?}", e);
                    }
                }
            }
            sleep(interval);
        }
    });
}
