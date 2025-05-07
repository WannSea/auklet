use chrono::Utc;
use std::env;
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
    fn to_line_protocol(&self, measurment: &String) -> String {
        let timestamp = Utc::now().timestamp_nanos_opt().unwrap();
        let data: String = self
            .measurements()
            .into_iter()
            .map(|m| format!("{}={}", m.name, m.value))
            .collect::<Vec<String>>()
            .join(",");
        format!("{} {} {}", measurment, data, timestamp)
    }
}

pub fn influx_log<T: Log>(shared: Arc<Mutex<T>>, measurement: String, interval: Duration) {
    let influx_url = env::var("INFLUX_URL").expect("no url provided");
    let influx_bucket = env::var("INFLUX_BUCKET").expect("no bucket provided");
    let influx_token = env::var("INFLUX_TOKEN").expect("no token provided");

    let url = format!("{influx_url}/api/v2/write?org=wannsea&bucket={influx_bucket}&precision=ns");

    thread::spawn(move || loop {
        let line = {
            let data = shared.lock().unwrap();
            data.to_line_protocol(&measurement)
        };

        let response = ureq::post(&url)
            .header("Authorization", format!("Token {influx_token}"))
            .header("Content-Type", "text/plain; charset=utf-8")
            .header("Accept", "application/json")
            .send(&line);

        match response {
            Ok(resp) if resp.status() == 204 => {
                //  println!("[Influx] Logged: {}", line);
            }
            Ok(resp) => {
                eprintln!("[Influx] Error {}: {} \n url:{}", resp.status(), line, url);
            }
            Err(e) => {
                eprintln!(
                    "[Influx] Network error: {:?} \n url:{} \n line:{}",
                    e, url, line
                );
            }
        }
        sleep(interval);
    });
}
