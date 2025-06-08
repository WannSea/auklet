use chrono::{DateTime, Utc};
use std::env;
use std::sync::mpsc::Receiver;
use std::thread;

pub struct DataPoint {
    name: String,
    timestamp: DateTime<Utc>,
    data: Vec<(String, f32)>,
}

impl DataPoint {
    pub fn new(name: String, data: impl Log) -> Self {
        Self {
            name,
            timestamp: Utc::now(),
            data: data.measurements(),
        }
    }

    pub fn new_single(name: String, field: String, value: f32) -> Self {
        Self {
            name,
            timestamp: Utc::now(),
            data: vec![(field, value)],
        }
    }

    pub fn to_influx_post(self) -> String {
        let string_data: String = self
            .data
            .into_iter()
            .map(|(name, value)| format!("{}={}", name, value))
            .collect::<Vec<String>>()
            .join(",");

        format!(
            "{} {} {}",
            self.name,
            string_data,
            self.timestamp.timestamp_nanos_opt().unwrap()
        )
    }
}

pub trait Log {
    fn measurements(&self) -> Vec<(String, f32)>;
}

pub struct InfluxHandler {}

impl InfluxHandler {
    pub fn run(self, influx_rx: Receiver<DataPoint>) {
        let influx_url = env::var("INFLUX_URL").expect("no url provided");
        let influx_bucket = env::var("INFLUX_BUCKET").expect("no bucket provided");
        let influx_token = env::var("INFLUX_TOKEN").expect("no token provided");
        let url =
            format!("{influx_url}/api/v2/write?org=wannsea&bucket={influx_bucket}&precision=ns");

        // maybe add aggregation to not ddos the influxdb
        thread::spawn(move || loop {
            let data_point = influx_rx.recv().unwrap();
            let line = data_point.to_influx_post();

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
        });
    }
}
