use crate::influx::{Log, Measurement};
use serde::Deserialize;
use std::{
    f32::consts::PI,
    sync::{Arc, Mutex},
};

#[derive(Deserialize, Debug)]
struct Pid {
    p: f32,
    i: f32,
    d: f32,
    i_limit: f32,
    #[serde(default)]
    i_term: f32,
    #[serde(default)]
    last_error: f32,
}

impl Pid {
    fn update(&mut self, setpoint: f32, measurement: f32, dt: f32) -> f32 {
        let error = setpoint - measurement;
        let p = error * self.p;

        self.i_term = self.i_term + dt * error;
        // anti windup
        self.i_term = self.i_term.clamp(-self.i_limit, self.i_limit);
        let i = self.i_term * self.i;

        let derivative = (self.last_error - error) / dt;
        let d = derivative * self.d;

        (p + i + d).clamp(-1.0, 1.0)
    }
}

#[derive(Debug, Deserialize)]
pub struct ControlAction {
    pub port: f32,
    pub starboard: f32,
    pub aft: f32,
    pub rudder: f32,
}

impl Log for ControlAction {
    fn measurements(&self) -> Vec<crate::influx::Measurement> {
        vec![
            Measurement {
                name: "Port",
                value: self.port / PI * 180.0,
            },
            Measurement {
                name: "Starboard",
                value: self.starboard / PI * 180.0,
            },
            Measurement {
                name: "Aft",
                value: self.aft / PI * 180.0,
            },
            Measurement {
                name: "Rudder",
                value: self.rudder / PI * 180.0,
            },
        ]
    }
}

impl From<[f32; 4]> for ControlAction {
    fn from(vec: [f32; 4]) -> Self {
        Self {
            port: vec[0],
            starboard: vec[1],
            aft: vec[2],
            rudder: vec[3],
        }
    }
}

#[derive(Debug, Clone, Copy, Deserialize)]
pub struct State {
    pub roll: f32,
    pub pitch: f32,
    pub yaw_rate: f32,
    pub altitude: f32,
}

impl Default for State {
    fn default() -> Self {
        Self {
            roll: 0.0,
            pitch: 0.0,
            yaw_rate: 0.0,
            altitude: 0.0,
        }
    }
}

impl Log for State {
    fn measurements(&self) -> Vec<crate::influx::Measurement> {
        vec![
            Measurement {
                name: "Roll",
                value: self.roll / PI * 180.0,
            },
            Measurement {
                name: "Pitch",
                value: self.pitch / PI * 180.0,
            },
            Measurement {
                name: "Yaw_Rate",
                value: self.yaw_rate / PI * 180.0,
            },
            Measurement {
                name: "altitude",
                value: self.altitude,
            },
        ]
    }
}

impl From<State> for [f32; 4] {
    fn from(state: State) -> Self {
        [state.roll, state.pitch, state.yaw_rate, state.altitude]
    }
}

#[derive(Debug, Deserialize)]
pub struct FlightController {
    roll: Pid,
    pitch: Pid,
    yaw: Pid,
    altitude: Pid,
    mix_matrix: [[f32; 4]; 4],

    #[serde(skip_deserializing)]
    pub current_pid: Arc<Mutex<State>>,
}

impl FlightController {
    pub fn update_controller(
        &mut self,
        setpoint: State,
        measurement: State,
        dt: f32,
    ) -> ControlAction {
        let pid = State {
            roll: self.roll.update(setpoint.roll, measurement.roll, dt),
            pitch: self.pitch.update(setpoint.pitch, measurement.pitch, dt),
            yaw_rate: self.yaw.update(setpoint.yaw_rate, measurement.yaw_rate, dt),
            altitude: self
                .altitude
                .update(setpoint.altitude, measurement.altitude, dt),
        };
        *self.current_pid.lock().unwrap() = pid;

        let mut action = [0.0; 4];
        let pid_array: [f32; 4] = pid.into();
        for i in 0..4 {
            for j in 0..4 {
                action[i] += self.mix_matrix[i][j] * pid_array[j];
            }
        }
        action.into()
    }
}
