use crate::{
    estimator::State,
    influx::{DataPoint, Log},
    receiver::Inputs,
};
use serde::Deserialize;
use std::{
    f32::consts::PI,
    ops::Add,
    sync::{
        mpsc::{Receiver, Sender, SyncSender},
        Arc, Mutex,
    },
    thread,
    time::SystemTime,
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

#[derive(Debug, Deserialize, Clone)]
pub struct ControlAction {
    pub port: f32,
    pub starboard: f32,
    pub aft: f32,
    pub rudder: f32,
}

impl Default for ControlAction {
    fn default() -> Self {
        Self {
            port: 0.0,
            starboard: 0.0,
            aft: 0.0,
            rudder: 0.0,
        }
    }
}

impl Log for ControlAction {
    fn measurements(&self) -> Vec<(String, f32)> {
        vec![
            ("Port".to_owned(), self.port),
            ("Starboard".to_owned(), self.starboard),
            ("Aft".to_owned(), self.aft),
            ("Rudder".to_owned(), self.rudder),
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
pub struct ConrollerState {
    pub roll: f32,
    pub pitch: f32,
    pub yaw_rate: f32,
    pub altitude: f32,
}

impl Default for ConrollerState {
    fn default() -> Self {
        Self {
            roll: 0.0,
            pitch: 0.0,
            yaw_rate: 0.0,
            altitude: 0.0,
        }
    }
}

impl Add for ConrollerState {
    type Output = Self;

    fn add(self, rhs: Self) -> Self::Output {
        Self {
            roll: self.roll + rhs.roll,
            pitch: self.pitch + rhs.pitch,
            yaw_rate: self.yaw_rate + rhs.yaw_rate,
            altitude: self.altitude + rhs.altitude,
        }
    }
}

impl Log for ConrollerState {
    fn measurements(&self) -> Vec<(String, f32)> {
        vec![
            ("Roll".to_owned(), self.roll),
            ("Pitch".to_owned(), self.pitch),
            ("Yaw_Rate".to_owned(), self.yaw_rate),
            ("altitude".to_owned(), self.altitude),
        ]
    }
}

impl From<ConrollerState> for [f32; 4] {
    fn from(state: ConrollerState) -> Self {
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
}

impl FlightController {
    pub fn run(
        mut self,
        state_estimation_rx: Receiver<State>,
        controls_tx: Sender<ControlAction>,
        influx_tx: SyncSender<DataPoint>,
        input: Arc<Mutex<Inputs>>,
    ) {
        let mut last_update = SystemTime::now();
        thread::spawn(move || -> ! {
            loop {
                // wait for new state estimation
                let state = state_estimation_rx.recv().unwrap();
                let dt = SystemTime::now()
                    .duration_since(last_update)
                    .unwrap()
                    .as_secs_f32();
                influx_tx
                    .send(DataPoint::new_single(
                        "pid_rate".to_owned(),
                        "rate_ms".to_owned(),
                        dt * 1000.0,
                    ))
                    .unwrap();
                let (roll, pitch, _yaw) = state.orientation.euler_angles();

                {
                    // get the most recent setpoint
                    let input = input.lock().unwrap();

                    let action: ControlAction = if input.controller_enable {
                        // calculate pids
                        let pid = ConrollerState {
                            roll: self.roll.update(input.setpoint.roll, roll / PI * 180.0, dt),
                            pitch: self
                                .pitch
                                .update(input.setpoint.pitch, pitch / PI * 180.0, dt),
                            yaw_rate: self.yaw.update(
                                input.setpoint.yaw_rate / PI * 180.0,
                                state.angular_vel[2],
                                dt,
                            ),
                            altitude: self.altitude.update(
                                input.setpoint.altitude,
                                state.position[2],
                                dt,
                            ),
                        };

                        influx_tx
                            .send(DataPoint::new("pid".to_owned(), pid))
                            .unwrap();

                        // apply mixing
                        let mut action = [0.0; 4];
                        let pid_array: [f32; 4] = pid.into();
                        for i in 0..4 {
                            for j in 0..4 {
                                action[i] += self.mix_matrix[i][j] * pid_array[j];
                            }
                        }
                        action.into()
                    } else {
                        self.reset();
                        ControlAction::default()
                    };

                    controls_tx.send(action.clone()).unwrap();
                    influx_tx
                        .send(DataPoint::new("action".to_owned(), action))
                        .unwrap();

                    last_update = SystemTime::now();
                }
            }
        });
    }

    pub fn reset(&mut self) {
        self.roll.i_term = 0.0;
        self.pitch.i_term = 0.0;
        self.yaw.i_term = 0.0;
        self.altitude.i_term = 0.0;
    }
}
#[cfg(test)]
mod tests {
    use super::Pid;

    #[test]
    fn test_clamp() {
        let mut pid = Pid {
            p: 10.0,
            i: 0.0,
            d: 0.0,
            i_limit: 1.0,
            i_term: 0.0,
            last_error: 0.0,
        };

        assert_eq!(-1.0, pid.update(0.0, 1.0, 1.0));
    }
}
