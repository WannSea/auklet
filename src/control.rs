use nalgebra::{vector, Matrix4, SVector};

struct MimoPid<const D: usize> {
    p: SVector<f32, D>,
    i: SVector<f32, D>,
    d: SVector<f32, D>,
    i_limit: SVector<f32, D>,
    i_term: SVector<f32, D>,
    last_error: SVector<f32, D>,
}

impl<const D: usize> MimoPid<D> {
    fn new(
        p: SVector<f32, D>,
        i: SVector<f32, D>,
        d: SVector<f32, D>,
        i_limit: SVector<f32, D>,
    ) -> Self {
        Self {
            p,
            i,
            d,
            i_limit,
            i_term: SVector::<f32, D>::zeros(),
            last_error: SVector::<f32, D>::zeros(),
        }
    }

    fn update(
        &mut self,
        setpoint: SVector<f32, D>,
        measurement: SVector<f32, D>,
        dt: f32,
    ) -> SVector<f32, D> {
        let error = setpoint - measurement;

        let p = error.component_mul(&self.p);

        self.i_term = self.i_term + dt * error;
        // anti windup
        self.i_term = self
            .i_term
            .zip_map(&self.i_limit, |i, limit| i.clamp(-limit, limit));
        let i = self.i_term.component_mul(&self.i);

        let derivative = (self.last_error - error) / dt;
        let d = derivative.component_mul(&self.d);

        p + i + d
    }
}

#[derive(Debug)]
pub struct ControlAction {
    pub port: f32,
    pub starboard: f32,
    pub aft: f32,
    pub rudder: f32,
}
impl From<ControlAction> for SVector<f32, 4> {
    fn from(action: ControlAction) -> Self {
        SVector::from_row_slice(&[action.port, action.starboard, action.aft, action.rudder])
    }
}
impl From<SVector<f32, 4>> for ControlAction {
    fn from(vec: SVector<f32, 4>) -> Self {
        Self {
            port: vec[0],
            starboard: vec[1],
            aft: vec[2],
            rudder: vec[3],
        }
    }
}

#[derive(Debug, Clone, Copy)]
pub struct State {
    pub roll: f32,
    pub pitch: f32,
    pub yaw_rate: f32,
    pub altitude: f32,
}

impl From<State> for SVector<f32, 4> {
    fn from(state: State) -> Self {
        SVector::from_row_slice(&[state.roll, state.pitch, state.yaw_rate, state.altitude])
    }
}

pub struct FlightController {
    pid: MimoPid<4>,
}

impl FlightController {
    pub fn new() -> Self {
        Self {
            //  (roll, pitch, yaw, altitude) to the ouputs
            pid: MimoPid::new(
                vector![0.6, 3.0, 0.3, 1.0],
                vector![0.0, 0.0, 0.0, 0.0],
                vector![0.0, 0.0, 0.0, 0.0],
                vector![25.0, 25.0, 25.0, 25.0],
            ),
        }
    }

    pub fn update_controller(
        &mut self,
        setpoint: State,
        measurement: State,
        dt: f32,
    ) -> ControlAction {
        // pid control
        let pid = self.pid.update(setpoint.into(), measurement.into(), dt);

        // mix the controllers (roll, pitch, yaw, altitude) to the ouputs
        let control_matrix = Matrix4::new(
            1.0, 0.4, 0.0, -1.0, // port
            -1.0, 0.4, 0.0, -1.0, // starboard
            0.0, -1.0, 0.0, -1.0, // aft
            0.0, 0.0, 1.0, 0.0, // rudder
        );

        (control_matrix * pid).into()
    }
}
