use core::f32::consts::PI;
use micromath::*;

pub struct Motor {
    pole_count: u16,
    phase_offset: i32
}

pub struct MotorState {
    pub position: i32
}

pub trait Control {
    fn phase_vector(&self, motor: &Motor, motor_state: &MotorState) -> [u16; 3];
}

pub struct TorqueControl {
    pub throttle: i32
}

pub trait Driver {
    fn set_voltages(&mut self, phase_vector: [u16; 3]);
}

impl MotorState {
    fn phase_state(&self, motor: &Motor) -> i32 {
        (self.position + motor.phase_offset).wrapping_mul(motor.pole_count as i32)
    }
}

impl Motor {
    pub fn new(pole_count: u16) -> Motor {
        Motor { pole_count, phase_offset: 0 }
    }
    pub fn set_zero(&mut self, position: i32) {
        self.phase_offset = -position;
    }


}

impl Control for TorqueControl {
    fn phase_vector(&self, motor: &Motor, motor_state: &MotorState) -> [u16; 3] {
        const PHASE_SHIFT: i32 = (i32::MAX / 3) * 2;
        const TORQUE_VECTOR_ANGLE: i32 = - (i32::MIN / 2);
        let torque_phase = motor_state.phase_state(motor).wrapping_add(TORQUE_VECTOR_ANGLE);
        [torque_phase.wrapping_sub(PHASE_SHIFT), torque_phase, torque_phase.wrapping_add(PHASE_SHIFT)]
            .map(|phase| {
                let rad = PI * (phase as f32) / (-(i32::MIN as f32));
                let magnitude  = (rad.sin() * (self.throttle as f32)) as i32;
                (magnitude >> 16) as u16 + (u16::MAX / 2)
            })
    }
}

/*
pub fn trapezoid(throttle: u16, phase: i32) -> u16 {
    const PHASE_SEGMENT_LENGTH: i32 = i32::MAX / 3;

    let phase_segment = phase / PHASE_SEGMENT_LENGTH;
    if phase_segment < -1 {
        throttle
    } else if phase_segment < 0 {
        ((throttle as i32) * ((3 * PHASE_SEGMENT_LENGTH - phase) >> 16 ) / (PHASE_SEGMENT_LENGTH >> 16)) as u16
    } else if phase_segment < 2 {
        0
    } else {
        ((throttle as i32) * ((phase - PHASE_SEGMENT_LENGTH * 5) >> 16) / (PHASE_SEGMENT_LENGTH >> 16)) as u16
    }
}


pub fn sinusoid(throttle: u16, phase: i32) -> u16 {
    let rad = PI * (phase as f32) / (i32::MAX as f32);
    ((1f32 + (rad.sin() / 2f32)) * (throttle as f32)) as u16
}
*/
