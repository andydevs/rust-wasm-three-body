// console.log
extern crate web_sys;
#[allow(unused_macros)]
macro_rules! log {
    ( $( $t:tt )* ) => {
        web_sys::console::log_1(&format!( $( $t )* ).into());
    }
}

mod utils;
mod constants;
mod body;
mod boundary;

use constants::*;
use wasm_bindgen::prelude::*;
type Vector2D = nalgebra::Vector2<f32>;
use body::Body;
use boundary::boundary_collision_correct;


/**
 * Compute effect of gravity on target from source
 */
fn gravity_equation(target: &Vector2D, source: &Vector2D) -> Vector2D {
    let rel = source - target;
    let dir = rel.normalize();
    let true_mag = rel.magnitude();
    let mag = true_mag.max(2.0*RADIUS);
    dir * GRAVITY / mag.powi(2)
}


#[wasm_bindgen]
pub struct ThreeBodySystem {
    bodies: [Body; 3],
    positions: [f32; 6],
    velocities: [f32; 6]
}

#[wasm_bindgen]
impl ThreeBodySystem {
    pub fn new() -> ThreeBodySystem {
        ThreeBodySystem {
            bodies: [ Body::default(), Body::default(), Body::default() ],
            positions: [0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
            velocities: [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
        }
    }

    pub fn get_radius(&self) -> f32 {
        RADIUS
    }

    pub fn get_state(&self) -> *const f32 {
        self.positions.as_ptr()
    }

    pub fn get_state_size(&self) -> i32 {
        6
    }

    fn state_to_bodies(&mut self) {
        for i in 0..3 {
            self.bodies[i] = Body::new(
                Vector2D::new(self.positions[2*i], self.positions[2*i + 1]),
                Vector2D::new(self.velocities[2*i], self.velocities[2*i + 1])
            );
        };
    }

    fn bodies_to_state(&mut self) {
        for i in 0..3 {
            self.positions[2*i] = self.bodies[i].position.x;
            self.positions[2*i + 1] = self.bodies[i].position.y;
        };
    }

    pub fn initialize_position(&mut self, x_a: f32, y_a: f32, x_b: f32, y_b: f32, x_c: f32, y_c: f32) {
        self.positions = [ x_a, y_a, x_b, y_b, x_c, y_c ];
        self.state_to_bodies();
    }

    pub fn initialize_velocity(&mut self, x_a: f32, y_a: f32, x_b: f32, y_b: f32, x_c: f32, y_c: f32) {
        self.velocities = [ x_a, y_a, x_b, y_b, x_c, y_c ];
        self.state_to_bodies();
    }

    pub fn physics_update(&mut self, dt: f32, width: f32, height: f32) {
        // Create bodies
        let ba = &self.bodies[0];
        let bb = &self.bodies[1];
        let bc = &self.bodies[2];

        // Compute updated forces
        let updated_ba = ba
            .updated(dt, |p, _v| {
                let f1 = gravity_equation(p, &bb.position);
                let f2 = gravity_equation(p, &bc.position);
                f1 + f2
            });
        let updated_bb = bb
            .updated(dt, |p, _v| {
                let f1 = gravity_equation(p, &ba.position);
                let f2 = gravity_equation(p, &bc.position);
                f1 + f2
            });
        let updated_bc = bc
            .updated(dt, |p, _v| {
                let f1 = gravity_equation(p, &ba.position);
                let f2 = gravity_equation(p, &bb.position);
                f1 + f2
            });

        let corrected_ba = boundary_collision_correct(&updated_ba, width, height);
        let corrected_bb = boundary_collision_correct(&updated_bb, width, height);
        let corrected_bc = boundary_collision_correct(&updated_bc, width, height);

        // Update bodies
        self.bodies[0] = if COLLISIONS { corrected_ba } else { updated_ba };
        self.bodies[1] = if COLLISIONS { corrected_bb } else { updated_bb };
        self.bodies[2] = if COLLISIONS { corrected_bc } else { updated_bc };

        // Update state
        self.bodies_to_state();
    }
}