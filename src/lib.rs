mod utils;

use std::ops::Add;
use std::ops::Div;
use std::ops::Mul;
use std::ops::Sub;

// Println
extern crate web_sys;
#[allow(unused_macros)]
macro_rules! log {
    ( $( $t:tt )* ) => {
        web_sys::console::log_1(&format!( $( $t )* ).into());
    }
}

use wasm_bindgen::prelude::*;

const GRAVITY: f32 = 20.0;
const RADIUS: f32 = 25.0;
const COLLISIONS: bool = true;
const COLLISION_ENERGY_LOSS: f32 = 0.80;

#[derive(Clone, Copy, Debug)]
struct Vector2D(f32, f32);

impl Default for Vector2D {
    fn default() -> Self {
        Self(0.0, 0.0)
    }
}

impl Add for Vector2D {
    type Output = Self;

    fn add(self, Self(x, y): Self) -> Self::Output {
        Self(self.0 + x, self.1 + y)
    }
}

impl Sub for Vector2D {
    type Output = Self;

    fn sub(self, Self(x, y): Self) -> Self::Output {
        Self(self.0 - x, self.1 - y)
    }
}

impl Mul<f32> for Vector2D {
    type Output = Self;

    fn mul(self, s: f32) -> Self::Output {
        Self(self.0*s, self.1*s)
    }
}

impl Mul<Vector2D> for f32 {
    type Output = Vector2D;

    fn mul(self, rhs: Vector2D) -> Self::Output {
        rhs*self    
    }
}

impl Div<f32> for Vector2D {
    type Output = Self;

    fn div(self, s: f32) -> Self::Output {
        Self(self.0/s, self.1/s)
    }
}

impl Vector2D {
    fn mag2(&self) -> f32 {
        self.0*self.0 + self.1*self.1
    }

    fn norm(self) -> Vector2D {
        self / self.mag2().sqrt()
    }
}


/**
 * Compute effect of gravity on target from source
 */
fn gravity_equation(target: Vector2D, source: Vector2D) -> Vector2D {
    // Determine distance and direction to source from target
    let rel = source - target;
    let dir = rel.norm();
    let true_mag = rel.mag2().sqrt();
    // Cap minimum distance to radius*2
    let mag = true_mag.max(2.0*RADIUS);

    // Compute force value due to gravity
    let gravity = GRAVITY / (mag*mag);

    // Scale direction based on gravity
    gravity * dir
}


#[derive(Debug, Default)]
struct Body {
    position: Vector2D,
    velocity: Vector2D
}

impl Body {
    pub fn new(pos: Vector2D, vel: Vector2D) -> Body {
        Body {
            position: pos,
            velocity: vel
        }
    }

    pub fn updated<ForceFunc>(&self, dt: f32, force_func: ForceFunc) -> Body
        where ForceFunc: Fn(Vector2D, Vector2D) -> Vector2D
    {
        let force = force_func(self.position, self.velocity);
        let new_position = self.position + self.velocity*dt;
        let new_velocity = self.velocity + force*dt;
        Body::new(new_position, new_velocity)
    }

    pub fn collision_correct(&self, width: f32, height: f32) -> Body
    {
        let xp = width/2.0 - RADIUS;
        let xn = -xp;
        let yp = height/2.0 - RADIUS;
        let yn = -yp;
        if COLLISIONS {
            if self.position.0 < xn {
                let new_position = Vector2D(xn, self.position.1);
                let new_velocity = Vector2D(-self.velocity.0, self.velocity.1);
                Body::new(new_position, COLLISION_ENERGY_LOSS*new_velocity)
            }
            else if self.position.0 > xp {
                
                let new_position = Vector2D(xp, self.position.1);
                let new_velocity = Vector2D(-self.velocity.0, self.velocity.1);
                Body::new(new_position, COLLISION_ENERGY_LOSS*new_velocity)
            }
            else if self.position.1 < yn {
                
                let new_position = Vector2D(self.position.0, yn);
                let new_velocity = Vector2D(self.velocity.0, -self.velocity.1);
                Body::new(new_position, COLLISION_ENERGY_LOSS*new_velocity)
            }
            else if self.position.1 > yp {
                
                let new_position = Vector2D(self.position.0, yp);
                let new_velocity = Vector2D(self.velocity.0, -self.velocity.1);
                Body::new(new_position, COLLISION_ENERGY_LOSS*new_velocity)
            }
            else {
                Body::new(self.position, self.velocity)
            }
        }
        else { Body::new(self.position, self.velocity) }
    }
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
                Vector2D(self.positions[2*i], self.positions[2*i + 1]),
                Vector2D(self.velocities[2*i], self.velocities[2*i + 1])
            );
        };
    }

    fn bodies_to_state(&mut self) {
        for i in 0..3 {
            self.positions[2*i] = self.bodies[i].position.0;
            self.positions[2*i + 1] = self.bodies[i].position.1;
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
        let new_ba = ba
            .updated(dt, |p, _v| {
                let f1 = gravity_equation(p, bb.position);
                let f2 = gravity_equation(p, bc.position);
                f1 + f2
            })
            .collision_correct(width, height);
        let new_bb = bb
            .updated(dt, |p, _v| {
                let f1 = gravity_equation(p, ba.position);
                let f2 = gravity_equation(p, bc.position);
                f1 + f2
            })
            .collision_correct(width, height);
        let new_bc = bc
            .updated(dt, |p, _v| {
                let f1 = gravity_equation(p, ba.position);
                let f2 = gravity_equation(p, bb.position);
                f1 + f2
            })
            .collision_correct(width, height);

        // Update bodies
        self.bodies[0] = new_ba;
        self.bodies[1] = new_bb;
        self.bodies[2] = new_bc;

        // Update state
        self.bodies_to_state();
    }
}