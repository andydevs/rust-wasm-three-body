mod utils;

use wasm_bindgen::prelude::*;

const RADIUS: f32 = 10.0;
const MASS: f32 = 10.0;
const GRAVITY: f32 = 100.0;

type Vector2D = [f32; 2];

fn vec2_scale(s: f32, v: Vector2D) -> Vector2D {
    [ v[0]*s, v[1]*s ]
}

fn vec2_add(a: Vector2D, b: Vector2D) -> Vector2D {
    [ a[0]+b[0], a[1]+b[1] ]
}

fn vec2_mag2(v: Vector2D) -> f32 {
    v.iter().map(|x| x*x).sum::<f32>()
}

fn vec2_norm(v: Vector2D) -> Vector2D {
    let mag = vec2_mag2(v).sqrt();
    let inv = 1.0/mag;
    vec2_scale(inv, v)
}

/**
 * Compute effect of gravity on target from source
 */
fn gravity_equation(target: Vector2D, source: Vector2D) -> Vector2D {
    // Determine distance and direction to source from target
    let rel = vec2_add(source, vec2_scale(-1.0, target));
    let dir = vec2_norm(rel);
    let true_mag = vec2_mag2(rel).sqrt();
    // Cap minimum distance to radius*2
    let mag = true_mag.max(2.0*RADIUS);

    // Compute force value due to gravity
    let gravity = GRAVITY * MASS / (mag*mag);

    // Scale direction based on gravity
    vec2_scale(gravity, dir)
}


#[wasm_bindgen]
pub struct ThreeBodySystem {
    positions: [f32; 6],
    velocities: [f32; 6]
}

#[wasm_bindgen]
impl ThreeBodySystem {
    pub fn new() -> ThreeBodySystem {
        ThreeBodySystem {
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

    pub fn initialize_position(&mut self, x_a: f32, y_a: f32, x_b: f32, y_b: f32, x_c: f32, y_c: f32) {
        self.positions = [
            x_a, y_a, x_b, y_b, x_c, y_c
        ]
    }

    pub fn initialize_velocity(&mut self, x_a: f32, y_a: f32, x_b: f32, y_b: f32, x_c: f32, y_c: f32) {
        self.velocities = [
            x_a, y_a, x_b, y_b, x_c, y_c
        ]
    }

    pub fn physics_update(&mut self, dt: f32) {
        // Get vectors for position and velocity of three bodies
        let pa = [self.positions[0], self.positions[1]];
        let pb = [self.positions[2], self.positions[3]];
        let pc = [self.positions[4], self.positions[5]];
        let va = [self.velocities[0], self.velocities[1]];
        let vb = [self.velocities[2], self.velocities[3]];
        let vc = [self.velocities[4], self.velocities[5]];

        // Compute forces due to gravity for each body
        let f_on_a_from_b = gravity_equation(pa, pb);
        let f_on_a_from_c = gravity_equation(pa, pc);
        let f_on_b_from_a = gravity_equation(pb, pa);
        let f_on_b_from_c = gravity_equation(pb, pc);
        let f_on_c_from_a = gravity_equation(pc, pa);
        let f_on_c_from_b = gravity_equation(pc, pb);

        // Sum forces for each object
        let f_on_a = vec2_add(f_on_a_from_b, f_on_a_from_c);
        let f_on_b = vec2_add(f_on_b_from_a, f_on_b_from_c);
        let f_on_c = vec2_add(f_on_c_from_a, f_on_c_from_b);

        // Determine velocity updates
        let aa = vec2_scale(1.0/MASS, f_on_a);
        let ab = vec2_scale(1.0/MASS, f_on_b);
        let ac = vec2_scale(1.0/MASS, f_on_c);

        // Update positions
        let upa = vec2_add(pa, vec2_scale(dt, va));
        let upb = vec2_add(pb, vec2_scale(dt, vb));
        let upc = vec2_add(pc, vec2_scale(dt, vc));

        // Update velocities
        let uva = vec2_add(va, vec2_scale(dt, aa));
        let uvb = vec2_add(vb, vec2_scale(dt, ab));
        let uvc = vec2_add(vc, vec2_scale(dt, ac));

        // Update state
        self.positions = [ upa[0], upa[1],
                           upb[0], upb[1],
                           upc[0], upc[1] ];
        self.velocities = [ uva[0], uva[1],
                            uvb[0], uvb[1],
                            uvc[0], uvc[1] ];
    }
}


#[wasm_bindgen]
pub fn greet() {
    
}
