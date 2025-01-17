use super::constants::*;
use super::body::Body;
use nalgebra::{Unit, Vector2};
type Vector2D = Vector2<f32>;

struct Boundary {
    normal: Unit<Vector2D>,
    offset: f32,
}

impl Boundary {
    fn new(norm: Unit<Vector2D>, ofs: f32) -> Self {
        Self { normal: norm, offset: ofs }
    }

    fn boundary_offset(&self, body: &Body) -> f32 {
        self.normal.dot(&body.position) + self.offset
    }

    fn body_crosses(&self, body: &Body) -> bool {
        self.boundary_offset(body) < 0.0
    }

    fn correct_body(&self, body: &Body) -> Body {
        let position_correction = self.normal.into_inner() * self.boundary_offset(body);
        let velocity_correction = 2.*self.normal.dot(&body.velocity) * self.normal.into_inner();
        let new_position = body.position - position_correction;
        let new_velocity = COLLISION_ENERGY_LOSS * (body.velocity - velocity_correction);
        Body::new(new_position, new_velocity)
    }
}

pub fn boundary_collision_correct(body: &Body, width: f32, height: f32) -> Body {
    let x_wall = width/2.0 - RADIUS;
    let y_wall = height/2.0 - RADIUS;
    let x_norm = Vector2D::x_axis();
    let y_norm = Vector2D::y_axis();
    let boundaries: [Boundary; 4] = [
        Boundary::new(x_norm, x_wall), Boundary::new(-x_norm, x_wall),
        Boundary::new(y_norm, y_wall), Boundary::new(-y_norm, y_wall)
    ];
    let opt_bound = boundaries.iter().find(|&b| b.body_crosses(body));
    if let Some(bound) = opt_bound { bound.correct_body(body) }
    else { *body }
}