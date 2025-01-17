type Vector2D = nalgebra::Vector2<f32>;

#[derive(Debug, Default, Clone, Copy)]
pub struct Body {
    pub position: Vector2D,
    pub velocity: Vector2D
}

impl Body {
    pub fn new(pos: Vector2D, vel: Vector2D) -> Body {
        Body { position: pos, velocity: vel }
    }

    pub fn updated<ForceFunc>(&self, dt: f32, force_func: ForceFunc) -> Body
        where ForceFunc: Fn(&Vector2D, &Vector2D) -> Vector2D
    {
        let force = force_func(&self.position, &self.velocity);
        let new_position = self.position + self.velocity*dt;
        let new_velocity = self.velocity + force*dt;
        Body::new(new_position, new_velocity)
    }
}