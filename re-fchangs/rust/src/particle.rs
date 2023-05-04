#[derive(Clone, Copy, Debug)]
pub struct Particle(pub f64, pub f64, pub f64, pub f64);

impl Default for Particle {
    fn default() -> Self {
        Self::new()
    }
}

impl Particle {
    pub fn new() -> Particle {
        Particle(0.0, 0.0, 0.0, 0.0)
    }

    pub fn x(&mut self) -> &mut f64 {
        &mut self.0
    }

    pub fn y(&mut self) -> &mut f64 {
        &mut self.1
    }

    pub fn w(&mut self) -> &mut f64 {
        &mut self.2
    }

    pub fn weight(&mut self) -> &mut f64 {
        &mut self.3
    }
}
