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

    pub fn x(&self) -> f64 {
        self.0
    }

    pub fn y(&self) -> f64 {
        self.1
    }

    pub fn w(&self) -> f64 {
        self.2
    }

    pub fn weight(&self) -> f64 {
        self.3
    }
}
