#[derive(Clone, Copy, Debug)]
pub struct TagDistance(pub f64, pub f64, pub f64);

impl TagDistance {
    pub fn x(&self) -> f64 {
        self.0
    }

    pub fn y(&self) -> f64 {
        self.1
    }

    pub fn distance(&self) -> f64 {
        self.2
    }
}
