#[derive(Clone, Copy, Debug)]
pub struct Tag(pub f64, pub f64, pub f64);

impl Tag {
    pub fn x(&self) -> f64 {
        self.0
    }

    pub fn y(&self) -> f64 {
        self.1
    }

    pub fn z(&self) -> f64 {
        self.2
    }
}
