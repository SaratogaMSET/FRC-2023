#[derive(Clone, Copy, Debug)]
pub struct Point3(pub f64, pub f64, pub f64);

impl Point3 {
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
