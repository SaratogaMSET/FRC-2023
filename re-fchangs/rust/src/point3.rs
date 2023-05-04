#[derive(Clone, Copy, Debug)]
pub struct Point3(pub f64, pub f64, pub f64);

impl Point3 {
    pub fn x(&mut self) -> &mut f64 {
        &mut self.0
    }

    pub fn y(&mut self) -> &mut f64 {
        &mut self.1
    }

    pub fn z(&mut self) -> &mut f64 {
        &mut self.2
    }
}
