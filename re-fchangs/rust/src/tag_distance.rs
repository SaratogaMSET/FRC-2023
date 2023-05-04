#[derive(Clone, Copy, Debug)]
pub struct TagDistance(pub f64, pub f64, pub f64);

impl TagDistance {
    pub fn x(&mut self) -> &mut f64 {
        &mut self.0
    }

    pub fn y(&mut self) -> &mut f64 {
        &mut self.1
    }

    pub fn distance(&mut self) -> &mut f64 {
        &mut self.2
    }
}
